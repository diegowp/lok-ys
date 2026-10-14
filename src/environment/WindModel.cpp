#include "LokYS/environment/WindModel.hpp"
#include <cmath>
#include <algorithm>

namespace LokYS::Environment {

namespace {

constexpr f64 PI = 3.14159265358979323846;
constexpr f64 DEG_TO_RAD = PI / 180.0;

f64 NormalizeDegrees360(f64 deg) noexcept {
    f64 result = std::fmod(deg, 360.0);
    if (result < 0.0) {
        result += 360.0;
    }
    return result;
}

f64 NormalizeDegrees180(f64 deg) noexcept {
    f64 result = NormalizeDegrees360(deg);
    if (result > 180.0) {
        result -= 360.0;
    }
    return result;
}

} // namespace

WindModel::WindModel()
    : m_rng(42) // Fixed seed for reproducible default behavior
{
}

WindModel::WindModel(u32 randomSeed)
    : m_rng(randomSeed)
{
}

void WindModel::ResetTurbulence() noexcept {
    m_gustNed = { 0.0, 0.0, 0.0 };
}

Math::Vec3d WindModel::CalculateSteadyWindNed(f64 altitudeM) const noexcept {
    if (m_windConfig.surfaceSpeedMps <= 0.0 && m_windConfig.aloftSpeedMps <= 0.0) {
        return { 0.0, 0.0, 0.0 };
    }

    f64 h = std::max(0.01, altitudeM);
    f64 z0 = std::max(0.001, m_windConfig.roughnessLengthM);
    f64 zRef = std::max(1.0, m_windConfig.referenceHeightM);

    f64 speed = 0.0;
    f64 directionDeg = m_windConfig.surfaceDirectionDeg;

    if (h <= 300.0) {
        // Planetary Boundary Layer (Logarithmic Wind Profile)
        if (h > z0) {
            f64 logRatio = std::log(h / z0) / std::log(zRef / z0);
            speed = m_windConfig.surfaceSpeedMps * std::max(0.0, logRatio);
        } else {
            speed = 0.0;
        }
    } else if (h < m_windConfig.aloftAltitudeM) {
        // Transition layer between boundary layer and aloft free stream
        f64 v300 = m_windConfig.surfaceSpeedMps * (std::log(300.0 / z0) / std::log(zRef / z0));
        f64 alpha = (h - 300.0) / std::max(1.0, m_windConfig.aloftAltitudeM - 300.0);
        alpha = std::clamp(alpha, 0.0, 1.0);

        speed = (1.0 - alpha) * v300 + alpha * m_windConfig.aloftSpeedMps;

        f64 dirDiff = NormalizeDegrees180(m_windConfig.aloftDirectionDeg - m_windConfig.surfaceDirectionDeg);
        directionDeg = NormalizeDegrees360(m_windConfig.surfaceDirectionDeg + alpha * dirDiff);
    } else {
        // Aloft free stream
        speed = m_windConfig.aloftSpeedMps;
        directionDeg = m_windConfig.aloftDirectionDeg;
    }

    // Convert from aeronautical "blowing from" direction to NED flow velocity
    f64 dirRad = directionDeg * DEG_TO_RAD;
    f64 vx = -speed * std::cos(dirRad);
    f64 vy = -speed * std::sin(dirRad);

    return { vx, vy, 0.0 };
}

f64 WindModel::CalculateThermalUpdraftMps(const Math::Vec3d& aircraftPosNed) const noexcept {
    if (m_turbConfig.thermalUpdraftStrengthMps <= 0.0 || m_turbConfig.thermalRadiusM <= 1.0) {
        return 0.0;
    }

    f64 altM = -aircraftPosNed.z;
    if (altM < 10.0 || altM > 2500.0) {
        return 0.0; // Inactive outside convection boundary
    }

    f64 dx = aircraftPosNed.x - m_turbConfig.thermalCenterNed.x;
    f64 dy = aircraftPosNed.y - m_turbConfig.thermalCenterNed.y;
    f64 distSq = dx * dx + dy * dy;
    f64 r = m_turbConfig.thermalRadiusM;

    // Updraft core with surrounding compensative sink
    f64 rNormSq = distSq / (r * r);
    f64 w = m_turbConfig.thermalUpdraftStrengthMps * std::exp(-0.5 * rNormSq);

    // Altitude profile (ramps up from ground, peaks at 800m, tapers at 2000m)
    f64 altFactor = 1.0;
    if (altM < 200.0) {
        altFactor = altM / 200.0;
    } else if (altM > 1500.0) {
        altFactor = std::max(0.0, (2500.0 - altM) / 1000.0);
    }

    return w * altFactor;
}

void WindModel::DecomposeRunwayWind(
    f64 windSpeedMps,
    f64 windFromDeg,
    f64 rwyHeadingDeg,
    f64& outHeadwindMps,
    f64& outCrosswindMps) noexcept
{
    f64 angleDiffRad = NormalizeDegrees180(windFromDeg - rwyHeadingDeg) * DEG_TO_RAD;
    outHeadwindMps = windSpeedMps * std::cos(angleDiffRad);
    outCrosswindMps = windSpeedMps * std::sin(angleDiffRad);
}

Math::Vec3d WindModel::EvaluateTotalWindNed(
    const Math::Vec3d& aircraftPosNed,
    f64 trueAirspeedMps,
    f64 dt) noexcept
{
    f64 altM = -aircraftPosNed.z;
    m_lastSteadyWindNed = CalculateSteadyWindNed(altM);

    // Update Dryden spectral turbulence filter
    if (m_turbConfig.severity != TurbulenceSeverity::None && dt > 0.0001) {
        f64 sigmaW = 0.0;
        switch (m_turbConfig.severity) {
        case TurbulenceSeverity::Light:    sigmaW = 0.50; break; // ~1 kt
        case TurbulenceSeverity::Moderate: sigmaW = 1.50; break; // ~3 kts
        case TurbulenceSeverity::Severe:   sigmaW = 3.00; break; // ~6 kts
        case TurbulenceSeverity::None:     sigmaW = 0.00; break;
        }

        if (m_turbConfig.gustFactor > 1.0) {
            sigmaW *= m_turbConfig.gustFactor;
        }

        f64 tas = std::max(10.0, trueAirspeedMps);
        f64 h = std::clamp(altM, 5.0, 1000.0);

        // Low altitude Dryden length scales (MIL-F-8785C)
        f64 Lw = h;
        f64 denom = std::pow(0.177 + 0.000823 * h, 1.2);
        f64 Lu = h / std::max(0.01, denom);
        f64 Lv = Lu;

        f64 sigmaU = sigmaW / std::pow(0.177 + 0.000823 * h, 0.4);
        f64 sigmaV = sigmaU;

        f64 tauU = std::max(0.05, Lu / tas);
        f64 tauV = std::max(0.05, Lv / tas);
        f64 tauW = std::max(0.05, Lw / tas);

        f64 stepDt = std::clamp(dt, 0.001, 0.05);

        // Discrete Ornstein-Uhlenbeck / Dryden step
        f64 etaU = m_normalDist(m_rng);
        f64 etaV = m_normalDist(m_rng);
        f64 etaW = m_normalDist(m_rng);

        m_gustNed.x += (-m_gustNed.x / tauU) * stepDt + sigmaU * std::sqrt(2.0 * stepDt / tauU) * etaU;
        m_gustNed.y += (-m_gustNed.y / tauV) * stepDt + sigmaV * std::sqrt(2.0 * stepDt / tauV) * etaV;
        m_gustNed.z += (-m_gustNed.z / tauW) * stepDt + sigmaW * std::sqrt(2.0 * stepDt / tauW) * etaW;
    } else {
        m_gustNed = { 0.0, 0.0, 0.0 };
    }

    m_lastThermalUpdraft = CalculateThermalUpdraftMps(aircraftPosNed);

    Math::Vec3d total = m_lastSteadyWindNed + m_gustNed;
    // In NED, vertical updraft corresponds to negative Z velocity
    total.z -= m_lastThermalUpdraft;

    return total;
}

} // namespace LokYS::Environment
