#include "LokYS/environment/AtmosphereModel.hpp"
#include <cmath>
#include <algorithm>

namespace LokYS::Environment {

namespace {

// ISA Barometric exponent n = -g0 / (L * R)
constexpr f64 BAROMETRIC_EXPONENT = -Constants::STANDARD_GRAVITY_ACCEL /
    (Constants::TROPOSPHERE_LAPSE_RATE_K_M * Constants::SPECIFIC_GAS_CONSTANT_AIR); // ~5.255876

// Base pressure at tropopause (11000m)
constexpr f64 TROPOPAUSE_BASE_PRESSURE_RATIO = 0.223361; // (216.65 / 288.15)^n

} // namespace

f64 AtmosphereModel::CalculateSpeedOfSound(f64 tempK) noexcept {
    if (tempK <= 0.0) {
        tempK = 1.0;
    }
    return std::sqrt(Constants::SPECIFIC_HEAT_RATIO_GAMMA * Constants::SPECIFIC_GAS_CONSTANT_AIR * tempK);
}

f64 AtmosphereModel::CalculateDynamicViscosity(f64 tempK) noexcept {
    if (tempK <= 0.0) {
        tempK = 1.0;
    }
    // Sutherland's law: mu = mu0 * (T/T0)^1.5 * (T0 + S) / (T + S)
    f64 tRatio = tempK / Constants::SUTHERLAND_REFERENCE_TEMP_K;
    f64 num = Constants::SUTHERLAND_REFERENCE_TEMP_K + Constants::SUTHERLAND_CONSTANT_K;
    f64 den = tempK + Constants::SUTHERLAND_CONSTANT_K;
    return Constants::SUTHERLAND_REFERENCE_VISC * std::pow(tRatio, 1.5) * (num / den);
}

f64 AtmosphereModel::CalculatePressureAltitudeFt(f64 pressurePa) noexcept {
    if (pressurePa <= 0.0) {
        return 80000.0;
    }
    f64 pRatio = pressurePa / Constants::STANDARD_SEA_LEVEL_PRESS_PA;
    f64 invN = 1.0 / BAROMETRIC_EXPONENT;
    f64 hM = (Constants::STANDARD_SEA_LEVEL_TEMP_K / -Constants::TROPOSPHERE_LAPSE_RATE_K_M) *
        (1.0 - std::pow(pRatio, invN));
    return hM * Constants::METERS_TO_FEET;
}

f64 AtmosphereModel::CalculateDensityAltitudeFt(f64 densityKgM3) noexcept {
    if (densityKgM3 <= 0.0) {
        return 80000.0;
    }
    f64 dRatio = densityKgM3 / Constants::STANDARD_SEA_LEVEL_DENSITY;
    f64 invExp = 1.0 / (BAROMETRIC_EXPONENT - 1.0); // exponent for density ratio in troposphere
    f64 hM = (Constants::STANDARD_SEA_LEVEL_TEMP_K / -Constants::TROPOSPHERE_LAPSE_RATE_K_M) *
        (1.0 - std::pow(dRatio, invExp));
    return hM * Constants::METERS_TO_FEET;
}

AtmosphericState AtmosphereModel::Evaluate(
    f64 altitudeM,
    f64 deltaIsaK,
    f64 seaLevelPressurePa) noexcept
{
    AtmosphericState state{};

    // Clamp minimum altitude to sea level or -500m (Death Valley / Dead Sea)
    f64 h = std::clamp(altitudeM, -500.0, 25000.0);

    if (h <= Constants::TROPOPAUSE_ALTITUDE_M) {
        // 1. Troposphere (0 to 11,000m)
        f64 stdTempK = Constants::STANDARD_SEA_LEVEL_TEMP_K + Constants::TROPOSPHERE_LAPSE_RATE_K_M * h;
        state.temperatureKelvin = stdTempK + deltaIsaK;

        f64 tempRatio = stdTempK / Constants::STANDARD_SEA_LEVEL_TEMP_K;
        state.pressurePa = seaLevelPressurePa * std::pow(tempRatio, BAROMETRIC_EXPONENT);
    } else {
        // 2. Tropopause / Lower Stratosphere (11,000m to 25,000m)
        state.temperatureKelvin = Constants::TROPOPAUSE_TEMP_K + deltaIsaK;

        f64 p11000 = seaLevelPressurePa * TROPOPAUSE_BASE_PRESSURE_RATIO;
        f64 deltaH = h - Constants::TROPOPAUSE_ALTITUDE_M;
        f64 expTerm = (-Constants::STANDARD_GRAVITY_ACCEL * deltaH) /
            (Constants::SPECIFIC_GAS_CONSTANT_AIR * Constants::TROPOPAUSE_TEMP_K);
        state.pressurePa = p11000 * std::exp(expTerm);
    }

    state.temperatureCelsius = state.temperatureKelvin - 273.15;
    state.pressureInHg = state.pressurePa / Constants::PASCALS_PER_INHG;
    state.densityKgM3 = state.pressurePa / (Constants::SPECIFIC_GAS_CONSTANT_AIR * state.temperatureKelvin);
    state.speedOfSoundMps = CalculateSpeedOfSound(state.temperatureKelvin);
    state.dynamicViscosity = CalculateDynamicViscosity(state.temperatureKelvin);
    state.pressureAltitudeFt = CalculatePressureAltitudeFt(state.pressurePa);
    state.densityAltitudeFt = CalculateDensityAltitudeFt(state.densityKgM3);

    return state;
}

} // namespace LokYS::Environment
