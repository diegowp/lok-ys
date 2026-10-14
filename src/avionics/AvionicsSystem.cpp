#include "LokYS/avionics/AvionicsSystem.hpp"
#include <algorithm>
#include <cmath>

namespace LokYS::Avionics {

AvionicsSystem::AvionicsSystem() {
    m_state.altimeter.kollsmanInHg = 29.92;
    m_state.heading.headingBugDeg = 0.0;
}

void AvionicsSystem::Update(
    const Physics::RigidBodyState& state,
    const Math::Vec3d& accelerationBody,
    f64 ambientPressurePa,
    f64 ambientDensityKgM3,
    f64 dt) noexcept
{
    if (dt <= 0.0) return;

    // 1. Derive Euler angles from attitude quaternion
    f64 yawRad = 0.0, pitchRad = 0.0, rollRad = 0.0;
    state.attitude.ToEuler(yawRad, pitchRad, rollRad);

    // 2. Airspeed Indicator (Pitot-Static)
    f64 trueAirspeedMps = state.velocityBody.Length();
    UpdateAirspeed(trueAirspeedMps, ambientDensityKgM3, dt);

    // 3. Attitude Indicator (Earth-Vertical Gyro)
    UpdateAttitude(state.attitude, state.angularVelocityBody, dt);

    // 4. Altimeter (Aneroid Barometer)
    UpdateAltimeter(ambientPressurePa, dt);

    // 5. Turn Coordinator (Canted Rate Gyro & Inclinometer Ball)
    UpdateTurnCoordinator(state.angularVelocityBody, accelerationBody, rollRad, pitchRad, dt);

    // 6. Heading Indicator (Directional Gyro)
    UpdateHeading(yawRad, dt);

    // 7. Vertical Speed Indicator (Capillary Metered Leak)
    Math::Vec3d vNed = state.attitude.Rotate(state.velocityBody);
    f64 vUpMps = -vNed.z; // -Z_NED is Up
    UpdateVerticalSpeed(vUpMps, dt);
}

void AvionicsSystem::ResetToSteadyState(
    const Physics::RigidBodyState& state,
    f64 ambientPressurePa,
    f64 ambientDensityKgM3) noexcept
{
    // Derive Euler angles
    f64 yawRad = 0.0, pitchRad = 0.0, rollRad = 0.0;
    state.attitude.ToEuler(yawRad, pitchRad, rollRad);

    // Airspeed
    f64 trueAirspeedMps = state.velocityBody.Length();
    f64 q = 0.5 * ambientDensityKgM3 * trueAirspeedMps * trueAirspeedMps;
    f64 iasMps = std::sqrt((2.0 * q) / Math::Constants::SEA_LEVEL_DENSITY);
    f64 iasKnots = Math::Units::MpsToKnots(iasMps);

    m_state.airspeed.trueAirspeedKnots = Math::Units::MpsToKnots(trueAirspeedMps);
    m_state.airspeed.dynamicPressurePa = q;
    m_state.airspeed.indicatedAirspeedKnots = iasKnots;
    if (iasKnots < 40.0) {
        m_state.airspeed.needleAngleDeg = (std::max(0.0, iasKnots) / 40.0) * 20.0;
    } else {
        m_state.airspeed.needleAngleDeg = 20.0 + std::clamp((iasKnots - 40.0) / 160.0, 0.0, 1.0) * 320.0;
    }

    // Attitude
    m_state.attitude.pitchDeg = Math::Units::RadToDeg(pitchRad);
    m_state.attitude.rollDeg = Math::Units::RadToDeg(rollRad);
    m_state.attitude.pitchDisplacementNorm = std::clamp(m_state.attitude.pitchDeg / 20.0, -1.0, 1.0);
    m_state.attitude.rollAngleRad = rollRad;
    m_state.attitude.isTumbled = (std::abs(m_state.attitude.pitchDeg) > 85.0);

    // Altimeter
    f64 pSettingPa = Math::Units::InHgToPascals(m_state.altimeter.kollsmanInHg);
    f64 pRatio = std::clamp(ambientPressurePa / pSettingPa, 1e-6, 2.0);
    f64 hMeters = 44330.77 * (1.0 - std::pow(pRatio, 0.190263));
    f64 hFeet = Math::Units::MetersToFeet(hMeters);

    f64 pStdRatio = std::clamp(ambientPressurePa / Math::Constants::STANDARD_SEA_LEVEL_PRESSURE, 1e-6, 2.0);
    f64 hStdMeters = 44330.77 * (1.0 - std::pow(pStdRatio, 0.190263));

    m_state.altimeter.staticPressurePa = ambientPressurePa;
    m_state.altimeter.indicatedAltitudeFeet = hFeet;
    m_state.altimeter.pressureAltitudeFeet = Math::Units::MetersToFeet(hStdMeters);

    f64 hWrap1k = std::fmod(std::fmod(hFeet, 1000.0) + 1000.0, 1000.0);
    f64 hWrap10k = std::fmod(std::fmod(hFeet, 10000.0) + 10000.0, 10000.0);
    f64 hWrap100k = std::fmod(std::fmod(hFeet, 100000.0) + 100000.0, 100000.0);
    m_state.altimeter.needle100FtAngleDeg = (hWrap1k / 1000.0) * 360.0;
    m_state.altimeter.needle1000FtAngleDeg = (hWrap10k / 10000.0) * 360.0;
    m_state.altimeter.needle10000FtAngleDeg = (hWrap100k / 100000.0) * 360.0;

    // Turn Coordinator
    m_state.turnCoordinator.turnRateDps = 0.0;
    m_state.turnCoordinator.symbolRollDeg = 0.0;
    m_state.turnCoordinator.slipSkidBallPosition = 0.0;
    m_state.turnCoordinator.lateralSpecificForceMps2 = 0.0;

    // Heading
    f64 headingDeg = Math::Units::RadToDeg(yawRad);
    headingDeg = std::fmod(std::fmod(headingDeg, 360.0) + 360.0, 360.0);
    m_state.heading.indicatedHeadingDeg = headingDeg;
    m_state.heading.gyroDriftDeg = 0.0;
    m_state.heading.compassCardAngleDeg = -headingDeg;

    // Vertical Speed
    Math::Vec3d vNed = state.attitude.Rotate(state.velocityBody);
    f64 vUpMps = -vNed.z;
    f64 vsiFpm = Math::Units::MpsToFeetPerMinute(vUpMps);
    m_state.verticalSpeed.rawVerticalSpeedFpm = vsiFpm;
    m_state.verticalSpeed.verticalSpeedFpm = vsiFpm;
    m_state.verticalSpeed.needleAngleDeg = std::clamp(vsiFpm / 2000.0, -1.0, 1.0) * 170.0;
}

void AvionicsSystem::SetKollsman(f64 inHg) noexcept {
    m_state.altimeter.kollsmanInHg = std::clamp(inHg, 27.0, 32.0);
}

void AvionicsSystem::AdjustKollsman(f64 deltaInHg) noexcept {
    SetKollsman(m_state.altimeter.kollsmanInHg + deltaInHg);
}

void AvionicsSystem::SetHeadingBug(f64 bugDeg) noexcept {
    m_state.heading.headingBugDeg = std::fmod(std::fmod(bugDeg, 360.0) + 360.0, 360.0);
}

void AvionicsSystem::AdjustHeadingBug(f64 deltaDeg) noexcept {
    SetHeadingBug(m_state.heading.headingBugDeg + deltaDeg);
}

void AvionicsSystem::AlignHeadingWithCompass() noexcept {
    m_state.heading.gyroDriftDeg = 0.0;
}

void AvionicsSystem::CageAttitude(bool caged) noexcept {
    m_state.attitude.isCaged = caged;
    if (caged) {
        m_state.attitude.pitchDeg = 0.0;
        m_state.attitude.rollDeg = 0.0;
        m_state.attitude.pitchDisplacementNorm = 0.0;
        m_state.attitude.rollAngleRad = 0.0;
        m_state.attitude.isTumbled = false;
    }
}

void AvionicsSystem::ToggleAttitudeCage() noexcept {
    CageAttitude(!m_state.attitude.isCaged);
}

void AvionicsSystem::SetTimeConstants(
    f64 tauAsi,
    f64 tauGyro,
    f64 tauAlt,
    f64 tauTc,
    f64 tauBall,
    f64 tauVsi) noexcept
{
    m_tauAsi = std::max(0.001, tauAsi);
    m_tauGyro = std::max(0.001, tauGyro);
    m_tauAlt = std::max(0.001, tauAlt);
    m_tauTc = std::max(0.001, tauTc);
    m_tauBall = std::max(0.001, tauBall);
    m_tauVsi = std::max(0.001, tauVsi);
}

void AvionicsSystem::UpdateAirspeed(f64 speedMps, f64 ambientDensity, f64 dt) noexcept {
    f64 q = 0.5 * ambientDensity * speedMps * speedMps;
    f64 iasMps = std::sqrt((2.0 * q) / Math::Constants::SEA_LEVEL_DENSITY);
    f64 targetIasKnots = Math::Units::MpsToKnots(iasMps);

    // Pitot-static line pneumatic acoustic lag filter
    f64 alpha = 1.0 - std::exp(-dt / m_tauAsi);
    m_state.airspeed.indicatedAirspeedKnots += (targetIasKnots - m_state.airspeed.indicatedAirspeedKnots) * alpha;
    m_state.airspeed.trueAirspeedKnots = Math::Units::MpsToKnots(speedMps);
    m_state.airspeed.dynamicPressurePa = q;

    // Dial needle angular deflection
    f64 ias = m_state.airspeed.indicatedAirspeedKnots;
    if (ias < 40.0) {
        m_state.airspeed.needleAngleDeg = (std::max(0.0, ias) / 40.0) * 20.0;
    } else {
        m_state.airspeed.needleAngleDeg = 20.0 + std::clamp((ias - 40.0) / 160.0, 0.0, 1.0) * 320.0;
    }
}

void AvionicsSystem::UpdateAttitude(const Math::Quat4d& attitude, const Math::Vec3d& /*angularVelBody*/, f64 dt) noexcept {
    if (m_state.attitude.isCaged) {
        m_state.attitude.pitchDeg = 0.0;
        m_state.attitude.rollDeg = 0.0;
        m_state.attitude.pitchDisplacementNorm = 0.0;
        m_state.attitude.rollAngleRad = 0.0;
        return;
    }

    f64 yawRad = 0.0, targetPitchRad = 0.0, targetRollRad = 0.0;
    attitude.ToEuler(yawRad, targetPitchRad, targetRollRad);

    f64 targetPitchDeg = Math::Units::RadToDeg(targetPitchRad);
    f64 targetRollDeg = Math::Units::RadToDeg(targetRollRad);

    // Gyro tumble detection at mechanical limits
    if (std::abs(targetPitchDeg) > 85.0) {
        m_state.attitude.isTumbled = true;
    }

    // Gimbal inertia lag
    f64 alpha = 1.0 - std::exp(-dt / m_tauGyro);
    m_state.attitude.pitchDeg += (targetPitchDeg - m_state.attitude.pitchDeg) * alpha;
    m_state.attitude.rollDeg += (targetRollDeg - m_state.attitude.rollDeg) * alpha;

    // Normalized visual horizon displacement
    m_state.attitude.pitchDisplacementNorm = std::clamp(m_state.attitude.pitchDeg / 20.0, -1.0, 1.0);
    m_state.attitude.rollAngleRad = Math::Units::DegToRad(m_state.attitude.rollDeg);
}

void AvionicsSystem::UpdateAltimeter(f64 staticPressurePa, f64 dt) noexcept {
    m_state.altimeter.staticPressurePa = staticPressurePa;

    f64 pSettingPa = Math::Units::InHgToPascals(m_state.altimeter.kollsmanInHg);
    f64 pRatio = std::clamp(staticPressurePa / pSettingPa, 1e-6, 2.0);
    f64 hMeters = 44330.77 * (1.0 - std::pow(pRatio, 0.190263));
    f64 targetAltFeet = Math::Units::MetersToFeet(hMeters);

    f64 pStdRatio = std::clamp(staticPressurePa / Math::Constants::STANDARD_SEA_LEVEL_PRESSURE, 1e-6, 2.0);
    f64 hStdMeters = 44330.77 * (1.0 - std::pow(pStdRatio, 0.190263));
    m_state.altimeter.pressureAltitudeFeet = Math::Units::MetersToFeet(hStdMeters);

    // Static port pneumatic damping lag
    f64 alpha = 1.0 - std::exp(-dt / m_tauAlt);
    m_state.altimeter.indicatedAltitudeFeet += (targetAltFeet - m_state.altimeter.indicatedAltitudeFeet) * alpha;

    f64 h = m_state.altimeter.indicatedAltitudeFeet;
    f64 hWrap1k = std::fmod(std::fmod(h, 1000.0) + 1000.0, 1000.0);
    f64 hWrap10k = std::fmod(std::fmod(h, 10000.0) + 10000.0, 10000.0);
    f64 hWrap100k = std::fmod(std::fmod(h, 100000.0) + 100000.0, 100000.0);

    m_state.altimeter.needle100FtAngleDeg = (hWrap1k / 1000.0) * 360.0;
    m_state.altimeter.needle1000FtAngleDeg = (hWrap10k / 10000.0) * 360.0;
    m_state.altimeter.needle10000FtAngleDeg = (hWrap100k / 100000.0) * 360.0;
}

void AvionicsSystem::UpdateTurnCoordinator(
    const Math::Vec3d& angularVelBody,
    const Math::Vec3d& accelerationBody,
    f64 rollRad,
    f64 pitchRad,
    f64 dt) noexcept
{
    // Canted rate gyro: canted 30 deg up to sense both yaw rate and roll rate
    constexpr f64 cantAngleRad = 30.0 * (Math::Constants::PI / 180.0);
    f64 sensedTurnRateRad = angularVelBody.z * std::cos(cantAngleRad) + angularVelBody.x * std::sin(cantAngleRad);
    f64 targetTurnRateDps = Math::Units::RadToDeg(sensedTurnRateRad);

    // Rate gyro spring-damper filter
    f64 alphaTc = 1.0 - std::exp(-dt / m_tauTc);
    m_state.turnCoordinator.turnRateDps += (targetTurnRateDps - m_state.turnCoordinator.turnRateDps) * alphaTc;

    // Standard rate turn (3.0 deg/s) deflects the miniature aircraft symbol by 20.0 degrees
    f64 turnFrac = m_state.turnCoordinator.turnRateDps / TurnCoordinatorState::StandardRateDps;
    m_state.turnCoordinator.symbolRollDeg = std::clamp(turnFrac * 20.0, -45.0, 45.0);

    // Inclinometer slip/skid ball:
    // Net lateral specific force: gravity component minus lateral kinematic acceleration
    f64 g = Math::Constants::STANDARD_GRAVITY;
    f64 latSpecificForce = g * std::sin(rollRad) * std::cos(pitchRad) - accelerationBody.y;
    m_state.turnCoordinator.lateralSpecificForceMps2 = latSpecificForce;

    // Ball deflection: 15 deg bank imbalance gives full scale (+/- 1.0)
    f64 fullScaleImbalance = g * std::sin(15.0 * (Math::Constants::PI / 180.0));
    f64 targetBallPos = std::clamp(latSpecificForce / fullScaleImbalance, -1.0, 1.0);

    // Viscous fluid damping
    f64 alphaBall = 1.0 - std::exp(-dt / m_tauBall);
    m_state.turnCoordinator.slipSkidBallPosition += (targetBallPos - m_state.turnCoordinator.slipSkidBallPosition) * alphaBall;
}

void AvionicsSystem::UpdateHeading(f64 yawRad, f64 dt) noexcept {
    // Gyro mechanical precession drift
    m_state.heading.gyroDriftDeg += m_gyroDriftRateDegPerSec * dt;

    f64 targetHeadingDeg = Math::Units::RadToDeg(yawRad);
    targetHeadingDeg = std::fmod(std::fmod(targetHeadingDeg + m_state.heading.gyroDriftDeg, 360.0) + 360.0, 360.0);

    // Angular difference with wrap-around
    f64 diff = targetHeadingDeg - m_state.heading.indicatedHeadingDeg;
    while (diff > 180.0) diff -= 360.0;
    while (diff < -180.0) diff += 360.0;

    f64 alpha = 1.0 - std::exp(-dt / 0.08); // 80 ms directional gyro lag
    m_state.heading.indicatedHeadingDeg = std::fmod(std::fmod(m_state.heading.indicatedHeadingDeg + diff * alpha, 360.0) + 360.0, 360.0);
    m_state.heading.compassCardAngleDeg = -m_state.heading.indicatedHeadingDeg;
}

void AvionicsSystem::UpdateVerticalSpeed(f64 verticalVelocityMps, f64 dt) noexcept {
    f64 targetFpm = Math::Units::MpsToFeetPerMinute(verticalVelocityMps);
    m_state.verticalSpeed.rawVerticalSpeedFpm = targetFpm;

    // Capillary metering leak pneumatic delay filter (tau = 2.5s)
    f64 alpha = 1.0 - std::exp(-dt / m_tauVsi);
    m_state.verticalSpeed.verticalSpeedFpm += (targetFpm - m_state.verticalSpeed.verticalSpeedFpm) * alpha;

    // Needle deflection: +/- 2,000 fpm full scale maps to +/- 170 degrees from 9 o'clock
    f64 frac = std::clamp(m_state.verticalSpeed.verticalSpeedFpm / VerticalSpeedIndicatorState::MaxScaleFpm, -1.0, 1.0);
    m_state.verticalSpeed.needleAngleDeg = frac * 170.0;
}

} // namespace LokYS::Avionics
