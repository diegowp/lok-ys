#include "LokYS/autopilot/AutopilotSystem.hpp"
#include "LokYS/math/Units.hpp"
#include <cmath>
#include <sstream>
#include <iomanip>
#include <algorithm>

namespace LokYS::Autopilot {

namespace {

[[nodiscard]] inline f64 NormalizeAngleDeg180(f64 deg) noexcept {
    if (!std::isfinite(deg)) return 0.0;
    f64 d = std::fmod(deg + 180.0, 360.0);
    if (d < 0.0) d += 360.0;
    return d - 180.0;
}

[[nodiscard]] inline f64 NormalizeAngleDeg360(f64 deg) noexcept {
    f64 d = std::fmod(deg, 360.0);
    if (d < 0.0) d += 360.0;
    return d;
}

} // anonymous namespace

AutopilotSystem::AutopilotSystem() {
    Initialize();
}

void AutopilotSystem::Initialize() noexcept {
    // 1. Roll inner-loop PID gains: drives bank angle error -> aileron deflection
    // Max bank commanded is 25 deg. At 10 deg bank error -> ~0.25 aileron deflection
    PidGains rollGains{};
    rollGains.kp = 0.025;
    rollGains.ki = 0.003;
    rollGains.kd = 0.012;
    rollGains.minIntegral = -0.25;
    rollGains.maxIntegral = +0.25;
    rollGains.minOutput = -0.50;
    rollGains.maxOutput = +0.50;
    rollGains.maxRate = 2.0;               // Max 2.0 full deflections per second
    rollGains.derivativeFilterAlpha = 0.25;
    m_rollPid.SetGains(rollGains);

    // 2. Pitch inner-loop PID gains: drives pitch attitude error -> elevator command
    // In LokYS, +elevatorCommand pitches the aircraft nose UP (+pitch rate, -tail surface)
    PidGains pitchGains{};
    pitchGains.kp = 0.040;
    pitchGains.ki = 0.005;
    pitchGains.kd = 0.018;
    pitchGains.minIntegral = -0.30;
    pitchGains.maxIntegral = +0.30;
    pitchGains.minOutput = -0.55;
    pitchGains.maxOutput = +0.55;
    pitchGains.maxRate = 2.0;
    pitchGains.derivativeFilterAlpha = 0.25;
    m_pitchPid.SetGains(pitchGains);

    // 3. Vertical Speed outer-loop PID gains: drives VS error (fpm) -> commanded pitch angle (deg)
    PidGains vsGains{};
    vsGains.kp = 0.004;                   // 500 fpm error -> ~2.0 deg pitch change
    vsGains.ki = 0.0006;
    vsGains.kd = 0.001;
    vsGains.minIntegral = -3.0;
    vsGains.maxIntegral = +3.0;
    vsGains.minOutput = -8.0;
    vsGains.maxOutput = +8.0;
    vsGains.maxRate = 5.0;
    vsGains.derivativeFilterAlpha = 0.20;
    m_vsPid.SetGains(vsGains);

    Reset();
}

void AutopilotSystem::Reset() noexcept {
    m_status.masterEngaged = false;
    m_status.flightDirectorActive = false;
    m_status.yawDamperActive = false;
    m_status.lateralMode = LateralMode::ROL;
    m_status.verticalMode = VerticalMode::ALT;
    m_status.lateralArmed = LateralArmedMode::None;
    m_status.verticalArmed = VerticalArmedMode::None;
    m_status.disconnectWarning = false;
    m_status.disconnectTimer = 0.0;
    m_status.lastDisconnectReason = DisconnectReason::None;
    m_status.altitudeCaptured = false;
    m_status.gsCaptured = false;

    m_status.targets.targetHeadingDeg = 360.0;
    m_status.targets.targetAltitudeFt = 3500.0;
    m_status.targets.targetVerticalSpeedFpm = 500.0;
    m_status.targets.targetPitchDeg = 3.0;
    m_status.targets.targetRollDeg = 0.0;

    m_rollPid.Reset();
    m_pitchPid.Reset();
    m_vsPid.Reset();

    m_output = {};
}

void AutopilotSystem::ToggleMaster() noexcept {
    SetMasterEngaged(!m_status.masterEngaged);
}

void AutopilotSystem::SetMasterEngaged(bool engaged) noexcept {
    if (m_status.masterEngaged == engaged) return;

    m_status.masterEngaged = engaged;
    if (engaged) {
        m_status.flightDirectorActive = true; // Auto-activate FD on AP engage
        m_status.disconnectWarning = false;
        m_status.lastDisconnectReason = DisconnectReason::None;
        m_rollPid.Reset();
        m_pitchPid.Reset();
        m_vsPid.Reset();
    } else {
        m_status.disconnectWarning = true;
        m_status.disconnectTimer = 2.0;
        m_status.lastDisconnectReason = DisconnectReason::PilotButtonToggle;
        m_output.engaged = false;
        m_output.elevatorCommand = 0.0;
        m_output.aileronCommand = 0.0;
    }
}

void AutopilotSystem::ToggleFlightDirector() noexcept {
    SetFlightDirectorActive(!m_status.flightDirectorActive);
}

void AutopilotSystem::SetFlightDirectorActive(bool active) noexcept {
    m_status.flightDirectorActive = active;
}

void AutopilotSystem::CycleLateralMode() noexcept {
    switch (m_status.lateralMode) {
    case LateralMode::ROL:
        SetLateralMode(LateralMode::HDG);
        break;
    case LateralMode::HDG:
        SetLateralMode(LateralMode::NAV);
        break;
    case LateralMode::NAV:
        SetLateralMode(LateralMode::APPR);
        break;
    case LateralMode::APPR:
    default:
        SetLateralMode(LateralMode::ROL);
        break;
    }
}

void AutopilotSystem::SetLateralMode(LateralMode mode) noexcept {
    m_status.lateralMode = mode;
    m_rollPid.Reset();
}

void AutopilotSystem::CycleVerticalMode() noexcept {
    switch (m_status.verticalMode) {
    case VerticalMode::PIT:
        SetVerticalMode(VerticalMode::ALT);
        break;
    case VerticalMode::ALT:
        SetVerticalMode(VerticalMode::VS);
        break;
    case VerticalMode::VS:
        SetVerticalMode(VerticalMode::GS);
        break;
    case VerticalMode::GS:
    default:
        SetVerticalMode(VerticalMode::PIT);
        break;
    }
}

void AutopilotSystem::SetVerticalMode(VerticalMode mode) noexcept {
    m_status.verticalMode = mode;
    m_pitchPid.Reset();
    m_vsPid.Reset();
    m_status.altitudeCaptured = false;
    m_status.gsCaptured = false;
}

void AutopilotSystem::AdjustTargetAltitude(f64 deltaFt) noexcept {
    m_status.targets.targetAltitudeFt = std::clamp(m_status.targets.targetAltitudeFt + deltaFt, 0.0, 20000.0);
    m_status.altitudeCaptured = false;
}

void AutopilotSystem::SetTargetAltitude(f64 targetFt) noexcept {
    m_status.targets.targetAltitudeFt = std::clamp(targetFt, 0.0, 20000.0);
    m_status.altitudeCaptured = false;
}

void AutopilotSystem::AdjustTargetHeading(f64 deltaDeg) noexcept {
    m_status.targets.targetHeadingDeg = NormalizeAngleDeg360(m_status.targets.targetHeadingDeg + deltaDeg);
}

void AutopilotSystem::SetTargetHeading(f64 targetDeg) noexcept {
    m_status.targets.targetHeadingDeg = NormalizeAngleDeg360(targetDeg);
}

void AutopilotSystem::AdjustTargetVerticalSpeed(f64 deltaFpm) noexcept {
    m_status.targets.targetVerticalSpeedFpm = std::clamp(m_status.targets.targetVerticalSpeedFpm + deltaFpm, -2500.0, +2500.0);
}

void AutopilotSystem::SetTargetVerticalSpeed(f64 targetFpm) noexcept {
    m_status.targets.targetVerticalSpeedFpm = std::clamp(targetFpm, -2500.0, +2500.0);
}

void AutopilotSystem::AdjustTargetPitch(f64 deltaDeg) noexcept {
    m_status.targets.targetPitchDeg = std::clamp(m_status.targets.targetPitchDeg + deltaDeg, -12.0, +18.0);
}

void AutopilotSystem::SetTargetPitch(f64 targetDeg) noexcept {
    m_status.targets.targetPitchDeg = std::clamp(targetDeg, -12.0, +18.0);
}

void AutopilotSystem::SynchronizeTargets(
    f64 indicatedAltitudeFt,
    f64 indicatedHeadingDeg,
    f64 verticalSpeedFpm,
    f64 pitchDeg,
    f64 rollDeg) noexcept
{
    m_status.targets.targetHeadingDeg = std::round(indicatedHeadingDeg);
    m_status.targets.targetAltitudeFt = std::round(indicatedAltitudeFt / 100.0) * 100.0;
    m_status.targets.targetVerticalSpeedFpm = std::round(verticalSpeedFpm / 50.0) * 50.0;
    m_status.targets.targetPitchDeg = pitchDeg;
    m_status.targets.targetRollDeg = (std::abs(rollDeg) < 6.0) ? 0.0 : rollDeg;
}

void AutopilotSystem::Update(
    f64 dt,
    const Physics::RigidBodyState& state,
    f64 /*indicatedAirspeedKnots*/,
    f64 indicatedAltitudeFt,
    f64 verticalSpeedFpm,
    f64 pitchDeg,
    f64 rollDeg,
    f64 indicatedHeadingDeg,
    const Navigation::NavReceiverState& nav1,
    const Input::FlightControls& pilotControls) noexcept
{
    if (dt <= 0.0) return;

    // Rate estimation using body angular rate from state (deg/s)
    m_rollRateDps = Math::Units::RadToDeg(state.angularVelocityBody.x);
    m_pitchRateDps = Math::Units::RadToDeg(state.angularVelocityBody.y);

    // 1. Pilot manual override / safety trip detection
    // If pilot applies significant stick force (> 0.25 deflection), trip AP
    if (m_status.masterEngaged) {
        if (std::abs(pilotControls.pitch) > 0.25 || std::abs(pilotControls.roll) > 0.25) {
            m_status.masterEngaged = false;
            m_status.disconnectWarning = true;
            m_status.disconnectTimer = 2.5;
            m_status.lastDisconnectReason = DisconnectReason::PilotManualOverride;
            m_output.engaged = false;
            m_output.elevatorCommand = 0.0;
            m_output.aileronCommand = 0.0;
        }
    }

    // Update disconnect warning timer
    if (m_status.disconnectWarning) {
        m_status.disconnectTimer -= dt;
        if (m_status.disconnectTimer <= 0.0) {
            m_status.disconnectWarning = false;
        }
    }

    // 2. Lateral Guidance Loop
    UpdateLateralGuidance(dt, indicatedHeadingDeg, rollDeg, m_rollRateDps, nav1);

    // 3. Vertical Guidance Loop
    UpdateVerticalGuidance(dt, indicatedAltitudeFt, verticalSpeedFpm, pitchDeg, m_pitchRateDps, nav1);

    // 4. Flight Director Command Cues
    m_status.fdCues.active = m_status.flightDirectorActive || m_status.masterEngaged;

    // 5. Output Assignment
    m_output.engaged = m_status.masterEngaged;
    if (!m_status.masterEngaged) {
        m_output.elevatorCommand = 0.0;
        m_output.aileronCommand = 0.0;
        m_output.rudderCommand = 0.0;
    }

    m_prevPitchDeg = pitchDeg;
    m_prevRollDeg = rollDeg;
}

void AutopilotSystem::UpdateLateralGuidance(
    f64 dt,
    f64 indicatedHeadingDeg,
    f64 rollDeg,
    f64 /*rollRateDps*/,
    const Navigation::NavReceiverState& nav1) noexcept
{
    f64 targetBankDeg = 0.0;

    switch (m_status.lateralMode) {
    case LateralMode::ROL:
        // Wing Leveler mode: if bank angle < 6 deg, hold 0 deg (level wings);
        // otherwise hold the engaged bank angle up to 25 deg
        if (std::abs(m_status.targets.targetRollDeg) < 6.0) {
            targetBankDeg = 0.0;
        } else {
            targetBankDeg = std::clamp(m_status.targets.targetRollDeg, -25.0, 25.0);
        }
        break;

    case LateralMode::HDG: {
        // Heading Select mode: turn to target heading bug via shortest turn arc
        f64 hdgError = NormalizeAngleDeg180(m_status.targets.targetHeadingDeg - indicatedHeadingDeg);
        // Standard rate turn limit: 1.4x heading error, clamped to +/- 25 degrees bank
        targetBankDeg = std::clamp(hdgError * 1.4, -25.0, 25.0);
        break;
    }

    case LateralMode::NAV: {
        // VOR / Localizer Tracking: read NAV1 CDI deviation (-5.0 to +5.0 dots)
        if (!nav1.cdi.navFlag && (nav1.cdi.toFlag || nav1.cdi.fromFlag)) {
            // CDI deflection error: normalized from +/- 5 dots
            f64 cdiNorm = std::clamp(nav1.cdi.courseDeviationDots / 5.0, -1.0, 1.0);
            f64 maxInterceptAngle = nav1.cdi.isIls ? 30.0 : 45.0;
            f64 interceptAngle = cdiNorm * maxInterceptAngle;

            // Commanded track angle: radial course minus intercept
            f64 desiredTrack = NormalizeAngleDeg360(nav1.cdi.obsDeg - interceptAngle);
            f64 trackError = NormalizeAngleDeg180(desiredTrack - indicatedHeadingDeg);
            targetBankDeg = std::clamp(trackError * 1.4, -22.0, 22.0);
        } else {
            // Fallback to wings level if signal lost
            targetBankDeg = 0.0;
        }
        break;
    }

    case LateralMode::APPR: {
        // Approach mode: precision localizer tracking
        if (!nav1.cdi.navFlag && nav1.cdi.isIls) {
            f64 locNorm = std::clamp(nav1.cdi.courseDeviationDots / 5.0, -1.0, 1.0);
            f64 interceptAngle = locNorm * 30.0;
            f64 desiredTrack = NormalizeAngleDeg360(nav1.cdi.obsDeg - interceptAngle);
            f64 trackError = NormalizeAngleDeg180(desiredTrack - indicatedHeadingDeg);
            targetBankDeg = std::clamp(trackError * 1.5, -20.0, 20.0);
        } else {
            targetBankDeg = 0.0;
        }
        break;
    }
    }

    // Clamp command bank angle
    targetBankDeg = std::clamp(targetBankDeg, -25.0, 25.0);
    m_status.fdCues.commandRollDeg = targetBankDeg;

    // Inner-loop Roll PID: drives bank error -> aileron deflection command
    f64 rollError = targetBankDeg - rollDeg;
    m_output.aileronCommand = m_rollPid.Update(rollError, dt);
}

void AutopilotSystem::UpdateVerticalGuidance(
    f64 dt,
    f64 indicatedAltitudeFt,
    f64 verticalSpeedFpm,
    f64 pitchDeg,
    f64 /*pitchRateDps*/,
    const Navigation::NavReceiverState& nav1) noexcept
{
    f64 targetPitchDeg = 0.0;

    switch (m_status.verticalMode) {
    case VerticalMode::PIT:
        // Direct pitch attitude hold
        targetPitchDeg = std::clamp(m_status.targets.targetPitchDeg, -10.0, 15.0);
        break;

    case VerticalMode::VS: {
        // Vertical speed hold mode
        f64 vsError = m_status.targets.targetVerticalSpeedFpm - verticalSpeedFpm;
        // Nominal trim pitch (~2.5 deg) modulated by VS controller
        f64 pitchDelta = m_vsPid.Update(vsError, dt);
        targetPitchDeg = std::clamp(2.5 + pitchDelta, -10.0, 15.0);
        break;
    }

    case VerticalMode::ALT: {
        // Altitude hold / capture mode
        f64 altError = m_status.targets.targetAltitudeFt - indicatedAltitudeFt;

        if (std::abs(altError) < 20.0) {
            m_status.altitudeCaptured = true;
        }

        // Commanded vertical speed: proportional capture up to +/- 1000 fpm
        f64 cmdVsFpm = std::clamp(altError * 3.5, -1000.0, 1000.0);
        f64 vsError = cmdVsFpm - verticalSpeedFpm;
        f64 pitchDelta = m_vsPid.Update(vsError, dt);
        targetPitchDeg = std::clamp(2.5 + pitchDelta, -8.0, 12.0);
        break;
    }

    case VerticalMode::GS: {
        // Glideslope tracking mode (standard 3.0 deg descent path)
        if (!nav1.cdi.gsFlag && nav1.cdi.isIls) {
            m_status.gsCaptured = true;
            // Glideslope deviation dots: negative fly-up, positive fly-down
            f64 gsNorm = std::clamp(nav1.cdi.glideslopeDeviationDots / 5.0, -1.0, 1.0);
            f64 cmdVsFpm = -600.0 - (gsNorm * 450.0); // Fly up (gsNorm < 0) increases climb rate
            f64 vsError = cmdVsFpm - verticalSpeedFpm;
            f64 pitchDelta = m_vsPid.Update(vsError, dt);
            targetPitchDeg = std::clamp(-1.5 + pitchDelta, -7.0, 4.0);
        } else {
            // Signal lost: maintain level flight or pitch hold
            targetPitchDeg = 2.0;
        }
        break;
    }
    }

    targetPitchDeg = std::clamp(targetPitchDeg, -10.0, 15.0);
    m_status.fdCues.commandPitchDeg = targetPitchDeg;

    // Inner-loop Pitch PID: drives pitch error -> elevator deflection command
    f64 pitchError = targetPitchDeg - pitchDeg;
    m_output.elevatorCommand = m_pitchPid.Update(pitchError, dt);
}

std::wstring AutopilotSystem::GetStatusString() const {
    std::wstringstream ss;
    if (m_status.masterEngaged) {
        ss << L"AP: [ON | ";
    } else if (m_status.disconnectWarning) {
        ss << L"AP: [DISC! | ";
    } else {
        ss << L"AP: [OFF | ";
    }

    // Lateral mode
    ss << LateralModeToString(m_status.lateralMode);
    if (m_status.lateralMode == LateralMode::HDG) {
        ss << L" " << static_cast<int>(m_status.targets.targetHeadingDeg) << L"\u00B0";
    }

    ss << L" | ";

    // Vertical mode
    ss << VerticalModeToString(m_status.verticalMode);
    if (m_status.verticalMode == VerticalMode::ALT) {
        ss << L" " << static_cast<int>(m_status.targets.targetAltitudeFt) << L"ft";
    } else if (m_status.verticalMode == VerticalMode::VS) {
        ss << L" " << (m_status.targets.targetVerticalSpeedFpm >= 0 ? L"+" : L"")
           << static_cast<int>(m_status.targets.targetVerticalSpeedFpm);
    }

    ss << L"]";

    if (m_status.flightDirectorActive) {
        ss << L" FD: ON";
    }

    return ss.str();
}

} // namespace LokYS::Autopilot
