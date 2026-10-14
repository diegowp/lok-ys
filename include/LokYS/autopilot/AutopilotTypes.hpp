#pragma once

#include "LokYS/core/Types.hpp"
#include <string>

namespace LokYS::Autopilot {

enum class LateralMode : u8 {
    ROL,    // Roll attitude hold / wing leveler (< 6 deg bank -> level wings)
    HDG,    // Heading select mode (turns towards target heading bug, max 25 deg bank)
    NAV,    // Navigation tracking mode (VOR radial / Localizer course tracking)
    APPR    // Precision approach mode (coupled ILS Localizer tracking)
};

enum class VerticalMode : u8 {
    PIT,    // Pitch attitude hold mode
    ALT,    // Altitude hold / capture mode
    VS,     // Vertical speed hold mode (climb/descent at target fpm)
    GS      // Precision approach mode (coupled ILS Glideslope tracking)
};

enum class LateralArmedMode : u8 {
    None,
    NAV,
    LOC
};

enum class VerticalArmedMode : u8 {
    None,
    ALT,
    GS
};

enum class DisconnectReason : u8 {
    None,
    PilotManualOverride,
    SensorInvalid,
    PilotButtonToggle
};

struct AutopilotTargets {
    f64 targetHeadingDeg{ 360.0 };       // [0, 360)
    f64 targetAltitudeFt{ 3500.0 };      // Indicated altitude target
    f64 targetVerticalSpeedFpm{ 500.0 }; // Target vertical speed (+/- fpm)
    f64 targetPitchDeg{ 3.0 };           // Target pitch attitude in PIT mode (+/- deg)
    f64 targetRollDeg{ 0.0 };            // Target roll attitude in ROL mode (+/- deg)
};

struct FlightDirectorCues {
    bool active{ false };
    f64 commandPitchDeg{ 0.0 };          // Commanded pitch attitude to display on EADI
    f64 commandRollDeg{ 0.0 };           // Commanded bank angle to display on EADI
};

struct AutopilotOutput {
    bool engaged{ false };
    f64 elevatorCommand{ 0.0 };          // Commanded elevator deflection [-1.0, 1.0]
    f64 aileronCommand{ 0.0 };           // Commanded aileron deflection [-1.0, 1.0]
    f64 rudderCommand{ 0.0 };            // Yaw damper rudder deflection [-1.0, 1.0]
    bool yawDamperActive{ false };
};

struct AutopilotStatus {
    bool masterEngaged{ false };
    bool flightDirectorActive{ false };
    bool yawDamperActive{ false };
    LateralMode lateralMode{ LateralMode::ROL };
    VerticalMode verticalMode{ VerticalMode::ALT };
    LateralArmedMode lateralArmed{ LateralArmedMode::None };
    VerticalArmedMode verticalArmed{ VerticalArmedMode::None };
    AutopilotTargets targets{};
    FlightDirectorCues fdCues{};
    bool disconnectWarning{ false };
    f64 disconnectTimer{ 0.0 };
    DisconnectReason lastDisconnectReason{ DisconnectReason::None };
    bool altitudeCaptured{ false };
    bool gsCaptured{ false };
};

[[nodiscard]] inline const char* LateralModeToString(LateralMode mode) noexcept {
    switch (mode) {
    case LateralMode::ROL:  return "ROL";
    case LateralMode::HDG:  return "HDG";
    case LateralMode::NAV:  return "NAV";
    case LateralMode::APPR: return "APPR";
    default:                return "---";
    }
}

[[nodiscard]] inline const char* VerticalModeToString(VerticalMode mode) noexcept {
    switch (mode) {
    case VerticalMode::PIT: return "PIT";
    case VerticalMode::ALT: return "ALT";
    case VerticalMode::VS:  return "VS";
    case VerticalMode::GS:  return "GS";
    default:                return "---";
    }
}

} // namespace LokYS::Autopilot
