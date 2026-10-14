#pragma once

#include "LokYS/core/Types.hpp"
#include "LokYS/autopilot/AutopilotTypes.hpp"
#include "LokYS/autopilot/PidController.hpp"
#include "LokYS/physics/RigidBodyState.hpp"
#include "LokYS/input/InputSystem.hpp"
#include "LokYS/navigation/NavTypes.hpp"
#include <string>

namespace LokYS::Autopilot {

class AutopilotSystem {
public:
    AutopilotSystem();
    ~AutopilotSystem() = default;

    void Initialize() noexcept;
    void Reset() noexcept;

    // Master Autopilot & Flight Director control
    void ToggleMaster() noexcept;
    void SetMasterEngaged(bool engaged) noexcept;
    [[nodiscard]] bool IsEngaged() const noexcept { return m_status.masterEngaged; }

    void ToggleFlightDirector() noexcept;
    void SetFlightDirectorActive(bool active) noexcept;
    [[nodiscard]] bool IsFlightDirectorActive() const noexcept { return m_status.flightDirectorActive; }

    // Mode selection & cycling
    void CycleLateralMode() noexcept;
    void SetLateralMode(LateralMode mode) noexcept;
    [[nodiscard]] LateralMode GetLateralMode() const noexcept { return m_status.lateralMode; }

    void CycleVerticalMode() noexcept;
    void SetVerticalMode(VerticalMode mode) noexcept;
    [[nodiscard]] VerticalMode GetVerticalMode() const noexcept { return m_status.verticalMode; }

    // Target bug adjustments
    void AdjustTargetAltitude(f64 deltaFt) noexcept;
    void SetTargetAltitude(f64 targetFt) noexcept;
    [[nodiscard]] f64 GetTargetAltitude() const noexcept { return m_status.targets.targetAltitudeFt; }

    void AdjustTargetHeading(f64 deltaDeg) noexcept;
    void SetTargetHeading(f64 targetDeg) noexcept;
    [[nodiscard]] f64 GetTargetHeading() const noexcept { return m_status.targets.targetHeadingDeg; }

    void AdjustTargetVerticalSpeed(f64 deltaFpm) noexcept;
    void SetTargetVerticalSpeed(f64 targetFpm) noexcept;
    [[nodiscard]] f64 GetTargetVerticalSpeed() const noexcept { return m_status.targets.targetVerticalSpeedFpm; }

    void AdjustTargetPitch(f64 deltaDeg) noexcept;
    void SetTargetPitch(f64 targetDeg) noexcept;
    [[nodiscard]] f64 GetTargetPitch() const noexcept { return m_status.targets.targetPitchDeg; }

    void SetTargets(const AutopilotTargets& targets) noexcept { m_status.targets = targets; }
    [[nodiscard]] const AutopilotTargets& GetTargets() const noexcept { return m_status.targets; }

    [[nodiscard]] const AutopilotStatus& GetStatus() const noexcept { return m_status; }
    [[nodiscard]] const AutopilotOutput& GetOutput() const noexcept { return m_output; }

    // Primary closed-loop simulation update (called at 120 Hz)
    void Update(
        f64 dt,
        const Physics::RigidBodyState& state,
        f64 indicatedAirspeedKnots,
        f64 indicatedAltitudeFt,
        f64 verticalSpeedFpm,
        f64 pitchDeg,
        f64 rollDeg,
        f64 indicatedHeadingDeg,
        const Navigation::NavReceiverState& nav1,
        const Input::FlightControls& pilotControls) noexcept;

    /// @brief Formats concise one-line status for window title / HUD telemetry.
    [[nodiscard]] std::wstring GetStatusString() const;

private:
    void UpdateLateralGuidance(
        f64 dt,
        f64 indicatedHeadingDeg,
        f64 rollDeg,
        f64 rollRateDps,
        const Navigation::NavReceiverState& nav1) noexcept;

    void UpdateVerticalGuidance(
        f64 dt,
        f64 indicatedAltitudeFt,
        f64 verticalSpeedFpm,
        f64 pitchDeg,
        f64 pitchRateDps,
        const Navigation::NavReceiverState& nav1) noexcept;

    void SynchronizeTargets(
        f64 indicatedAltitudeFt,
        f64 indicatedHeadingDeg,
        f64 verticalSpeedFpm,
        f64 pitchDeg,
        f64 rollDeg) noexcept;

    AutopilotStatus m_status{};
    AutopilotOutput m_output{};

    // Inner loop PID controllers
    PidController m_rollPid{};
    PidController m_pitchPid{};
    PidController m_vsPid{};
    PidController m_yawDamperPid{};

    f64 m_prevPitchDeg{ 0.0 };
    f64 m_prevRollDeg{ 0.0 };
    f64 m_rollRateDps{ 0.0 };
    f64 m_pitchRateDps{ 0.0 };
};

} // namespace LokYS::Autopilot
