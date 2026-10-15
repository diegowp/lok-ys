#pragma once

#include "LokYS/core/Types.hpp"
#include "LokYS/serial/SerialTypes.hpp"
#include "LokYS/input/ActionMapping.hpp"

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <string>

namespace LokYS::Input {

struct FlightControls {
    f64 pitch{ 0.0 };    // [-1.0, 1.0]: -1 nose down (push), +1 nose up (pull)
    f64 roll{ 0.0 };     // [-1.0, 1.0]: -1 left bank, +1 right bank
    f64 yaw{ 0.0 };      // [-1.0, 1.0]: -1 left rudder, +1 right rudder
    f64 throttle{ 0.0 }; // [0.0, 1.0]: 0 idle, 1 full power
    f64 mixture{ 1.0 };  // [0.0, 1.0]: 0 idle cutoff, 1 full rich
    u32 flapSetting{ 0 };// 0 (0 deg), 1 (10 deg), 2 (20 deg), 3 (30 deg)
    f64 brakes{ 0.0 };   // [0.0, 1.0]: wheel brake pressure
};


struct InputCalibration {
    f64 deadzone{ 0.04 };       // 4% deadzone around center
    f64 sensitivityExp{ 1.5 };   // Response curve exponent (1.0 = linear, 2.0 = quadratic)
    bool invertPitch{ false };
    bool invertRoll{ false };
    bool invertYaw{ false };
};

class InputSystem {
public:
    InputSystem();
    ~InputSystem();

    [[nodiscard]] bool Initialize(HWND hwnd);
    void Poll(f64 dt) noexcept;

    [[nodiscard]] const FlightControls& GetControls() const noexcept { return m_controls; }
    [[nodiscard]] const std::wstring& GetActiveDeviceName() const noexcept { return m_activeDeviceName; }
    [[nodiscard]] bool HasHardwareJoystick() const noexcept { return m_joystickConnected; }

    void SetCalibration(const InputCalibration& calib) noexcept { m_calibration = calib; }
    [[nodiscard]] const InputCalibration& GetCalibration() const noexcept { return m_calibration; }

    [[nodiscard]] bool WasCameraToggleTriggered() const noexcept { return m_cameraToggleTriggered; }
    [[nodiscard]] bool WasResetTriggered() const noexcept { return m_resetTriggered; }
    [[nodiscard]] bool WasPanelToggleTriggered() const noexcept { return m_panelToggleTriggered; }
    [[nodiscard]] bool WasAvionicsAlignTriggered() const noexcept { return m_avionicsAlignTriggered; }
    [[nodiscard]] bool WasNavSwapTriggered() const noexcept { return m_navSwapTriggered; }
    [[nodiscard]] bool WasNavIdentTriggered() const noexcept { return m_navIdentTriggered; }
    [[nodiscard]] bool WasWeatherToggleTriggered() const noexcept { return m_weatherToggleTriggered; }
    [[nodiscard]] bool WasFogToggleTriggered() const noexcept { return m_fogToggleTriggered; }
    [[nodiscard]] bool WasNetworkToggleTriggered() const noexcept { return m_networkToggleTriggered; }
    [[nodiscard]] bool WasApToggleTriggered() const noexcept { return m_apToggleTriggered; }
    [[nodiscard]] bool WasFdToggleTriggered() const noexcept { return m_fdToggleTriggered; }
    [[nodiscard]] bool WasApLateralCycleTriggered() const noexcept { return m_apLateralCycleTriggered; }
    [[nodiscard]] bool WasApVerticalCycleTriggered() const noexcept { return m_apVerticalCycleTriggered; }
    [[nodiscard]] bool WasAircraftCycleTriggered() const noexcept { return m_aircraftCycleTriggered; }
    [[nodiscard]] bool WasGearToggleTriggered() const noexcept { return m_gearToggleTriggered; }
    [[nodiscard]] f64 GetAltitudeBugDelta() const noexcept { return m_altitudeBugDelta; }
    [[nodiscard]] f64 GetKollsmanDelta() const noexcept { return m_kollsmanDelta; }
    [[nodiscard]] f64 GetHeadingBugDelta() const noexcept { return m_headingBugDelta; }
    [[nodiscard]] f64 GetObsDelta() const noexcept { return m_obsDelta; }
    [[nodiscard]] f64 GetNavFrequencyDelta() const noexcept { return m_navFreqDelta; }

    // Dynamic Camera Orbit & Zoom Controls
    [[nodiscard]] f32 GetCameraOrbitYawDelta() const noexcept { return m_cameraOrbitYawDelta; }
    [[nodiscard]] f32 GetCameraOrbitPitchDelta() const noexcept { return m_cameraOrbitPitchDelta; }
    [[nodiscard]] f32 GetCameraZoomDelta() const noexcept { return m_cameraZoomDelta; }
    [[nodiscard]] bool WasOrbitResetTriggered() const noexcept { return m_orbitResetTriggered; }

    // GameState & Pre-Flight Menu Triggers
    [[nodiscard]] bool WasPauseToggleTriggered() const noexcept { return m_pauseToggleTriggered; }
    [[nodiscard]] bool WasMenuSelectTriggered() const noexcept { return m_menuSelectTriggered; }
    [[nodiscard]] bool WasMenuBackTriggered() const noexcept { return m_menuBackTriggered; }
    [[nodiscard]] bool WasAirportCycleTriggered() const noexcept { return m_airportCycleTriggered; }
    [[nodiscard]] bool WasSpawnModeToggleTriggered() const noexcept { return m_spawnModeToggleTriggered; }

    [[nodiscard]] bool WasSettingsToggleTriggered() const noexcept { return m_settingsToggleTriggered; }

    [[nodiscard]] const ActionMap& GetActionMap() const noexcept { return m_actionMap; }
    void SetActionMap(const ActionMap& actionMap) noexcept { m_actionMap = actionMap; }

    [[nodiscard]] f32 GetMouseX() const noexcept { return m_mouseX; }
    [[nodiscard]] f32 GetMouseY() const noexcept { return m_mouseY; }
    [[nodiscard]] bool IsLeftMouseDown() const noexcept { return m_leftMouseDown; }
    [[nodiscard]] bool WasLeftMouseClicked() const noexcept { return m_leftMouseClicked; }

    // Manual control setters for automation / autopilot
    void SetControls(const FlightControls& controls) noexcept { m_controls = controls; }

    [[nodiscard]] bool IsParkingBrakeEngaged() const noexcept { return m_parkingBrakeEngaged; }
    void SetParkingBrake(bool engaged) noexcept {
        m_parkingBrakeEngaged = engaged;
        m_controls.brakes = engaged ? 1.0 : 0.0;
    }

    /// @brief Ingests hardware serial controls from microcontroller panel.
    void ApplySerialControls(
        const Serial::InboundFlightControlsPayload& controls,
        const Serial::InboundNavControlsPayload& nav) noexcept;

private:
    void PollKeyboard(f64 dt) noexcept;
    void PollJoystick() noexcept;

    [[nodiscard]] f64 ApplyAxisShaping(f64 rawValue, bool invert) const noexcept;

    HWND m_hwnd{ nullptr };
    FlightControls m_controls{};
    InputCalibration m_calibration{};

    bool m_joystickConnected{ false };
    u32 m_joystickId{ 0 };
    std::wstring m_activeDeviceName{ L"Keyboard & Mouse" };

    // Key toggle state latching
    bool m_prevFlapUpKey{ false };
    bool m_prevFlapDownKey{ false };
    bool m_prevCameraKey{ false };
    bool m_prevResetKey{ false };
    bool m_prevPanelKey{ false };
    bool m_prevAlignKey{ false };
    bool m_prevNavSwapKey{ false };
    bool m_prevNavIdentKey{ false };
    bool m_prevWeatherKey{ false };
    bool m_cameraToggleTriggered{ false };
    bool m_resetTriggered{ false };
    bool m_panelToggleTriggered{ false };
    bool m_avionicsAlignTriggered{ false };
    bool m_navSwapTriggered{ false };
    bool m_navIdentTriggered{ false };
    bool m_weatherToggleTriggered{ false };
    f64 m_kollsmanDelta{ 0.0 };
    f64 m_headingBugDelta{ 0.0 };
    f64 m_obsDelta{ 0.0 };
    f64 m_navFreqDelta{ 0.0 };
    bool m_prevFogKey{ false };
    bool m_fogToggleTriggered{ false };
    bool m_prevNetworkKey{ false };
    bool m_networkToggleTriggered{ false };
    bool m_prevApKey{ false };
    bool m_apToggleTriggered{ false };
    bool m_prevFdKey{ false };
    bool m_fdToggleTriggered{ false };
    bool m_prevApLateralKey{ false };
    bool m_apLateralCycleTriggered{ false };
    bool m_prevApVerticalKey{ false };
    bool m_apVerticalCycleTriggered{ false };
    f64 m_altitudeBugDelta{ 0.0 };
    bool m_prevAircraftCycleKey{ false };
    bool m_aircraftCycleTriggered{ false };
    bool m_prevGearToggleKey{ false };
    bool m_gearToggleTriggered{ false };
    bool m_parkingBrakeEngaged{ true };
    bool m_prevSpaceKey{ false };
    bool m_prevEscapeKey{ false };
    bool m_pauseToggleTriggered{ false };
    bool m_prevReturnKey{ false };
    bool m_menuSelectTriggered{ false };
    bool m_prevTabKey{ false };
    bool m_airportCycleTriggered{ false };
    bool m_spawnModeToggleTriggered{ false };
    bool m_menuBackTriggered{ false };

    // Camera orbit & zoom tracking
    f32 m_cameraOrbitYawDelta{ 0.0f };
    f32 m_cameraOrbitPitchDelta{ 0.0f };
    f32 m_cameraZoomDelta{ 0.0f };
    bool m_orbitResetTriggered{ false };
    bool m_prevSettingsKey{ false };
    bool m_settingsToggleTriggered{ false };
    ActionMap m_actionMap{};
    f32 m_mouseX{ 0.0f };
    f32 m_mouseY{ 0.0f };
    int m_lastMouseX{ 0 };
    int m_lastMouseY{ 0 };
    bool m_rightMouseDown{ false };
    bool m_leftMouseDown{ false };
    bool m_prevLeftMouseDown{ false };
    bool m_leftMouseClicked{ false };
};


} // namespace LokYS::Input
