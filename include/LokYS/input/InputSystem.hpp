#pragma once

#include "LokYS/core/Types.hpp"
#include "LokYS/serial/SerialTypes.hpp"

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
    [[nodiscard]] f64 GetKollsmanDelta() const noexcept { return m_kollsmanDelta; }
    [[nodiscard]] f64 GetHeadingBugDelta() const noexcept { return m_headingBugDelta; }
    [[nodiscard]] f64 GetObsDelta() const noexcept { return m_obsDelta; }
    [[nodiscard]] f64 GetNavFrequencyDelta() const noexcept { return m_navFreqDelta; }

    // Manual control setters for automation / autopilot
    void SetControls(const FlightControls& controls) noexcept { m_controls = controls; }

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
    bool m_cameraToggleTriggered{ false };
    bool m_resetTriggered{ false };
    bool m_panelToggleTriggered{ false };
    bool m_avionicsAlignTriggered{ false };
    bool m_navSwapTriggered{ false };
    bool m_navIdentTriggered{ false };
    f64 m_kollsmanDelta{ 0.0 };
    f64 m_headingBugDelta{ 0.0 };
    f64 m_obsDelta{ 0.0 };
    f64 m_navFreqDelta{ 0.0 };
};


} // namespace LokYS::Input
