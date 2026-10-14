#include "LokYS/input/InputSystem.hpp"
#include <mmsystem.h>
#include <cmath>
#include <algorithm>

namespace LokYS::Input {

InputSystem::InputSystem() = default;

InputSystem::~InputSystem() = default;

bool InputSystem::Initialize(HWND hwnd) {
    m_hwnd = hwnd;

    // Detect hardware USB flight joystick / gamepad
    JOYCAPSW caps{};
    if (joyGetDevCapsW(JOYSTICKID1, &caps, sizeof(caps)) == JOYERR_NOERROR) {
        m_joystickConnected = true;
        m_joystickId = JOYSTICKID1;
        m_activeDeviceName = caps.szPname;
    } else if (joyGetDevCapsW(JOYSTICKID2, &caps, sizeof(caps)) == JOYERR_NOERROR) {
        m_joystickConnected = true;
        m_joystickId = JOYSTICKID2;
        m_activeDeviceName = caps.szPname;
    } else {
        m_joystickConnected = false;
        m_activeDeviceName = L"Keyboard & Mouse Primary";
    }

    return true;
}

f64 InputSystem::ApplyAxisShaping(f64 rawValue, bool invert) const noexcept {
    f64 val = invert ? -rawValue : rawValue;
    f64 absVal = std::abs(val);

    if (absVal <= m_calibration.deadzone) {
        return 0.0;
    }

    f64 remapped = (absVal - m_calibration.deadzone) / (1.0 - m_calibration.deadzone);
    remapped = std::clamp(remapped, 0.0, 1.0);
    f64 shaped = std::pow(remapped, m_calibration.sensitivityExp);

    return std::copysign(shaped, val);
}

void InputSystem::Poll(f64 dt) noexcept {
    m_cameraToggleTriggered = false;
    m_resetTriggered = false;
    m_panelToggleTriggered = false;
    m_avionicsAlignTriggered = false;
    m_navSwapTriggered = false;
    m_navIdentTriggered = false;
    m_kollsmanDelta = 0.0;
    m_headingBugDelta = 0.0;
    m_obsDelta = 0.0;
    m_navFreqDelta = 0.0;

    // Hardware joystick polling
    if (m_joystickConnected) {
        PollJoystick();
    }

    // Always poll keyboard for fallback and modifier controls (flaps, throttle keys, trim)
    PollKeyboard(dt);
}


void InputSystem::PollJoystick() noexcept {
    JOYINFOEX info{};
    info.dwSize = sizeof(JOYINFOEX);
    info.dwFlags = JOY_RETURNALL;

    if (joyGetPosEx(m_joystickId, &info) == JOYERR_NOERROR) {
        // Roll: dwXpos [0 .. 65535] -> [-1.0 .. 1.0]
        f64 rawRoll = (static_cast<f64>(info.dwXpos) - 32767.5) / 32767.5;
        m_controls.roll = ApplyAxisShaping(rawRoll, m_calibration.invertRoll);

        // Pitch: dwYpos [0 .. 65535] -> [-1.0 .. 1.0] (invert so pull back is +1.0 nose up)
        f64 rawPitch = (static_cast<f64>(info.dwYpos) - 32767.5) / 32767.5;
        m_controls.pitch = ApplyAxisShaping(-rawPitch, m_calibration.invertPitch);

        // Yaw / Rudder: dwRpos if available
        if (info.dwRpos > 0) {
            f64 rawYaw = (static_cast<f64>(info.dwRpos) - 32767.5) / 32767.5;
            m_controls.yaw = ApplyAxisShaping(rawYaw, m_calibration.invertYaw);
        }

        // Throttle: dwZpos [0 (full) .. 65535 (idle)] -> [0.0 .. 1.0]
        if (info.dwZpos > 0) {
            f64 rawThrottle = 1.0 - (static_cast<f64>(info.dwZpos) / 65535.0);
            m_controls.throttle = std::clamp(rawThrottle, 0.0, 1.0);
        }

        // Brakes: Trigger button (JOY_BUTTON1)
        if (info.dwButtons & JOY_BUTTON1) {
            m_controls.brakes = 1.0;
        } else {
            m_controls.brakes = 0.0;
        }
    } else {
        // Hardware disconnected
        m_joystickConnected = false;
        m_activeDeviceName = L"Keyboard Fallback";
    }
}

void InputSystem::PollKeyboard(f64 dt) noexcept {
    const f64 rampRate = 2.5 * dt; // Rate of deflection ramp
    const f64 returnRate = 3.5 * dt; // Rate of centering spring return

    auto IsDown = [](int vk) -> bool {
        return (GetAsyncKeyState(vk) & 0x8000) != 0;
    };

    // If hardware joystick is active, don't let keyboard center overwrite joystick roll/pitch
    if (!m_joystickConnected) {
        // Pitch: Up Arrow / W (pitch down), Down Arrow / S (pitch up)
        if (IsDown(VK_UP) || IsDown('W')) {
            m_controls.pitch = std::max(-1.0, m_controls.pitch - rampRate);
        } else if (IsDown(VK_DOWN) || IsDown('S')) {
            m_controls.pitch = std::min(1.0, m_controls.pitch + rampRate);
        } else {
            // Spring return to center
            if (m_controls.pitch > 0.0) {
                m_controls.pitch = std::max(0.0, m_controls.pitch - returnRate);
            } else if (m_controls.pitch < 0.0) {
                m_controls.pitch = std::min(0.0, m_controls.pitch + returnRate);
            }
        }

        // Roll: Left Arrow / A (left bank), Right Arrow / D (right bank)
        if (IsDown(VK_LEFT) || IsDown('A')) {
            m_controls.roll = std::max(-1.0, m_controls.roll - rampRate);
        } else if (IsDown(VK_RIGHT) || IsDown('D')) {
            m_controls.roll = std::min(1.0, m_controls.roll + rampRate);
        } else {
            // Spring return to center
            if (m_controls.roll > 0.0) {
                m_controls.roll = std::max(0.0, m_controls.roll - returnRate);
            } else if (m_controls.roll < 0.0) {
                m_controls.roll = std::min(0.0, m_controls.roll + returnRate);
            }
        }

        // Yaw: Q (left rudder), E (right rudder)
        if (IsDown('Q')) {
            m_controls.yaw = std::max(-1.0, m_controls.yaw - rampRate);
        } else if (IsDown('E')) {
            m_controls.yaw = std::min(1.0, m_controls.yaw + rampRate);
        } else {
            if (m_controls.yaw > 0.0) {
                m_controls.yaw = std::max(0.0, m_controls.yaw - returnRate);
            } else if (m_controls.yaw < 0.0) {
                m_controls.yaw = std::min(0.0, m_controls.yaw + returnRate);
            }
        }
    }

    // Throttle controls: PageUp (increase), PageDown (decrease), '0' (100%), '-' (idle)
    if (IsDown(VK_PRIOR)) { // Page Up
        m_controls.throttle = std::min(1.0, m_controls.throttle + 0.5 * dt);
    } else if (IsDown(VK_NEXT)) { // Page Down
        m_controls.throttle = std::max(0.0, m_controls.throttle - 0.5 * dt);
    } else if (IsDown('0')) {
        m_controls.throttle = 1.0;
    } else if (IsDown(VK_OEM_MINUS)) {
        m_controls.throttle = 0.0;
    }

    // Mixture controls: Home (Rich / Increase), End (Lean / Decrease)
    if (IsDown(VK_HOME)) {
        m_controls.mixture = std::min(1.0, m_controls.mixture + 0.5 * dt);
    } else if (IsDown(VK_END)) {
        m_controls.mixture = std::max(0.0, m_controls.mixture - 0.5 * dt);
    }


    // Flaps: F (extend one notch), V (retract one notch)
    bool fDown = IsDown('F');
    if (fDown && !m_prevFlapDownKey) {
        if (m_controls.flapSetting < 3) {
            m_controls.flapSetting++;
        }
    }
    m_prevFlapDownKey = fDown;

    bool vDown = IsDown('V');
    if (vDown && !m_prevFlapUpKey) {
        if (m_controls.flapSetting > 0) {
            m_controls.flapSetting--;
        }
    }
    m_prevFlapUpKey = vDown;

    // Wheel Brakes: Spacebar
    if (IsDown(VK_SPACE)) {
        m_controls.brakes = 1.0;
    } else if (!m_joystickConnected) {
        m_controls.brakes = 0.0;
    }

    // Camera Toggle: 'C' key
    bool cDown = IsDown('C');
    if (cDown && !m_prevCameraKey) {
        m_cameraToggleTriggered = true;
    }
    m_prevCameraKey = cDown;

    // Flight Reset: 'R' key
    bool rDown = IsDown('R');
    if (rDown && !m_prevResetKey) {
        m_resetTriggered = true;
    }
    m_prevResetKey = rDown;

    // Instrument Panel Toggle: 'H' key
    bool hDown = IsDown('H');
    if (hDown && !m_prevPanelKey) {
        m_panelToggleTriggered = true;
    }
    m_prevPanelKey = hDown;

    // Avionics Align Gyro: 'B' key
    bool bDown = IsDown('B');
    if (bDown && !m_prevAlignKey) {
        m_avionicsAlignTriggered = true;
    }
    m_prevAlignKey = bDown;

    // Kollsman Barometric Adjustment: '[' (decrease) and ']' (increase)
    if (IsDown(VK_OEM_4)) { // '['
        m_kollsmanDelta -= 0.05 * dt;
    }
    if (IsDown(VK_OEM_6)) { // ']'
        m_kollsmanDelta += 0.05 * dt;
    }

    // Heading Bug Adjustment: ',' (decrease) and '.' (increase)
    if (IsDown(VK_OEM_COMMA)) {
        m_headingBugDelta -= 20.0 * dt;
    }
    if (IsDown(VK_OEM_PERIOD)) {
        m_headingBugDelta += 20.0 * dt;
    }

    // NAV Frequency Swap: 'T' key
    bool tDown = IsDown('T');
    if (tDown && !m_prevNavSwapKey) {
        m_navSwapTriggered = true;
    }
    m_prevNavSwapKey = tDown;

    // NAV Audio Ident Toggle: 'Y' key
    bool yDown = IsDown('Y');
    if (yDown && !m_prevNavIdentKey) {
        m_navIdentTriggered = true;
    }
    m_prevNavIdentKey = yDown;

    // OBS Course Adjustment: 'K' (decrease) and 'L' (increase)
    if (IsDown('K')) {
        m_obsDelta -= 40.0 * dt;
    }
    if (IsDown('L')) {
        m_obsDelta += 40.0 * dt;
    }

    // NAV Standby Frequency Adjustment: 'U' (decrease) and 'I' (increase)
    if (IsDown('U')) {
        m_navFreqDelta -= 0.5 * dt;
    }
    if (IsDown('I')) {
        m_navFreqDelta += 0.5 * dt;
    }
}

void InputSystem::ApplySerialControls(
    const Serial::InboundFlightControlsPayload& controls,
    const Serial::InboundNavControlsPayload& nav) noexcept
{
    // Convert 16-bit integer ADC axes into normalized floating point
    if (controls.validAxesMask & (1 << 0)) { // Pitch
        m_controls.pitch = std::clamp(static_cast<f64>(controls.pitchAxis) / 32767.0, -1.0, 1.0);
    }
    if (controls.validAxesMask & (1 << 1)) { // Roll
        m_controls.roll = std::clamp(static_cast<f64>(controls.rollAxis) / 32767.0, -1.0, 1.0);
    }
    if (controls.validAxesMask & (1 << 2)) { // Yaw
        m_controls.yaw = std::clamp(static_cast<f64>(controls.yawAxis) / 32767.0, -1.0, 1.0);
    }
    if (controls.validAxesMask & (1 << 3)) { // Throttle
        m_controls.throttle = std::clamp(static_cast<f64>(controls.throttleAxis) / 65535.0, 0.0, 1.0);
    }
    if (controls.validAxesMask & (1 << 4)) { // Mixture
        m_controls.mixture = std::clamp(static_cast<f64>(controls.mixtureAxis) / 65535.0, 0.0, 1.0);
    }
    if (controls.validAxesMask & (1 << 5)) { // Brakes
        f64 maxBrake = std::max(
            static_cast<f64>(controls.leftBrake) / 65535.0,
            static_cast<f64>(controls.rightBrake) / 65535.0);
        m_controls.brakes = std::clamp(maxBrake, 0.0, 1.0);
    }
    if (controls.validAxesMask & (1 << 6)) { // Flaps
        m_controls.flapSetting = static_cast<u32>(std::clamp(static_cast<int>(controls.flapSetting), 0, 3));
    }

    if (nav.navSwapTriggered) {
        m_navSwapTriggered = true;
    }
    if (nav.navIdentTriggered) {
        m_navIdentTriggered = true;
    }
    if (nav.obsDeltaTicks != 0) {
        m_obsDelta += static_cast<f64>(nav.obsDeltaTicks);
    }
    if (nav.freqDeltaTicks != 0) {
        m_navFreqDelta += static_cast<f64>(nav.freqDeltaTicks) * 0.05; // 50 kHz steps
    }
}

} // namespace LokYS::Input
