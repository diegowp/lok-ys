#include "LokYS/input/InputSystem.hpp"
#include <mmsystem.h>
#include <cmath>
#include <algorithm>

namespace LokYS::Input {

InputSystem::InputSystem() {
    m_parkingBrakeEngaged = true;
    m_controls.brakes = 1.0;
}

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
    m_weatherToggleTriggered = false;
    m_fogToggleTriggered = false;
    m_networkToggleTriggered = false;
    m_apToggleTriggered = false;
    m_fdToggleTriggered = false;
    m_apLateralCycleTriggered = false;
    m_apVerticalCycleTriggered = false;
    m_altitudeBugDelta = 0.0;
    m_kollsmanDelta = 0.0;
    m_headingBugDelta = 0.0;
    m_obsDelta = 0.0;
    m_navFreqDelta = 0.0;
    m_aircraftCycleTriggered = false;
    m_gearToggleTriggered = false;
    m_cameraOrbitYawDelta = 0.0f;
    m_cameraOrbitPitchDelta = 0.0f;
    m_cameraZoomDelta = 0.0f;
    m_orbitResetTriggered = false;
    m_pauseToggleTriggered = false;
    m_menuSelectTriggered = false;
    m_menuBackTriggered = false;
    m_airportCycleTriggered = false;
    m_spawnModeToggleTriggered = false;
    m_settingsToggleTriggered = false;

    // Track mouse coordinates and left click
    if (m_hwnd) {
        POINT pt{};
        if (::GetCursorPos(&pt) && ::ScreenToClient(m_hwnd, &pt)) {
            m_mouseX = static_cast<f32>(pt.x);
            m_mouseY = static_cast<f32>(pt.y);
        }
        m_leftMouseDown = (::GetAsyncKeyState(VK_LBUTTON) & 0x8000) != 0;
        m_leftMouseClicked = m_leftMouseDown && !m_prevLeftMouseDown;
        m_prevLeftMouseDown = m_leftMouseDown;
    }

    // Mouse Right Drag for 360-degree Orbit
    if (GetKeyState(VK_RBUTTON) < 0) {
        POINT pt{};
        if (GetCursorPos(&pt) && m_hwnd) {
            ScreenToClient(m_hwnd, &pt);
            if (m_rightMouseDown) {
                int dx = pt.x - m_lastMouseX;
                int dy = pt.y - m_lastMouseY;
                m_cameraOrbitYawDelta += static_cast<f32>(dx) * 0.005f;
                m_cameraOrbitPitchDelta += static_cast<f32>(dy) * 0.005f;
            }
            m_lastMouseX = pt.x;
            m_lastMouseY = pt.y;
            m_rightMouseDown = true;
        }
    } else {
        m_rightMouseDown = false;
    }

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

        // Brakes: Trigger button (JOY_BUTTON1) or persistent Parking Brake latch
        f64 joyBrake = (info.dwButtons & JOY_BUTTON1) ? 1.0 : 0.0;
        if (m_parkingBrakeEngaged) {
            m_controls.brakes = 1.0;
        } else {
            m_controls.brakes = joyBrake;
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

    // Throttle controls: Shift / F3 / PageUp (increase), Ctrl / F2 / PageDown (decrease), F4 / '0' (100%), F1 / '-' (idle)
    if (IsDown(VK_F3) || IsDown(VK_PRIOR) || (IsDown(VK_SHIFT) && !IsDown('F') && !IsDown('V') && !IsDown('U'))) {
        m_controls.throttle = std::min(1.0, m_controls.throttle + 0.5 * dt);
    } else if (IsDown(VK_F2) || IsDown(VK_NEXT) || IsDown(VK_CONTROL)) {
        m_controls.throttle = std::max(0.0, m_controls.throttle - 0.5 * dt);
    } else if (IsDown(VK_F4) || IsDown('0')) {
        m_controls.throttle = 1.0;
    } else if (IsDown(VK_F1) || IsDown(VK_OEM_MINUS)) {
        m_controls.throttle = 0.0;
    }

    // Mixture controls: Home (Rich / Increase), End (Lean / Decrease)
    if (IsDown(VK_HOME)) {
        m_controls.mixture = std::min(1.0, m_controls.mixture + 0.5 * dt);
    } else if (IsDown(VK_END)) {
        m_controls.mixture = std::max(0.0, m_controls.mixture - 0.5 * dt);
    }

    // Flaps: 'F' (extend one notch) or 'V' / Shift+'F' (retract one notch)
    bool fDown = IsDown('F');
    if (fDown && !m_prevFlapDownKey) {
        if (IsDown(VK_SHIFT)) {
            if (m_controls.flapSetting > 0) {
                m_controls.flapSetting--;
            }
        } else {
            if (m_controls.flapSetting < 3) {
                m_controls.flapSetting++;
            }
        }
    }
    m_prevFlapDownKey = fDown;

    bool vDown = IsDown('V') && !IsDown(VK_SHIFT);
    if (vDown && !m_prevFlapUpKey) {
        if (m_controls.flapSetting > 0) {
            m_controls.flapSetting--;
        }
    }
    m_prevFlapUpKey = vDown;

    // Aircraft Archetype Cycle: Shift+'V' or F9 key (Cessna 172 -> Baron 58 -> Extra 330)
    bool acCycleDown = (IsDown('V') && IsDown(VK_SHIFT)) || IsDown(VK_F9);
    if (acCycleDown && !m_prevAircraftCycleKey) {
        m_aircraftCycleTriggered = true;
    }
    m_prevAircraftCycleKey = acCycleDown;

    // Retractable Landing Gear Toggle: 'U' key (Baron 58)
    bool uDown = IsDown('U') && !IsDown(VK_SHIFT);
    if (uDown && !m_prevGearToggleKey) {
        m_gearToggleTriggered = true;
    }
    m_prevGearToggleKey = uDown;

    // Parking Brake: Spacebar edge-triggered toggle (rising edge only)
    bool spaceDown = IsDown(VK_SPACE);
    if (spaceDown && !m_prevSpaceKey) {
        m_parkingBrakeEngaged = !m_parkingBrakeEngaged;
    }
    m_prevSpaceKey = spaceDown;

    // Manual Toe/Wheel Brakes: 'B' key (both), ',' (left toe brake), '.' (right toe brake)
    f64 manualBrake = IsDown('B') ? 1.0 : 0.0;
    f64 manualLeft = (IsDown('B') || IsDown(VK_OEM_COMMA)) ? 1.0 : 0.0;
    f64 manualRight = (IsDown('B') || IsDown(VK_OEM_PERIOD)) ? 1.0 : 0.0;

    // Resolved brake control: locked at 1.0 if parking brake active, else manual input
    if (m_parkingBrakeEngaged) {
        m_controls.brakes = 1.0;
        m_controls.brakeLeft = 1.0;
        m_controls.brakeRight = 1.0;
    } else if (!m_joystickConnected) {
        m_controls.brakes = manualBrake;
        m_controls.brakeLeft = manualLeft;
        m_controls.brakeRight = manualRight;
    } else {
        m_controls.brakeLeft = m_controls.brakes;
        m_controls.brakeRight = m_controls.brakes;
    }

    // Camera Toggle: 'C' key
    bool cDown = IsDown('C');
    if (cDown && !m_prevCameraKey) {
        m_cameraToggleTriggered = true;
    }
    m_prevCameraKey = cDown;

    // Flight Reset: 'R' key (or Toggle Spawn Mode in Pre-Flight Menu)
    bool rDown = IsDown('R');
    if (rDown && !m_prevResetKey) {
        m_resetTriggered = true;
        m_spawnModeToggleTriggered = true;
    }
    m_prevResetKey = rDown;

    // GameState Flow: Escape (Pause / Back)
    bool escDown = IsDown(VK_ESCAPE);
    if (escDown && !m_prevEscapeKey) {
        m_pauseToggleTriggered = true;
    }
    m_prevEscapeKey = escDown;

    // Menu Select / Start Flight: Return / Enter
    bool retDown = IsDown(VK_RETURN);
    if (retDown && !m_prevReturnKey) {
        m_menuSelectTriggered = true;
    }
    m_prevReturnKey = retDown;

    // Airport Cycle: Tab key
    bool tabDown = IsDown(VK_TAB);
    if (tabDown && !m_prevTabKey) {
        m_airportCycleTriggered = true;
    }
    m_prevTabKey = tabDown;

    // Menu Back: 'M' or Backspace
    bool backDown = IsDown('M') || IsDown(VK_BACK);
    if (backDown) {
        m_menuBackTriggered = true;
    }

    // Settings Toggle: 'O' key
    bool oDown = IsDown('O');
    if (oDown && !m_prevSettingsKey) {
        m_settingsToggleTriggered = true;
    }
    m_prevSettingsKey = oDown;

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

    // NAV Standby Frequency Adjustment: Shift+'U' (decrease) and 'I' (increase)
    if (IsDown('U') && IsDown(VK_SHIFT)) {
        m_navFreqDelta -= 0.5 * dt;
    }
    if (IsDown('I')) {
        m_navFreqDelta += 0.5 * dt;
    }

    // Weather Preset Toggle: F8 key
    bool wDown = IsDown(VK_F8);
    if (wDown && !m_prevWeatherKey) {
        m_weatherToggleTriggered = true;
    }
    m_prevWeatherKey = wDown;

    // Atmospheric Fog Toggle: 'G' key
    bool gDown = IsDown('G');
    if (gDown && !m_prevFogKey) {
        m_fogToggleTriggered = true;
    }
    m_prevFogKey = gDown;

    // Multiplayer Network Mode Toggle: 'N' key
    bool nDown = IsDown('N');
    if (nDown && !m_prevNetworkKey) {
        m_networkToggleTriggered = true;
    }
    m_prevNetworkKey = nDown;

    // Milestone M21 Autopilot Master Toggle: 'Z' key
    bool zDown = IsDown('Z');
    if (zDown && !m_prevApKey) {
        m_apToggleTriggered = true;
    }
    m_prevApKey = zDown;

    // Flight Director Toggle: 'J' key
    bool jDown = IsDown('J');
    if (jDown && !m_prevFdKey) {
        m_fdToggleTriggered = true;
    }
    m_prevFdKey = jDown;

    // Autopilot Lateral Mode Cycle: 'X' key
    bool xDown = IsDown('X');
    if (xDown && !m_prevApLateralKey) {
        m_apLateralCycleTriggered = true;
    }
    m_prevApLateralKey = xDown;

    // Autopilot Vertical Mode Cycle: 'M' key
    bool mDown = IsDown('M');
    if (mDown && !m_prevApVerticalKey) {
        m_apVerticalCycleTriggered = true;
    }
    m_prevApVerticalKey = mDown;

    // Target Altitude Bug Adjustment: 'O' (decrease 100 ft) and 'P' (increase 100 ft)
    if (IsDown('O')) {
        m_altitudeBugDelta -= 200.0 * dt;
    }
    if (IsDown('P')) {
        m_altitudeBugDelta += 200.0 * dt;
    }

    // Dynamic Camera Orbit Controls: Alt + Arrow Keys or NumPad (4, 6, 8, 2)
    if (IsDown(VK_MENU)) { // Alt key held down
        if (IsDown(VK_LEFT))  m_cameraOrbitYawDelta -= static_cast<f32>(1.5 * dt);
        if (IsDown(VK_RIGHT)) m_cameraOrbitYawDelta += static_cast<f32>(1.5 * dt);
        if (IsDown(VK_UP))    m_cameraOrbitPitchDelta -= static_cast<f32>(1.5 * dt);
        if (IsDown(VK_DOWN))  m_cameraOrbitPitchDelta += static_cast<f32>(1.5 * dt);
    }
    if (IsDown(VK_NUMPAD4)) m_cameraOrbitYawDelta -= static_cast<f32>(2.0 * dt);
    if (IsDown(VK_NUMPAD6)) m_cameraOrbitYawDelta += static_cast<f32>(2.0 * dt);
    if (IsDown(VK_NUMPAD8)) m_cameraOrbitPitchDelta -= static_cast<f32>(2.0 * dt);
    if (IsDown(VK_NUMPAD2)) m_cameraOrbitPitchDelta += static_cast<f32>(2.0 * dt);

    // Orbit Reset: Backspace, NumPad 5, or Middle Mouse Button
    if (IsDown(VK_BACK) || IsDown(VK_NUMPAD5) || IsDown(VK_MBUTTON)) {
        m_orbitResetTriggered = true;
    }

    // Dynamic Camera Zoom: NumPad +/- or Alt/Ctrl with +/-
    if (IsDown(VK_ADD) || (IsDown(VK_OEM_PLUS) && (IsDown(VK_CONTROL) || IsDown(VK_MENU)))) {
        m_cameraZoomDelta -= static_cast<f32>(0.5 * dt); // Zoom in
    }
    if (IsDown(VK_SUBTRACT) || (IsDown(VK_OEM_MINUS) && (IsDown(VK_CONTROL) || IsDown(VK_MENU)))) {
        m_cameraZoomDelta += static_cast<f32>(0.5 * dt); // Zoom out
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
