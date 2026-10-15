#include "LokYS/input/ActionMapping.hpp"

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <sstream>
#include <algorithm>

namespace LokYS::Input {

namespace {

std::string VKeyToName(u32 vk) {
    if (vk >= 'A' && vk <= 'Z') {
        return std::string(1, static_cast<char>(vk));
    }
    if (vk >= '0' && vk <= '9') {
        return std::string(1, static_cast<char>(vk));
    }
    switch (vk) {
    case VK_SPACE: return "SPACE";
    case VK_RETURN: return "ENTER";
    case VK_ESCAPE: return "ESC";
    case VK_TAB: return "TAB";
    case VK_BACK: return "BACKSPACE";
    case VK_UP: return "UP";
    case VK_DOWN: return "DOWN";
    case VK_LEFT: return "LEFT";
    case VK_RIGHT: return "RIGHT";
    case VK_OEM_4: return "[";
    case VK_OEM_6: return "]";
    case VK_OEM_COMMA: return ",";
    case VK_OEM_PERIOD: return ".";
    case VK_OEM_PLUS: return "+";
    case VK_OEM_MINUS: return "-";
    case VK_F1: return "F1";
    case VK_F2: return "F2";
    case VK_F3: return "F3";
    case VK_F4: return "F4";
    case VK_F5: return "F5";
    case VK_F6: return "F6";
    case VK_F7: return "F7";
    case VK_F8: return "F8";
    case VK_F9: return "F9";
    case VK_F10: return "F10";
    case VK_F11: return "F11";
    case VK_F12: return "F12";
    case VK_NUMPAD0: return "NUM0";
    case VK_NUMPAD1: return "NUM1";
    case VK_NUMPAD2: return "NUM2";
    case VK_NUMPAD3: return "NUM3";
    case VK_NUMPAD4: return "NUM4";
    case VK_NUMPAD5: return "NUM5";
    case VK_NUMPAD6: return "NUM6";
    case VK_NUMPAD7: return "NUM7";
    case VK_NUMPAD8: return "NUM8";
    case VK_NUMPAD9: return "NUM9";
    default:
        return "KEY_" + std::to_string(vk);
    }
}

u32 NameToVKey(const std::string& name) {
    if (name.length() == 1) {
        char c = static_cast<char>(std::toupper(static_cast<unsigned char>(name[0])));
        if ((c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9')) {
            return static_cast<u32>(c);
        }
        if (c == '[') return VK_OEM_4;
        if (c == ']') return VK_OEM_6;
        if (c == ',') return VK_OEM_COMMA;
        if (c == '.') return VK_OEM_PERIOD;
        if (c == '+') return VK_OEM_PLUS;
        if (c == '-') return VK_OEM_MINUS;
    }
    if (name == "SPACE") return VK_SPACE;
    if (name == "ENTER") return VK_RETURN;
    if (name == "ESC") return VK_ESCAPE;
    if (name == "TAB") return VK_TAB;
    if (name == "BACKSPACE") return VK_BACK;
    if (name == "UP") return VK_UP;
    if (name == "DOWN") return VK_DOWN;
    if (name == "LEFT") return VK_LEFT;
    if (name == "RIGHT") return VK_RIGHT;
    if (name == "F8") return VK_F8;
    if (name == "NUM0") return VK_NUMPAD0;
    if (name == "NUM1") return VK_NUMPAD1;
    if (name == "NUM2") return VK_NUMPAD2;
    if (name == "NUM3") return VK_NUMPAD3;
    if (name == "NUM4") return VK_NUMPAD4;
    if (name == "NUM5") return VK_NUMPAD5;
    if (name == "NUM6") return VK_NUMPAD6;
    if (name == "NUM7") return VK_NUMPAD7;
    if (name == "NUM8") return VK_NUMPAD8;
    if (name == "NUM9") return VK_NUMPAD9;
    return 0;
}

} // namespace

std::string KeyBinding::ToString() const {
    if (vkey == 0) return "NONE";
    std::string result;
    if (ctrl) result += "CTRL+";
    if (alt) result += "ALT+";
    if (shift) result += "SHIFT+";
    result += VKeyToName(vkey);
    return result;
}

KeyBinding KeyBinding::FromString(const std::string& str) {
    KeyBinding kb{};
    if (str.empty() || str == "NONE") return kb;

    std::string token;
    std::stringstream ss(str);
    std::vector<std::string> parts;
    while (std::getline(ss, token, '+')) {
        parts.push_back(token);
    }

    for (size_t i = 0; i < parts.size(); ++i) {
        if (parts[i] == "CTRL") kb.ctrl = true;
        else if (parts[i] == "ALT") kb.alt = true;
        else if (parts[i] == "SHIFT") kb.shift = true;
        else {
            kb.vkey = NameToVKey(parts[i]);
        }
    }
    return kb;
}

ActionMap::ActionMap() {
    SetDefaultBindings();
}

void ActionMap::SetDefaultBindings() {
    m_bindings.clear();

    m_bindings[InputAction::PitchUp]             = KeyBinding{ VK_DOWN, false, false, false };
    m_bindings[InputAction::PitchDown]           = KeyBinding{ VK_UP, false, false, false };
    m_bindings[InputAction::RollLeft]            = KeyBinding{ VK_LEFT, false, false, false };
    m_bindings[InputAction::RollRight]           = KeyBinding{ VK_RIGHT, false, false, false };
    m_bindings[InputAction::YawLeft]             = KeyBinding{ static_cast<u32>('A'), false, false, false };
    m_bindings[InputAction::YawRight]            = KeyBinding{ static_cast<u32>('D'), false, false, false };
    m_bindings[InputAction::ThrottleUp]          = KeyBinding{ static_cast<u32>('W'), false, false, false };
    m_bindings[InputAction::ThrottleDown]        = KeyBinding{ static_cast<u32>('S'), false, false, false };
    m_bindings[InputAction::FlapsUp]             = KeyBinding{ static_cast<u32>('F'), false, false, false };
    m_bindings[InputAction::FlapsDown]           = KeyBinding{ static_cast<u32>('V'), false, false, false };
    m_bindings[InputAction::ToggleGear]          = KeyBinding{ static_cast<u32>('U'), false, false, false };
    m_bindings[InputAction::ToggleParkingBrake]  = KeyBinding{ VK_SPACE, false, false, false };
    m_bindings[InputAction::ToeBrakes]           = KeyBinding{ static_cast<u32>('B'), false, false, false };
    m_bindings[InputAction::ToggleCamera]        = KeyBinding{ static_cast<u32>('C'), false, false, false };
    m_bindings[InputAction::ResetFlight]         = KeyBinding{ static_cast<u32>('R'), false, false, false };
    m_bindings[InputAction::TogglePause]         = KeyBinding{ VK_ESCAPE, false, false, false };
    m_bindings[InputAction::MenuSelect]          = KeyBinding{ VK_RETURN, false, false, false };
    m_bindings[InputAction::MenuBack]            = KeyBinding{ static_cast<u32>('M'), false, false, false };
    m_bindings[InputAction::ToggleSettings]      = KeyBinding{ static_cast<u32>('O'), false, false, false };
    m_bindings[InputAction::CycleInstruments]    = KeyBinding{ static_cast<u32>('H'), false, false, false };
    m_bindings[InputAction::ToggleAutopilot]     = KeyBinding{ static_cast<u32>('Z'), false, false, false };
    m_bindings[InputAction::ToggleFlightDirector]= KeyBinding{ static_cast<u32>('X'), false, false, false };
    m_bindings[InputAction::CycleWeather]        = KeyBinding{ VK_F8, false, false, false };
    m_bindings[InputAction::ToggleFog]           = KeyBinding{ static_cast<u32>('G'), false, false, false };
}

void ActionMap::SetBinding(InputAction action, const KeyBinding& binding) {
    m_bindings[action] = binding;
}

const KeyBinding& ActionMap::GetBinding(InputAction action) const {
    auto it = m_bindings.find(action);
    if (it != m_bindings.end()) {
        return it->second;
    }
    return m_fallback;
}

const char* ActionMap::GetActionName(InputAction action) noexcept {
    switch (action) {
    case InputAction::PitchUp:             return "Pitch Up";
    case InputAction::PitchDown:           return "Pitch Down";
    case InputAction::RollLeft:            return "Roll Left";
    case InputAction::RollRight:           return "Roll Right";
    case InputAction::YawLeft:             return "Rudder Left";
    case InputAction::YawRight:            return "Rudder Right";
    case InputAction::ThrottleUp:          return "Throttle Up";
    case InputAction::ThrottleDown:        return "Throttle Down";
    case InputAction::FlapsUp:             return "Flaps Retract";
    case InputAction::FlapsDown:           return "Flaps Extend";
    case InputAction::ToggleGear:          return "Landing Gear";
    case InputAction::ToggleParkingBrake:  return "Parking Brake";
    case InputAction::ToeBrakes:           return "Wheel Brakes";
    case InputAction::ToggleCamera:        return "Cycle Camera";
    case InputAction::ResetFlight:         return "Restart Flight";
    case InputAction::TogglePause:         return "Pause / Back";
    case InputAction::MenuSelect:          return "Select / Confirm";
    case InputAction::MenuBack:            return "Menu Return";
    case InputAction::ToggleSettings:      return "Settings Menu";
    case InputAction::CycleInstruments:    return "Instrument Mode";
    case InputAction::ToggleAutopilot:     return "Autopilot Master";
    case InputAction::ToggleFlightDirector:return "Flight Director";
    case InputAction::CycleWeather:        return "Cycle Weather";
    case InputAction::ToggleFog:           return "Toggle Fog";
    default:                               return "Unknown Action";
    }
}

const char* ActionMap::GetActionDescription(InputAction action) noexcept {
    switch (action) {
    case InputAction::PitchUp:             return "Pull nose up (elevator)";
    case InputAction::PitchDown:           return "Push nose down (elevator)";
    case InputAction::RollLeft:            return "Bank left (ailerons)";
    case InputAction::RollRight:           return "Bank right (ailerons)";
    case InputAction::YawLeft:             return "Deflect rudder to left";
    case InputAction::YawRight:            return "Deflect rudder to right";
    case InputAction::ThrottleUp:          return "Increase engine thrust";
    case InputAction::ThrottleDown:        return "Decrease engine thrust";
    case InputAction::FlapsUp:             return "Retract trailing edge flaps";
    case InputAction::FlapsDown:           return "Extend trailing edge flaps";
    case InputAction::ToggleGear:          return "Retract or extend landing gear";
    case InputAction::ToggleParkingBrake:  return "Toggle parking brake state";
    case InputAction::ToeBrakes:           return "Hold differential wheel brakes";
    case InputAction::ToggleCamera:        return "Toggle cockpit and chase views";
    case InputAction::ResetFlight:         return "Reset aircraft to runway/cruise";
    case InputAction::TogglePause:         return "Pause simulation or go back";
    case InputAction::MenuSelect:          return "Confirm selection or start flight";
    case InputAction::MenuBack:            return "Return to main pre-flight menu";
    case InputAction::ToggleSettings:      return "Open settings & controls menu";
    case InputAction::CycleInstruments:    return "Cycle PFD, HUD, Analog, Off";
    case InputAction::ToggleAutopilot:     return "Engage or disengage autopilot";
    case InputAction::ToggleFlightDirector:return "Toggle autopilot flight director cues";
    case InputAction::CycleWeather:        return "Cycle atmospheric presets";
    case InputAction::ToggleFog:           return "Toggle volumetric fog mode";
    default:                               return "";
    }
}

} // namespace LokYS::Input
