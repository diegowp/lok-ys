#pragma once

#include "LokYS/core/Types.hpp"
#include <string>
#include <unordered_map>

namespace LokYS::Input {

enum class InputAction : u32 {
    PitchUp,
    PitchDown,
    RollLeft,
    RollRight,
    YawLeft,
    YawRight,
    ThrottleUp,
    ThrottleDown,
    FlapsUp,
    FlapsDown,
    ToggleGear,
    ToggleParkingBrake,
    ToeBrakes,
    ToggleCamera,
    ResetFlight,
    TogglePause,
    MenuSelect,
    MenuBack,
    ToggleSettings,
    CycleInstruments,
    ToggleAutopilot,
    ToggleFlightDirector,
    CycleWeather,
    ToggleFog,
    Count
};

struct KeyBinding {
    u32 vkey{ 0 };
    bool shift{ false };
    bool ctrl{ false };
    bool alt{ false };

    [[nodiscard]] std::string ToString() const;
    [[nodiscard]] static KeyBinding FromString(const std::string& str);
    [[nodiscard]] bool Matches(u32 vk, bool s, bool c, bool a) const noexcept {
        return (vkey == vk) && (shift == s) && (ctrl == c) && (alt == a);
    }
};

class ActionMap {
public:
    ActionMap();

    void SetDefaultBindings();
    void ResetToDefaults() { SetDefaultBindings(); }
    void SetBinding(InputAction action, const KeyBinding& binding);
    [[nodiscard]] const KeyBinding& GetBinding(InputAction action) const;
    [[nodiscard]] static const char* GetActionName(InputAction action) noexcept;
    [[nodiscard]] static const char* GetActionDescription(InputAction action) noexcept;

    [[nodiscard]] const std::unordered_map<InputAction, KeyBinding>& GetAllBindings() const noexcept {
        return m_bindings;
    }

private:
    std::unordered_map<InputAction, KeyBinding> m_bindings;
    KeyBinding m_fallback{};
};

} // namespace LokYS::Input
