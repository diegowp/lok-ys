#pragma once

#include "LokYS/core/Types.hpp"
#include <string>
#include <cstdint>

namespace LokYS::Core {

enum class GameState : uint8_t {
    MainMenu,
    Simulation,
    Paused,
    Settings
};

[[nodiscard]] constexpr const char* GameStateToString(GameState state) noexcept {
    switch (state) {
        case GameState::MainMenu:   return "Main Menu";
        case GameState::Simulation: return "Simulation";
        case GameState::Paused:     return "Paused";
        case GameState::Settings:   return "Settings";
        default:                    return "Unknown";
    }
}

enum class SpawnMode : uint8_t {
    RunwayTakeoff,
    AirborneCruise
};

[[nodiscard]] constexpr const char* SpawnModeToString(SpawnMode mode) noexcept {
    switch (mode) {
        case SpawnMode::RunwayTakeoff: return "Runway Takeoff";
        case SpawnMode::AirborneCruise: return "3000 ft Airborne Cruise";
        default: return "Unknown";
    }
}

struct FlightSetupConfiguration {
    size_t selectedAircraftIndex{ 0 };
    std::string airportIcao{ "KLOK" };
    uint32_t selectedRunwayIndex{ 0 };
    uint8_t weatherPresetIndex{ 0 }; // 0: Clear Calm, 1: Standard, 2: Crosswind, 3: Gusty, 4: Hot/High
    SpawnMode spawnMode{ SpawnMode::RunwayTakeoff };
    float initialFuelPercentage{ 1.0f }; // [0.0, 1.0]
};

class GameStateManager {
public:
    constexpr GameStateManager() noexcept = default;
    explicit constexpr GameStateManager(GameState initialState) noexcept
        : m_currentState(initialState), m_previousState(initialState) {}

    [[nodiscard]] constexpr GameState GetCurrentState() const noexcept { return m_currentState; }
    [[nodiscard]] constexpr GameState GetPreviousState() const noexcept { return m_previousState; }
    [[nodiscard]] constexpr bool IsInSimulation() const noexcept { return m_currentState == GameState::Simulation; }
    [[nodiscard]] constexpr bool IsPaused() const noexcept { return m_currentState == GameState::Paused; }
    [[nodiscard]] constexpr bool IsInMenu() const noexcept { return m_currentState == GameState::MainMenu; }
    [[nodiscard]] constexpr bool IsInSettings() const noexcept { return m_currentState == GameState::Settings; }

    constexpr void TransitionTo(GameState newState) noexcept {
        if (m_currentState != newState) {
            m_previousState = m_currentState;
            m_currentState = newState;
        }
    }

    constexpr void TogglePause() noexcept {
        if (m_currentState == GameState::Simulation) {
            TransitionTo(GameState::Paused);
        } else if (m_currentState == GameState::Paused) {
            TransitionTo(GameState::Simulation);
        }
    }

    constexpr void ReturnToPrevious() noexcept {
        TransitionTo(m_previousState);
    }

    [[nodiscard]] FlightSetupConfiguration& GetFlightConfig() noexcept { return m_flightConfig; }
    [[nodiscard]] const FlightSetupConfiguration& GetFlightConfig() const noexcept { return m_flightConfig; }
    void SetFlightConfig(const FlightSetupConfiguration& config) noexcept { m_flightConfig = config; }

private:
    GameState m_currentState{ GameState::MainMenu };
    GameState m_previousState{ GameState::MainMenu };
    FlightSetupConfiguration m_flightConfig{};
};

} // namespace LokYS::Core
