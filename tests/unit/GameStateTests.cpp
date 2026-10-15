#include "LokYS/core/GameState.hpp"
#include <iostream>
#include <string>
#include <cassert>

using namespace LokYS::Core;

static int g_testsPassed = 0;
static int g_testsFailed = 0;

#define TEST_ASSERT(cond, msg) \
    do { \
        if (!(cond)) { \
            std::cerr << "[FAIL] " << msg << " (line " << __LINE__ << ")\n"; \
            g_testsFailed++; \
        } else { \
            g_testsPassed++; \
        } \
    } while (0)

void TestInitialStateAndConstructors() {
    GameStateManager mgrDefault;
    TEST_ASSERT(mgrDefault.GetCurrentState() == GameState::MainMenu, "Default state is MainMenu");
    TEST_ASSERT(mgrDefault.IsInMenu(), "IsInMenu() returns true initially");
    TEST_ASSERT(!mgrDefault.IsInSimulation(), "IsInSimulation() returns false initially");
    TEST_ASSERT(!mgrDefault.IsPaused(), "IsPaused() returns false initially");

    GameStateManager mgrSim(GameState::Simulation);
    TEST_ASSERT(mgrSim.GetCurrentState() == GameState::Simulation, "Explicit state is Simulation");
    TEST_ASSERT(mgrSim.IsInSimulation(), "IsInSimulation() returns true for Simulation manager");
}

void TestStateTransitions() {
    GameStateManager mgr;

    // MainMenu -> Simulation
    mgr.TransitionTo(GameState::Simulation);
    TEST_ASSERT(mgr.GetCurrentState() == GameState::Simulation, "State is Simulation after transition");
    TEST_ASSERT(mgr.GetPreviousState() == GameState::MainMenu, "Previous state tracked as MainMenu");

    // Simulation -> Paused
    mgr.TransitionTo(GameState::Paused);
    TEST_ASSERT(mgr.GetCurrentState() == GameState::Paused, "State is Paused after transition");
    TEST_ASSERT(mgr.GetPreviousState() == GameState::Simulation, "Previous state tracked as Simulation");

    // Paused -> Settings
    mgr.TransitionTo(GameState::Settings);
    TEST_ASSERT(mgr.GetCurrentState() == GameState::Settings, "State is Settings");
    TEST_ASSERT(mgr.GetPreviousState() == GameState::Paused, "Previous state tracked as Paused");

    // Return to previous
    mgr.ReturnToPrevious();
    TEST_ASSERT(mgr.GetCurrentState() == GameState::Paused, "Returned to Paused");
}

void TestPauseToggle() {
    GameStateManager mgr(GameState::Simulation);
    TEST_ASSERT(mgr.IsInSimulation(), "Starts in Simulation");

    // Pause toggle 1: Simulation -> Paused
    mgr.TogglePause();
    TEST_ASSERT(mgr.IsPaused(), "TogglePause() transitions to Paused");
    TEST_ASSERT(!mgr.IsInSimulation(), "No longer in simulation");

    // Pause toggle 2: Paused -> Simulation
    mgr.TogglePause();
    TEST_ASSERT(mgr.IsInSimulation(), "TogglePause() resumes Simulation");
    TEST_ASSERT(!mgr.IsPaused(), "No longer paused");
}

void TestFlightSetupConfiguration() {
    GameStateManager mgr;
    FlightSetupConfiguration config{};
    config.selectedAircraftIndex = 2; // Extra 330SC
    config.airportIcao = "KMES";      // Mesa Highlands
    config.selectedRunwayIndex = 0;   // Rwy 09/27
    config.weatherPresetIndex = 3;    // Gusty
    config.spawnMode = SpawnMode::AirborneCruise;
    config.initialFuelPercentage = 0.85f;

    mgr.SetFlightConfig(config);
    const auto& fetched = mgr.GetFlightConfig();

    TEST_ASSERT(fetched.selectedAircraftIndex == 2, "Config aircraft index stored");
    TEST_ASSERT(fetched.airportIcao == "KMES", "Config airport ICAO stored");
    TEST_ASSERT(fetched.selectedRunwayIndex == 0, "Config runway index stored");
    TEST_ASSERT(fetched.weatherPresetIndex == 3, "Config weather index stored");
    TEST_ASSERT(fetched.spawnMode == SpawnMode::AirborneCruise, "Config spawn mode stored");
    TEST_ASSERT(std::abs(fetched.initialFuelPercentage - 0.85f) < 1e-4f, "Config fuel percentage stored");
}

void TestStringConversions() {
    TEST_ASSERT(std::string(GameStateToString(GameState::MainMenu)) == "Main Menu", "GameState MainMenu string");
    TEST_ASSERT(std::string(GameStateToString(GameState::Simulation)) == "Simulation", "GameState Simulation string");
    TEST_ASSERT(std::string(GameStateToString(GameState::Paused)) == "Paused", "GameState Paused string");
    TEST_ASSERT(std::string(GameStateToString(GameState::Settings)) == "Settings", "GameState Settings string");

    TEST_ASSERT(std::string(SpawnModeToString(SpawnMode::RunwayTakeoff)) == "Runway Takeoff", "SpawnMode RunwayTakeoff string");
    TEST_ASSERT(std::string(SpawnModeToString(SpawnMode::AirborneCruise)) == "3000 ft Airborne Cruise", "SpawnMode AirborneCruise string");
}

int main() {
    std::cout << "========================================\n";
    std::cout << " LokYS GameState & Pre-Flight Flow Tests\n";
    std::cout << "========================================\n";

    TestInitialStateAndConstructors();
    TestStateTransitions();
    TestPauseToggle();
    TestFlightSetupConfiguration();
    TestStringConversions();

    std::cout << "\n----------------------------------------\n";
    std::cout << "Tests Passed: " << g_testsPassed << "\n";
    std::cout << "Tests Failed: " << g_testsFailed << "\n";
    std::cout << "----------------------------------------\n";

    return (g_testsFailed == 0) ? 0 : 1;
}
