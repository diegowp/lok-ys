#include "LokYS/input/ActionMapping.hpp"
#include "LokYS/input/ConfigPersistence.hpp"

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <iostream>
#include <fstream>
#include <string>
#include <filesystem>
#include <cassert>

using namespace LokYS::Input;
using LokYS::u32;

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

void TestActionMapDefaults() {
    ActionMap map;

    const auto& bindings = map.GetAllBindings();
    TEST_ASSERT(!bindings.empty(), "ActionMap has default bindings populated");
    TEST_ASSERT(bindings.size() >= static_cast<size_t>(InputAction::Count), "All actions are bound");

    KeyBinding pitchUp = map.GetBinding(InputAction::PitchUp);
    TEST_ASSERT(pitchUp.vkey != 0, "PitchUp has a valid key");

    KeyBinding brake = map.GetBinding(InputAction::ToggleParkingBrake);
    TEST_ASSERT(brake.vkey == VK_SPACE, "Parking brake is default SPACE");

    KeyBinding pause = map.GetBinding(InputAction::TogglePause);
    TEST_ASSERT(pause.vkey == VK_ESCAPE, "Pause is default ESC");

    for (u32 a = 0; a < static_cast<u32>(InputAction::Count); ++a) {
        auto act = static_cast<InputAction>(a);
        const char* name = ActionMap::GetActionName(act);
        TEST_ASSERT(name != nullptr && std::string(name) != "Unknown Action", "Action has valid name");
        const char* desc = ActionMap::GetActionDescription(act);
        TEST_ASSERT(desc != nullptr, "Action has valid description");
    }
}

void TestKeyBindingStringFormatting() {
    KeyBinding kbSpace{ VK_SPACE, false, false, false };
    TEST_ASSERT(kbSpace.ToString() == "SPACE", "Space binding string matches");

    KeyBinding kbCtrlShiftW{ 'W', true, true, false };
    TEST_ASSERT(kbCtrlShiftW.ToString() == "CTRL+SHIFT+W", "Modifier string formatting matches");

    KeyBinding parsed = KeyBinding::FromString("CTRL+SHIFT+W");
    TEST_ASSERT(parsed.ctrl == true, "Parsed ctrl modifier");
    TEST_ASSERT(parsed.shift == true, "Parsed shift modifier");
    TEST_ASSERT(parsed.alt == false, "Parsed alt modifier is false");
    TEST_ASSERT(parsed.vkey == 'W', "Parsed key W");

    KeyBinding parsedSpace = KeyBinding::FromString("SPACE");
    TEST_ASSERT(parsedSpace.vkey == VK_SPACE, "Parsed SPACE key");

    KeyBinding parsedNone = KeyBinding::FromString("NONE");
    TEST_ASSERT(parsedNone.vkey == 0, "Parsed NONE is 0");
}

void TestRebinding() {
    ActionMap map;
    KeyBinding custom{ 'P', false, false, false };
    map.SetBinding(InputAction::ToggleParkingBrake, custom);

    KeyBinding retrieved = map.GetBinding(InputAction::ToggleParkingBrake);
    TEST_ASSERT(retrieved.vkey == 'P', "Parking brake remapped to P");
    TEST_ASSERT(retrieved.ToString() == "P", "ToString matches remapped key");

    // Reset to defaults
    map.SetDefaultBindings();
    TEST_ASSERT(map.GetBinding(InputAction::ToggleParkingBrake).vkey == VK_SPACE, "Reset restored SPACE");
}

void TestJsonPersistenceRoundtrip() {
    SimulatorConfig originalConfig{};
    originalConfig.masterVolume = 0.72f;
    originalConfig.cockpitDisplayMode = 1; // HUD
    originalConfig.deadzone = 0.08;
    originalConfig.sensitivityExp = 1.8;
    originalConfig.invertPitch = true;
    originalConfig.defaultAircraft = "Beechcraft Baron 58";
    originalConfig.defaultAirportIcao = "KMES";
    originalConfig.defaultRunwayIndex = 1;
    originalConfig.defaultWeatherPresetIndex = 2;
    originalConfig.defaultSpawnMode = 1; // Cruise

    originalConfig.actionMap.SetBinding(InputAction::ToggleCamera, KeyBinding{ 'K', false, false, false });

    std::string testPath = "scratch/test_controls_config.json";
    bool saveOk = ConfigPersistence::SaveToFile(testPath, originalConfig);
    TEST_ASSERT(saveOk, "ConfigPersistence::SaveToFile succeeded");

    SimulatorConfig loadedConfig{};
    bool loadOk = ConfigPersistence::LoadFromFile(testPath, loadedConfig);
    TEST_ASSERT(loadOk, "ConfigPersistence::LoadFromFile succeeded");

    TEST_ASSERT(std::abs(loadedConfig.masterVolume - 0.72f) < 0.001f, "Loaded master volume matches");
    TEST_ASSERT(loadedConfig.cockpitDisplayMode == 1, "Loaded cockpit display mode matches");
    TEST_ASSERT(std::abs(loadedConfig.deadzone - 0.08) < 0.001, "Loaded deadzone matches");
    TEST_ASSERT(std::abs(loadedConfig.sensitivityExp - 1.8) < 0.001, "Loaded sensitivityExp matches");
    TEST_ASSERT(loadedConfig.invertPitch == true, "Loaded invertPitch matches");
    TEST_ASSERT(loadedConfig.defaultAircraft == "Beechcraft Baron 58", "Loaded defaultAircraft matches");
    TEST_ASSERT(loadedConfig.defaultAirportIcao == "KMES", "Loaded defaultAirportIcao matches");
    TEST_ASSERT(loadedConfig.defaultRunwayIndex == 1, "Loaded defaultRunwayIndex matches");
    TEST_ASSERT(loadedConfig.defaultWeatherPresetIndex == 2, "Loaded defaultWeatherPresetIndex matches");
    TEST_ASSERT(loadedConfig.defaultSpawnMode == 1, "Loaded defaultSpawnMode matches");

    KeyBinding loadedCamera = loadedConfig.actionMap.GetBinding(InputAction::ToggleCamera);
    TEST_ASSERT(loadedCamera.vkey == 'K', "Loaded remapped camera binding matches");
}

void TestCorruptedAndMissingJsonResilience() {
    SimulatorConfig cfg{};
    bool loadMissing = ConfigPersistence::LoadFromFile("non_existent_directory/missing_file.json", cfg);
    TEST_ASSERT(!loadMissing, "Missing file returns false without crashing");

    // Invalid JSON text
    std::string badPath = "scratch/corrupted_config.json";
    std::ofstream badFile(badPath, std::ios::out | std::ios::trunc);
    badFile << "{ this is not valid json : [ 1, 2, }";
    badFile.close();

    bool loadCorrupted = ConfigPersistence::LoadFromFile(badPath, cfg);
    TEST_ASSERT(!loadCorrupted, "Corrupted JSON returns false gracefully");
}

int main() {
    std::cout << "========================================\n";
    std::cout << " LokYS Action Mapping & Persistence Tests\n";
    std::cout << "========================================\n\n";

    TestActionMapDefaults();
    TestKeyBindingStringFormatting();
    TestRebinding();
    TestJsonPersistenceRoundtrip();
    TestCorruptedAndMissingJsonResilience();

    std::cout << "----------------------------------------\n";
    std::cout << "Tests Passed: " << g_testsPassed << "\n";
    std::cout << "Tests Failed: " << g_testsFailed << "\n";
    std::cout << "----------------------------------------\n";

    return (g_testsFailed == 0) ? 0 : 1;
}
