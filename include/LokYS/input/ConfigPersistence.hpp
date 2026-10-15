#pragma once

#include "LokYS/core/Types.hpp"
#include "LokYS/input/ActionMapping.hpp"
#include <string>

namespace LokYS::Input {

struct SimulatorConfig {
    ActionMap actionMap{};
    f64 deadzone{ 0.04 };
    f64 sensitivityExp{ 1.5 };
    bool invertPitch{ false };
    bool invertRoll{ false };
    bool invertYaw{ false };
    f32 masterVolume{ 0.85f };
    std::string defaultAircraft{ "Cessna 172SP Skyhawk" };
    std::string defaultAirportIcao{ "KLOK" };
    u32 defaultRunwayIndex{ 0 };
    u32 defaultWeatherPresetIndex{ 0 };
    u32 defaultSpawnMode{ 0 }; // 0: Runway, 1: Cruise
    u32 cockpitDisplayMode{ 0 }; // 0: PFD, 1: HUD, 2: Analog, 3: Off
};

class ConfigPersistence {
public:
    [[nodiscard]] static bool LoadFromFile(const std::string& filePath, SimulatorConfig& outConfig);
    [[nodiscard]] static bool SaveToFile(const std::string& filePath, const SimulatorConfig& config);
    [[nodiscard]] static std::string GetDefaultConfigPath();
};

} // namespace LokYS::Input
