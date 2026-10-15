#include "LokYS/input/ConfigPersistence.hpp"
#include "LokYS/core/JsonParser.hpp"
#include <fstream>
#include <filesystem>

namespace LokYS::Input {

std::string ConfigPersistence::GetDefaultConfigPath() {
    return "config/controls.json";
}

bool ConfigPersistence::LoadFromFile(const std::string& filePath, SimulatorConfig& outConfig) {
    std::ifstream file(filePath, std::ios::in | std::ios::binary);
    if (!file.is_open()) {
        return false;
    }

    std::string content((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    file.close();

    std::string err;
    Core::JsonValue root = Core::JsonValue::Parse(content, &err);
    if (!root.IsObject()) {
        return false;
    }

    // Audio & General
    if (root.HasKey("masterVolume")) {
        outConfig.masterVolume = root["masterVolume"].AsFloat(outConfig.masterVolume);
    }
    if (root.HasKey("cockpitDisplayMode")) {
        outConfig.cockpitDisplayMode = root["cockpitDisplayMode"].AsU32(outConfig.cockpitDisplayMode);
    }

    // Calibration
    if (root.HasKey("calibration") && root["calibration"].IsObject()) {
        const auto& calibObj = root["calibration"];
        outConfig.deadzone = calibObj["deadzone"].AsNumber(outConfig.deadzone);
        outConfig.sensitivityExp = calibObj["sensitivityExp"].AsNumber(outConfig.sensitivityExp);
        outConfig.invertPitch = calibObj["invertPitch"].AsBool(outConfig.invertPitch);
        outConfig.invertRoll = calibObj["invertRoll"].AsBool(outConfig.invertRoll);
        outConfig.invertYaw = calibObj["invertYaw"].AsBool(outConfig.invertYaw);
    }

    // Pre-Flight Defaults
    if (root.HasKey("preflight") && root["preflight"].IsObject()) {
        const auto& pfObj = root["preflight"];
        if (pfObj.HasKey("aircraft")) outConfig.defaultAircraft = pfObj["aircraft"].AsString(outConfig.defaultAircraft);
        if (pfObj.HasKey("airportIcao")) outConfig.defaultAirportIcao = pfObj["airportIcao"].AsString(outConfig.defaultAirportIcao);
        if (pfObj.HasKey("runwayIndex")) outConfig.defaultRunwayIndex = pfObj["runwayIndex"].AsU32(outConfig.defaultRunwayIndex);
        if (pfObj.HasKey("weatherPresetIndex")) outConfig.defaultWeatherPresetIndex = pfObj["weatherPresetIndex"].AsU32(outConfig.defaultWeatherPresetIndex);
        if (pfObj.HasKey("spawnMode")) outConfig.defaultSpawnMode = pfObj["spawnMode"].AsU32(outConfig.defaultSpawnMode);
    }

    // Action Bindings
    if (root.HasKey("bindings") && root["bindings"].IsObject()) {
        const auto& bindingsObj = root["bindings"].AsObject();
        for (u32 a = 0; a < static_cast<u32>(InputAction::Count); ++a) {
            auto action = static_cast<InputAction>(a);
            std::string actionName = ActionMap::GetActionName(action);
            auto it = bindingsObj.find(actionName);
            if (it != bindingsObj.end() && it->second.IsString()) {
                KeyBinding kb = KeyBinding::FromString(it->second.AsString());
                if (kb.vkey != 0) {
                    outConfig.actionMap.SetBinding(action, kb);
                }
            }
        }
    }

    return true;
}

bool ConfigPersistence::SaveToFile(const std::string& filePath, const SimulatorConfig& config) {
    try {
        std::filesystem::path p(filePath);
        if (p.has_parent_path()) {
            std::filesystem::create_directories(p.parent_path());
        }
    } catch (...) {
        return false;
    }

    Core::JsonValue root(std::unordered_map<std::string, Core::JsonValue>{});
    root["version"] = Core::JsonValue("1.0.0");
    root["masterVolume"] = Core::JsonValue(static_cast<double>(config.masterVolume));
    root["cockpitDisplayMode"] = Core::JsonValue(config.cockpitDisplayMode);

    // Calibration
    Core::JsonValue calib(std::unordered_map<std::string, Core::JsonValue>{});
    calib["deadzone"] = Core::JsonValue(config.deadzone);
    calib["sensitivityExp"] = Core::JsonValue(config.sensitivityExp);
    calib["invertPitch"] = Core::JsonValue(config.invertPitch);
    calib["invertRoll"] = Core::JsonValue(config.invertRoll);
    calib["invertYaw"] = Core::JsonValue(config.invertYaw);
    root["calibration"] = std::move(calib);

    // Pre-Flight Defaults
    Core::JsonValue pf(std::unordered_map<std::string, Core::JsonValue>{});
    pf["aircraft"] = Core::JsonValue(config.defaultAircraft);
    pf["airportIcao"] = Core::JsonValue(config.defaultAirportIcao);
    pf["runwayIndex"] = Core::JsonValue(config.defaultRunwayIndex);
    pf["weatherPresetIndex"] = Core::JsonValue(config.defaultWeatherPresetIndex);
    pf["spawnMode"] = Core::JsonValue(config.defaultSpawnMode);
    root["preflight"] = std::move(pf);

    // Action Bindings
    Core::JsonValue bindings(std::unordered_map<std::string, Core::JsonValue>{});
    const auto& map = config.actionMap.GetAllBindings();
    for (const auto& [action, kb] : map) {
        std::string actionName = ActionMap::GetActionName(action);
        bindings[actionName] = Core::JsonValue(kb.ToString());
    }
    root["bindings"] = std::move(bindings);

    std::string jsonStr = root.Serialize(true);
    std::ofstream file(filePath, std::ios::out | std::ios::trunc | std::ios::binary);
    if (!file.is_open()) {
        return false;
    }
    file.write(jsonStr.data(), jsonStr.size());
    file.close();
    return true;
}

} // namespace LokYS::Input
