#pragma once

#include "LokYS/core/Types.hpp"
#include "LokYS/aircraft/AircraftTypes.hpp"
#include <string>
#include <string_view>
#include <optional>
#include <vector>

namespace LokYS::Assets {

struct YsGearPosition {
    Math::Vec3d leftMainFRD{ 0.0, 0.0, 0.0 };
    Math::Vec3d rightMainFRD{ 0.0, 0.0, 0.0 };
    Math::Vec3d noseOrTailFRD{ 0.0, 0.0, 0.0 };
    bool isTaildragger{ false };
};

struct ParsedAircraftDat {
    Aircraft::AircraftConfig config;
    YsGearPosition gearPositions;
    std::string cockpitPanelPath;
    std::string visualModelDnm;
    std::string collisionMeshSrf;
    std::string cockpitInteriorSrf;
    f64 tireFriction{ 0.09 };
};

class AircraftDatParser {
public:
    // Parse an aircraft .dat file from a text buffer
    [[nodiscard]] static std::optional<ParsedAircraftDat> ParseFromString(
        std::string_view text,
        std::string_view fallbackName = "Custom YS Aircraft");

    // Parse an aircraft .dat file from a file path
    [[nodiscard]] static std::optional<ParsedAircraftDat> ParseFromFile(const std::string& filePath);

private:
    static void EstimateInertiaTensor(
        f64 massKg,
        f64 wingspanM,
        f64 lengthM,
        Math::Mat3d& outTensor) noexcept;
};

} // namespace LokYS::Assets
