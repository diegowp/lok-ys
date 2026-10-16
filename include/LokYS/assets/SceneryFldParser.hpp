#pragma once

#include "LokYS/core/Types.hpp"
#include "LokYS/math/Vector3.hpp"
#include "LokYS/scenery/AirportDatabase.hpp"
#include <string>
#include <string_view>
#include <vector>
#include <optional>

namespace LokYS::Assets {

struct TerrMeshBlock {
    std::string name;
    Math::Vec3d originNed{ 0.0, 0.0, 0.0 };
    f64 stepX{ 500.0 };
    f64 stepZ{ 500.0 };
    u32 cols{ 4 };
    u32 rows{ 4 };
    std::vector<f32> elevations; // row-major elevation grid in meters

    [[nodiscard]] f32 SampleElevation(f64 xNed, f64 yNed) const noexcept;
};

struct PlacedSceneryObject {
    std::string modelFile;
    std::string tag;
    Math::Vec3d positionNed{ 0.0, 0.0, 0.0 };
    f64 headingDeg{ 0.0 };
    f64 pitchDeg{ 0.0 };
    f64 rollDeg{ 0.0 };
};

struct SceneryField {
    std::string name;
    Math::Vec3f gndColor{ 0.01f, 0.13f, 0.72f };
    Math::Vec3f skyColor{ 0.14f, 0.48f, 0.82f };
    std::vector<Scenery::RunwayConfig> runways;
    std::vector<PlacedSceneryObject> placedObjects;
    std::vector<TerrMeshBlock> terrainBlocks;

    [[nodiscard]] f32 SampleElevationAt(f64 xNed, f64 yNed) const noexcept;
    [[nodiscard]] Scenery::AirportConfig ToAirportConfig(
        std::string_view icao = "YFLD",
        std::string_view airportName = "YSField Airport") const;
};

class SceneryFldParser {
public:
    // Parse a .fld scenery field from a text buffer
    [[nodiscard]] static std::optional<SceneryField> ParseFromString(
        std::string_view text,
        std::string_view fieldName = "UnnamedField");

    // Parse a .fld scenery field from disk
    [[nodiscard]] static std::optional<SceneryField> ParseFromFile(const std::string& filePath);
};

} // namespace LokYS::Assets
