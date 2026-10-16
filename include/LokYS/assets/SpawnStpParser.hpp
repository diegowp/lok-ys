#pragma once

#include "LokYS/core/Types.hpp"
#include "LokYS/math/Vector3.hpp"
#include <string>
#include <string_view>
#include <vector>
#include <optional>

namespace LokYS::Assets {

struct SpawnPoint {
    std::string name;
    Math::Vec3d positionNed{ 0.0, 0.0, 0.0 }; // North (m), East (m), Down (m)
    f64 altitudeMslM{ 0.0 };                  // Elevation above Mean Sea Level (m)
    f64 headingDeg{ 0.0 };                    // Magnetic heading [0, 360)
    f64 pitchDeg{ 0.0 };                      // Initial pitch [deg]
    f64 rollDeg{ 0.0 };                       // Initial roll [deg]
    f64 initialSpeedMps{ 0.0 };               // Initial True Airspeed [m/s]
    f64 initialThrottle{ 0.0 };               // Initial throttle [0.0, 1.0]
    bool gearDown{ true };                    // Landing gear extended
    bool isAirborne{ false };                 // True if airspawn, false if on ground/runway
};

struct ScenerySpawnList {
    std::vector<SpawnPoint> points;

    [[nodiscard]] const SpawnPoint* FindPoint(std::string_view name) const noexcept;
    [[nodiscard]] std::vector<SpawnPoint> GetRunwaySpawns() const;
    [[nodiscard]] std::vector<SpawnPoint> GetAirSpawns() const;
};

class SpawnStpParser {
public:
    // Parse an STP starting points script from text
    [[nodiscard]] static std::optional<ScenerySpawnList> ParseFromString(std::string_view text);

    // Parse an STP starting points script from disk
    [[nodiscard]] static std::optional<ScenerySpawnList> ParseFromFile(const std::string& filePath);
};

} // namespace LokYS::Assets
