#pragma once

#include "LokYS/core/Types.hpp"
#include "LokYS/math/Vector3.hpp"
#include <string>
#include <vector>
#include <optional>

namespace LokYS::Scenery {

enum class RunwaySurface : u8 {
    Asphalt,
    Concrete,
    TurfGrass
};

struct RunwayConfig {
    std::string ident;          // e.g. "36/18" or "09/27"
    Math::Vec3d centerNed;      // Center position in NED frame (m)
    f64 lengthM{ 2000.0 };
    f64 widthM{ 45.0 };
    f64 headingDeg{ 0.0 };      // Magnetic runway heading
    f64 elevationM{ 0.0 };      // Elevation above MSL
    RunwaySurface surface{ RunwaySurface::Asphalt };

    bool hasAls{ true };        // 720m ALSF-1 Approach Lighting System
    bool hasPapi{ true };       // 4-unit PAPI light array
    f64 papiGlideslopeDeg{ 3.0 };
    bool hasReil{ false };      // Runway End Identifier Lights (flashing strobes)
    bool hasIls{ true };
    f64 ilsFrequencyMhz{ 109.50 };
};

struct AirportConfig {
    std::string icao;           // e.g. "KLOK", "KMES", "KVAL"
    std::string name;           // e.g. "LokYS Regional", "Mesa Highlands", "Valley Ranch"
    Math::Vec3d locationNed;    // Geographic reference location in NED (m)
    f64 elevationM{ 0.0 };      // Airport reference elevation (m)
    std::vector<RunwayConfig> runways;
    f64 vorFrequencyMhz{ 0.0 };
    std::string vorIdent;
};

class AirportDatabase {
public:
    AirportDatabase();
    ~AirportDatabase() = default;

    [[nodiscard]] const std::vector<AirportConfig>& GetAirports() const noexcept { return m_airports; }
    [[nodiscard]] const std::vector<AirportConfig>& GetAllAirports() const noexcept { return m_airports; }

    [[nodiscard]] std::optional<AirportConfig> FindAirport(const std::string& icao) const noexcept;
    [[nodiscard]] const AirportConfig& GetNearestAirport(const Math::Vec3d& posNed) const noexcept;

    /// @brief Computes multi-airport plateau elevation flattening.
    /// Smoothly blends airport mesas (e.g. Mesa Highlands at +350m) into regional terrain.
    [[nodiscard]] f32 SampleAirportElevationOffset(f32 xGfx, f32 zGfx, f32& outFlattenWeight) const noexcept;

    [[nodiscard]] f64 SampleAirportElevationOffset(f64 xNed, f64 yNed) const noexcept {
        // Overload taking coordinates in NED frame (x=North, y=East)
        // Graphics coords: Z_gfx = X_ned, X_gfx = -Y_ned
        f32 weight = 0.0f;
        f32 elev = SampleAirportElevationOffset(static_cast<f32>(-yNed), static_cast<f32>(xNed), weight);
        return static_cast<f64>(elev * weight);
    }

private:
    void InitializeAirports();

    std::vector<AirportConfig> m_airports;
};

} // namespace LokYS::Scenery
