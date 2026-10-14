#include "LokYS/scenery/AirportDatabase.hpp"
#include "LokYS/math/CoordinateFrames.hpp"
#include <cmath>
#include <algorithm>

namespace LokYS::Scenery {

AirportDatabase::AirportDatabase() {
    InitializeAirports();
}

void AirportDatabase::InitializeAirports() {
    // -------------------------------------------------------------
    // 1. KLOK - LokYS Regional Airport (Hub / Primary International)
    // -------------------------------------------------------------
    AirportConfig klok{};
    klok.icao = "KLOK";
    klok.name = "LokYS Regional Airport";
    klok.locationNed = Math::Vec3d(0.0, 0.0, 0.0);
    klok.elevationM = 0.0;
    klok.vorFrequencyMhz = 114.50;
    klok.vorIdent = "LOK";

    RunwayConfig rwy36{};
    rwy36.ident = "36/18";
    rwy36.centerNed = Math::Vec3d(0.0, 0.0, 0.0);
    rwy36.lengthM = 2000.0;
    rwy36.widthM = 45.0;
    rwy36.headingDeg = 0.0;
    rwy36.elevationM = 0.0;
    rwy36.surface = RunwaySurface::Asphalt;
    rwy36.hasAls = true;
    rwy36.hasPapi = true;
    rwy36.papiGlideslopeDeg = 3.0;
    rwy36.hasReil = false;
    rwy36.hasIls = true;
    rwy36.ilsFrequencyMhz = 109.50;
    klok.runways.push_back(rwy36);

    m_airports.push_back(klok);

    // -------------------------------------------------------------
    // 2. KMES - Mesa Highlands Airport (Mountain Plateau Airfield)
    // -------------------------------------------------------------
    AirportConfig kmes{};
    kmes.icao = "KMES";
    kmes.name = "Mesa Highlands Airport";
    // Located 14 km North, 8 km East at +350m MSL elevation (-350m Z in NED)
    kmes.locationNed = Math::Vec3d(14000.0, 8000.0, -350.0);
    kmes.elevationM = 350.0;
    kmes.vorFrequencyMhz = 116.20;
    kmes.vorIdent = "MES";

    RunwayConfig rwy09{};
    rwy09.ident = "09/27";
    rwy09.centerNed = Math::Vec3d(14000.0, 8000.0, -350.0);
    rwy09.lengthM = 1400.0;
    rwy09.widthM = 30.0;
    rwy09.headingDeg = 90.0;
    rwy09.elevationM = 350.0;
    rwy09.surface = RunwaySurface::Asphalt;
    rwy09.hasAls = false;
    rwy09.hasPapi = true;
    rwy09.papiGlideslopeDeg = 3.0;
    rwy09.hasReil = true; // REIL flashing strobes
    rwy09.hasIls = false;
    kmes.runways.push_back(rwy09);

    m_airports.push_back(kmes);

    // -------------------------------------------------------------
    // 3. KVAL - Valley Ranch Airstrip (Rural Grass Turf Strip)
    // -------------------------------------------------------------
    AirportConfig kval{};
    kval.icao = "KVAL";
    kval.name = "Valley Ranch Airstrip";
    // Located 10 km South, 6 km West at +60m MSL elevation (-60m Z in NED)
    kval.locationNed = Math::Vec3d(-10000.0, -6000.0, -60.0);
    kval.elevationM = 60.0;
    kval.vorFrequencyMhz = 0.0; // Uncontrolled VFR airstrip
    kval.vorIdent = "";

    RunwayConfig rwy04{};
    rwy04.ident = "04/22";
    rwy04.centerNed = Math::Vec3d(-10000.0, -6000.0, -60.0);
    rwy04.lengthM = 700.0;
    rwy04.widthM = 25.0;
    rwy04.headingDeg = 40.0;
    rwy04.elevationM = 60.0;
    rwy04.surface = RunwaySurface::TurfGrass;
    rwy04.hasAls = false;
    rwy04.hasPapi = false;
    rwy04.hasReil = false;
    rwy04.hasIls = false;
    kval.runways.push_back(rwy04);

    m_airports.push_back(kval);
}

std::optional<AirportConfig> AirportDatabase::FindAirport(const std::string& icao) const noexcept {
    for (const auto& apt : m_airports) {
        if (apt.icao == icao) {
            return apt;
        }
    }
    return std::nullopt;
}

const AirportConfig& AirportDatabase::GetNearestAirport(const Math::Vec3d& posNed) const noexcept {
    size_t bestIdx = 0;
    f64 bestDistSq = 1e30;

    for (size_t i = 0; i < m_airports.size(); ++i) {
        f64 dx = posNed.x - m_airports[i].locationNed.x;
        f64 dy = posNed.y - m_airports[i].locationNed.y;
        f64 dSq = dx * dx + dy * dy;
        if (dSq < bestDistSq) {
            bestDistSq = dSq;
            bestIdx = i;
        }
    }

    return m_airports[bestIdx];
}

f32 AirportDatabase::SampleAirportElevationOffset(f32 xGfx, f32 zGfx, f32& outFlattenWeight) const noexcept {
    outFlattenWeight = 0.0f;
    f32 targetElev = 0.0f;

    // Convert graphics (X, Z) into NED (X=North, Y=East)
    // Ned.X = Z_gfx, Ned.Y = -X_gfx
    const f64 nedX = static_cast<f64>(zGfx);
    const f64 nedY = static_cast<f64>(-xGfx);

    constexpr f64 DEG_TO_RAD = 3.14159265358979323846 / 180.0;
    constexpr f64 BLEND_MARGIN = 1200.0; // Smooth 1.2km transition between airport plateau and regional terrain

    for (const auto& apt : m_airports) {
        for (const auto& rwy : apt.runways) {
            const f64 dxCenter = nedX - rwy.centerNed.x;
            const f64 dyCenter = nedY - rwy.centerNed.y;

            // Rotate coordinates into runway-aligned frame (u = along runway, v = across runway)
            const f64 headingRad = rwy.headingDeg * DEG_TO_RAD;
            const f64 cosH = std::cos(headingRad);
            const f64 sinH = std::sin(headingRad);

            const f64 uAlong = dxCenter * cosH + dyCenter * sinH;
            const f64 vAcross = -dxCenter * sinH + dyCenter * cosH;

            // Flat reservation box covers runway length + 500m overruns, and runway width + 350m taxiways/apron
            const f64 halfLengthFlat = (rwy.lengthM * 0.5) + 500.0;
            const f64 halfWidthFlat = std::max(350.0, (rwy.widthM * 0.5) + 250.0);

            const f64 du = std::max(0.0, std::abs(uAlong) - halfLengthFlat);
            const f64 dv = std::max(0.0, std::abs(vAcross) - halfWidthFlat);
            const f64 distToBox = std::sqrt(du * du + dv * dv);

            if (distToBox <= 0.0) {
                outFlattenWeight = 1.0f;
                return static_cast<f32>(rwy.elevationM);
            } else if (distToBox < BLEND_MARGIN) {
                const f64 frac = distToBox / BLEND_MARGIN;
                const f64 w = 0.5 * (1.0 + std::cos(frac * 3.14159265358979323846));
                const f32 fw = static_cast<f32>(w);
                if (fw > outFlattenWeight) {
                    outFlattenWeight = fw;
                    targetElev = static_cast<f32>(rwy.elevationM);
                }
            }
        }
    }

    return targetElev;
}

} // namespace LokYS::Scenery
