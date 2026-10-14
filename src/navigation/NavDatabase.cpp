#include "LokYS/navigation/NavDatabase.hpp"
#include <cmath>
#include <algorithm>

namespace LokYS::Navigation {

namespace {

constexpr f64 METERS_PER_NM = 1852.0;
constexpr f64 METERS_TO_FEET = 3.280839895013123;

} // namespace

NavDatabase::NavDatabase() {
    PopulateDefaults();
}

void NavDatabase::PopulateDefaults() {
    m_facilities.clear();

    // 1. LokYS Regional VOR/DME ("LOK") - 114.50 MHz
    // Co-located 1.2 km East of runway center
    NavFacility lokVor{};
    lokVor.id = "LOK";
    lokVor.name = "LokYS Regional VOR/DME";
    lokVor.type = NavFacilityType::VOR_DME;
    lokVor.frequencyMhz = 114.50;
    lokVor.positionNed = { 0.0, 1200.0, -10.0 };
    lokVor.elevationM = 10.0;
    lokVor.magneticVariationDeg = 0.0;
    lokVor.rangeNm = 40.0;
    lokVor.hasDme = true;
    m_facilities.push_back(lokVor);

    // 2. Instrument Landing System Runway 36 ("ILOK") - 109.50 MHz
    // Runway 36: Landing heading 000.0 True/Magnetic (approaching from South, flying North)
    NavFacility ilokIls{};
    ilokIls.id = "ILOK";
    ilokIls.name = "ILS Runway 36";
    ilokIls.type = NavFacilityType::ILS;
    ilokIls.frequencyMhz = 109.50;
    // Localizer antenna situated 100m past North departure end (X = +1100m, Y = 0)
    ilokIls.positionNed = { 1100.0, 0.0, 0.0 };
    ilokIls.elevationM = 0.0;
    ilokIls.magneticVariationDeg = 0.0;
    ilokIls.rangeNm = 18.0; // Standard 18 NM localizer service volume
    ilokIls.hasDme = true;  // Paired DME measuring distance to approach threshold
    ilokIls.inboundCourseDeg = 000.0;
    ilokIls.glideslopeAngleDeg = 3.0;
    // Glideslope antenna located 300m past approach threshold (X = -700m), offset 75m East
    ilokIls.glideslopeAntennaNed = { -700.0, 75.0, 0.0 };
    m_facilities.push_back(ilokIls);

    // 3. Palo Alto VOR/DME ("PAO") - 115.70 MHz
    // Situated ~10 NM South-West (X = -18520m, Y = -5000m)
    NavFacility paoVor{};
    paoVor.id = "PAO";
    paoVor.name = "Palo Alto VOR/DME";
    paoVor.type = NavFacilityType::VOR_DME;
    paoVor.frequencyMhz = 115.70;
    paoVor.positionNed = { -18520.0, -5000.0, -5.0 };
    paoVor.elevationM = 5.0;
    paoVor.magneticVariationDeg = 0.0;
    paoVor.rangeNm = 50.0;
    paoVor.hasDme = true;
    m_facilities.push_back(paoVor);

    // 4. San Jose VOR/DME ("SJC") - 114.10 MHz
    // Situated ~20 NM South-East (X = -37040m, Y = +15000m)
    NavFacility sjcVor{};
    sjcVor.id = "SJC";
    sjcVor.name = "San Jose VOR/DME";
    sjcVor.type = NavFacilityType::VOR_DME;
    sjcVor.frequencyMhz = 114.10;
    sjcVor.positionNed = { -37040.0, 15000.0, -18.0 };
    sjcVor.elevationM = 18.0;
    sjcVor.magneticVariationDeg = 0.0;
    sjcVor.rangeNm = 60.0;
    sjcVor.hasDme = true;
    m_facilities.push_back(sjcVor);
}

void NavDatabase::AddFacility(const NavFacility& facility) {
    m_facilities.push_back(facility);
}

const NavFacility* NavDatabase::FindByFrequency(
    f64 freqMhz,
    const Math::Vec3d& aircraftPosNed) const noexcept
{
    const NavFacility* bestFacility = nullptr;
    f64 maxStrength = 0.0;

    for (const auto& fac : m_facilities) {
        if (std::abs(fac.frequencyMhz - freqMhz) < 0.025) {
            f64 strength = CalculateSignalStrength(aircraftPosNed, fac);
            if (strength > maxStrength) {
                maxStrength = strength;
                bestFacility = &fac;
            }
        }
    }

    return bestFacility;
}

f64 NavDatabase::CalculateSignalStrength(
    const Math::Vec3d& aircraftPosNed,
    const NavFacility& facility) noexcept
{
    f64 dx = aircraftPosNed.x - facility.positionNed.x;
    f64 dy = aircraftPosNed.y - facility.positionNed.y;
    f64 distNm = std::sqrt(dx * dx + dy * dy) / METERS_PER_NM;

    // Line of sight radio horizon: d = 1.23 * (sqrt(h_ac) + sqrt(h_st))
    f64 acAltFt = std::max(0.0, -aircraftPosNed.z * METERS_TO_FEET);
    f64 stAltFt = std::max(0.0, facility.elevationM * METERS_TO_FEET);
    f64 radioHorizonNm = 1.23 * (std::sqrt(acAltFt) + std::sqrt(stAltFt));

    f64 effectiveMaxRange = std::min(facility.rangeNm, radioHorizonNm);
    if (distNm > effectiveMaxRange || effectiveMaxRange <= 0.0) {
        return 0.0;
    }

    // For ILS, check reception coverage sector (+/- 35 deg of centerline)
    if (facility.type == NavFacilityType::ILS) {
        // Runway 36 approach is from the South (X < 1100)
        f64 toAntennaX = facility.positionNed.x - aircraftPosNed.x;
        f64 toAntennaY = aircraftPosNed.y - facility.positionNed.y;
        f64 azimuthFromCenterlineDeg = std::abs(std::atan2(toAntennaY, toAntennaX) * 180.0 / 3.141592653589793);
        if (azimuthFromCenterlineDeg > 35.0 || toAntennaX <= 0.0) {
            return 0.0; // Outside ILS service volume
        }
    }

    // Normalized signal strength [0, 1]
    f64 normDist = distNm / effectiveMaxRange;
    return std::clamp(1.0 - (normDist * normDist), 0.05, 1.0);
}

MarkerBeaconType NavDatabase::QueryMarkerBeacon(
    const Math::Vec3d& aircraftPosNed) const noexcept
{
    // Approach along Runway 36 centerline: |Y| < 250m, altitude < 1500m AGL
    if (std::abs(aircraftPosNed.y) > 250.0 || aircraftPosNed.z > 0.0 || aircraftPosNed.z < -1500.0) {
        return MarkerBeaconType::None;
    }

    f64 x = aircraftPosNed.x;

    // Outer Marker (OM): ~4.5 NM out (X = -8408m +/- 250m)
    if (x >= -8658.0 && x <= -8158.0) {
        return MarkerBeaconType::Outer;
    }

    // Middle Marker (MM): ~0.5 NM out (X = -1800m +/- 150m)
    if (x >= -1950.0 && x <= -1650.0) {
        return MarkerBeaconType::Middle;
    }

    // Inner Marker (IM): At threshold (X = -1000m +/- 80m)
    if (x >= -1080.0 && x <= -920.0) {
        return MarkerBeaconType::Inner;
    }

    return MarkerBeaconType::None;
}

} // namespace LokYS::Navigation
