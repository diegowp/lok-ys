#include "LokYS/navigation/RadioNavSystem.hpp"
#include <cmath>
#include <algorithm>

namespace LokYS::Navigation {

namespace {

constexpr f64 PI = 3.14159265358979323846;
constexpr f64 RAD_TO_DEG = 180.0 / PI;
constexpr f64 DEG_TO_RAD = PI / 180.0;
constexpr f64 METERS_PER_NM = 1852.0;

} // namespace

f64 RadioNavSystem::NormalizeDegrees360(f64 deg) noexcept {
    f64 result = std::fmod(deg, 360.0);
    if (result < 0.0) {
        result += 360.0;
    }
    return result;
}

f64 RadioNavSystem::NormalizeDegrees180(f64 deg) noexcept {
    f64 result = NormalizeDegrees360(deg);
    if (result > 180.0) {
        result -= 360.0;
    }
    return result;
}

f64 RadioNavSystem::CalculateVorRadial(
    const Math::Vec3d& aircraftNed,
    const Math::Vec3d& vorNed,
    f64 magVarDeg) noexcept
{
    f64 dx = aircraftNed.x - vorNed.x; // North
    f64 dy = aircraftNed.y - vorNed.y; // East

    if (dx * dx + dy * dy < 1.0) {
        return 0.0; // Directly overhead
    }

    f64 trueBearing = std::atan2(dy, dx) * RAD_TO_DEG;
    return NormalizeDegrees360(trueBearing - magVarDeg);
}

void RadioNavSystem::ResolveVorToFrom(
    f64 obsDeg,
    f64 radialDeg,
    bool& toFlag,
    bool& fromFlag,
    bool& navFlag) noexcept
{
    f64 diff = NormalizeDegrees180(radialDeg - obsDeg);

    // 8-degree cone of ambiguity around 90 and 270 degrees
    if (std::abs(diff) < 82.0) {
        toFlag = false;
        fromFlag = true;
        navFlag = false;
    } else if (std::abs(diff) > 98.0) {
        toFlag = true;
        fromFlag = false;
        navFlag = false;
    } else {
        toFlag = false;
        fromFlag = false;
        navFlag = true; // Beam crossing / cone of ambiguity
    }
}

f64 RadioNavSystem::CalculateCdiDeflection(
    f64 obsDeg,
    f64 radialDeg,
    bool toFlag) noexcept
{
    f64 errDeg = 0.0;
    if (toFlag) {
        // Flying inbound towards station on course obsDeg
        errDeg = NormalizeDegrees180(radialDeg - NormalizeDegrees360(obsDeg + 180.0));
    } else {
        // Flying outbound away from station on radial obsDeg
        errDeg = NormalizeDegrees180(radialDeg - obsDeg);
    }

    // Standard VOR calibration: 1 dot = 2.0 degrees deviation
    // If aircraft is right of track (errDeg > 0), needle deflects LEFT (negative dots)
    f64 dots = -errDeg / 2.0;
    return std::clamp(dots, -5.0, 5.0);
}

f64 RadioNavSystem::CalculateLocalizerDeflection(
    const Math::Vec3d& aircraftNed,
    const Math::Vec3d& locAntennaNed,
    f64 rwyHeadingDeg) noexcept
{
    f64 rwyRad = rwyHeadingDeg * DEG_TO_RAD;
    f64 cosH = std::cos(rwyRad);
    f64 sinH = std::sin(rwyRad);

    // Vector from aircraft to localizer antenna
    f64 dx = locAntennaNed.x - aircraftNed.x;
    f64 dy = locAntennaNed.y - aircraftNed.y;

    // Along-track and cross-track decomposition
    f64 alongTrack = dx * cosH + dy * sinH;
    f64 crossTrack = -dx * sinH + dy * cosH;

    if (alongTrack <= 50.0) {
        return 0.0; // Past antenna
    }

    f64 angleDeg = std::atan2(crossTrack, alongTrack) * RAD_TO_DEG;

    // Standard ILS Localizer calibration: 1 dot = 0.5 degrees deviation (+/- 2.5 deg full scale)
    // Needle points towards the runway centerline (fly-left / fly-right guidance)
    f64 dots = angleDeg / 0.5;
    return std::clamp(dots, -5.0, 5.0);
}

f64 RadioNavSystem::CalculateGlideslopeDeflection(
    const Math::Vec3d& aircraftNed,
    const Math::Vec3d& gsAntennaNed,
    [[maybe_unused]] f64 rwyHeadingDeg,
    f64 targetGsAngleDeg) noexcept
{
    f64 dx = gsAntennaNed.x - aircraftNed.x;
    f64 dy = gsAntennaNed.y - aircraftNed.y;
    f64 horizDist = std::sqrt(dx * dx + dy * dy);

    if (horizDist < 50.0 || dx <= 0.0) {
        return 0.0; // Over or past antenna
    }

    // Height above glideslope antenna (NED Z is positive down, so -Z is altitude)
    f64 altitudeAboveAntenna = -aircraftNed.z - (-gsAntennaNed.z);
    if (altitudeAboveAntenna < 0.0) {
        altitudeAboveAntenna = 0.0;
    }

    f64 currentAngleDeg = std::atan2(altitudeAboveAntenna, horizDist) * RAD_TO_DEG;
    f64 angleErrorDeg = currentAngleDeg - targetGsAngleDeg;

    // Standard ILS Glideslope calibration: 1 dot = 0.14 degrees deviation (+/- 0.7 deg full scale)
    // Above glidepath (angleErrorDeg > 0) -> Fly DOWN (needle deflects down / negative dots)
    f64 dots = -angleErrorDeg / 0.14;
    return std::clamp(dots, -5.0, 5.0);
}

f64 RadioNavSystem::CalculateDmeSlantRangeNm(
    const Math::Vec3d& aircraftNed,
    const Math::Vec3d& dmeNed) noexcept
{
    f64 dx = aircraftNed.x - dmeNed.x;
    f64 dy = aircraftNed.y - dmeNed.y;
    f64 dz = aircraftNed.z - dmeNed.z;
    f64 slantDistM = std::sqrt(dx * dx + dy * dy + dz * dz);
    return slantDistM / METERS_PER_NM;
}

RadioNavSystem::RadioNavSystem() {
    Initialize();
}

void RadioNavSystem::Initialize() {
    m_database.PopulateDefaults();

    // NAV1 initial frequencies: Active = 114.50 MHz (LOK VOR), Standby = 109.50 MHz (ILOK ILS 36)
    m_nav1.activeFrequencyMhz = 114.50;
    m_nav1.standbyFrequencyMhz = 109.50;
    m_nav1.cdi.obsDeg = 360.0;
    m_nav1.cdi.navFlag = true;
    m_nav1.cdi.gsFlag = true;

    // NAV2 initial frequencies
    m_nav2.activeFrequencyMhz = 115.70; // PAO VOR
    m_nav2.standbyFrequencyMhz = 114.10; // SJC VOR
    m_nav2.cdi.obsDeg = 180.0;
    m_nav2.cdi.navFlag = true;
    m_nav2.cdi.gsFlag = true;

    m_dmeInitialized = false;
}

void RadioNavSystem::SwapNav1Frequencies() noexcept {
    std::swap(m_nav1.activeFrequencyMhz, m_nav1.standbyFrequencyMhz);
    m_dmeInitialized = false;
}

void RadioNavSystem::AdjustNav1StandbyFrequency(f64 deltaMhz) noexcept {
    m_nav1.standbyFrequencyMhz += deltaMhz;
    // Keep in standard aeronautical VHF NAV band: 108.00 to 117.95 MHz
    if (m_nav1.standbyFrequencyMhz < 108.00) {
        m_nav1.standbyFrequencyMhz = 117.95;
    } else if (m_nav1.standbyFrequencyMhz > 117.95) {
        m_nav1.standbyFrequencyMhz = 108.00;
    }
}

void RadioNavSystem::AdjustNav1Obs(f64 deltaDeg) noexcept {
    m_nav1.cdi.obsDeg = NormalizeDegrees360(m_nav1.cdi.obsDeg + deltaDeg);
}

void RadioNavSystem::ToggleNav1AudioIdent() noexcept {
    m_nav1.audioIdentEnabled = !m_nav1.audioIdentEnabled;
}

void RadioNavSystem::Update(
    const Math::Vec3d& aircraftPosNed,
    [[maybe_unused]] const Math::Vec3d& velocityNed,
    f64 dt)
{
    // 1. Process NAV1 Receiver
    const NavFacility* fac = m_database.FindByFrequency(m_nav1.activeFrequencyMhz, aircraftPosNed);
    if (fac != nullptr) {
        m_nav1.stationIdent = fac->id;
        m_nav1.signalStrength = NavDatabase::CalculateSignalStrength(aircraftPosNed, *fac);

        if (fac->type == NavFacilityType::ILS) {
            // ILS Localizer & Glideslope processing
            m_nav1.cdi.isIls = true;
            m_nav1.cdi.radialDeg = fac->inboundCourseDeg;
            m_nav1.cdi.courseDeviationDots = CalculateLocalizerDeflection(
                aircraftPosNed,
                fac->positionNed,
                fac->inboundCourseDeg);
            m_nav1.cdi.navFlag = false;

            // Glideslope
            m_nav1.cdi.glideslopeDeviationDots = CalculateGlideslopeDeflection(
                aircraftPosNed,
                fac->glideslopeAntennaNed,
                fac->inboundCourseDeg,
                fac->glideslopeAngleDeg);
            m_nav1.cdi.gsFlag = false;

            m_nav1.cdi.toFlag = true;
            m_nav1.cdi.fromFlag = false;
        } else {
            // Standard VOR Processing
            m_nav1.cdi.isIls = false;
            m_nav1.cdi.radialDeg = CalculateVorRadial(
                aircraftPosNed,
                fac->positionNed,
                fac->magneticVariationDeg);

            ResolveVorToFrom(
                m_nav1.cdi.obsDeg,
                m_nav1.cdi.radialDeg,
                m_nav1.cdi.toFlag,
                m_nav1.cdi.fromFlag,
                m_nav1.cdi.navFlag);

            if (!m_nav1.cdi.navFlag) {
                m_nav1.cdi.courseDeviationDots = CalculateCdiDeflection(
                    m_nav1.cdi.obsDeg,
                    m_nav1.cdi.radialDeg,
                    m_nav1.cdi.toFlag);
            } else {
                m_nav1.cdi.courseDeviationDots = 0.0;
            }

            m_nav1.cdi.glideslopeDeviationDots = 0.0;
            m_nav1.cdi.gsFlag = true; // No GS on pure VOR
        }

        // DME Interrogation
        if (fac->hasDme) {
            m_nav1.dme.hasSignal = true;
            Math::Vec3d dmePos = fac->positionNed;
            if (fac->type == NavFacilityType::ILS) {
                // ILS DME references runway threshold (X = -1000m)
                dmePos = { -1000.0, 0.0, 0.0 };
            }
            f64 slantRange = CalculateDmeSlantRangeNm(aircraftPosNed, dmePos);
            m_nav1.dme.slantRangeNm = slantRange;

            if (m_dmeInitialized && dt > 0.001) {
                f64 closureRateNmPerSec = (m_prevDmeRangeNm - slantRange) / dt;
                f64 closingSpeedKnots = closureRateNmPerSec * 3600.0;
                // Filter closing ground speed
                m_nav1.dme.groundSpeedKnots = std::max(0.0, closingSpeedKnots);
                if (m_nav1.dme.groundSpeedKnots > 10.0) {
                    m_nav1.dme.timeToStationMinutes = (slantRange / m_nav1.dme.groundSpeedKnots) * 60.0;
                } else {
                    m_nav1.dme.timeToStationMinutes = 0.0;
                }
            } else {
                m_dmeInitialized = true;
                m_nav1.dme.groundSpeedKnots = 0.0;
                m_nav1.dme.timeToStationMinutes = 0.0;
            }
            m_prevDmeRangeNm = slantRange;
        } else {
            m_nav1.dme.hasSignal = false;
            m_nav1.dme.slantRangeNm = 0.0;
            m_nav1.dme.groundSpeedKnots = 0.0;
            m_nav1.dme.timeToStationMinutes = 0.0;
        }
    } else {
        // Out of range or no station on tuned frequency
        m_nav1.stationIdent.clear();
        m_nav1.signalStrength = 0.0;
        m_nav1.cdi.navFlag = true;
        m_nav1.cdi.gsFlag = true;
        m_nav1.cdi.toFlag = false;
        m_nav1.cdi.fromFlag = false;
        m_nav1.cdi.courseDeviationDots = 0.0;
        m_nav1.cdi.glideslopeDeviationDots = 0.0;
        m_nav1.dme.hasSignal = false;
        m_nav1.dme.slantRangeNm = 0.0;
        m_nav1.dme.groundSpeedKnots = 0.0;
        m_nav1.dme.timeToStationMinutes = 0.0;
    }

    // 2. Query Marker Beacons
    MarkerBeaconType marker = m_database.QueryMarkerBeacon(aircraftPosNed);
    m_nav1.marker.activeMarker = marker;
    m_nav1.marker.lightBlue = (marker == MarkerBeaconType::Outer);
    m_nav1.marker.lightAmber = (marker == MarkerBeaconType::Middle);
    m_nav1.marker.lightWhite = (marker == MarkerBeaconType::Inner);

    if (marker == MarkerBeaconType::Outer) {
        m_nav1.marker.audioToneHz = 400.0;
    } else if (marker == MarkerBeaconType::Middle) {
        m_nav1.marker.audioToneHz = 1300.0;
    } else if (marker == MarkerBeaconType::Inner) {
        m_nav1.marker.audioToneHz = 3000.0;
    } else {
        m_nav1.marker.audioToneHz = 0.0;
    }
}

} // namespace LokYS::Navigation
