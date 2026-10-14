#include "LokYS/navigation/RadioNavSystem.hpp"
#include <iostream>
#include <cmath>

#define TEST_ASSERT(cond, msg) \
    do { \
        if (!(cond)) { \
            std::cerr << "Assertion failed: " << (msg) << " (" << __FILE__ << ":" << __LINE__ << ")\n"; \
            return false; \
        } \
    } while (0)

namespace {

using namespace LokYS;
using namespace LokYS::Navigation;

constexpr f64 EPSILON = 1e-2;

bool FloatNear(f64 a, f64 b, f64 eps = EPSILON) {
    return std::fabs(a - b) <= eps;
}

bool TestVorRadialCalculations() {
    std::cout << "[RUNNING] TestVorRadialCalculations...\n";

    Math::Vec3d vorPos{ 0.0, 0.0, 0.0 };

    // Due North (+X) -> Radial 000 / 360
    f64 rNorth = RadioNavSystem::CalculateVorRadial({ 5000.0, 0.0, 0.0 }, vorPos);
    TEST_ASSERT(FloatNear(rNorth, 0.0) || FloatNear(rNorth, 360.0), "Due North must be 000 deg radial");

    // Due East (+Y) -> Radial 090
    f64 rEast = RadioNavSystem::CalculateVorRadial({ 0.0, 5000.0, 0.0 }, vorPos);
    TEST_ASSERT(FloatNear(rEast, 90.0), "Due East must be 090 deg radial");

    // Due South (-X) -> Radial 180
    f64 rSouth = RadioNavSystem::CalculateVorRadial({ -5000.0, 0.0, 0.0 }, vorPos);
    TEST_ASSERT(FloatNear(rSouth, 180.0), "Due South must be 180 deg radial");

    // Due West (-Y) -> Radial 270
    f64 rWest = RadioNavSystem::CalculateVorRadial({ 0.0, -5000.0, 0.0 }, vorPos);
    TEST_ASSERT(FloatNear(rWest, 270.0), "Due West must be 270 deg radial");

    // North-East -> Radial 045
    f64 rNorthEast = RadioNavSystem::CalculateVorRadial({ 5000.0, 5000.0, 0.0 }, vorPos);
    TEST_ASSERT(FloatNear(rNorthEast, 45.0), "North-East must be 045 deg radial");

    // Magnetic declination / variation application (5 deg East variation)
    f64 rMagVar = RadioNavSystem::CalculateVorRadial({ 0.0, 5000.0, 0.0 }, vorPos, 5.0);
    TEST_ASSERT(FloatNear(rMagVar, 85.0), "Magnetic variation of 5 deg East should shift radial to 085");

    std::cout << "[PASSED] TestVorRadialCalculations\n";
    return true;
}

bool TestVorToFromAndAmbiguity() {
    std::cout << "[RUNNING] TestVorToFromAndAmbiguity...\n";

    f64 obs = 90.0;
    bool toFlag = false, fromFlag = false, navFlag = false;

    // Sensed radial 090 with OBS 090 (flying outbound away from station) -> FROM
    RadioNavSystem::ResolveVorToFrom(obs, 90.0, toFlag, fromFlag, navFlag);
    TEST_ASSERT(fromFlag && !toFlag && !navFlag, "OBS 090 on radial 090 must yield FROM flag");

    // Sensed radial 270 with OBS 090 (flying inbound towards station on 090 track) -> TO
    RadioNavSystem::ResolveVorToFrom(obs, 270.0, toFlag, fromFlag, navFlag);
    TEST_ASSERT(toFlag && !fromFlag && !navFlag, "OBS 090 on radial 270 must yield TO flag");

    // Sensed radial 000 with OBS 090 (90 deg perpendicular: beam crossing) -> NAV flag
    RadioNavSystem::ResolveVorToFrom(obs, 0.0, toFlag, fromFlag, navFlag);
    TEST_ASSERT(navFlag && !toFlag && !fromFlag, "OBS 090 on radial 000 must trigger NAV flag ambiguity");

    // Sensed radial 180 with OBS 090 (perpendicular beam crossing) -> NAV flag
    RadioNavSystem::ResolveVorToFrom(obs, 180.0, toFlag, fromFlag, navFlag);
    TEST_ASSERT(navFlag && !toFlag && !fromFlag, "OBS 090 on radial 180 must trigger NAV flag ambiguity");

    std::cout << "[PASSED] TestVorToFromAndAmbiguity\n";
    return true;
}

bool TestCdiDeflectionScaling() {
    std::cout << "[RUNNING] TestCdiDeflectionScaling...\n";

    f64 obs = 90.0;

    // Outbound on course (FROM, radial 090) -> Centered (0.0 dots)
    f64 devCenter = RadioNavSystem::CalculateCdiDeflection(obs, 90.0, false);
    TEST_ASSERT(FloatNear(devCenter, 0.0), "CDI must be centered on radial");

    // 4 degrees right of radial (radial 094) -> Needle should deflect LEFT (-2.0 dots)
    f64 devRight = RadioNavSystem::CalculateCdiDeflection(obs, 94.0, false);
    TEST_ASSERT(FloatNear(devRight, -2.0), "4 deg right of track must deflect needle 2 dots left");

    // 4 degrees left of radial (radial 086) -> Needle should deflect RIGHT (+2.0 dots)
    f64 devLeft = RadioNavSystem::CalculateCdiDeflection(obs, 86.0, false);
    TEST_ASSERT(FloatNear(devLeft, 2.0), "4 deg left of track must deflect needle 2 dots right");

    // 10 degrees deviation -> Full scale (5.0 dots)
    f64 devFullScale = RadioNavSystem::CalculateCdiDeflection(obs, 80.0, false);
    TEST_ASSERT(FloatNear(devFullScale, 5.0), "10 deg left of track must peg needle at 5.0 dots");

    // 20 degrees deviation -> Clamped at 5.0 dots
    f64 devPegged = RadioNavSystem::CalculateCdiDeflection(obs, 70.0, false);
    TEST_ASSERT(FloatNear(devPegged, 5.0), "Excess deviation must be clamped at 5.0 dots");

    // Inbound (TO, radial 270) -> Centered (0.0 dots)
    f64 devInboundCenter = RadioNavSystem::CalculateCdiDeflection(obs, 270.0, true);
    TEST_ASSERT(FloatNear(devInboundCenter, 0.0), "CDI must be centered inbound on reciprocal radial");

    // Inbound right of track (radial 274) -> Needle deflects left (-2.0 dots)
    f64 devInboundRight = RadioNavSystem::CalculateCdiDeflection(obs, 274.0, true);
    TEST_ASSERT(FloatNear(devInboundRight, -2.0), "Inbound right of track must deflect left");

    std::cout << "[PASSED] TestCdiDeflectionScaling\n";
    return true;
}

bool TestIlsLocalizerAndGlideslope() {
    std::cout << "[RUNNING] TestIlsLocalizerAndGlideslope...\n";

    // Runway 36 localizer antenna at departure end (X = +1100m, Y = 0)
    Math::Vec3d locAntenna{ 1100.0, 0.0, 0.0 };
    f64 rwyHdg = 000.0;

    // Aircraft on extended centerline (Y = 0, X = -4000m): Centered localizer
    f64 locCenter = RadioNavSystem::CalculateLocalizerDeflection({ -4000.0, 0.0, -300.0 }, locAntenna, rwyHdg);
    TEST_ASSERT(FloatNear(locCenter, 0.0), "On-centerline aircraft must have 0.0 localizer deflection");

    // Aircraft 1.0 degree right of centerline -> 2.0 dots LEFT (-2.0 dots, since 1 dot = 0.5 deg)
    // At distance 5100m from antenna (X = -4000m), 1.0 deg corresponds to Y = 5100 * tan(1.0 deg) ~= 89.0m
    f64 yOffset1Deg = 5100.0 * std::tan(1.0 * 3.141592653589793 / 180.0);
    f64 locRight = RadioNavSystem::CalculateLocalizerDeflection({ -4000.0, yOffset1Deg, -300.0 }, locAntenna, rwyHdg);
    TEST_ASSERT(FloatNear(locRight, -2.0, 0.05), "1.0 deg offset on localizer must produce 2.0 dots deflection");

    // Glideslope antenna at (-700m, 75m, 0m). Target GS = 3.0 deg.
    Math::Vec3d gsAntenna{ -700.0, 75.0, 0.0 };

    // At distance 5000m from antenna (X = -5700m), 3.0 deg altitude is 5000 * tan(3.0 deg) ~= 262.0m (-Z = 262.0m)
    f64 onGlideHeight = 5000.0 * std::tan(3.0 * 3.141592653589793 / 180.0);
    f64 gsCenter = RadioNavSystem::CalculateGlideslopeDeflection({ -5700.0, 0.0, -onGlideHeight }, gsAntenna, rwyHdg, 3.0);
    TEST_ASSERT(FloatNear(gsCenter, 0.0, 0.05), "Aircraft on 3.0 deg path must have centered GS needle");

    // Aircraft ABOVE path by 0.28 deg (altitude corresponding to 3.28 deg) -> Fly DOWN (-2.0 dots, 1 dot = 0.14 deg)
    f64 aboveHeight = 5000.0 * std::tan(3.28 * 3.141592653589793 / 180.0);
    f64 gsAbove = RadioNavSystem::CalculateGlideslopeDeflection({ -5700.0, 0.0, -aboveHeight }, gsAntenna, rwyHdg, 3.0);
    TEST_ASSERT(FloatNear(gsAbove, -2.0, 0.05), "0.28 deg high on glideslope must command 2 dots fly-down");

    // Aircraft BELOW path by 0.28 deg (altitude corresponding to 2.72 deg) -> Fly UP (+2.0 dots)
    f64 belowHeight = 5000.0 * std::tan(2.72 * 3.141592653589793 / 180.0);
    f64 gsBelow = RadioNavSystem::CalculateGlideslopeDeflection({ -5700.0, 0.0, -belowHeight }, gsAntenna, rwyHdg, 3.0);
    TEST_ASSERT(FloatNear(gsBelow, 2.0, 0.05), "0.28 deg low on glideslope must command 2 dots fly-up");

    std::cout << "[PASSED] TestIlsLocalizerAndGlideslope\n";
    return true;
}

bool TestDmeSlantRange() {
    std::cout << "[RUNNING] TestDmeSlantRange...\n";

    Math::Vec3d dmePos{ 0.0, 0.0, 0.0 };
    // 3000m North, 4000m East, 1000m altitude (-1000m Z)
    // Slant range = sqrt(3000^2 + 4000^2 + 1000^2) = sqrt(26,000,000) ~= 5099.02m / 1852m ~= 2.753 NM
    f64 rangeNm = RadioNavSystem::CalculateDmeSlantRangeNm({ 3000.0, 4000.0, -1000.0 }, dmePos);
    TEST_ASSERT(FloatNear(rangeNm, 5099.02 / 1852.0, 0.01), "DME slant range calculation accuracy");

    std::cout << "[PASSED] TestDmeSlantRange\n";
    return true;
}

bool TestMarkerBeacons() {
    std::cout << "[RUNNING] TestMarkerBeacons...\n";

    NavDatabase db;

    // Over Outer Marker (OM: X = -8408m)
    MarkerBeaconType mOuter = db.QueryMarkerBeacon({ -8408.0, 0.0, -350.0 });
    TEST_ASSERT(mOuter == MarkerBeaconType::Outer, "Outer marker detection at ~4.5 NM");

    // Over Middle Marker (MM: X = -1800m)
    MarkerBeaconType mMiddle = db.QueryMarkerBeacon({ -1800.0, 0.0, -70.0 });
    TEST_ASSERT(mMiddle == MarkerBeaconType::Middle, "Middle marker detection at ~0.5 NM");

    // Over Inner Marker (IM: X = -1000m)
    MarkerBeaconType mInner = db.QueryMarkerBeacon({ -1000.0, 0.0, -15.0 });
    TEST_ASSERT(mInner == MarkerBeaconType::Inner, "Inner marker detection at threshold");

    // Away from approach (off-centerline Y = 2000m)
    MarkerBeaconType mNone = db.QueryMarkerBeacon({ -8408.0, 2000.0, -350.0 });
    TEST_ASSERT(mNone == MarkerBeaconType::None, "Marker beacon must not activate off-course");

    std::cout << "[PASSED] TestMarkerBeacons\n";
    return true;
}

bool TestRadioNavSystemIntegration() {
    std::cout << "[RUNNING] TestRadioNavSystemIntegration...\n";

    RadioNavSystem nav;
    nav.Initialize();

    // 1. Initial State: NAV1 tuned to 114.50 MHz (LOK VOR), Standby 109.50 MHz (ILOK ILS 36)
    const auto& nav1 = nav.GetNav1();
    TEST_ASSERT(FloatNear(nav1.activeFrequencyMhz, 114.50), "NAV1 default active frequency");
    TEST_ASSERT(FloatNear(nav1.standbyFrequencyMhz, 109.50), "NAV1 default standby frequency");

    // Position aircraft 5 NM North of LOK VOR (VOR is at X=0, Y=1200, Z=-10)
    Math::Vec3d acPos{ 5.0 * 1852.0, 1200.0, -914.4 };
    nav.Update(acPos, { 50.0, 0.0, 0.0 }, 1.0 / 60.0);

    TEST_ASSERT(nav.GetNav1().stationIdent == "LOK", "Acquired LOK VOR ident");
    TEST_ASSERT(!nav.GetNav1().cdi.isIls, "LOK should be recognized as VOR, not ILS");
    TEST_ASSERT(FloatNear(nav.GetNav1().dme.slantRangeNm, 5.0, 0.1), "DME distance approx 5.0 NM");

    // 2. Flip-Flop frequency swap to 109.50 (ILOK ILS)
    nav.SwapNav1Frequencies();
    TEST_ASSERT(FloatNear(nav.GetNav1().activeFrequencyMhz, 109.50), "NAV1 active swapped to 109.50");
    TEST_ASSERT(FloatNear(nav.GetNav1().standbyFrequencyMhz, 114.50), "NAV1 standby swapped to 114.50");

    // Position aircraft on 5 NM final approach for Runway 36 (X = -10000m, Y = 0m, 3.0 deg descent)
    f64 ilsDistFromThreshold = 9000.0;
    f64 ilsAltitude = ilsDistFromThreshold * std::tan(3.0 * 3.141592653589793 / 180.0);
    nav.Update({ -10000.0, 0.0, -ilsAltitude }, { 40.0, 0.0, 0.0 }, 1.0 / 60.0);

    TEST_ASSERT(nav.GetNav1().stationIdent == "ILOK", "Acquired ILOK ILS ident");
    TEST_ASSERT(nav.GetNav1().cdi.isIls, "ILOK should be recognized as ILS");
    TEST_ASSERT(!nav.GetNav1().cdi.navFlag, "Localizer valid (no NAV flag)");
    TEST_ASSERT(!nav.GetNav1().cdi.gsFlag, "Glideslope valid (no GS flag)");
    TEST_ASSERT(FloatNear(nav.GetNav1().cdi.courseDeviationDots, 0.0, 0.1), "Localizer centered on final");

    // 3. Pilot adjustments
    nav.AdjustNav1Obs(10.0);
    TEST_ASSERT(FloatNear(nav.GetNav1().cdi.obsDeg, 10.0) || FloatNear(nav.GetNav1().cdi.obsDeg, 370.0 - 360.0), "OBS adjusted by +10 deg");

    nav.AdjustNav1StandbyFrequency(0.50);
    TEST_ASSERT(FloatNear(nav.GetNav1().standbyFrequencyMhz, 115.00), "Standby frequency adjusted by +0.50 MHz");

    nav.ToggleNav1AudioIdent();
    TEST_ASSERT(nav.GetNav1().audioIdentEnabled, "Audio ident toggled ON");

    std::cout << "[PASSED] TestRadioNavSystemIntegration\n";
    return true;
}

} // namespace

int main() {
    std::cout << "============================================================\n";
    std::cout << "LokYS Native Windows Flight Simulator — Radio Nav Unit Tests\n";
    std::cout << "============================================================\n";

    bool success = true;
    success &= TestVorRadialCalculations();
    success &= TestVorToFromAndAmbiguity();
    success &= TestCdiDeflectionScaling();
    success &= TestIlsLocalizerAndGlideslope();
    success &= TestDmeSlantRange();
    success &= TestMarkerBeacons();
    success &= TestRadioNavSystemIntegration();

    std::cout << "============================================================\n";
    if (success) {
        std::cout << "[ALL 7 RADIO NAVIGATION TEST SUITES PASSED (100%)]\n";
        return 0;
    } else {
        std::cerr << "[ONE OR MORE RADIO NAVIGATION TESTS FAILED]\n";
        return 1;
    }
}
