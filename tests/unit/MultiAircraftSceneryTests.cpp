#include "LokYS/aircraft/AircraftTypes.hpp"
#include "LokYS/aircraft/AircraftRegistry.hpp"
#include "LokYS/scenery/AirportDatabase.hpp"
#include "LokYS/graphics/ApproachLightingSystem.hpp"
#include "LokYS/graphics/TerrainStreamingSystem.hpp"
#include "LokYS/aerodynamics/AerodynamicsSystem.hpp"
#include "LokYS/math/Units.hpp"
#include "LokYS/math/Vector3.hpp"
#include <iostream>
#include <cmath>
#include <vector>

using namespace LokYS;
using namespace LokYS::Aircraft;
using namespace LokYS::Scenery;
using namespace LokYS::Graphics;
using namespace LokYS::Aerodynamics;
using namespace LokYS::Math;

#define TEST_ASSERT(cond, msg) \
    do { \
        if (!(cond)) { \
            std::cerr << "[FAIL] " << msg << " (" << __FILE__ << ":" << __LINE__ << ")\n"; \
            return false; \
        } \
    } while (0)

#define TEST_NEAR(actual, expected, tol, msg) \
    do { \
        f64 a = static_cast<f64>(actual); \
        f64 e = static_cast<f64>(expected); \
        if (std::abs(a - e) > (tol)) { \
            std::cerr << "[FAIL] " << msg << " (Expected " << e << ", got " << a \
                      << ", diff=" << std::abs(a - e) << " > " << (tol) << ")\n"; \
            return false; \
        } \
    } while (0)

namespace {

// 1. Validates all 3 aircraft configurations in AircraftRegistry
bool TestAircraftRegistryConfigurations() {
    std::cout << "[RUNNING] TestAircraftRegistryConfigurations...\n";

    AircraftRegistry registry;

    // Default aircraft is Cessna 172
    TEST_ASSERT(registry.GetActiveType() == AircraftType::Cessna172Skyhawk, "Default aircraft should be C172");
    const auto& c172 = registry.GetActiveConfig();
    TEST_ASSERT(c172.category == AircraftCategory::SingleEnginePiston, "C172 category must be SEP");
    TEST_ASSERT(c172.propulsion.engineCount == 1, "C172 must have 1 engine");
    TEST_NEAR(c172.propulsion.horsepowerPerEngine, 160.0, 0.1, "C172 must be 160 hp");
    TEST_ASSERT(!c172.hasRetractableGear, "C172 must have fixed landing gear");
    TEST_NEAR(c172.vSpeeds.Vs, 48.0, 0.1, "C172 Vs clean stall speed");
    TEST_NEAR(c172.vSpeeds.Vne, 163.0, 0.1, "C172 Vne never exceed speed");

    // Cycle to Baron 58
    registry.CycleNextAircraft();
    TEST_ASSERT(registry.GetActiveType() == AircraftType::Baron58, "Next aircraft should be Baron 58");
    const auto& b58 = registry.GetActiveConfig();
    TEST_ASSERT(b58.category == AircraftCategory::MultiEnginePiston, "Baron 58 category must be MEP");
    TEST_ASSERT(b58.propulsion.engineCount == 2, "Baron 58 must have 2 engines");
    TEST_NEAR(b58.propulsion.horsepowerPerEngine, 300.0, 0.1, "Baron 58 must be 300 hp per engine");
    TEST_NEAR(b58.propulsion.lateralOffsetM, 1.83, 0.01, "Baron 58 engine lateral offset must be 1.83m");
    TEST_ASSERT(b58.hasRetractableGear, "Baron 58 must have retractable landing gear");
    TEST_NEAR(b58.vSpeeds.Vmc, 84.0, 0.1, "Baron 58 Vmc critical engine speed must be 84 kt");
    TEST_NEAR(b58.vSpeeds.Vne, 223.0, 0.1, "Baron 58 Vne never exceed speed must be 223 kt");

    // Cycle to Extra 330SC
    registry.CycleNextAircraft();
    TEST_ASSERT(registry.GetActiveType() == AircraftType::Extra330SC, "Next aircraft should be Extra 330SC");
    const auto& e330 = registry.GetActiveConfig();
    TEST_ASSERT(e330.category == AircraftCategory::Aerobatic, "Extra 330 category must be Aerobatic");
    TEST_ASSERT(e330.propulsion.engineCount == 1, "Extra 330 must have 1 engine");
    TEST_NEAR(e330.propulsion.horsepowerPerEngine, 315.0, 0.1, "Extra 330 must be 315 hp");
    TEST_ASSERT(!e330.hasRetractableGear, "Extra 330 must have fixed gear with fairings");
    TEST_ASSERT(e330.propulsion.isAerobaticInvertedFuel, "Extra 330 must have inverted fuel/oil system");
    TEST_NEAR(e330.aero.CL0, 0.0, 1e-4, "Extra 330 symmetric airfoil CL0 must be 0.0");
    TEST_NEAR(e330.mass.emptyMassKg, 650.0, 0.1, "Extra 330 empty mass 650 kg");

    // Cycle back to C172
    registry.CycleNextAircraft();
    TEST_ASSERT(registry.GetActiveType() == AircraftType::Cessna172Skyhawk, "Cycle should loop back to C172");

    std::cout << "[PASS] TestAircraftRegistryConfigurations\n";
    return true;
}

// 2. Validates multi-engine asymmetric thrust and yaw moment
bool TestMultiEngineAsymmetricThrust() {
    std::cout << "[RUNNING] TestMultiEngineAsymmetricThrust...\n";

    AircraftRegistry registry;
    registry.SetActiveAircraft(AircraftType::Baron58);

    // Case A: Symmetric thrust (Left 2000 N, Right 2000 N) -> Asymmetric yaw moment must be zero
    Math::Vec3d momentsSym = registry.ComputeMultiEngineForces(2000.0, 2000.0);
    TEST_NEAR(momentsSym.z, 0.0, 1e-4, "Symmetric thrust produces zero asymmetric yaw moment");

    // Case B: Single-engine failure (Left 3000 N, Right 0 N)
    // Left engine offset is y = -1.83 m. Thrust forward (+X) at y = -1.83 m produces a right yaw moment (+Z):
    // N = T_left * offset = 3000 N * 1.83 m = +5490 N*m
    Math::Vec3d momentsAsymLeft = registry.ComputeMultiEngineForces(3000.0, 0.0);
    TEST_NEAR(momentsAsymLeft.z, 5490.0, 1.0, "Left engine thrust produces +5490 N*m yawing moment to the right");

    // Case C: Critical right engine running (Left 0 N, Right 3000 N)
    // Right engine offset is y = +1.83 m. Thrust forward (+X) at y = +1.83 m produces a left yaw moment (-Z):
    // N = -T_right * offset = -3000 N * 1.83 m = -5490 N*m
    Math::Vec3d momentsAsymRight = registry.ComputeMultiEngineForces(0.0, 3000.0);
    TEST_NEAR(momentsAsymRight.z, -5490.0, 1.0, "Right engine thrust produces -5490 N*m yawing moment to the left");

    // Case D: Single engine aircraft (Cessna 172) always produces zero asymmetric multi-engine yaw
    registry.SetActiveAircraft(AircraftType::Cessna172Skyhawk);
    Math::Vec3d momentsC172 = registry.ComputeMultiEngineForces(2500.0, 0.0);
    TEST_NEAR(momentsC172.z, 0.0, 1e-4, "Single-engine aircraft produces 0 asymmetric multi-engine yaw");

    std::cout << "[PASS] TestMultiEngineAsymmetricThrust\n";
    return true;
}

// 3. Validates retractable gear aerodynamic parasite drag reduction
bool TestRetractableGearAerodynamics() {
    std::cout << "[RUNNING] TestRetractableGearAerodynamics...\n";

    AircraftRegistry registry;
    registry.SetActiveAircraft(AircraftType::Baron58);

    // Initial state for Baron 58: gear extended (DOWN)
    TEST_ASSERT(registry.IsGearExtended(), "Baron 58 initial gear should be DOWN");
    const auto& configDown = registry.GetActiveConfig();
    TEST_NEAR(configDown.aero.CD0_gearDown, 0.038, 1e-4, "Gear down CD0 should be 0.038");

    // Toggle gear UP
    registry.ToggleLandingGear();
    TEST_ASSERT(!registry.IsGearExtended(), "Gear should now be RETRACTED (UP)");

    // Aerodynamics system with gear UP vs gear DOWN
    AerodynamicsSystem aero;
    aero.SetConfig(configDown.geometry, configDown.aero, false); // Gear UP

    f64 airspeed = Units::KnotsToMps(160.0); // 160 kt cruise
    Math::Vec3d relWind(airspeed, 0.0, 0.0);
    Math::Vec3d omega(0.0, 0.0, 0.0);
    ControlDeflections controls{};

    AeroOutput outClean = aero.Evaluate(relWind, omega, controls);

    // Now reconfigure with gear DOWN
    aero.SetConfig(configDown.geometry, configDown.aero, true); // Gear DOWN
    AeroOutput outDirty = aero.Evaluate(relWind, omega, controls);

    // Parasite drag with gear extended must be significantly higher than gear retracted:
    // CD_dirty - CD_clean = 0.038 - 0.024 = +0.014
    f64 deltaCd = outDirty.coeffs.CD - outClean.coeffs.CD;
    TEST_NEAR(deltaCd, 0.014, 1e-4, "Gear extension increases parasite drag by exactly Delta CD0 = 0.014");
    TEST_ASSERT(outDirty.coeffs.CD > outClean.coeffs.CD, "Dirty drag must exceed clean drag");

    std::cout << "[PASS] TestRetractableGearAerodynamics\n";
    return true;
}

// 4. Validates Extra 330SC symmetric airfoil and aerobatic characteristics
bool TestAerobaticSymmetricAirfoil() {
    std::cout << "[RUNNING] TestAerobaticSymmetricAirfoil...\n";

    AircraftRegistry registry;
    registry.SetActiveAircraft(AircraftType::Extra330SC);
    const auto& config = registry.GetActiveConfig();

    AerodynamicsSystem aero;
    aero.SetConfig(config.geometry, config.aero, false);

    f64 airspeed = Units::KnotsToMps(120.0);
    Math::Vec3d omega(0.0, 0.0, 0.0);
    ControlDeflections controls{};

    // 1. Zero lift at zero angle of attack for symmetric airfoil NACA 0012
    AeroOutput out0 = aero.Evaluate(Math::Vec3d(airspeed, 0.0, 0.0), omega, controls);
    TEST_NEAR(out0.coeffs.CL, 0.0, 1e-4, "Symmetric airfoil lift at alpha=0 must be exactly zero");
    TEST_NEAR(out0.coeffs.Cm, 0.0, 1e-4, "Symmetric airfoil pitching moment at alpha=0 must be zero");

    // 2. Symmetric lift magnitude for positive and negative alpha (+/- 8 deg)
    f64 alphaRad = Units::DegToRad(8.0);
    Math::Vec3d relWindPos(airspeed * std::cos(alphaRad), 0.0, airspeed * std::sin(alphaRad));
    Math::Vec3d relWindNeg(airspeed * std::cos(-alphaRad), 0.0, airspeed * std::sin(-alphaRad));

    AeroOutput outPos = aero.Evaluate(relWindPos, omega, controls);
    AeroOutput outNeg = aero.Evaluate(relWindNeg, omega, controls);

    TEST_NEAR(outPos.coeffs.CL, -outNeg.coeffs.CL, 1e-4, "Lift magnitude at +8 deg must equal negative lift at -8 deg");

    // 3. Roll rate capability derivative Clp and Clda
    TEST_ASSERT(std::abs(config.aero.Clda) >= 0.28, "Extra 330 must have high roll authority Clda >= 0.28");

    std::cout << "[PASS] TestAerobaticSymmetricAirfoil\n";
    return true;
}

// 5. Validates regional AirportDatabase and plateau elevation blending
bool TestAirportDatabaseAndElevationBlending() {
    std::cout << "[RUNNING] TestAirportDatabaseAndElevationBlending...\n";

    AirportDatabase db;
    const auto& airports = db.GetAllAirports();
    TEST_ASSERT(airports.size() >= 3, "Database must contain at least 3 regional airports");

    // Validate KLOK (LokYS Regional Airport)
    auto klokOpt = db.FindAirport("KLOK");
    TEST_ASSERT(klokOpt.has_value(), "KLOK airport must exist");
    const auto& klok = *klokOpt;
    TEST_NEAR(klok.elevationM, 0.0, 0.1, "KLOK elevation 0.0 m");
    TEST_NEAR(klok.locationNed.x, 0.0, 0.1, "KLOK X=0");
    TEST_NEAR(klok.locationNed.y, 0.0, 0.1, "KLOK Y=0");
    TEST_ASSERT(!klok.runways.empty() && klok.runways[0].hasIls, "KLOK has ILS RWY 36");

    // Validate KMES (Mesa Highlands Mountain Airport)
    auto kmesOpt = db.FindAirport("KMES");
    TEST_ASSERT(kmesOpt.has_value(), "KMES airport must exist");
    const auto& kmes = *kmesOpt;
    TEST_NEAR(kmes.elevationM, 350.0, 0.1, "KMES elevation +350.0 m");
    TEST_NEAR(kmes.locationNed.x, 14000.0, 0.1, "KMES X=14000 (North)");
    TEST_NEAR(kmes.locationNed.y, 8000.0, 0.1, "KMES Y=8000 (East)");
    TEST_ASSERT(!kmes.runways.empty() && !kmes.runways[0].hasIls, "KMES is non-ILS visual mountain field");

    // Validate KVAL (Valley Ranch Grass Airstrip)
    auto kvalOpt = db.FindAirport("KVAL");
    TEST_ASSERT(kvalOpt.has_value(), "KVAL airport must exist");
    const auto& kval = *kvalOpt;
    TEST_NEAR(kval.elevationM, 60.0, 0.1, "KVAL elevation +60.0 m");
    TEST_NEAR(kval.locationNed.x, -10000.0, 0.1, "KVAL X=-10000 (South)");
    TEST_NEAR(kval.locationNed.y, -6000.0, 0.1, "KVAL Y=-6000 (West)");

    // Test plateau elevation offset blending (using NED coordinates):
    // At KMES center (14000, 8000), elevation offset must be exactly +350m
    f64 elevMesaCenter = db.SampleAirportElevationOffset(14000.0, 8000.0);
    TEST_NEAR(elevMesaCenter, 350.0, 0.5, "Elevation offset at KMES center must be +350m");

    // 5 km away from KMES (14000, 14000), elevation offset should be blended down to 0
    f64 elevFar = db.SampleAirportElevationOffset(14000.0, 14000.0);
    TEST_NEAR(elevFar, 0.0, 1.0, "Elevation offset far from KMES must decay to 0m");

    // At KVAL center (-10000, -6000), elevation offset must be +60m
    f64 elevValCenter = db.SampleAirportElevationOffset(-10000.0, -6000.0);
    TEST_NEAR(elevValCenter, 60.0, 0.5, "Elevation offset at KVAL center must be +60m");

    std::cout << "[PASS] TestAirportDatabaseAndElevationBlending\n";
    return true;
}

// 6. Validates Approach Lighting System (ALSF-1) 720m array and sequenced strobe wave
bool TestApproachLightingSystemTimings() {
    std::cout << "[RUNNING] TestApproachLightingSystemTimings...\n";

    ApproachLightingSystem als;

    // Station count verification: 24 stations spaced 30m apart (0 to 720m)
    TEST_ASSERT(als.GetStationCount() == 24, "ALSF-1 must have 24 stations");

    // Verify sequenced strobe units: 21 capacitor discharge units
    TEST_ASSERT(als.GetStrobeCount() == 21, "ALSF-1 sequenced strobe array must have 21 units");

    // Test "The Rabbit" traveling flash sequence:
    // Total sequence period is 1.0s. At time t = 0.01s, strobe fires.
    als.Update(0.01f);
    int activeIndexStart = als.GetActiveStrobeIndex();
    TEST_ASSERT(activeIndexStart >= 0 && activeIndexStart < 21, "Active strobe index must be within range [0, 20]");
    TEST_ASSERT(als.IsReilFlashing(), "REIL should flash at start of second");

    // Step forward 0.5 seconds
    als.Update(0.50f);
    int activeIndexMid = als.GetActiveStrobeIndex();
    TEST_ASSERT(activeIndexMid != activeIndexStart, "Strobe wave must advance across time");
    TEST_ASSERT(!als.IsReilFlashing(), "REIL should not be flashing at mid-cycle");

    std::cout << "[PASS] TestApproachLightingSystemTimings\n";
    return true;
}

// 7. Validates TerrainStreamingSystem 64-tile grid and LOD distance management
bool TestTerrainStreamingTileLODs() {
    std::cout << "[RUNNING] TestTerrainStreamingTileLODs...\n";

    TerrainStreamingSystem terrain;
    TEST_ASSERT(terrain.GetTotalTileCount() == 64, "Regional terrain must have 64 tiles (8x8 grid)");
    TEST_NEAR(terrain.GetWorldSizeMeters(), 64000.0, 1.0, "World extent must be 64 km (64,000m)");
    TEST_NEAR(terrain.GetTileSizeMeters(), 8000.0, 1.0, "Tile size must be 8 km (8,000m)");

    // Sample terrain elevation at KLOK runway (0, 0): runway plateau must be exactly 0.0m
    f32 elevKlok = terrain.SampleElevation(0.0f, 0.0f);
    TEST_NEAR(elevKlok, 0.0f, 0.1f, "KLOK runway center elevation must be 0.0m");

    // Sample terrain elevation at KMES (Z_gfx=14000, X_gfx=-8000): elevated mesa plateau must be ~350m
    f32 elevKmes = terrain.SampleElevation(-8000.0f, 14000.0f);
    TEST_NEAR(elevKmes, 350.0f, 2.0f, "KMES elevated mesa plateau must be ~350m");

    // Camera at KLOK: tiles closest to (0, 0) should be LOD 0, distant tiles LOD 1
    terrain.UpdateLODs(0.0f, 0.0f);
    u32 lod0Count = terrain.GetLOD0Count();
    u32 lod1Count = terrain.GetLOD1Count();
    TEST_ASSERT(lod0Count > 0, "Near tiles must select high-detail LOD 0");
    TEST_ASSERT(lod1Count > 0, "Distant tiles must select performance LOD 1");
    TEST_ASSERT(lod0Count + lod1Count == 64, "Total active tiles must equal 64");

    std::cout << "[PASS] TestTerrainStreamingTileLODs\n";
    return true;
}

} // namespace

int main() {
    std::cout << "========================================\n";
    std::cout << " LokYS Flight Simulator - Milestone M22\n";
    std::cout << " Multi-Aircraft & Scenery Unit Tests\n";
    std::cout << "========================================\n";

    int passed = 0;
    int total = 7;

    if (TestAircraftRegistryConfigurations()) ++passed;
    if (TestMultiEngineAsymmetricThrust()) ++passed;
    if (TestRetractableGearAerodynamics()) ++passed;
    if (TestAerobaticSymmetricAirfoil()) ++passed;
    if (TestAirportDatabaseAndElevationBlending()) ++passed;
    if (TestApproachLightingSystemTimings()) ++passed;
    if (TestTerrainStreamingTileLODs()) ++passed;

    std::cout << "========================================\n";
    std::cout << " Test Results: " << passed << " / " << total << " suites passed.\n";
    std::cout << "========================================\n";

    return (passed == total) ? 0 : 1;
}
