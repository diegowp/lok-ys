#include "LokYS/core/Types.hpp"
#include "LokYS/math/Vector3.hpp"
#include "LokYS/math/Matrix3x3.hpp"
#include "LokYS/math/Matrix4x4.hpp"
#include "LokYS/math/Quaternion.hpp"
#include "LokYS/math/Units.hpp"
#include "LokYS/physics/RigidBody.hpp"
#include "LokYS/physics/GroundReactions.hpp"
#include "LokYS/aerodynamics/AerodynamicsSystem.hpp"
#include "LokYS/propulsion/PropulsionSystem.hpp"
#include "LokYS/assets/AircraftDatParser.hpp"
#include "LokYS/assets/DnmParser.hpp"
#include "LokYS/assets/SrfParser.hpp"
#include "LokYS/assets/CockpitIstParser.hpp"
#include "LokYS/assets/AcftPackage.hpp"
#include "LokYS/assets/SceneryFldParser.hpp"
#include "LokYS/assets/SpawnStpParser.hpp"
#include "LokYS/assets/LfsPackage.hpp"
#include "LokYS/tools/AircraftEditor.hpp"
#include "LokYS/tools/SceneryEditor.hpp"
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <chrono>
#include <cmath>
#include <cstdlib>

#define TEST_ASSERT(expr) \
    do { \
        if (!(expr)) { \
            std::cerr << "Assertion failed at " << __FILE__ << ":" << __LINE__ \
                      << " - " << #expr << std::endl; \
            std::exit(1); \
        } \
    } while (0)

using namespace LokYS;

namespace {

std::string FindAssetPath(const std::string& relativePath) {
    // Check current directory
    if (std::ifstream(relativePath).good()) return relativePath;
    // Check parent directory
    std::string parentPath = "../" + relativePath;
    if (std::ifstream(parentPath).good()) return parentPath;
    return relativePath;
}

} // namespace

void TestRealAircraftIngestion() {
    std::cout << "[TEST] Running TestRealAircraftIngestion (P5.1)..." << std::endl;

    struct AircraftTestCase {
        std::string baseName;
        f64 minMassKg;
        f64 maxMassKg;
        f64 minWingspanM;
    };

    std::vector<AircraftTestCase> testCases = {
        { "cessna172r", 500.0, 1500.0, 9.0 },
        { "f16", 4000.0, 15000.0, 8.0 },
        { "b747", 100000.0, 450000.0, 50.0 },
        { "ys11", 10000.0, 30000.0, 20.0 }
    };

    for (const auto& tc : testCases) {
        std::string datPath = FindAssetPath("Ysflight/aircraft/" + tc.baseName + ".dat");
        std::string dnmPath = FindAssetPath("Ysflight/aircraft/" + tc.baseName + ".dnm");
        std::string istPath = FindAssetPath("Ysflight/aircraft/" + tc.baseName + ".ist");

        // 1. Verify DAT parser
        auto parsedDat = Assets::AircraftDatParser::ParseFromFile(datPath);
        TEST_ASSERT(parsedDat.has_value());
        const auto& cfg = parsedDat->config;
        TEST_ASSERT(!cfg.name.empty());
        TEST_ASSERT(cfg.mass.emptyMassKg >= tc.minMassKg && cfg.mass.emptyMassKg <= tc.maxMassKg);
        TEST_ASSERT(cfg.geometry.wingspanM >= tc.minWingspanM);
        TEST_ASSERT(cfg.geometry.wingAreaM2 > 5.0);

        // 2. Verify DNM parser
        auto parsedDnm = Assets::DnmParser::ParseFromFile(dnmPath);
        TEST_ASSERT(parsedDnm.has_value());
        TEST_ASSERT(!parsedDnm->nodes.empty());
        TEST_ASSERT(parsedDnm->GetTotalVertices() > 50);
        TEST_ASSERT(parsedDnm->GetTotalIndices() > 50);
        TEST_ASSERT(parsedDnm->aabbMin.x < parsedDnm->aabbMax.x ||
                    parsedDnm->aabbMin.y < parsedDnm->aabbMax.y ||
                    parsedDnm->aabbMin.z < parsedDnm->aabbMax.z);

        // 3. Verify Cockpit IST parser
        auto parsedIst = Assets::CockpitIstParser::ParseFromFile(istPath);
        if (parsedIst.has_value()) {
            TEST_ASSERT(parsedIst->airspeed.isPresent || parsedIst->attitude.isPresent || parsedIst->altimeter.isPresent);
        }

        std::cout << "  [INGESTION OK] Aircraft: " << cfg.name 
                  << " | Mass: " << cfg.mass.emptyMassKg << " kg"
                  << " | Span: " << cfg.geometry.wingspanM << " m"
                  << " | DNM Nodes: " << parsedDnm->nodes.size()
                  << " | Verts: " << parsedDnm->GetTotalVertices() << std::endl;
    }

    // 4. Mathematical Coordinate Transform Identity Check
    // YS Body (+X Right, +Y Up, +Z Nose) -> FRD (+X Nose, +Y Right, +Z Down)
    // R = [ 0 0 1; 1 0 0; 0 -1 0 ]
    Math::Vec3d vYS{ 12.34, 56.78, 90.12 };
    Math::Vec3d vFRD{ vYS.z, vYS.x, -vYS.y };

    // Invariant: Vector lengths must be mathematically identical
    f64 lenYS = vYS.Length();
    f64 lenFRD = vFRD.Length();
    TEST_ASSERT(std::abs(lenYS - lenFRD) < 1e-12);

    // Matrix orthogonality: R * R^T = Identity
    Math::Mat3d R_ys_to_frd(
        0.0, 0.0, 1.0,
        1.0, 0.0, 0.0,
        0.0, -1.0, 0.0
    );
    Math::Mat3d R_transpose = R_ys_to_frd.Transposed();
    Math::Mat3d R_mult = R_ys_to_frd * R_transpose;
    Math::Mat3d identity = Math::Mat3d::Identity();

    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            TEST_ASSERT(std::abs(R_mult.m[r][c] - identity.m[r][c]) < 1e-12);
        }
    }

    std::cout << "[PASS] TestRealAircraftIngestion passed successfully." << std::endl;
}

void TestRealSceneryIngestion() {
    std::cout << "[TEST] Running TestRealSceneryIngestion (P5.1)..." << std::endl;

    std::vector<std::string> sceneryNames = {
        "small", "hawaii", "aomori", "heathrow"
    };

    for (const auto& name : sceneryNames) {
        std::string fldPath = FindAssetPath("Ysflight/scenery/" + name + ".fld");
        std::string stpPath = FindAssetPath("Ysflight/scenery/" + name + ".stp");

        auto parsedFld = Assets::SceneryFldParser::ParseFromFile(fldPath);
        TEST_ASSERT(parsedFld.has_value());
        TEST_ASSERT(!parsedFld->name.empty());

        auto parsedStp = Assets::SpawnStpParser::ParseFromFile(stpPath);
        TEST_ASSERT(parsedStp.has_value());
        TEST_ASSERT(!parsedStp->points.empty());

        // Verify terrain / runways
        auto airport = parsedFld->ToAirportConfig("Y" + name, parsedFld->name);
        TEST_ASSERT(!airport.runways.empty());

        std::cout << "  [INGESTION OK] Scenery: " << parsedFld->name
                  << " | TerrBlocks: " << parsedFld->terrainBlocks.size()
                  << " | Runways: " << parsedFld->runways.size()
                  << " | Spawns: " << parsedStp->points.size() << std::endl;
    }

    std::cout << "[PASS] TestRealSceneryIngestion passed successfully." << std::endl;
}

void TestHeadlessFlightSimulationBenchmark() {
    std::cout << "[TEST] Running TestHeadlessFlightSimulationBenchmark (P5.2)..." << std::endl;

    // 1. Ingest real Cessna 172R and Small scenery
    std::string datPath = FindAssetPath("Ysflight/aircraft/cessna172r.dat");
    std::string fldPath = FindAssetPath("Ysflight/scenery/small.fld");
    std::string stpPath = FindAssetPath("Ysflight/scenery/small.stp");

    auto parsedDat = Assets::AircraftDatParser::ParseFromFile(datPath);
    TEST_ASSERT(parsedDat.has_value());
    auto parsedFld = Assets::SceneryFldParser::ParseFromFile(fldPath);
    TEST_ASSERT(parsedFld.has_value());
    auto parsedStp = Assets::SpawnStpParser::ParseFromFile(stpPath);
    TEST_ASSERT(parsedStp.has_value());

    // 2. Select first runway spawn point
    auto runwaySpawns = parsedStp->GetRunwaySpawns();
    TEST_ASSERT(!runwaySpawns.empty());
    const auto& spawn = runwaySpawns[0];

    // 3. Setup Physics RigidBody
    const auto& acftConfig = parsedDat->config;
    Physics::RigidBody body(acftConfig.mass.emptyMassKg, acftConfig.mass.inertiaTensor);
    body.SetGravityEnabled(true);

    // Initial State: Stationary spawn
    Physics::RigidBodyState state{};
    state.positionNed = spawn.positionNed;
    state.velocityBody = Math::Vec3d{ 0.0, 0.0, 0.0 };
    state.angularVelocityBody = Math::Vec3d{ 0.0, 0.0, 0.0 };
    f64 headingRad = spawn.headingDeg * (Math::Constants::PI / 180.0);
    state.attitude = Math::Quat4d::FromEuler(headingRad, 0.0, 0.0);
    body.SetState(state);

    // 4. Setup Ground Reactions & Aerodynamics & Propulsion
    Physics::GroundReactionsSystem groundReactions;
    Aerodynamics::AerodynamicsSystem aero;
    aero.SetConfig(acftConfig.geometry, acftConfig.aero, true);

    Propulsion::PropulsionSystem propulsion;
    propulsion.SetAircraftEngineConfig(acftConfig.propulsion);
    propulsion.SetRpm(acftConfig.propulsion.idleRpm);

    // 5. Run 240-step simulation benchmark (120 Hz, total 2.0 seconds)
    constexpr f64 dt = 1.0 / 120.0;
    constexpr u32 totalTicks = 240;

    auto tStart = std::chrono::high_resolution_clock::now();

    for (u32 tick = 0; tick < totalTicks; ++tick) {
        Physics::RigidBodyState curState = body.GetState();

        // Control schedule:
        // Steps 0-119: Stationary spawn with parking brake active (throttle 0, brake 1)
        // Steps 120-239: Brake released, full throttle acceleration (throttle 1, brake 0)
        f64 throttle = (tick < 120) ? 0.0 : 1.0;
        f64 brake = (tick < 120) ? 1.0 : 0.0;

        if (tick == 120) {
            propulsion.SetRpm(acftConfig.propulsion.maxRatedRpm);
        }

        // Evaluate Ground Reactions
        f32 terrainElev = parsedFld->SampleElevationAt(curState.positionNed.x, curState.positionNed.y);
        f64 terrainNedZ = -static_cast<f64>(terrainElev);
        auto grOutput = groundReactions.Evaluate(curState, 0.0, brake, dt, terrainNedZ);

        // Evaluate Aerodynamics
        Aerodynamics::ControlDeflections controls{};
        Math::Vec3d relWind = -curState.velocityBody;
        auto aeroOutput = aero.Evaluate(relWind, curState.angularVelocityBody, controls);

        // Evaluate Propulsion
        Propulsion::PropulsionControls propControls{};
        propControls.throttle = throttle;
        auto propOutput = propulsion.Evaluate(
            propControls,
            curState.velocityBody,
            curState.angularVelocityBody,
            Math::Constants::SEA_LEVEL_DENSITY,
            Math::Constants::STANDARD_SEA_LEVEL_PRESSURE,
            dt);

        // State-dependent RK4 force summation
        body.ClearForcesAndMoments();
        body.AddForceBody(grOutput.totalForceBody);
        body.AddMomentBody(grOutput.totalMomentBody);
        body.AddForceBody(aeroOutput.forcesBody);
        body.AddMomentBody(aeroOutput.momentsBody);
        body.AddForceBody(propOutput.thrustForceBody);
        body.AddMomentBody(propOutput.momentsBody);

        body.IntegrateRK4(dt);

        const auto& nextState = body.GetState();

        // Strict Numerical Health Invariant: Zero NaN / Inf
        TEST_ASSERT(!std::isnan(nextState.positionNed.x) && !std::isinf(nextState.positionNed.x));
        TEST_ASSERT(!std::isnan(nextState.positionNed.y) && !std::isinf(nextState.positionNed.y));
        TEST_ASSERT(!std::isnan(nextState.positionNed.z) && !std::isinf(nextState.positionNed.z));
        TEST_ASSERT(!std::isnan(nextState.velocityBody.x) && !std::isinf(nextState.velocityBody.x));
        TEST_ASSERT(!std::isnan(nextState.velocityBody.y) && !std::isinf(nextState.velocityBody.y));
        TEST_ASSERT(!std::isnan(nextState.velocityBody.z) && !std::isinf(nextState.velocityBody.z));
        TEST_ASSERT(!std::isnan(nextState.attitude.w) && !std::isinf(nextState.attitude.w));
        TEST_ASSERT(!std::isnan(nextState.angularVelocityBody.x) && !std::isinf(nextState.angularVelocityBody.x));

        // Invariant: At tick 119 (end of parking brake test), aircraft must remain stationary
        if (tick == 119) {
            TEST_ASSERT(std::abs(nextState.velocityBody.x) < 0.1);
            TEST_ASSERT(std::abs(nextState.velocityBody.y) < 0.1);
        }
    }

    auto tEnd = std::chrono::high_resolution_clock::now();
    f64 totalDurationMs = std::chrono::duration<f64, std::milli>(tEnd - tStart).count();
    f64 avgTickUs = (totalDurationMs / totalTicks) * 1000.0;

    const auto& finalState = body.GetState();
    // After brake release and 120 ticks of 100% throttle, aircraft must accelerate forward
    TEST_ASSERT(finalState.velocityBody.x > 2.0);

    // Performance budget check: latency must be strictly < 1.0 ms (1000 us) per tick
    TEST_ASSERT(avgTickUs < 1000.0);

    std::cout << "  Benchmark Completed: 240 ticks in " << totalDurationMs << " ms ("
              << avgTickUs << " us/tick, budget < 1000 us/tick)." << std::endl;
    std::cout << "  Final speed: " << (finalState.velocityBody.x * 1.943844) << " kts (takeoff acceleration verified)." << std::endl;
    std::cout << "[PASS] TestHeadlessFlightSimulationBenchmark passed successfully." << std::endl;
}

int main() {
    std::cout << "========================================================" << std::endl;
    std::cout << "Running LokYS YSFlight Interoperability Phase 5 QA & Benchmark" << std::endl;
    std::cout << "========================================================" << std::endl;

    TestRealAircraftIngestion();
    TestRealSceneryIngestion();
    TestHeadlessFlightSimulationBenchmark();

    std::cout << "========================================================" << std::endl;
    std::cout << "ALL PHASE 5 TESTS & BENCHMARKS PASSED (100% SUCCESS)" << std::endl;
    std::cout << "========================================================" << std::endl;

    return 0;
}
