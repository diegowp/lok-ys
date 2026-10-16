#include "LokYS/assets/SpawnStpParser.hpp"
#include "LokYS/assets/SceneryFldParser.hpp"
#include "LokYS/assets/LfsPackage.hpp"
#include <iostream>
#include <cstdlib>
#include <cmath>
#include <filesystem>

#define TEST_ASSERT(expr) do { \
    if (!(expr)) { \
        std::cerr << "[ASSERTION FAILED] " #expr " at " __FILE__ ":" << __LINE__ << std::endl; \
        std::exit(1); \
    } \
} while(0)

using namespace LokYS;
using namespace LokYS::Assets;

void TestSpawnStpParser() {
    std::cout << "[TEST] Running TestSpawnStpParser...\n";

    std::string_view syntheticStp =
        "N RW36_01\n"
        "C POSITION 1000m 0m 2000m\n"
        "C ATTITUDE 0deg 0deg 0deg\n"
        "C INITSPED 0.0MACH\n"
        "C CTLTHROT 0.0\n"
        "N AIR_10K\n"
        "C POSITION 500m 3048m 1500m\n"
        "C ATTITUDE 180deg 0deg 0deg\n"
        "C INITSPED 150kt\n"
        "C CTLTHROT 0.8\n"
        "C CTLLDGEA FALSE\n";

    auto listOpt = SpawnStpParser::ParseFromString(syntheticStp);
    TEST_ASSERT(listOpt.has_value());
    TEST_ASSERT(listOpt->points.size() == 2);

    const auto* rw = listOpt->FindPoint("RW36_01");
    TEST_ASSERT(rw != nullptr);
    TEST_ASSERT(!rw->isAirborne);
    // YS -> NED: X_NED = +Z_YS (2000), Y_NED = +X_YS (1000), Z_NED = -Y_YS (0)
    TEST_ASSERT(std::abs(rw->positionNed.x - 2000.0) < 1e-4);
    TEST_ASSERT(std::abs(rw->positionNed.y - 1000.0) < 1e-4);
    TEST_ASSERT(std::abs(rw->positionNed.z - 0.0) < 1e-4);
    TEST_ASSERT(std::abs(rw->headingDeg - 0.0) < 1e-4);

    const auto* air = listOpt->FindPoint("AIR_10K");
    TEST_ASSERT(air != nullptr);
    TEST_ASSERT(air->isAirborne);
    TEST_ASSERT(std::abs(air->altitudeMslM - 3048.0) < 1e-4);
    TEST_ASSERT(std::abs(air->headingDeg - 180.0) < 1e-4);
    TEST_ASSERT(!air->gearDown);

    // Test real file from Ysflight if present
    if (std::filesystem::exists("Ysflight/scenery/small.stp")) {
        auto realStpOpt = SpawnStpParser::ParseFromFile("Ysflight/scenery/small.stp");
        TEST_ASSERT(realStpOpt.has_value());
        TEST_ASSERT(!realStpOpt->points.empty());

        auto rwSpawns = realStpOpt->GetRunwaySpawns();
        auto airSpawns = realStpOpt->GetAirSpawns();

        TEST_ASSERT(!rwSpawns.empty());
        TEST_ASSERT(!airSpawns.empty());

        std::cout << "  Loaded real small.stp: " << realStpOpt->points.size()
                  << " total spawn points (" << rwSpawns.size() << " runway, "
                  << airSpawns.size() << " airborne).\n";
    }

    std::cout << "[PASS] TestSpawnStpParser passed successfully.\n";
}

void TestSceneryFldParser() {
    std::cout << "[TEST] Running TestSceneryFldParser...\n";

    std::string_view syntheticFld =
        "FIELD\n"
        "GND 10 20 30\n"
        "SKY 100 150 200\n"
        "PCK \"test.ter\" 10\n"
        "TerrMesh\n"
        "NBL 2 2\n"
        "TMS 100.0 100.0\n"
        "BLO 0.0\n"
        "BLO 50.0\n"
        "BLO 100.0\n"
        "BLO 150.0\n"
        "END\n"
        "TER\n"
        "FIL \"test.ter\"\n"
        "POS 0 0 0 0 0 0\n"
        "END\n"
        "RGN\n"
        "ARE -25.0 -1000.0 25.0 1000.0\n"
        "POS 500 0 1000 0\n"
        "TAG \"RW\"\n"
        "END\n";

    auto fldOpt = SceneryFldParser::ParseFromString(syntheticFld, "TestField");
    TEST_ASSERT(fldOpt.has_value());

    const auto& fld = *fldOpt;
    TEST_ASSERT(std::abs(fld.gndColor.x - (10.0f / 255.0f)) < 1e-3f);
    TEST_ASSERT(std::abs(fld.skyColor.x - (100.0f / 255.0f)) < 1e-3f);
    TEST_ASSERT(fld.terrainBlocks.size() == 1);
    TEST_ASSERT(fld.runways.size() == 1);

    const auto& rnw = fld.runways[0];
    TEST_ASSERT(std::abs(rnw.widthM - 50.0) < 1e-4);
    TEST_ASSERT(std::abs(rnw.lengthM - 2000.0) < 1e-4);
    // YS -> NED: centerNed.x = +Z_YS (1000), centerNed.y = +X_YS (500)
    TEST_ASSERT(std::abs(rnw.centerNed.x - 1000.0) < 1e-4);
    TEST_ASSERT(std::abs(rnw.centerNed.y - 500.0) < 1e-4);

    // Test real file from Ysflight if present
    if (std::filesystem::exists("Ysflight/scenery/small.fld")) {
        auto realFldOpt = SceneryFldParser::ParseFromFile("Ysflight/scenery/small.fld");
        TEST_ASSERT(realFldOpt.has_value());
        const auto& smallFld = *realFldOpt;

        TEST_ASSERT(!smallFld.terrainBlocks.empty());
        TEST_ASSERT(!smallFld.runways.empty());

        auto apt = smallFld.ToAirportConfig("YSM", "Small Regional");
        TEST_ASSERT(apt.icao == "YSM");
        TEST_ASSERT(!apt.runways.empty());

        std::cout << "  Loaded real small.fld: " << smallFld.terrainBlocks.size()
                  << " terrain elevation blocks, " << smallFld.runways.size()
                  << " runways, " << smallFld.placedObjects.size() << " placed objects.\n";
    }

    std::cout << "[PASS] TestSceneryFldParser passed successfully.\n";
}

void TestLfsPackage() {
    std::cout << "[TEST] Running TestLfsPackage...\n";

    // Test loading real Small Scenery from Ysflight folder
    if (std::filesystem::exists("Ysflight/scenery/small.fld") &&
        std::filesystem::exists("Ysflight/scenery/small.stp")) {
        LfsPackage pkg;
        bool ok = pkg.LoadFromDirectory("Ysflight/scenery", "small.fld", "small.stp");
        TEST_ASSERT(ok);
        TEST_ASSERT(pkg.IsValid());

        const auto& fld = pkg.GetField();
        TEST_ASSERT(!fld.runways.empty());

        const auto& spawns = pkg.GetSpawns();
        TEST_ASSERT(!spawns.points.empty());

        const auto& apt = pkg.GetAirportConfig();
        TEST_ASSERT(!apt.runways.empty());

        std::cout << "  Loaded complete Small scenery package: "
                  << fld.runways.size() << " runways, "
                  << spawns.points.size() << " spawn points ready for simulation.\n";
    }

    std::cout << "[PASS] TestLfsPackage passed successfully.\n";
}

int main() {
    std::cout << "========================================================\n";
    std::cout << "Running LokYS YSFlight Interoperability Phase 3 Tests\n";
    std::cout << "========================================================\n";

    TestSpawnStpParser();
    TestSceneryFldParser();
    TestLfsPackage();

    std::cout << "========================================================\n";
    std::cout << "ALL PHASE 3 TESTS PASSED (100% SUCCESS)\n";
    std::cout << "========================================================\n";
    return 0;
}
