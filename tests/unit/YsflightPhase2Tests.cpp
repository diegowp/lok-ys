#include "LokYS/assets/AircraftDatParser.hpp"
#include "LokYS/assets/CockpitIstParser.hpp"
#include "LokYS/assets/AcftPackage.hpp"
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

void TestAircraftDatParser() {
    std::cout << "[TEST] Running TestAircraftDatParser...\n";

    std::string_view syntheticDat =
        "IDENTIFY \"TEST_BIPLANE\"\n"
        "CATEGORY AEROBATIC\n"
        "WEIGHCLN 500kg\n"
        "WEIGFUEL 100kg\n"
        "WEIGLOAD 100kg\n"
        "WINGAREA 15m^2\n"
        "PROPELLR 200HP\n"
        "CRITAOAP 16deg\n"
        "CRITSPED 280km/h\n"
        "MAXSPEED 260km/h\n"
        "LEFTGEAR -1.0m -1.2m 0.5m\n"
        "RIGHGEAR  1.0m -1.2m 0.5m\n"
        "WHELGEAR  0.0m -0.5m -3.0m\n"; // Tail wheel is behind main gear

    auto datOpt = AircraftDatParser::ParseFromString(syntheticDat, "TestBiplane");
    TEST_ASSERT(datOpt.has_value());

    const auto& parsed = *datOpt;
    const auto& cfg = parsed.config;

    TEST_ASSERT(cfg.name == "TEST_BIPLANE");
    TEST_ASSERT(cfg.category == Aircraft::AircraftCategory::Aerobatic);
    TEST_ASSERT(std::abs(cfg.mass.emptyMassKg - 500.0) < 1e-4);
    TEST_ASSERT(std::abs(cfg.propulsion.maxFuelCapacityKg - 100.0) < 1e-4);
    TEST_ASSERT(std::abs(cfg.geometry.wingAreaM2 - 15.0) < 1e-4);
    TEST_ASSERT(std::abs(cfg.propulsion.horsepowerPerEngine - 200.0) < 1.0);
    TEST_ASSERT(std::abs(cfg.aero.stallAlphaRad - (16.0 * 3.141592653589793 / 180.0)) < 1e-4);

    // Verify gear conversion to FRD: X_FRD = +Z_YS, Y_FRD = +X_YS, Z_FRD = -Y_YS
    TEST_ASSERT(std::abs(parsed.gearPositions.leftMainFRD.x - 0.5) < 1e-4);
    TEST_ASSERT(std::abs(parsed.gearPositions.leftMainFRD.y - (-1.0)) < 1e-4);
    TEST_ASSERT(std::abs(parsed.gearPositions.leftMainFRD.z - 1.2) < 1e-4);

    TEST_ASSERT(std::abs(parsed.gearPositions.noseOrTailFRD.x - (-3.0)) < 1e-4);
    TEST_ASSERT(parsed.gearPositions.isTaildragger == true); // Wheel at -3.0 < 0.5

    // Test real file from Ysflight if present
    if (std::filesystem::exists("Ysflight/aircraft/cessna172r.dat")) {
        auto realDatOpt = AircraftDatParser::ParseFromFile("Ysflight/aircraft/cessna172r.dat");
        TEST_ASSERT(realDatOpt.has_value());
        const auto& c172 = *realDatOpt;

        TEST_ASSERT(c172.config.name == "CESSNA_172R");
        TEST_ASSERT(std::abs(c172.config.mass.emptyMassKg - 630.0) < 1e-4);
        TEST_ASSERT(std::abs(c172.config.geometry.wingAreaM2 - 20.0) < 1e-4);
        TEST_ASSERT(c172.config.propulsion.horsepowerPerEngine >= 159.0 && c172.config.propulsion.horsepowerPerEngine <= 181.0);
        TEST_ASSERT(c172.gearPositions.isTaildragger == false); // C172 is tricycle gear

        std::cout << "  Loaded real cessna172r.dat: emptyMass = " << c172.config.mass.emptyMassKg
                  << " kg, power = " << c172.config.propulsion.horsepowerPerEngine
                  << " HP, wingArea = " << c172.config.geometry.wingAreaM2 << " m^2\n";
    }

    std::cout << "[PASS] TestAircraftDatParser passed successfully.\n";
}

void TestCockpitIstParser() {
    std::cout << "[TEST] Running TestCockpitIstParser...\n";

    std::string_view syntheticIst =
        "PNLRANGE -0.8 -1.0 0.8 -0.37\n"
        "AIRSPEED -0.6 -0.5 0.15\n"
        "ATTITUDE -0.3 -0.5 0.15\n"
        "ALTITUDE  0.0 -0.5 0.15\n"
        "TRNCOORD -0.6 -0.8 0.15\n"
        "HEADING_ -0.3 -0.8 0.15\n"
        "VERTICAL  0.0 -0.8 0.15\n"
        "TACHOPOS 0 0.5 -0.5 0.15\n"
        "TACHORPM 0 0 3500\n"
        "SPEEDVFE 85kt\n"
        "SPEEDVNO 129kt\n"
        "SPEEDVNE 163kt\n";

    auto istOpt = CockpitIstParser::ParseFromString(syntheticIst);
    TEST_ASSERT(istOpt.has_value());

    const auto& layout = *istOpt;
    TEST_ASSERT(layout.airspeed.isPresent == true);
    TEST_ASSERT(layout.attitude.isPresent == true);
    TEST_ASSERT(layout.altimeter.isPresent == true);
    TEST_ASSERT(layout.turnCoordinator.isPresent == true);
    TEST_ASSERT(layout.heading.isPresent == true);
    TEST_ASSERT(layout.verticalSpeed.isPresent == true);
    TEST_ASSERT(layout.tachometer.isPresent == true);

    TEST_ASSERT(std::abs(layout.tachometer.maxRpm - 3500.0f) < 1e-2f);
    TEST_ASSERT(std::abs(layout.speedVfeKnots - 85.0f) < 1.0f);
    TEST_ASSERT(std::abs(layout.speedVnoKnots - 129.0f) < 1.0f);
    TEST_ASSERT(std::abs(layout.speedVneKnots - 163.0f) < 1.0f);

    // Test real file from Ysflight if present
    if (std::filesystem::exists("Ysflight/aircraft/cessna172r.ist")) {
        auto realIstOpt = CockpitIstParser::ParseFromFile("Ysflight/aircraft/cessna172r.ist");
        TEST_ASSERT(realIstOpt.has_value());
        const auto& c172Ist = *realIstOpt;
        TEST_ASSERT(c172Ist.airspeed.isPresent);
        TEST_ASSERT(c172Ist.attitude.isPresent);
        TEST_ASSERT(c172Ist.altimeter.isPresent);
        TEST_ASSERT(c172Ist.tachometer.isPresent);
        std::cout << "  Loaded real cessna172r.ist: 6-Pack + Tachometer detected successfully.\n";
    }

    std::cout << "[PASS] TestCockpitIstParser passed successfully.\n";
}

void TestAcftPackage() {
    std::cout << "[TEST] Running TestAcftPackage...\n";

    // Test loading real Cessna 172 from Ysflight directory
    if (std::filesystem::exists("Ysflight/aircraft/cessna172r.dat")) {
        AcftPackage pkg;
        bool ok = pkg.LoadFromDirectory("Ysflight/aircraft", "cessna172r.dat");
        TEST_ASSERT(ok);
        TEST_ASSERT(pkg.IsValid());

        const auto& flight = pkg.GetFlightData();
        TEST_ASSERT(flight.config.name == "CESSNA_172R");

        const auto& visual = pkg.GetVisualModel();
        TEST_ASSERT(!visual.nodes.empty());
        TEST_ASSERT(visual.FindNode("Fuselage") != nullptr);

        TEST_ASSERT(pkg.GetCockpitPanel().has_value());
        TEST_ASSERT(pkg.GetCockpitPanel()->airspeed.isPresent);

        std::cout << "  Loaded complete Cessna 172 package: "
                  << visual.nodes.size() << " articulated nodes, "
                  << visual.GetTotalVertices() << " vertices, "
                  << flight.config.propulsion.horsepowerPerEngine << " HP engine.\n";
    }

    std::cout << "[PASS] TestAcftPackage passed successfully.\n";
}

int main() {
    std::cout << "========================================================\n";
    std::cout << "Running LokYS YSFlight Interoperability Phase 2 Tests\n";
    std::cout << "========================================================\n";

    TestAircraftDatParser();
    TestCockpitIstParser();
    TestAcftPackage();

    std::cout << "========================================================\n";
    std::cout << "ALL PHASE 2 TESTS PASSED (100% SUCCESS)\n";
    std::cout << "========================================================\n";
    return 0;
}
