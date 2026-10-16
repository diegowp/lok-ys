#include "LokYS/vfs/ZipWriter.hpp"
#include "LokYS/vfs/ZipVfsReader.hpp"
#include "LokYS/tools/AircraftEditor.hpp"
#include "LokYS/tools/SceneryEditor.hpp"
#include "LokYS/assets/AcftPackage.hpp"
#include "LokYS/assets/LfsPackage.hpp"
#include <iostream>
#include <cstdlib>
#include <cmath>
#include <cassert>

#define TEST_ASSERT(expr) \
    do { \
        if (!(expr)) { \
            std::cerr << "Assertion failed at " << __FILE__ << ":" << __LINE__ \
                      << " - " << #expr << std::endl; \
            std::exit(1); \
        } \
    } while (0)

using namespace LokYS;

void TestZipWriter() {
    std::cout << "[TEST] Running TestZipWriter..." << std::endl;

    // 1. Verify standard CRC-32 test vector: "123456789" -> 0xCBF43926
    const char* testVector = "123456789";
    std::span<const uint8_t> vecBytes(reinterpret_cast<const uint8_t*>(testVector), 9);
    u32 crc = Vfs::ZipWriter::ComputeCRC32(vecBytes);
    TEST_ASSERT(crc == 0xCBF43926u);

    // 2. Build in-memory ZIP archive
    Vfs::ZipWriter writer;
    writer.AddText("manifest.json", "{\"type\":\"test\"}");
    writer.AddText("data/readme.txt", "Hello LokYS VFS from ZipWriter!");
    writer.AddText("data/config.ini", "speed=120\naltitude=5000\n");

    TEST_ASSERT(writer.GetEntryCount() == 3);
    TEST_ASSERT(writer.HasFile("manifest.json"));
    TEST_ASSERT(writer.HasFile("data/readme.txt"));

    std::vector<uint8_t> zipBuffer = writer.FinalizeToMemory();
    TEST_ASSERT(!zipBuffer.empty());
    TEST_ASSERT(zipBuffer.size() > 100);

    // 3. Round-trip: Read back with ZipVfsReader
    Vfs::ZipVfsReader reader;
    bool loadOk = reader.LoadFromMemory(zipBuffer);
    TEST_ASSERT(loadOk);

    TEST_ASSERT(reader.HasFile("manifest.json"));
    TEST_ASSERT(reader.HasFile("data/readme.txt"));
    TEST_ASSERT(reader.HasFile("data/config.ini"));

    auto manifestText = reader.ReadText("manifest.json");
    TEST_ASSERT(manifestText.has_value());
    TEST_ASSERT(*manifestText == "{\"type\":\"test\"}");

    auto readmeText = reader.ReadText("data/readme.txt");
    TEST_ASSERT(readmeText.has_value());
    TEST_ASSERT(*readmeText == "Hello LokYS VFS from ZipWriter!");

    std::cout << "  ZipWriter created " << zipBuffer.size() << " byte PKZIP archive, decoded successfully." << std::endl;
    std::cout << "[PASS] TestZipWriter passed successfully." << std::endl;
}

void TestAircraftEditor() {
    std::cout << "[TEST] Running TestAircraftEditor..." << std::endl;

    Tools::AircraftEditor editor;

    // 1. Load real YSFlight assets: cessna172r
    bool loaded = editor.LoadFromFiles(
        "Ysflight/aircraft/cessna172r.dat",
        "Ysflight/aircraft/cessna172r.dnm",
        "Ysflight/aircraft/cessna172r.ist"
    );

    if (!loaded) {
        // Fallback to relative test path
        loaded = editor.LoadFromFiles(
            "../Ysflight/aircraft/cessna172r.dat",
            "../Ysflight/aircraft/cessna172r.dnm",
            "../Ysflight/aircraft/cessna172r.ist"
        );
    }
    TEST_ASSERT(loaded);

    // 2. Inspect Hierarchy
    size_t nodeCount = editor.GetNodeCount();
    TEST_ASSERT(nodeCount > 0);
    std::string treeStr = editor.GenerateHierarchyTree();
    TEST_ASSERT(!treeStr.empty());
    TEST_ASSERT(treeStr.find("DNM HIERARCHY TREE") != std::string::npos);

    // 3. Test Animation Debugger
    // Set Flaps (CLA 5) to full deflection (1.0)
    editor.SetAnimationChannel(5, 1.0f);
    TEST_ASSERT(editor.GetAnimationChannel(5) == 1.0f);

    Math::Mat4f node0World = editor.ComputeNodeWorldTransform(0);
    TEST_ASSERT(std::abs(node0World.m[3][3] - 1.0f) < 1e-4f);

    // 4. Test Aerodynamics Polars & Stability
    auto polars = editor.ComputeAeroPolars(-20.0, 25.0, 5.0);
    TEST_ASSERT(!polars.empty());

    bool stable = editor.ValidateLongitudinalStability();
    TEST_ASSERT(stable); // C_ma should be negative for pitch stiffness
    f64 staticMargin = editor.GetStaticMargin();
    TEST_ASSERT(staticMargin > 0.0);

    // 5. Test Mutators
    editor.SetEngineHorsepower(180.0);
    editor.SetEmptyMassKg(765.0);
    editor.SetCruiseSpeedKnots(122.0);

    auto summary = editor.GetSummary();
    TEST_ASSERT(summary.emptyMassKg == 765.0);
    TEST_ASSERT(summary.cruiseSpeedKts == 122.0);
    TEST_ASSERT(summary.isLongitudinallyStable);

    // 6. Test Export to .acft.zip and Round-Trip Validation
    std::vector<uint8_t> acftZip = editor.ExportToAcftPackage();
    TEST_ASSERT(!acftZip.empty());

    Vfs::ZipVfsReader vfs;
    TEST_ASSERT(vfs.LoadFromMemory(acftZip));
    TEST_ASSERT(vfs.HasFile("manifest.json"));
    TEST_ASSERT(vfs.HasFile("aircraft.dat"));
    TEST_ASSERT(vfs.HasFile("visual.dnm"));
    TEST_ASSERT(vfs.HasFile("cockpit.ist"));

    Assets::AcftPackage reloadedPkg;
    TEST_ASSERT(reloadedPkg.LoadFromArchive(vfs));
    TEST_ASSERT(reloadedPkg.IsValid());
    TEST_ASSERT(reloadedPkg.GetFlightData().config.mass.emptyMassKg == 765.0);

    std::cout << "  AircraftEditor loaded " << nodeCount << " DNM nodes, validated stability, exported "
              << acftZip.size() << " bytes .acft.zip package." << std::endl;
    std::cout << "[PASS] TestAircraftEditor passed successfully." << std::endl;
}

void TestSceneryEditor() {
    std::cout << "[TEST] Running TestSceneryEditor..." << std::endl;

    Tools::SceneryEditor editor;

    // 1. Load real YSFlight scenery: small.fld and small.stp
    bool loaded = editor.LoadFromFiles("Ysflight/scenery/small.fld", "Ysflight/scenery/small.stp");
    if (!loaded) {
        loaded = editor.LoadFromFiles("../Ysflight/scenery/small.fld", "../Ysflight/scenery/small.stp");
    }
    TEST_ASSERT(loaded);

    // 2. Test Terrain Elevation Queries & Sculpting
    f32 initialElev = editor.SampleElevationAt(0.0, 0.0);
    editor.SculptElevation(0.0, 0.0, 500.0, 25.0f);
    f32 sculptedElev = editor.SampleElevationAt(0.0, 0.0);
    TEST_ASSERT(sculptedElev >= initialElev);

    // 3. Test Object Placement & Ground Snapping
    size_t initObjCount = editor.GetPlacedObjectCount();
    Assets::PlacedSceneryObject towerObj;
    towerObj.tag = "ATC_TOWER";
    towerObj.positionNed = Math::Vec3d{ 1000.0, 500.0, 0.0 };
    towerObj.headingDeg = 45.0;

    size_t newIdx = editor.AddPlacedObject(towerObj);
    TEST_ASSERT(editor.GetPlacedObjectCount() == initObjCount + 1);

    bool snapOk = editor.SnapObjectToTerrain(newIdx);
    TEST_ASSERT(snapOk);
    const auto* placed = editor.GetPlacedObject(newIdx);
    TEST_ASSERT(placed != nullptr);
    TEST_ASSERT(placed->tag == "ATC_TOWER");

    // 4. Test Spawn Point Management
    size_t initSpawnCount = editor.GetSpawnPointCount();
    Assets::SpawnPoint newSpawn;
    newSpawn.name = "CUSTOM_RUNWAY_SPAWN_01";
    newSpawn.positionNed = Math::Vec3d{ 500.0, 500.0, -10.0 };
    newSpawn.headingDeg = 180.0;
    newSpawn.initialSpeedMps = 0.0;
    newSpawn.initialThrottle = 0.0;
    newSpawn.gearDown = true;
    newSpawn.isAirborne = false;

    size_t spawnIdx = editor.AddSpawnPoint(newSpawn);
    TEST_ASSERT(editor.GetSpawnPointCount() == initSpawnCount + 1);
    const auto* querySpawn = editor.GetSpawnPoint(spawnIdx);
    TEST_ASSERT(querySpawn != nullptr);
    TEST_ASSERT(querySpawn->name == "CUSTOM_RUNWAY_SPAWN_01");

    // 5. Test One-Click Compilation to .lfs and Round-Trip Validation
    std::vector<uint8_t> lfsArchive = editor.CompileToLfsPackage();
    TEST_ASSERT(!lfsArchive.empty());

    Assets::LfsPackage lfsPkg;
    bool lfsLoadOk = lfsPkg.LoadFromMemory(lfsArchive);
    TEST_ASSERT(lfsLoadOk);
    TEST_ASSERT(lfsPkg.IsValid());
    TEST_ASSERT(lfsPkg.GetSpawnList().points.size() == initSpawnCount + 1);

    auto stats = editor.GetStats();
    TEST_ASSERT(stats.placedObjectCount == initObjCount + 1);
    TEST_ASSERT(stats.spawnPointCount == initSpawnCount + 1);

    std::cout << "  SceneryEditor sculpted terrain, added placed object, compiled "
              << lfsArchive.size() << " bytes .lfs package." << std::endl;
    std::cout << "[PASS] TestSceneryEditor passed successfully." << std::endl;
}

int main() {
    std::cout << "========================================================" << std::endl;
    std::cout << "Running LokYS YSFlight Interoperability Phase 4 Tests" << std::endl;
    std::cout << "========================================================" << std::endl;

    TestZipWriter();
    TestAircraftEditor();
    TestSceneryEditor();

    std::cout << "========================================================" << std::endl;
    std::cout << "ALL PHASE 4 TESTS PASSED (100% SUCCESS)" << std::endl;
    std::cout << "========================================================" << std::endl;

    return 0;
}
