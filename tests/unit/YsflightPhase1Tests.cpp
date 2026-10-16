#include "LokYS/vfs/ZipVfsReader.hpp"
#include "LokYS/assets/AssetTokenizer.hpp"
#include "LokYS/assets/SrfParser.hpp"
#include "LokYS/assets/DnmParser.hpp"
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
using namespace LokYS::Vfs;
using namespace LokYS::Assets;

void TestAssetTokenizer() {
    std::cout << "[TEST] Running TestAssetTokenizer...\n";

    // 1. Comment stripping
    std::string_view line1 = "WEIGHCLN 630kg # Clean weight of Cessna 172";
    auto clean1 = AssetTokenizer::StripComments(line1);
    TEST_ASSERT(clean1 == "WEIGHCLN 630kg");

    std::string_view line2 = "REM This is a remark";
    auto clean2 = AssetTokenizer::StripComments(line2);
    TEST_ASSERT(clean2.empty());

    // 2. Token splitting with quotes
    std::string_view line3 = "IDENTIFY \"CESSNA_172R\" CATEGORY NORMAL";
    auto tokens3 = AssetTokenizer::SplitTokens(line3);
    TEST_ASSERT(tokens3.size() == 4);
    TEST_ASSERT(tokens3[0] == "IDENTIFY");
    TEST_ASSERT(tokens3[1] == "CESSNA_172R");
    TEST_ASSERT(tokens3[2] == "CATEGORY");
    TEST_ASSERT(tokens3[3] == "NORMAL");

    // 3. Unit conversions
    double hp = AssetTokenizer::ParseValueWithUnit("180HP");
    TEST_ASSERT(std::abs(hp - (180.0 * 745.699872)) < 1.0);

    double massKg = AssetTokenizer::ParseValueWithUnit("630kg");
    TEST_ASSERT(std::abs(massKg - 630.0) < 1e-4);

    double massTon = AssetTokenizer::ParseValueWithUnit("2.5t");
    TEST_ASSERT(std::abs(massTon - 2500.0) < 1e-4);

    double altFt = AssetTokenizer::ParseValueWithUnit("8000ft");
    TEST_ASSERT(std::abs(altFt - (8000.0 * 0.3048)) < 1e-4);

    double spdKmh = AssetTokenizer::ParseValueWithUnit("250km/h");
    TEST_ASSERT(std::abs(spdKmh - (250.0 / 3.6)) < 1e-4);

    double spdKt = AssetTokenizer::ParseValueWithUnit("120kt");
    TEST_ASSERT(std::abs(spdKt - (120.0 * 0.514444444444444)) < 1e-4);

    double aoaDeg = AssetTokenizer::ParseValueWithUnit("15deg");
    TEST_ASSERT(std::abs(aoaDeg - (15.0 * 3.141592653589793 / 180.0)) < 1e-4);

    double pct = AssetTokenizer::ParseValueWithUnit("75%");
    TEST_ASSERT(std::abs(pct - 0.75) < 1e-4);

    // 4. Boolean parsing
    TEST_ASSERT(AssetTokenizer::ParseBool("TRUE") == true);
    TEST_ASSERT(AssetTokenizer::ParseBool("FALSE") == false);
    TEST_ASSERT(AssetTokenizer::ParseBool("1") == true);
    TEST_ASSERT(AssetTokenizer::ParseBool("0") == false);

    // 5. Vec3 parsing
    std::vector<std::string> posTokens = { "POSITION", "100m", "50ft", "-20m" };
    auto vec = AssetTokenizer::ParseVec3(posTokens, 1);
    TEST_ASSERT(std::abs(vec.x - 100.0) < 1e-4);
    TEST_ASSERT(std::abs(vec.y - (50.0 * 0.3048)) < 1e-4);
    TEST_ASSERT(std::abs(vec.z - (-20.0)) < 1e-4);

    std::cout << "[PASS] TestAssetTokenizer passed successfully.\n";
}

void TestSrfParser() {
    std::cout << "[TEST] Running TestSrfParser...\n";

    // Synthetic SRF
    std::string_view syntheticSrf =
        "SURF\n"
        "V 0 0 0\n"
        "V 10 0 0\n"
        "V 10 10 0\n"
        "V 0 10 0\n"
        "F\n"
        "V 0 1 2 3\n"
        "C 255 128 0\n"
        "E\n";

    auto modelOpt = SrfParser::ParseFromString(syntheticSrf, "TestQuad");
    TEST_ASSERT(modelOpt.has_value());
    TEST_ASSERT(modelOpt->IsValid());
    TEST_ASSERT(modelOpt->polygonCount == 1);
    TEST_ASSERT(modelOpt->vertices.size() == 4);
    TEST_ASSERT(modelOpt->indices.size() == 6); // 1 quad = 2 triangles = 6 indices

    // Verify vertex color
    TEST_ASSERT(std::abs(modelOpt->vertices[0].color.x - 1.0f) < 1e-3f);
    TEST_ASSERT(std::abs(modelOpt->vertices[0].color.y - (128.0f / 255.0f)) < 1e-3f);
    TEST_ASSERT(std::abs(modelOpt->vertices[0].color.z - 0.0f) < 1e-3f);

    // Test real file from Ysflight if present
    if (std::filesystem::exists("Ysflight/ground/bridge1.srf")) {
        auto realModelOpt = SrfParser::ParseFromFile("Ysflight/ground/bridge1.srf");
        TEST_ASSERT(realModelOpt.has_value());
        TEST_ASSERT(realModelOpt->IsValid());
        TEST_ASSERT(realModelOpt->polygonCount > 0);
        TEST_ASSERT(!realModelOpt->vertices.empty());
        TEST_ASSERT(!realModelOpt->indices.empty());
        std::cout << "  Loaded real bridge1.srf: " << realModelOpt->vertices.size()
                  << " vertices, " << realModelOpt->indices.size() << " indices, "
                  << realModelOpt->polygonCount << " polygons.\n";
    }

    std::cout << "[PASS] TestSrfParser passed successfully.\n";
}

void TestDnmParser() {
    std::cout << "[TEST] Running TestDnmParser...\n";

    // Synthetic DNM with two articulated nodes
    std::string_view syntheticDnm =
        "DYNAMODEL\n"
        "DNMVER 1\n"
        "PCK \"Fuselage\" 10\n"
        "Surf\n"
        "V 0 0 0\n"
        "V 5 0 0\n"
        "V 0 5 0\n"
        "F\n"
        "V 0 1 2\n"
        "C 200 200 200\n"
        "E\n"
        "PCK \"Rudder\" 10\n"
        "Surf\n"
        "V 0 0 0\n"
        "V 0 2 0\n"
        "V 0 2 -1\n"
        "F\n"
        "V 0 1 2\n"
        "C 255 0 0\n"
        "E\n"
        "SRF \"Fuselage\"\n"
        "CLA 0\n"
        "NCH 1\n"
        "CLD \"Rudder\"\n"
        "POS 0 0 0 0 0 0\n"
        "END\n"
        "SRF \"Rudder\"\n"
        "CLA 8\n"
        "POS 0 1 -3 0 0 0\n"
        "CNT 0 1 -3\n"
        "PAX 0 1 0\n"
        "END\n";

    auto dnmOpt = DnmParser::ParseFromString(syntheticDnm, "TestAircraft");
    TEST_ASSERT(dnmOpt.has_value());
    TEST_ASSERT(dnmOpt->nodes.size() == 2);

    const auto* fuse = dnmOpt->FindNode("Fuselage");
    TEST_ASSERT(fuse != nullptr);
    TEST_ASSERT(fuse->animationClass == 0);
    TEST_ASSERT(fuse->childrenNames.size() == 1);
    TEST_ASSERT(fuse->childrenNames[0] == "Rudder");
    TEST_ASSERT(fuse->vertices.size() == 3);
    TEST_ASSERT(fuse->indices.size() == 3);

    const auto* rud = dnmOpt->FindNode("Rudder");
    TEST_ASSERT(rud != nullptr);
    TEST_ASSERT(rud->animationClass == 8); // Rudder
    TEST_ASSERT(std::abs(rud->position.y - 1.0f) < 1e-4f);
    TEST_ASSERT(std::abs(rud->position.z - (-3.0f)) < 1e-4f);
    TEST_ASSERT(std::abs(rud->pivotAxis.y - 1.0f) < 1e-4f);

    // Test real file from Ysflight if present
    if (std::filesystem::exists("Ysflight/aircraft/cessna172r.dnm")) {
        auto realDnmOpt = DnmParser::ParseFromFile("Ysflight/aircraft/cessna172r.dnm");
        TEST_ASSERT(realDnmOpt.has_value());
        TEST_ASSERT(!realDnmOpt->nodes.empty());

        const auto* c172Fuse = realDnmOpt->FindNode("Fuselage");
        TEST_ASSERT(c172Fuse != nullptr);

        const auto* c172Elev = realDnmOpt->FindNode("Elevator");
        TEST_ASSERT(c172Elev != nullptr);
        TEST_ASSERT(c172Elev->animationClass == 6); // Elevator class

        const auto* c172Rud = realDnmOpt->FindNode("Rudder");
        TEST_ASSERT(c172Rud != nullptr);
        TEST_ASSERT(c172Rud->animationClass == 8); // Rudder class

        std::cout << "  Loaded real cessna172r.dnm: " << realDnmOpt->nodes.size()
                  << " nodes, " << realDnmOpt->GetTotalVertices() << " total vertices, "
                  << realDnmOpt->GetTotalIndices() << " total indices.\n";
    }

    std::cout << "[PASS] TestDnmParser passed successfully.\n";
}

void TestZipVfsReader() {
    std::cout << "[TEST] Running TestZipVfsReader...\n";

    // Minimal valid PKZIP archive containing one uncompressed (STORED) file: "hello.txt" -> "Hello World!"
    // Construct standard ZIP in memory
    const uint8_t mockZip[] = {
        // Local File Header
        0x50, 0x4b, 0x03, 0x04, // signature
        0x14, 0x00,             // version needed (2.0)
        0x00, 0x00,             // flags
        0x00, 0x00,             // method 0 (stored)
        0x00, 0x00, 0x00, 0x00, // mod time/date
        0x1c, 0x3d, 0x44, 0x47, // crc32
        0x0c, 0x00, 0x00, 0x00, // compressed size (12)
        0x0c, 0x00, 0x00, 0x00, // uncompressed size (12)
        0x09, 0x00,             // file name length (9)
        0x00, 0x00,             // extra field length (0)
        // File name: "hello.txt"
        'h', 'e', 'l', 'l', 'o', '.', 't', 'x', 't',
        // Payload: "Hello World!"
        'H', 'e', 'l', 'l', 'o', ' ', 'W', 'o', 'r', 'l', 'd', '!',

        // Central Directory File Header
        0x50, 0x4b, 0x01, 0x02, // signature
        0x14, 0x00,             // version made by
        0x14, 0x00,             // version needed
        0x00, 0x00,             // flags
        0x00, 0x00,             // method 0 (stored)
        0x00, 0x00, 0x00, 0x00, // mod time/date
        0x1c, 0x3d, 0x44, 0x47, // crc32
        0x0c, 0x00, 0x00, 0x00, // compressed size (12)
        0x0c, 0x00, 0x00, 0x00, // uncompressed size (12)
        0x09, 0x00,             // file name length (9)
        0x00, 0x00,             // extra field length (0)
        0x00, 0x00,             // file comment length (0)
        0x00, 0x00,             // disk number start
        0x00, 0x00,             // internal file attributes
        0x00, 0x00, 0x00, 0x00, // external file attributes
        0x00, 0x00, 0x00, 0x00, // relative offset of local header (0)
        // File name: "hello.txt"
        'h', 'e', 'l', 'l', 'o', '.', 't', 'x', 't',

        // End of Central Directory Record (EOCD)
        0x50, 0x4b, 0x05, 0x06, // signature
        0x00, 0x00,             // disk number
        0x00, 0x00,             // disk with CD
        0x01, 0x00,             // total entries on this disk (1)
        0x01, 0x00,             // total entries (1)
        0x37, 0x00, 0x00, 0x00, // size of central directory (55 bytes)
        0x33, 0x00, 0x00, 0x00, // offset of central directory (51 bytes)
        0x00, 0x00              // comment length (0)
    };

    ZipVfsReader reader;
    bool ok = reader.LoadFromMemory(mockZip);
    TEST_ASSERT(ok);
    TEST_ASSERT(reader.HasFile("hello.txt"));
    TEST_ASSERT(reader.HasFile("HELLO.TXT")); // case-insensitive

    auto textOpt = reader.ReadText("hello.txt");
    TEST_ASSERT(textOpt.has_value());
    TEST_ASSERT(*textOpt == "Hello World!");

    auto files = reader.ListFiles();
    TEST_ASSERT(files.size() == 1);
    TEST_ASSERT(files[0] == "hello.txt");

    std::cout << "[PASS] TestZipVfsReader passed successfully.\n";
}

int main() {
    std::cout << "========================================================\n";
    std::cout << "Running LokYS YSFlight Interoperability Phase 1 Tests\n";
    std::cout << "========================================================\n";

    TestAssetTokenizer();
    TestSrfParser();
    TestDnmParser();
    TestZipVfsReader();

    std::cout << "========================================================\n";
    std::cout << "ALL PHASE 1 TESTS PASSED (100% SUCCESS)\n";
    std::cout << "========================================================\n";
    return 0;
}
