#include "LokYS/graphics/GltfModel.hpp"
#include "LokYS/graphics/AircraftModelGenerator.hpp"
#include "LokYS/core/JsonParser.hpp"
#include <iostream>
#include <cmath>

using namespace LokYS;
using namespace LokYS::Core;
using namespace LokYS::Graphics;

static int g_testsPassed = 0;
static int g_testsFailed = 0;

#define TEST_ASSERT(cond, msg) \
    do { \
        if (!(cond)) { \
            std::cerr << "[FAIL] " << msg << " (line " << __LINE__ << ")\n"; \
            g_testsFailed++; \
        } else { \
            g_testsPassed++; \
        } \
    } while (0)

#define TEST_NEAR(a, b, eps, msg) \
    do { \
        double diff = std::abs(static_cast<double>(a) - static_cast<double>(b)); \
        if (diff > static_cast<double>(eps)) { \
            std::cerr << "[FAIL] " << msg << " | Expected " << b << ", got " << a << " (diff=" << diff << " > " << eps << ")\n"; \
            g_testsFailed++; \
        } else { \
            g_testsPassed++; \
        } \
    } while (0)

void TestJsonParser() {
    std::string jsonStr = R"({
        "asset": { "version": "2.0" },
        "scene": 0,
        "nodes": [
            { "name": "Fuselage", "mesh": 0, "translation": [0.0, 1.5, -2.0] },
            { "name": "Wings", "mesh": 1, "scale": [1.0, 1.0, 1.0] }
        ],
        "enabled": true,
        "speed": 125.5
    })";

    std::string err;
    JsonValue root = JsonValue::Parse(jsonStr, &err);
    TEST_ASSERT(!root.IsNull(), "JSON parsed successfully");
    TEST_ASSERT(err.empty(), "No JSON parser error");

    TEST_ASSERT(root.HasKey("asset"), "JSON has 'asset' key");
    TEST_ASSERT(root["asset"]["version"].AsString() == "2.0", "Asset version matches '2.0'");
    TEST_ASSERT(root["scene"].AsInt() == 0, "Default scene is 0");
    TEST_ASSERT(root["enabled"].AsBool() == true, "Boolean field parsed");
    TEST_NEAR(root["speed"].AsFloat(), 125.5f, 1e-4, "Float field parsed");

    const auto& nodes = root["nodes"].AsArray();
    TEST_ASSERT(nodes.size() == 2, "Array has 2 elements");
    TEST_ASSERT(nodes[0]["name"].AsString() == "Fuselage", "First node is Fuselage");
    TEST_ASSERT(nodes[1]["name"].AsString() == "Wings", "Second node is Wings");
}

void TestGlbGenerationAndLoading() {
    // 1. Generate Cessna 172 GLB
    std::vector<u8> glbData = AircraftModelGenerator::GenerateCessna172Glb();
    TEST_ASSERT(!glbData.empty(), "GLB data generated");
    TEST_ASSERT(glbData.size() > 1024, "GLB size is credible (> 1KB)");

    // 2. Validate binary header directly in memory
    u32 magic = 0;
    u32 version = 0;
    u32 length = 0;
    std::memcpy(&magic, glbData.data() + 0, 4);
    std::memcpy(&version, glbData.data() + 4, 4);
    std::memcpy(&length, glbData.data() + 8, 4);

    TEST_ASSERT(magic == 0x46546C67, "Valid glTF 2.0 magic 'glTF'");
    TEST_ASSERT(version == 2, "Valid glTF version 2");
    TEST_ASSERT(length == glbData.size(), "GLB declared length matches buffer size");

    // 3. Load via GltfModel
    GltfModel model;
    std::string loadErr;
    bool loaded = model.LoadFromGlbMemory(glbData.data(), glbData.size(), &loadErr);
    TEST_ASSERT(loaded, "GltfModel loaded Cessna 172 GLB without errors");
    TEST_ASSERT(loadErr.empty(), "No model loading error message");
    TEST_ASSERT(model.IsLoaded(), "Model reports IsLoaded true");

    // 4. Verify model contents
    TEST_ASSERT(model.GetNodes().size() == 12, "Model has 12 nodes (1 root + 11 parts)");
    TEST_ASSERT(model.GetMeshes().size() == 11, "Model has 11 distinct submeshes");
    TEST_ASSERT(model.GetMaterials().size() >= 3, "Model contains PBR materials");

    size_t totalVertices = model.GetTotalVertexCount();
    size_t totalTriangles = model.GetTotalTriangleCount();
    TEST_ASSERT(totalVertices > 200, "Sufficient vertex density (>200 vertices)");
    TEST_ASSERT(totalTriangles > 100, "Sufficient triangle density (>100 triangles)");

    // Verify critical nodes are present
    TEST_ASSERT(model.FindNodeIndex("Fuselage") >= 0, "Fuselage node exists");
    TEST_ASSERT(model.FindNodeIndex("Wings") >= 0, "Wings node exists");
    TEST_ASSERT(model.FindNodeIndex("Aileron_L") >= 0, "Aileron_L node exists");
    TEST_ASSERT(model.FindNodeIndex("Aileron_R") >= 0, "Aileron_R node exists");
    TEST_ASSERT(model.FindNodeIndex("Elevator") >= 0, "Elevator node exists");
    TEST_ASSERT(model.FindNodeIndex("Rudder") >= 0, "Rudder node exists");
    TEST_ASSERT(model.FindNodeIndex("Flap_L") >= 0, "Flap_L node exists");
    TEST_ASSERT(model.FindNodeIndex("Flap_R") >= 0, "Flap_R node exists");
    TEST_ASSERT(model.FindNodeIndex("Propeller") >= 0, "Propeller node exists");
    TEST_ASSERT(model.FindNodeIndex("Cockpit") >= 0, "Cockpit node exists");
    TEST_ASSERT(model.FindNodeIndex("LandingGear") >= 0, "LandingGear node exists");
}

void TestControlSurfaceArticulations() {
    std::vector<u8> glbData = AircraftModelGenerator::GenerateCessna172Glb();
    GltfModel model;
    TEST_ASSERT(model.LoadFromGlbMemory(glbData.data(), glbData.size()), "Loaded GLB for articulation testing");

    i32 elevIdx = model.FindNodeIndex("Elevator");
    i32 aileronLIdx = model.FindNodeIndex("Aileron_L");
    i32 aileronRIdx = model.FindNodeIndex("Aileron_R");
    i32 rudderIdx = model.FindNodeIndex("Rudder");
    i32 flapLIdx = model.FindNodeIndex("Flap_L");
    i32 propIdx = model.FindNodeIndex("Propeller");

    // Rest pose: deflections are zero
    TEST_NEAR(model.GetNodes()[elevIdx].currentDeflectionRad, 0.0f, 1e-5, "Elevator initially neutral");
    TEST_NEAR(model.GetNodes()[rudderIdx].currentDeflectionRad, 0.0f, 1e-5, "Rudder initially neutral");

    // 1. Pitch Up (+1.0 elevator pull)
    model.ApplyAircraftControls(1.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    TEST_ASSERT(model.GetNodes()[elevIdx].currentDeflectionRad < 0.0f, "Elevator deflects trailing-edge up on stick pull");

    // 2. Roll Right (+1.0 roll right)
    model.ApplyAircraftControls(0.0f, 1.0f, 0.0f, 0.0f, 0.0f);
    f32 leftAileronDeflect = model.GetNodes()[aileronLIdx].currentDeflectionRad;
    f32 rightAileronDeflect = model.GetNodes()[aileronRIdx].currentDeflectionRad;
    TEST_ASSERT(leftAileronDeflect > 0.0f, "Left aileron deflects downward on right roll");
    TEST_ASSERT(rightAileronDeflect < 0.0f, "Right aileron deflects upward on right roll");

    // 3. Rudder Right (+1.0 yaw right)
    model.ApplyAircraftControls(0.0f, 0.0f, 1.0f, 0.0f, 0.0f);
    TEST_ASSERT(model.GetNodes()[rudderIdx].currentDeflectionRad < 0.0f, "Rudder deflects trailing-edge right");

    // 4. Flaps Full Down (+1.0 flap)
    model.ApplyAircraftControls(0.0f, 0.0f, 0.0f, 1.0f, 0.0f);
    TEST_ASSERT(model.GetNodes()[flapLIdx].currentDeflectionRad > 0.4f, "Flaps deploy downwards > 20 degrees");

    // 5. Propeller Rotation (pi / 2 rad)
    model.ApplyAircraftControls(0.0f, 0.0f, 0.0f, 0.0f, 1.5707963f);
    TEST_NEAR(model.GetNodes()[propIdx].currentDeflectionRad, 1.5707963f, 1e-4, "Propeller rotates to exact requested angle");
}

void TestCorruptedGlbRejection() {
    GltfModel model;
    std::string err;

    // 1. Null data or tiny size
    TEST_ASSERT(!model.LoadFromGlbMemory(nullptr, 0, &err), "Null buffer rejected");
    u8 tiny[8] = { 0 };
    TEST_ASSERT(!model.LoadFromGlbMemory(tiny, sizeof(tiny), &err), "Header under 12 bytes rejected");

    // 2. Bad magic bytes
    u8 badMagic[12] = { 'F', 'A', 'I', 'L', 2, 0, 0, 0, 12, 0, 0, 0 };
    TEST_ASSERT(!model.LoadFromGlbMemory(badMagic, sizeof(badMagic), &err), "Non-glTF magic rejected");

    // 3. Declared length larger than buffer
    u8 badLen[12] = { 'g', 'l', 'T', 'F', 2, 0, 0, 0, 0xFF, 0xFF, 0, 0 };
    TEST_ASSERT(!model.LoadFromGlbMemory(badLen, sizeof(badLen), &err), "Length overflow rejected");
}

void TestBoundingBoxAndNormalization() {
    // 1. Generate procedural Cessna 172
    std::vector<u8> glbData = AircraftModelGenerator::GenerateCessna172Glb();
    GltfModel model;
    std::string err;
    TEST_ASSERT(model.LoadFromGlbMemory(glbData.data(), glbData.size(), &err), "Load GLB for bounds test");

    // 2. Verify initial AABB
    const auto& bounds = model.GetBounds();
    TEST_ASSERT(bounds.dimensions.x > 10.0f && bounds.dimensions.x < 12.0f, "Detected wingspan ~11.0m");
    TEST_ASSERT(bounds.dimensions.y > 1.5f && bounds.dimensions.y < 4.0f, "Detected height ~2.0-3.5m");
    TEST_ASSERT(bounds.dimensions.z > 5.0f && bounds.dimensions.z < 8.0f, "Detected length ~6.0-7.0m");

    // 3. Test AutoNormalizeToSpan on standard metric model (should stay identity)
    model.AutoNormalizeToSpan(11.0f);
    TEST_NEAR(model.GetNormalizationTransform().m[0][0], 1.0f, 1e-4, "Scale X is 1.0 for metric model");
    TEST_NEAR(model.GetNormalizationTransform().m[1][1], 1.0f, 1e-4, "Scale Y is 1.0 for metric model");
    TEST_NEAR(model.GetNormalizationTransform().m[2][2], 1.0f, 1e-4, "Scale Z is 1.0 for metric model");

    // 4. Test AutoNormalizeToSpan with target span = 5.5m (scale factor ~0.5)
    model.AutoNormalizeToSpan(5.5f);
    f32 scaleX = model.GetNormalizationTransform().m[0][0];
    TEST_NEAR(scaleX, 0.5f, 0.05f, "AutoNormalize scales down to 5.5m wingspan");
    TEST_NEAR(model.GetEffectiveDimensions().x, 5.5f, 0.1f, "Effective wingspan reported as 5.5m");
}

void TestNodeMatrixSupport() {
    std::string jsonStr = R"({
        "asset": { "version": "2.0" },
        "nodes": [
            {
                "name": "RootMatrixNode",
                "matrix": [
                    1.0, 0.0, 0.0, 0.0,
                    0.0, -1.0, 0.0, 0.0,
                    0.0, 0.0, 1.0, 0.0,
                    15.0, 25.0, 35.0, 1.0
                ]
            }
        ]
    })";

    GltfModel model;
    std::string err;
    bool parsed = model.ParseJson(jsonStr, nullptr, 0, &err);
    TEST_ASSERT(parsed, "Parsed glTF JSON with node matrix");
    TEST_ASSERT(model.GetNodes().size() == 1, "Model has 1 node");
    const auto& node = model.GetNodes()[0];
    TEST_ASSERT(node.hasMatrix, "Node detected hasMatrix=true");
    TEST_NEAR(node.matrix.m[3][0], 15.0f, 1e-4, "Matrix translation X matches");
    TEST_NEAR(node.matrix.m[3][1], 25.0f, 1e-4, "Matrix translation Y matches");
    TEST_NEAR(node.matrix.m[3][2], 35.0f, 1e-4, "Matrix translation Z matches");
    TEST_NEAR(node.matrix.m[1][1], -1.0f, 1e-4, "Matrix rotation matches inverted Y");

    model.UpdateForwardKinematics();
    TEST_NEAR(node.worldTransform.m[3][0], 15.0f, 1e-4, "WorldTransform matches local matrix X");
    TEST_NEAR(node.worldTransform.m[1][1], -1.0f, 1e-4, "WorldTransform matches local matrix Y rot");
}

int main() {
    std::cout << "========================================\n";
    std::cout << " LokYS glTF 2.0 Aircraft Model Tests\n";
    std::cout << "========================================\n";

    TestJsonParser();
    TestGlbGenerationAndLoading();
    TestControlSurfaceArticulations();
    TestCorruptedGlbRejection();
    TestBoundingBoxAndNormalization();
    TestNodeMatrixSupport();

    std::cout << "\n----------------------------------------\n";
    std::cout << "Tests Passed: " << g_testsPassed << "\n";
    std::cout << "Tests Failed: " << g_testsFailed << "\n";
    std::cout << "----------------------------------------\n";

    return (g_testsFailed == 0) ? 0 : 1;
}
