#include "LokYS/graphics/AircraftModelGenerator.hpp"
#include "LokYS/graphics/GltfTypes.hpp"
#include "LokYS/core/JsonParser.hpp"
#include "LokYS/graphics/Mesh.hpp"
#include "LokYS/math/Vector3.hpp"
#include <fstream>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <algorithm>

namespace LokYS::Graphics {

namespace {

// Helper to add a quad (2 triangles) to vertices and indices
void AddQuad(
    std::vector<Vertex3D>& verts,
    std::vector<u32>& inds,
    const Math::Vec3f& p0,
    const Math::Vec3f& p1,
    const Math::Vec3f& p2,
    const Math::Vec3f& p3,
    const Math::Vec4f& color)
{
    u32 base = static_cast<u32>(verts.size());

    Math::Vec3f v1 = p1 - p0;
    Math::Vec3f v2 = p2 - p0;
    Math::Vec3f normal = v1.Cross(v2).Normalized();

    verts.push_back({ p0, normal, color });
    verts.push_back({ p1, normal, color });
    verts.push_back({ p2, normal, color });
    verts.push_back({ p3, normal, color });

    inds.push_back(base + 0);
    inds.push_back(base + 1);
    inds.push_back(base + 2);

    inds.push_back(base + 0);
    inds.push_back(base + 2);
    inds.push_back(base + 3);
}

// Helper to add a box with 6 faces
void AddBox(
    std::vector<Vertex3D>& verts,
    std::vector<u32>& inds,
    const Math::Vec3f& min,
    const Math::Vec3f& max,
    const Math::Vec4f& color)
{
    // Front (+Z)
    AddQuad(verts, inds,
        { min.x, min.y, max.z }, { max.x, min.y, max.z },
        { max.x, max.y, max.z }, { min.x, max.y, max.z }, color);
    // Back (-Z)
    AddQuad(verts, inds,
        { max.x, min.y, min.z }, { min.x, min.y, min.z },
        { min.x, max.y, min.z }, { max.x, max.y, min.z }, color);
    // Left (-X)
    AddQuad(verts, inds,
        { min.x, min.y, min.z }, { min.x, min.y, max.z },
        { min.x, max.y, max.z }, { min.x, max.y, min.z }, color);
    // Right (+X)
    AddQuad(verts, inds,
        { max.x, min.y, max.z }, { max.x, min.y, min.z },
        { max.x, max.y, min.z }, { max.x, max.y, max.z }, color);
    // Top (+Y)
    AddQuad(verts, inds,
        { min.x, max.y, max.z }, { max.x, max.y, max.z },
        { max.x, max.y, min.z }, { min.x, max.y, min.z }, color);
    // Bottom (-Y)
    AddQuad(verts, inds,
        { min.x, min.y, min.z }, { max.x, min.y, min.z },
        { max.x, min.y, max.z }, { min.x, min.y, max.z }, color);
}

// Helper to add a cylinder (for propeller spinner or fuselage sections)
void AddCylinder(
    std::vector<Vertex3D>& verts,
    std::vector<u32>& inds,
    const Math::Vec3f& baseCenter,
    f32 radius,
    f32 length,
    int segments,
    const Math::Vec4f& color)
{
    constexpr f32 twoPi = 6.2831853f;
    f32 angleStep = twoPi / static_cast<f32>(segments);

    for (int i = 0; i < segments; ++i) {
        f32 a0 = static_cast<f32>(i) * angleStep;
        f32 a1 = static_cast<f32>(i + 1) * angleStep;

        f32 c0 = std::cos(a0), s0 = std::sin(a0);
        f32 c1 = std::cos(a1), s1 = std::sin(a1);

        Math::Vec3f p0{ baseCenter.x + radius * c0, baseCenter.y + radius * s0, baseCenter.z };
        Math::Vec3f p1{ baseCenter.x + radius * c1, baseCenter.y + radius * s1, baseCenter.z };
        Math::Vec3f p2{ baseCenter.x + radius * c1, baseCenter.y + radius * s1, baseCenter.z + length };
        Math::Vec3f p3{ baseCenter.x + radius * c0, baseCenter.y + radius * s0, baseCenter.z + length };

        AddQuad(verts, inds, p0, p1, p2, p3, color);
    }
}

struct SubmeshData {
    std::string nodeName;
    std::vector<Vertex3D> vertices;
    std::vector<u32> indices;
    Math::Vec3f translation{ 0.0f, 0.0f, 0.0f };
    Math::Vec3f pivot{ 0.0f, 0.0f, 0.0f };
    Math::Vec3f hingeAxis{ 1.0f, 0.0f, 0.0f };
    i32 materialIndex{ 0 };
};

// Generic GLB binary packager for any set of aircraft submeshes
std::vector<u8> SerializeSubmeshesToGlb(const std::vector<SubmeshData>& submeshes, const std::string& modelName) {
    std::vector<u8> binBuffer;

    struct AccessorMeta {
        size_t bufferViewIndex;
        size_t byteOffset;
        GltfComponentType compType;
        size_t count;
        std::string type;
        std::vector<f32> minVals;
        std::vector<f32> maxVals;
    };

    struct BufferViewMeta {
        size_t byteOffset;
        size_t byteLength;
        size_t byteStride;
        u32 target;
    };

    std::vector<BufferViewMeta> bufferViews;
    std::vector<AccessorMeta> accessors;

    struct MeshPrimitiveMeta {
        size_t posAccessorIdx;
        size_t normAccessorIdx;
        size_t colAccessorIdx;
        size_t indAccessorIdx;
        i32 materialIdx;
    };

    std::vector<MeshPrimitiveMeta> meshPrimMetas;

    for (const auto& sub : submeshes) {
        // 1. Positions View & Accessor (VEC3 FLOAT)
        size_t posOffset = binBuffer.size();
        size_t posBytes = sub.vertices.size() * sizeof(f32) * 3;

        std::vector<f32> minP{ 1e9f, 1e9f, 1e9f };
        std::vector<f32> maxP{ -1e9f, -1e9f, -1e9f };

        for (const auto& v : sub.vertices) {
            f32 p[3] = { v.position.x, v.position.y, v.position.z };
            const u8* pBytes = reinterpret_cast<const u8*>(p);
            binBuffer.insert(binBuffer.end(), pBytes, pBytes + 12);

            for (int k = 0; k < 3; ++k) {
                minP[k] = std::min(minP[k], p[k]);
                maxP[k] = std::max(maxP[k], p[k]);
            }
        }
        size_t posBvIdx = bufferViews.size();
        bufferViews.push_back({ posOffset, posBytes, 0, 34962 }); // ARRAY_BUFFER

        size_t posAccIdx = accessors.size();
        accessors.push_back({ posBvIdx, 0, GltfComponentType::Float, sub.vertices.size(), "VEC3", minP, maxP });

        // 2. Normals View & Accessor (VEC3 FLOAT)
        size_t normOffset = binBuffer.size();
        size_t normBytes = sub.vertices.size() * sizeof(f32) * 3;
        for (const auto& v : sub.vertices) {
            f32 n[3] = { v.normal.x, v.normal.y, v.normal.z };
            const u8* nBytes = reinterpret_cast<const u8*>(n);
            binBuffer.insert(binBuffer.end(), nBytes, nBytes + 12);
        }
        size_t normBvIdx = bufferViews.size();
        bufferViews.push_back({ normOffset, normBytes, 0, 34962 });

        size_t normAccIdx = accessors.size();
        accessors.push_back({ normBvIdx, 0, GltfComponentType::Float, sub.vertices.size(), "VEC3", {}, {} });

        // 3. Colors View & Accessor (VEC4 FLOAT)
        size_t colOffset = binBuffer.size();
        size_t colBytes = sub.vertices.size() * sizeof(f32) * 4;
        for (const auto& v : sub.vertices) {
            f32 c[4] = { v.color.x, v.color.y, v.color.z, v.color.w };
            const u8* cBytes = reinterpret_cast<const u8*>(c);
            binBuffer.insert(binBuffer.end(), cBytes, cBytes + 16);
        }
        size_t colBvIdx = bufferViews.size();
        bufferViews.push_back({ colOffset, colBytes, 0, 34962 });

        size_t colAccIdx = accessors.size();
        accessors.push_back({ colBvIdx, 0, GltfComponentType::Float, sub.vertices.size(), "VEC4", {}, {} });

        // 4. Indices View & Accessor (SCALAR UNSIGNED_INT)
        size_t indOffset = binBuffer.size();
        size_t indBytes = sub.indices.size() * sizeof(u32);
        const u8* iBytes = reinterpret_cast<const u8*>(sub.indices.data());
        binBuffer.insert(binBuffer.end(), iBytes, iBytes + indBytes);

        size_t indBvIdx = bufferViews.size();
        bufferViews.push_back({ indOffset, indBytes, 0, 34963 }); // ELEMENT_ARRAY_BUFFER

        size_t indAccIdx = accessors.size();
        accessors.push_back({ indBvIdx, 0, GltfComponentType::UnsignedInt, sub.indices.size(), "SCALAR", {}, {} });

        meshPrimMetas.push_back({ posAccIdx, normAccIdx, colAccIdx, indAccIdx, sub.materialIndex });

        // Pad to 4 bytes boundary
        while ((binBuffer.size() % 4) != 0) {
            binBuffer.push_back(0);
        }
    }

    // JSON Manifest Chunk
    Core::JsonValue jsonRoot(std::unordered_map<std::string, Core::JsonValue>{});

    // Asset
    {
        std::unordered_map<std::string, Core::JsonValue> asset;
        asset["version"] = Core::JsonValue("2.0");
        asset["generator"] = Core::JsonValue(std::string("LokYS Generator - ") + modelName);
        jsonRoot["asset"] = Core::JsonValue(std::move(asset));
    }

    // Materials
    {
        std::vector<Core::JsonValue> materials;
        auto AddMat = [&](const std::string& name, const Math::Vec4f& col, f32 met, f32 rough) {
            std::unordered_map<std::string, Core::JsonValue> m;
            m["name"] = Core::JsonValue(name);
            std::unordered_map<std::string, Core::JsonValue> pbr;
            std::vector<Core::JsonValue> baseCol{
                Core::JsonValue(static_cast<f64>(col.x)),
                Core::JsonValue(static_cast<f64>(col.y)),
                Core::JsonValue(static_cast<f64>(col.z)),
                Core::JsonValue(static_cast<f64>(col.w))
            };
            pbr["baseColorFactor"] = Core::JsonValue(std::move(baseCol));
            pbr["metallicFactor"] = Core::JsonValue(static_cast<f64>(met));
            pbr["roughnessFactor"] = Core::JsonValue(static_cast<f64>(rough));
            m["pbrMetallicRoughness"] = Core::JsonValue(std::move(pbr));
            materials.push_back(Core::JsonValue(std::move(m)));
        };

        AddMat("Mat_Fuselage", { 0.94f, 0.95f, 0.97f, 1.0f }, 0.1f, 0.4f);
        AddMat("Mat_Propeller", { 0.10f, 0.10f, 0.10f, 1.0f }, 0.2f, 0.6f);
        AddMat("Mat_Glass", { 0.40f, 0.60f, 0.80f, 0.85f }, 0.9f, 0.1f);
        AddMat("Mat_Cockpit", { 0.15f, 0.16f, 0.18f, 1.0f }, 0.0f, 0.9f);
        AddMat("Mat_Trim", { 0.12f, 0.30f, 0.65f, 1.0f }, 0.2f, 0.5f);

        jsonRoot["materials"] = Core::JsonValue(std::move(materials));
    }

    // Meshes
    {
        std::vector<Core::JsonValue> meshes;
        for (size_t i = 0; i < submeshes.size(); ++i) {
            std::unordered_map<std::string, Core::JsonValue> meshObj;
            meshObj["name"] = Core::JsonValue(submeshes[i].nodeName);

            std::vector<Core::JsonValue> primitives;
            std::unordered_map<std::string, Core::JsonValue> prim;

            std::unordered_map<std::string, Core::JsonValue> attrs;
            attrs["POSITION"] = Core::JsonValue(static_cast<f64>(meshPrimMetas[i].posAccessorIdx));
            attrs["NORMAL"] = Core::JsonValue(static_cast<f64>(meshPrimMetas[i].normAccessorIdx));
            attrs["COLOR_0"] = Core::JsonValue(static_cast<f64>(meshPrimMetas[i].colAccessorIdx));
            prim["attributes"] = Core::JsonValue(std::move(attrs));

            prim["indices"] = Core::JsonValue(static_cast<f64>(meshPrimMetas[i].indAccessorIdx));
            prim["material"] = Core::JsonValue(static_cast<f64>(meshPrimMetas[i].materialIdx));

            primitives.push_back(Core::JsonValue(std::move(prim)));
            meshObj["primitives"] = Core::JsonValue(std::move(primitives));
            meshes.push_back(Core::JsonValue(std::move(meshObj)));
        }
        jsonRoot["meshes"] = Core::JsonValue(std::move(meshes));
    }

    // Nodes
    {
        std::vector<Core::JsonValue> nodes;

        // Node 0: Root node containing all submesh parts
        std::unordered_map<std::string, Core::JsonValue> rootNodeObj;
        rootNodeObj["name"] = Core::JsonValue("Root");
        std::vector<Core::JsonValue> rootChildren;
        for (size_t i = 0; i < submeshes.size(); ++i) {
            rootChildren.push_back(Core::JsonValue(static_cast<f64>(i + 1)));
        }
        rootNodeObj["children"] = Core::JsonValue(std::move(rootChildren));
        nodes.push_back(Core::JsonValue(std::move(rootNodeObj)));

        for (size_t i = 0; i < submeshes.size(); ++i) {
            std::unordered_map<std::string, Core::JsonValue> nodeObj;
            nodeObj["name"] = Core::JsonValue(submeshes[i].nodeName);
            nodeObj["mesh"] = Core::JsonValue(static_cast<f64>(i));

            if (submeshes[i].translation.x != 0.0f || submeshes[i].translation.y != 0.0f || submeshes[i].translation.z != 0.0f) {
                std::vector<Core::JsonValue> trans{
                    Core::JsonValue(static_cast<f64>(submeshes[i].translation.x)),
                    Core::JsonValue(static_cast<f64>(submeshes[i].translation.y)),
                    Core::JsonValue(static_cast<f64>(submeshes[i].translation.z))
                };
                nodeObj["translation"] = Core::JsonValue(std::move(trans));
            }

            nodes.push_back(Core::JsonValue(std::move(nodeObj)));
        }
        jsonRoot["nodes"] = Core::JsonValue(std::move(nodes));
    }

    // Scenes & Scene
    {
        std::vector<Core::JsonValue> scenes;
        std::unordered_map<std::string, Core::JsonValue> sceneObj;
        sceneObj["name"] = Core::JsonValue("Scene");

        std::vector<Core::JsonValue> rootNodes;
        rootNodes.push_back(Core::JsonValue(0.0));
        sceneObj["nodes"] = Core::JsonValue(std::move(rootNodes));
        scenes.push_back(Core::JsonValue(std::move(sceneObj)));

        jsonRoot["scenes"] = Core::JsonValue(std::move(scenes));
        jsonRoot["scene"] = Core::JsonValue(0.0);
    }

    // BufferViews
    {
        std::vector<Core::JsonValue> bvArr;
        for (const auto& bv : bufferViews) {
            std::unordered_map<std::string, Core::JsonValue> bvObj;
            bvObj["buffer"] = Core::JsonValue(0.0);
            bvObj["byteOffset"] = Core::JsonValue(static_cast<f64>(bv.byteOffset));
            bvObj["byteLength"] = Core::JsonValue(static_cast<f64>(bv.byteLength));
            if (bv.target != 0) {
                bvObj["target"] = Core::JsonValue(static_cast<f64>(bv.target));
            }
            bvArr.push_back(Core::JsonValue(std::move(bvObj)));
        }
        jsonRoot["bufferViews"] = Core::JsonValue(std::move(bvArr));
    }

    // Accessors
    {
        std::vector<Core::JsonValue> accArr;
        for (const auto& acc : accessors) {
            std::unordered_map<std::string, Core::JsonValue> accObj;
            accObj["bufferView"] = Core::JsonValue(static_cast<f64>(acc.bufferViewIndex));
            accObj["byteOffset"] = Core::JsonValue(static_cast<f64>(acc.byteOffset));
            accObj["componentType"] = Core::JsonValue(static_cast<f64>(static_cast<u32>(acc.compType)));
            accObj["count"] = Core::JsonValue(static_cast<f64>(acc.count));
            accObj["type"] = Core::JsonValue(acc.type);

            if (!acc.minVals.empty()) {
                std::vector<Core::JsonValue> minArr;
                for (f32 val : acc.minVals) minArr.push_back(Core::JsonValue(static_cast<f64>(val)));
                accObj["min"] = Core::JsonValue(std::move(minArr));
            }
            if (!acc.maxVals.empty()) {
                std::vector<Core::JsonValue> maxArr;
                for (f32 val : acc.maxVals) maxArr.push_back(Core::JsonValue(static_cast<f64>(val)));
                accObj["max"] = Core::JsonValue(std::move(maxArr));
            }

            accArr.push_back(Core::JsonValue(std::move(accObj)));
        }
        jsonRoot["accessors"] = Core::JsonValue(std::move(accArr));
    }

    // Buffers
    {
        std::vector<Core::JsonValue> bArr;
        std::unordered_map<std::string, Core::JsonValue> bObj;
        bObj["byteLength"] = Core::JsonValue(static_cast<f64>(binBuffer.size()));
        bArr.push_back(Core::JsonValue(std::move(bObj)));
        jsonRoot["buffers"] = Core::JsonValue(std::move(bArr));
    }

    std::string jsonStr = jsonRoot.Serialize();

    // Pad JSON string with spaces up to 4-byte boundary
    while ((jsonStr.size() % 4) != 0) {
        jsonStr.push_back(' ');
    }

    // Total GLB = Header (12) + JSON Chunk Header (8) + JSON Len + BIN Chunk Header (8) + BIN Len
    u32 jsonChunkLen = static_cast<u32>(jsonStr.size());
    u32 binChunkLen = static_cast<u32>(binBuffer.size());
    u32 totalLength = 12 + 8 + jsonChunkLen + 8 + binChunkLen;

    std::vector<u8> glb(totalLength);
    size_t outOffset = 0;

    // 1. GLB Header
    u32 magic = 0x46546C67; // "glTF"
    u32 version = 2;
    std::memcpy(glb.data() + 0, &magic, 4);
    std::memcpy(glb.data() + 4, &version, 4);
    std::memcpy(glb.data() + 8, &totalLength, 4);
    outOffset = 12;

    // 2. Chunk 0: JSON Chunk
    u32 jsonChunkType = 0x4E4F534A; // "JSON"
    std::memcpy(glb.data() + outOffset, &jsonChunkLen, 4);
    std::memcpy(glb.data() + outOffset + 4, &jsonChunkType, 4);
    outOffset += 8;
    std::memcpy(glb.data() + outOffset, jsonStr.data(), jsonChunkLen);
    outOffset += jsonChunkLen;

    // 3. Chunk 1: BIN Chunk
    u32 binChunkType = 0x004E4942; // "BIN\0"
    std::memcpy(glb.data() + outOffset, &binChunkLen, 4);
    std::memcpy(glb.data() + outOffset + 4, &binChunkType, 4);
    outOffset += 8;
    std::memcpy(glb.data() + outOffset, binBuffer.data(), binChunkLen);
    outOffset += binChunkLen;

    return glb;
}

} // namespace

// =========================================================================
// 1. Cessna 172 Skyhawk Model Generator
// =========================================================================
std::vector<u8> AircraftModelGenerator::GenerateCessna172Glb() {
    const Math::Vec4f colAviationWhite{ 0.94f, 0.95f, 0.97f, 1.0f };
    const Math::Vec4f colAeroBlue{ 0.12f, 0.30f, 0.65f, 1.0f };
    const Math::Vec4f colGoldTrim{ 0.85f, 0.70f, 0.15f, 1.0f };
    const Math::Vec4f colDarkCockpit{ 0.15f, 0.16f, 0.18f, 1.0f };
    const Math::Vec4f colGlassTint{ 0.40f, 0.60f, 0.80f, 0.85f };
    const Math::Vec4f colRubberTire{ 0.12f, 0.12f, 0.12f, 1.0f };
    const Math::Vec4f colChromeStrut{ 0.75f, 0.76f, 0.78f, 1.0f };
    const Math::Vec4f colPropBlack{ 0.10f, 0.10f, 0.10f, 1.0f };
    const Math::Vec4f colSafetyYellow{ 0.95f, 0.85f, 0.10f, 1.0f };

    std::vector<SubmeshData> submeshes;

    // 1. Fuselage Main Body
    {
        SubmeshData sub{};
        sub.nodeName = "Fuselage";
        sub.materialIndex = 0;

        AddBox(sub.vertices, sub.indices, { -0.52f, -0.45f, -1.10f }, { +0.52f, +0.65f, +1.20f }, colAviationWhite);
        AddBox(sub.vertices, sub.indices, { -0.55f, -0.15f, -1.05f }, { +0.55f, -0.05f, +1.15f }, colAeroBlue);
        AddBox(sub.vertices, sub.indices, { -0.45f, -0.40f, +1.20f }, { +0.45f, +0.35f, +2.15f }, colAviationWhite);
        AddBox(sub.vertices, sub.indices, { -0.32f, -0.22f, -4.40f }, { +0.32f, +0.35f, -1.10f }, colAviationWhite);
        AddBox(sub.vertices, sub.indices, { -0.06f, +0.35f, -4.40f }, { +0.06f, +1.85f, -3.20f }, colAeroBlue);
        AddBox(sub.vertices, sub.indices, { -1.65f, +0.30f, -4.40f }, { +1.65f, +0.42f, -3.50f }, colAviationWhite);

        submeshes.push_back(std::move(sub));
    }

    // 2. High Wings and Struts
    {
        SubmeshData sub{};
        sub.nodeName = "Wings";
        sub.materialIndex = 0;

        AddBox(sub.vertices, sub.indices, { -5.50f, +0.72f, -0.70f }, { +5.50f, +0.88f, +0.80f }, colAviationWhite);
        AddBox(sub.vertices, sub.indices, { -1.82f, -0.35f, +0.10f }, { -1.78f, +0.72f, +0.18f }, colChromeStrut);
        AddBox(sub.vertices, sub.indices, { +1.78f, -0.35f, +0.10f }, { +1.82f, +0.72f, +0.18f }, colChromeStrut);

        submeshes.push_back(std::move(sub));
    }

    // 3. Aileron_L
    {
        SubmeshData sub{};
        sub.nodeName = "Aileron_L";
        sub.pivot = { -4.0f, +0.75f, -0.35f };
        sub.hingeAxis = { 1.0f, 0.0f, 0.0f };
        sub.materialIndex = 0;
        AddBox(sub.vertices, sub.indices, { -5.35f, +0.71f, -0.75f }, { -2.80f, +0.79f, -0.35f }, colAviationWhite);
        submeshes.push_back(std::move(sub));
    }

    // 4. Aileron_R
    {
        SubmeshData sub{};
        sub.nodeName = "Aileron_R";
        sub.pivot = { +4.0f, +0.75f, -0.35f };
        sub.hingeAxis = { 1.0f, 0.0f, 0.0f };
        sub.materialIndex = 0;
        AddBox(sub.vertices, sub.indices, { +2.80f, +0.71f, -0.75f }, { +5.35f, +0.79f, -0.35f }, colAviationWhite);
        submeshes.push_back(std::move(sub));
    }

    // 5. Flap_L
    {
        SubmeshData sub{};
        sub.nodeName = "Flap_L";
        sub.pivot = { -1.7f, +0.75f, -0.35f };
        sub.hingeAxis = { 1.0f, 0.0f, 0.0f };
        sub.materialIndex = 0;
        AddBox(sub.vertices, sub.indices, { -2.75f, +0.70f, -0.80f }, { -0.65f, +0.80f, -0.35f }, colAviationWhite);
        submeshes.push_back(std::move(sub));
    }

    // 6. Flap_R
    {
        SubmeshData sub{};
        sub.nodeName = "Flap_R";
        sub.pivot = { +1.7f, +0.75f, -0.35f };
        sub.hingeAxis = { 1.0f, 0.0f, 0.0f };
        sub.materialIndex = 0;
        AddBox(sub.vertices, sub.indices, { +0.65f, +0.70f, -0.80f }, { +2.75f, +0.80f, -0.35f }, colAviationWhite);
        submeshes.push_back(std::move(sub));
    }

    // 7. Elevator
    {
        SubmeshData sub{};
        sub.nodeName = "Elevator";
        sub.pivot = { 0.0f, +0.36f, -4.45f };
        sub.hingeAxis = { 1.0f, 0.0f, 0.0f };
        sub.materialIndex = 0;
        AddBox(sub.vertices, sub.indices, { -1.72f, +0.33f, -4.92f }, { +1.72f, +0.39f, -4.45f }, colAviationWhite);
        submeshes.push_back(std::move(sub));
    }

    // 8. Rudder
    {
        SubmeshData sub{};
        sub.nodeName = "Rudder";
        sub.pivot = { 0.0f, +0.38f, -4.50f };
        sub.hingeAxis = { 0.0f, 1.0f, 0.0f };
        sub.materialIndex = 0;
        AddBox(sub.vertices, sub.indices, { -0.06f, +0.40f, -4.95f }, { +0.06f, +1.92f, -4.50f }, colAeroBlue);
        submeshes.push_back(std::move(sub));
    }

    // 9. Propeller
    {
        SubmeshData sub{};
        sub.nodeName = "Propeller";
        sub.pivot = { 0.0f, -0.05f, +2.15f };
        sub.hingeAxis = { 0.0f, 0.0f, 1.0f };
        sub.materialIndex = 1;

        AddCylinder(sub.vertices, sub.indices, { 0.0f, -0.05f, +2.15f }, 0.22f, 0.25f, 12, colAviationWhite);
        AddBox(sub.vertices, sub.indices, { -0.08f, -0.95f, +2.22f }, { +0.08f, +0.85f, +2.28f }, colPropBlack);
        AddBox(sub.vertices, sub.indices, { -0.08f, +0.75f, +2.22f }, { +0.08f, +0.85f, +2.28f }, colSafetyYellow);
        AddBox(sub.vertices, sub.indices, { -0.08f, -0.95f, +2.22f }, { +0.08f, -0.85f, +2.28f }, colSafetyYellow);

        submeshes.push_back(std::move(sub));
    }

    // 10. Landing Gear
    {
        SubmeshData sub{};
        sub.nodeName = "LandingGear";
        sub.materialIndex = 1;

        AddCylinder(sub.vertices, sub.indices, { 0.0f, -0.45f, +1.45f }, 0.04f, 0.40f, 8, colChromeStrut);
        AddBox(sub.vertices, sub.indices, { -0.09f, -0.94f, +1.35f }, { +0.09f, -0.65f, +1.55f }, colRubberTire);
        AddBox(sub.vertices, sub.indices, { -1.18f, -0.94f, -0.45f }, { -0.98f, -0.60f, -0.15f }, colRubberTire);
        AddBox(sub.vertices, sub.indices, { +0.98f, -0.94f, -0.45f }, { +1.18f, -0.60f, -0.15f }, colRubberTire);

        submeshes.push_back(std::move(sub));
    }

    // 11. Cockpit Interior
    {
        SubmeshData sub{};
        sub.nodeName = "Cockpit";
        sub.materialIndex = 3;
        AddBox(sub.vertices, sub.indices, { -0.48f, +0.10f, +0.45f }, { +0.48f, +0.32f, +0.80f }, colDarkCockpit);
        submeshes.push_back(std::move(sub));
    }

    return SerializeSubmeshesToGlb(submeshes, "Cessna 172 Skyhawk");
}

bool AircraftModelGenerator::SaveCessna172Glb(const std::wstring& outputPath) {
    try {
        std::filesystem::path p(outputPath);
        if (p.has_parent_path()) {
            std::filesystem::create_directories(p.parent_path());
        }
        std::vector<u8> glb = GenerateCessna172Glb();
        std::ofstream out(outputPath, std::ios::binary);
        if (!out.is_open()) return false;
        out.write(reinterpret_cast<const char*>(glb.data()), glb.size());
        return true;
    } catch (...) {
        return false;
    }
}

bool AircraftModelGenerator::SaveCessna172Glb(const std::string& outputPath) {
    std::wstring wpath(outputPath.begin(), outputPath.end());
    return SaveCessna172Glb(wpath);
}

// =========================================================================
// 2. Beechcraft Baron 58 Model Generator (Twin-Engine Piston)
// =========================================================================
std::vector<u8> AircraftModelGenerator::GenerateBaron58Glb() {
    const Math::Vec4f colWhite{ 0.95f, 0.95f, 0.97f, 1.0f };
    const Math::Vec4f colNavyBlue{ 0.08f, 0.16f, 0.38f, 1.0f };
    const Math::Vec4f colGold{ 0.85f, 0.72f, 0.20f, 1.0f };
    const Math::Vec4f colDarkGrey{ 0.16f, 0.16f, 0.18f, 1.0f };
    const Math::Vec4f colGlass{ 0.35f, 0.55f, 0.75f, 0.85f };
    const Math::Vec4f colChrome{ 0.78f, 0.80f, 0.82f, 1.0f };

    std::vector<SubmeshData> submeshes;

    // 1. Sleek Twin Fuselage & Cabin
    {
        SubmeshData sub{};
        sub.nodeName = "Fuselage";
        sub.materialIndex = 0;

        // Main cabin body (length ~8.8m)
        AddBox(sub.vertices, sub.indices, { -0.62f, -0.45f, -1.20f }, { +0.62f, +0.65f, +1.50f }, colWhite);
        // Navy blue executive cheatline stripe
        AddBox(sub.vertices, sub.indices, { -0.65f, -0.08f, -1.15f }, { +0.65f, +0.06f, +1.45f }, colNavyBlue);
        // Gold pinstripe
        AddBox(sub.vertices, sub.indices, { -0.66f, +0.06f, -1.15f }, { +0.66f, +0.10f, +1.45f }, colGold);

        // Radar nose cone
        AddBox(sub.vertices, sub.indices, { -0.48f, -0.38f, +1.50f }, { +0.48f, +0.35f, +3.20f }, colWhite);
        AddBox(sub.vertices, sub.indices, { -0.32f, -0.22f, +3.20f }, { +0.32f, +0.20f, +3.70f }, colDarkGrey);

        // Tapered aft fuselage & vertical fin root
        AddBox(sub.vertices, sub.indices, { -0.36f, -0.25f, -4.60f }, { +0.36f, +0.38f, -1.20f }, colWhite);
        AddBox(sub.vertices, sub.indices, { -0.07f, +0.38f, -4.60f }, { +0.07f, +2.10f, -3.20f }, colNavyBlue);
        // Swept horizontal stabilizer root
        AddBox(sub.vertices, sub.indices, { -2.10f, +0.28f, -4.60f }, { +2.10f, +0.42f, -3.60f }, colWhite);

        // Cockpit windshield
        AddBox(sub.vertices, sub.indices, { -0.58f, +0.28f, +0.80f }, { +0.58f, +0.68f, +1.45f }, colGlass);

        submeshes.push_back(std::move(sub));
    }

    // 2. Low Wings (Dihedral Low-Wing Mount)
    {
        SubmeshData sub{};
        sub.nodeName = "Wings";
        sub.materialIndex = 0;

        // Left low wing (span 11.53m)
        AddBox(sub.vertices, sub.indices, { -5.76f, -0.38f, -0.65f }, { -0.60f, -0.22f, +0.85f }, colWhite);
        // Right low wing
        AddBox(sub.vertices, sub.indices, { +0.60f, -0.38f, -0.65f }, { +5.76f, -0.22f, +0.85f }, colWhite);

        // Navy blue wingtips
        AddBox(sub.vertices, sub.indices, { -5.78f, -0.36f, -0.60f }, { -5.60f, -0.24f, +0.80f }, colNavyBlue);
        AddBox(sub.vertices, sub.indices, { +5.60f, -0.36f, -0.60f }, { +5.78f, -0.24f, +0.80f }, colNavyBlue);

        submeshes.push_back(std::move(sub));
    }

    // 3. Left Engine Nacelle (Continental IO-550 @ Y = -1.83m)
    {
        SubmeshData sub{};
        sub.nodeName = "Nacelle_L";
        sub.materialIndex = 0;

        AddBox(sub.vertices, sub.indices, { -2.30f, -0.48f, -0.50f }, { -1.36f, +0.20f, +2.05f }, colWhite);
        // Air intake and oil cooler chin
        AddBox(sub.vertices, sub.indices, { -2.15f, -0.46f, +1.80f }, { -1.50f, -0.25f, +2.10f }, colDarkGrey);

        submeshes.push_back(std::move(sub));
    }

    // 4. Right Engine Nacelle (Continental IO-550 @ Y = +1.83m)
    {
        SubmeshData sub{};
        sub.nodeName = "Nacelle_R";
        sub.materialIndex = 0;

        AddBox(sub.vertices, sub.indices, { +1.36f, -0.48f, -0.50f }, { +2.30f, +0.20f, +2.05f }, colWhite);
        AddBox(sub.vertices, sub.indices, { +1.50f, -0.46f, +1.80f }, { +2.15f, -0.25f, +2.10f }, colDarkGrey);

        submeshes.push_back(std::move(sub));
    }

    // 5. Left Propeller (3-Blade Constant Speed)
    {
        SubmeshData sub{};
        sub.nodeName = "Propeller"; // Primary propeller
        sub.pivot = { -1.83f, -0.12f, +2.10f };
        sub.hingeAxis = { 0.0f, 0.0f, 1.0f };
        sub.materialIndex = 1;

        AddCylinder(sub.vertices, sub.indices, { -1.83f, -0.12f, +2.08f }, 0.20f, 0.24f, 12, colChrome);
        AddBox(sub.vertices, sub.indices, { -1.91f, -1.05f, +2.14f }, { -1.75f, +0.95f, +2.20f }, colDarkGrey);
        AddBox(sub.vertices, sub.indices, { -1.91f, +0.85f, +2.14f }, { -1.75f, +0.95f, +2.20f }, colGold);

        submeshes.push_back(std::move(sub));
    }

    // 6. Right Propeller (3-Blade Constant Speed)
    {
        SubmeshData sub{};
        sub.nodeName = "Propeller_R";
        sub.pivot = { +1.83f, -0.12f, +2.10f };
        sub.hingeAxis = { 0.0f, 0.0f, 1.0f };
        sub.materialIndex = 1;

        AddCylinder(sub.vertices, sub.indices, { +1.83f, -0.12f, +2.08f }, 0.20f, 0.24f, 12, colChrome);
        AddBox(sub.vertices, sub.indices, { +1.75f, -1.05f, +2.14f }, { +1.91f, +0.95f, +2.20f }, colDarkGrey);
        AddBox(sub.vertices, sub.indices, { +1.75f, +0.85f, +2.14f }, { +1.91f, +0.95f, +2.20f }, colGold);

        submeshes.push_back(std::move(sub));
    }

    // 7. Aileron_L
    {
        SubmeshData sub{};
        sub.nodeName = "Aileron_L";
        sub.pivot = { -4.3f, -0.30f, -0.65f };
        sub.hingeAxis = { 1.0f, 0.0f, 0.0f };
        sub.materialIndex = 0;
        AddBox(sub.vertices, sub.indices, { -5.60f, -0.35f, -0.95f }, { -3.00f, -0.25f, -0.65f }, colWhite);
        submeshes.push_back(std::move(sub));
    }

    // 8. Aileron_R
    {
        SubmeshData sub{};
        sub.nodeName = "Aileron_R";
        sub.pivot = { +4.3f, -0.30f, -0.65f };
        sub.hingeAxis = { 1.0f, 0.0f, 0.0f };
        sub.materialIndex = 0;
        AddBox(sub.vertices, sub.indices, { +3.00f, -0.35f, -0.95f }, { +5.60f, -0.25f, -0.65f }, colWhite);
        submeshes.push_back(std::move(sub));
    }

    // 9. Flap_L
    {
        SubmeshData sub{};
        sub.nodeName = "Flap_L";
        sub.pivot = { -1.8f, -0.30f, -0.65f };
        sub.hingeAxis = { 1.0f, 0.0f, 0.0f };
        sub.materialIndex = 0;
        AddBox(sub.vertices, sub.indices, { -2.95f, -0.36f, -1.00f }, { -0.65f, -0.24f, -0.65f }, colWhite);
        submeshes.push_back(std::move(sub));
    }

    // 10. Flap_R
    {
        SubmeshData sub{};
        sub.nodeName = "Flap_R";
        sub.pivot = { +1.8f, -0.30f, -0.65f };
        sub.hingeAxis = { 1.0f, 0.0f, 0.0f };
        sub.materialIndex = 0;
        AddBox(sub.vertices, sub.indices, { +0.65f, -0.36f, -1.00f }, { +2.95f, -0.24f, -0.65f }, colWhite);
        submeshes.push_back(std::move(sub));
    }

    // 11. Elevator
    {
        SubmeshData sub{};
        sub.nodeName = "Elevator";
        sub.pivot = { 0.0f, +0.35f, -4.60f };
        sub.hingeAxis = { 1.0f, 0.0f, 0.0f };
        sub.materialIndex = 0;
        AddBox(sub.vertices, sub.indices, { -2.15f, +0.31f, -5.10f }, { +2.15f, +0.39f, -4.60f }, colWhite);
        submeshes.push_back(std::move(sub));
    }

    // 12. Rudder
    {
        SubmeshData sub{};
        sub.nodeName = "Rudder";
        sub.pivot = { 0.0f, +0.40f, -4.65f };
        sub.hingeAxis = { 0.0f, 1.0f, 0.0f };
        sub.materialIndex = 0;
        AddBox(sub.vertices, sub.indices, { -0.07f, +0.42f, -5.20f }, { +0.07f, +2.18f, -4.65f }, colNavyBlue);
        submeshes.push_back(std::move(sub));
    }

    // 13. Retractable Nose Landing Gear
    {
        SubmeshData sub{};
        sub.nodeName = "Gear_Nose";
        sub.materialIndex = 1;
        AddCylinder(sub.vertices, sub.indices, { 0.0f, -0.42f, +2.40f }, 0.045f, 0.45f, 8, colChrome);
        AddBox(sub.vertices, sub.indices, { -0.10f, -0.95f, +2.30f }, { +0.10f, -0.60f, +2.52f }, colDarkGrey);
        submeshes.push_back(std::move(sub));
    }

    // 14. Retractable Left Main Landing Gear (under wing/nacelle)
    {
        SubmeshData sub{};
        sub.nodeName = "Gear_L";
        sub.materialIndex = 1;
        AddCylinder(sub.vertices, sub.indices, { -1.83f, -0.42f, +0.20f }, 0.055f, 0.48f, 8, colChrome);
        AddBox(sub.vertices, sub.indices, { -1.95f, -0.96f, +0.08f }, { -1.71f, -0.58f, +0.32f }, colDarkGrey);
        submeshes.push_back(std::move(sub));
    }

    // 15. Retractable Right Main Landing Gear
    {
        SubmeshData sub{};
        sub.nodeName = "Gear_R";
        sub.materialIndex = 1;
        AddCylinder(sub.vertices, sub.indices, { +1.83f, -0.42f, +0.20f }, 0.055f, 0.48f, 8, colChrome);
        AddBox(sub.vertices, sub.indices, { +1.71f, -0.96f, +0.08f }, { +1.95f, -0.58f, +0.32f }, colDarkGrey);
        submeshes.push_back(std::move(sub));
    }

    return SerializeSubmeshesToGlb(submeshes, "Beechcraft Baron 58");
}

bool AircraftModelGenerator::SaveBaron58Glb(const std::wstring& outputPath) {
    try {
        std::filesystem::path p(outputPath);
        if (p.has_parent_path()) {
            std::filesystem::create_directories(p.parent_path());
        }
        std::vector<u8> glb = GenerateBaron58Glb();
        std::ofstream out(outputPath, std::ios::binary);
        if (!out.is_open()) return false;
        out.write(reinterpret_cast<const char*>(glb.data()), glb.size());
        return true;
    } catch (...) {
        return false;
    }
}

bool AircraftModelGenerator::SaveBaron58Glb(const std::string& outputPath) {
    std::wstring wpath(outputPath.begin(), outputPath.end());
    return SaveBaron58Glb(wpath);
}

// =========================================================================
// 3. Extra 330SC Model Generator (Unlimited Competition Aerobatic)
// =========================================================================
std::vector<u8> AircraftModelGenerator::GenerateExtra330Glb() {
    const Math::Vec4f colRacingRed{ 0.90f, 0.12f, 0.10f, 1.0f };
    const Math::Vec4f colPureWhite{ 0.98f, 0.98f, 0.98f, 1.0f };
    const Math::Vec4f colCarbonBlack{ 0.10f, 0.10f, 0.12f, 1.0f };
    const Math::Vec4f colClearCanopy{ 0.20f, 0.35f, 0.50f, 0.70f };
    const Math::Vec4f colChrome{ 0.80f, 0.80f, 0.82f, 1.0f };

    std::vector<SubmeshData> submeshes;

    // 1. Compact Aerobatic Monocoque Fuselage
    {
        SubmeshData sub{};
        sub.nodeName = "Fuselage";
        sub.materialIndex = 0;

        // Slender fuselage (length ~6.9m)
        AddBox(sub.vertices, sub.indices, { -0.42f, -0.42f, -0.80f }, { +0.42f, +0.45f, +1.20f }, colRacingRed);
        AddBox(sub.vertices, sub.indices, { -0.44f, -0.15f, -0.75f }, { +0.44f, +0.10f, +1.15f }, colPureWhite);

        // Aerobatic engine cowl & spinner mount
        AddBox(sub.vertices, sub.indices, { -0.38f, -0.38f, +1.20f }, { +0.38f, +0.38f, +2.20f }, colRacingRed);
        AddBox(sub.vertices, sub.indices, { -0.35f, -0.35f, +2.18f }, { +0.35f, -0.10f, +2.25f }, colCarbonBlack); // Chin scoop

        // Tapered aft fuselage & vertical fin root
        AddBox(sub.vertices, sub.indices, { -0.20f, -0.22f, -3.60f }, { +0.20f, +0.32f, -0.80f }, colRacingRed);
        // Oversized vertical tail fin (knife edge authority)
        AddBox(sub.vertices, sub.indices, { -0.05f, +0.32f, -3.60f }, { +0.05f, +2.15f, -2.40f }, colRacingRed);
        AddBox(sub.vertices, sub.indices, { -0.06f, +0.80f, -3.55f }, { +0.06f, +1.80f, -2.45f }, colPureWhite); // Sunburst stripe
        // Symmetrical horizontal stabilizer root
        AddBox(sub.vertices, sub.indices, { -1.50f, +0.08f, -3.60f }, { +1.50f, +0.20f, -2.80f }, colRacingRed);

        // Clear bubble teardrop canopy
        AddBox(sub.vertices, sub.indices, { -0.35f, +0.42f, -0.60f }, { +0.35f, +0.82f, +0.95f }, colClearCanopy);

        submeshes.push_back(std::move(sub));
    }

    // 2. Mid-Wing Symmetrical Wings (Zero Dihedral, Symmetrical Airfoil)
    {
        SubmeshData sub{};
        sub.nodeName = "Wings";
        sub.materialIndex = 0;

        // Left symmetrical wing (span 7.5m)
        AddBox(sub.vertices, sub.indices, { -3.75f, -0.10f, -0.45f }, { -0.40f, +0.10f, +0.70f }, colRacingRed);
        AddBox(sub.vertices, sub.indices, { -3.65f, -0.09f, -0.10f }, { -0.45f, +0.09f, +0.30f }, colPureWhite); // Top wing chevron

        // Right symmetrical wing
        AddBox(sub.vertices, sub.indices, { +0.40f, -0.10f, -0.45f }, { +3.75f, +0.10f, +0.70f }, colRacingRed);
        AddBox(sub.vertices, sub.indices, { +0.45f, -0.09f, -0.10f }, { +3.65f, +0.09f, +0.30f }, colPureWhite);

        // Squared competition carbon wingtips
        AddBox(sub.vertices, sub.indices, { -3.76f, -0.12f, -0.40f }, { -3.68f, +0.12f, +0.65f }, colCarbonBlack);
        AddBox(sub.vertices, sub.indices, { +3.68f, -0.12f, -0.40f }, { +3.76f, +0.12f, +0.65f }, colCarbonBlack);

        submeshes.push_back(std::move(sub));
    }

    // 3. Full-Span Aileron_L (>400 deg/s roll rate authority)
    {
        SubmeshData sub{};
        sub.nodeName = "Aileron_L";
        sub.pivot = { -2.2f, 0.0f, -0.45f };
        sub.hingeAxis = { 1.0f, 0.0f, 0.0f };
        sub.materialIndex = 0;
        AddBox(sub.vertices, sub.indices, { -3.65f, -0.08f, -0.80f }, { -0.50f, +0.08f, -0.45f }, colPureWhite);
        submeshes.push_back(std::move(sub));
    }

    // 4. Full-Span Aileron_R
    {
        SubmeshData sub{};
        sub.nodeName = "Aileron_R";
        sub.pivot = { +2.2f, 0.0f, -0.45f };
        sub.hingeAxis = { 1.0f, 0.0f, 0.0f };
        sub.materialIndex = 0;
        AddBox(sub.vertices, sub.indices, { +0.50f, -0.08f, -0.80f }, { +3.65f, +0.08f, -0.45f }, colPureWhite);
        submeshes.push_back(std::move(sub));
    }

    // 5. Symmetrical Elevator
    {
        SubmeshData sub{};
        sub.nodeName = "Elevator";
        sub.pivot = { 0.0f, +0.14f, -3.60f };
        sub.hingeAxis = { 1.0f, 0.0f, 0.0f };
        sub.materialIndex = 0;
        AddBox(sub.vertices, sub.indices, { -1.55f, +0.09f, -4.05f }, { +1.55f, +0.19f, -3.60f }, colPureWhite);
        submeshes.push_back(std::move(sub));
    }

    // 6. Oversized Rudder
    {
        SubmeshData sub{};
        sub.nodeName = "Rudder";
        sub.pivot = { 0.0f, +0.32f, -3.60f };
        sub.hingeAxis = { 0.0f, 1.0f, 0.0f };
        sub.materialIndex = 0;
        AddBox(sub.vertices, sub.indices, { -0.06f, +0.34f, -4.15f }, { +0.06f, +2.20f, -3.60f }, colPureWhite);
        submeshes.push_back(std::move(sub));
    }

    // 7. 3-Blade Composite Propeller with Racing Red Spinner
    {
        SubmeshData sub{};
        sub.nodeName = "Propeller";
        sub.pivot = { 0.0f, 0.0f, +2.25f };
        sub.hingeAxis = { 0.0f, 0.0f, 1.0f };
        sub.materialIndex = 1;

        // Pointed aerodynamic spinner
        AddCylinder(sub.vertices, sub.indices, { 0.0f, 0.0f, +2.20f }, 0.22f, 0.28f, 12, colRacingRed);
        // 3 composite scimitar blades
        AddBox(sub.vertices, sub.indices, { -0.07f, -1.02f, +2.25f }, { +0.07f, +0.98f, +2.30f }, colCarbonBlack);
        AddBox(sub.vertices, sub.indices, { -1.00f, -0.06f, +2.25f }, { +0.00f, +0.06f, +2.30f }, colCarbonBlack);
        AddBox(sub.vertices, sub.indices, { -0.07f, +0.88f, +2.25f }, { +0.07f, +0.98f, +2.30f }, colPureWhite);

        submeshes.push_back(std::move(sub));
    }

    // 8. Fixed Lightweight Landing Gear with Teardrop Wheel Fairings (Pants)
    {
        SubmeshData sub{};
        sub.nodeName = "Gear";
        sub.materialIndex = 1;

        // Composite main legs
        AddCylinder(sub.vertices, sub.indices, { -0.35f, -0.38f, +0.40f }, 0.035f, 0.65f, 8, colChrome);
        AddCylinder(sub.vertices, sub.indices, { +0.35f, -0.38f, +0.40f }, 0.035f, 0.65f, 8, colChrome);

        // Teardrop aerodynamic wheel fairings
        AddBox(sub.vertices, sub.indices, { -1.18f, -0.92f, +0.25f }, { -0.98f, -0.58f, +0.65f }, colRacingRed);
        AddBox(sub.vertices, sub.indices, { +0.98f, -0.92f, +0.25f }, { +1.18f, -0.58f, +0.65f }, colRacingRed);
        // Exposed rubber tire bottoms
        AddBox(sub.vertices, sub.indices, { -1.15f, -0.96f, +0.35f }, { -1.01f, -0.80f, +0.55f }, colCarbonBlack);
        AddBox(sub.vertices, sub.indices, { +1.01f, -0.96f, +0.35f }, { +1.15f, -0.80f, +0.55f }, colCarbonBlack);

        submeshes.push_back(std::move(sub));
    }

    return SerializeSubmeshesToGlb(submeshes, "Extra 330SC");
}

bool AircraftModelGenerator::SaveExtra330Glb(const std::wstring& outputPath) {
    try {
        std::filesystem::path p(outputPath);
        if (p.has_parent_path()) {
            std::filesystem::create_directories(p.parent_path());
        }
        std::vector<u8> glb = GenerateExtra330Glb();
        std::ofstream out(outputPath, std::ios::binary);
        if (!out.is_open()) return false;
        out.write(reinterpret_cast<const char*>(glb.data()), glb.size());
        return true;
    } catch (...) {
        return false;
    }
}

bool AircraftModelGenerator::SaveExtra330Glb(const std::string& outputPath) {
    std::wstring wpath(outputPath.begin(), outputPath.end());
    return SaveExtra330Glb(wpath);
}

} // namespace LokYS::Graphics
