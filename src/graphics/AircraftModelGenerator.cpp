#include "LokYS/graphics/AircraftModelGenerator.hpp"
#include "LokYS/graphics/GltfTypes.hpp"
#include "LokYS/core/JsonParser.hpp"
#include "LokYS/graphics/Mesh.hpp"
#include "LokYS/math/Vector3.hpp"
#include <fstream>
#include <cmath>
#include <cstring>
#include <filesystem>

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
    // Top (+Y)
    AddQuad(verts, inds,
        { min.x, max.y, max.z }, { max.x, max.y, max.z },
        { max.x, max.y, min.z }, { min.x, max.y, min.z }, color);
    // Bottom (-Y)
    AddQuad(verts, inds,
        { min.x, min.y, min.z }, { max.x, min.y, min.z },
        { max.x, min.y, max.z }, { min.x, min.y, max.z }, color);
    // Right (+X)
    AddQuad(verts, inds,
        { max.x, min.y, max.z }, { max.x, min.y, min.z },
        { max.x, max.y, min.z }, { max.x, max.y, max.z }, color);
    // Left (-X)
    AddQuad(verts, inds,
        { min.x, min.y, min.z }, { min.x, min.y, max.z },
        { min.x, max.y, max.z }, { min.x, max.y, min.z }, color);
}

// Helper to add a cylinder/wheel
void AddCylinder(
    std::vector<Vertex3D>& verts,
    std::vector<u32>& inds,
    const Math::Vec3f& center,
    f32 radius,
    f32 width,
    int segments,
    const Math::Vec4f& color,
    int axis = 0)
{
    f32 halfW = width * 0.5f;

    for (int i = 0; i < segments; ++i) {
        f32 a0 = (static_cast<f32>(i) / segments) * 6.2831853f;
        f32 a1 = (static_cast<f32>(i + 1) / segments) * 6.2831853f;

        f32 c0 = std::cos(a0), s0 = std::sin(a0);
        f32 c1 = std::cos(a1), s1 = std::sin(a1);

        Math::Vec3f p0, p1, p2, p3;
        if (axis == 0) { // Wheel rolling around X axis
            p0 = center + Math::Vec3f(-halfW, radius * s0, radius * c0);
            p1 = center + Math::Vec3f(+halfW, radius * s0, radius * c0);
            p2 = center + Math::Vec3f(+halfW, radius * s1, radius * c1);
            p3 = center + Math::Vec3f(-halfW, radius * s1, radius * c1);
        } else {
            p0 = center + Math::Vec3f(radius * c0, -halfW, radius * s0);
            p1 = center + Math::Vec3f(radius * c0, +halfW, radius * s0);
            p2 = center + Math::Vec3f(radius * c1, +halfW, radius * s1);
            p3 = center + Math::Vec3f(radius * c1, -halfW, radius * s1);
        }

        AddQuad(verts, inds, p0, p1, p2, p3, color);
    }
}

} // namespace

std::vector<u8> AircraftModelGenerator::GenerateCessna172Glb() {
    // Colors
    const Math::Vec4f colAviationWhite{ 0.94f, 0.95f, 0.97f, 1.0f };
    const Math::Vec4f colAeroBlue{ 0.12f, 0.30f, 0.65f, 1.0f };
    const Math::Vec4f colGoldTrim{ 0.85f, 0.70f, 0.15f, 1.0f };
    const Math::Vec4f colDarkCockpit{ 0.15f, 0.16f, 0.18f, 1.0f };
    const Math::Vec4f colGlassTint{ 0.40f, 0.60f, 0.80f, 0.85f };
    const Math::Vec4f colRubberTire{ 0.12f, 0.12f, 0.12f, 1.0f };
    const Math::Vec4f colChromeStrut{ 0.75f, 0.76f, 0.78f, 1.0f };
    const Math::Vec4f colPropBlack{ 0.10f, 0.10f, 0.10f, 1.0f };
    const Math::Vec4f colSafetyYellow{ 0.95f, 0.85f, 0.10f, 1.0f };

    // Structure for generating submesh data
    struct SubmeshData {
        std::string nodeName;
        std::vector<Vertex3D> vertices;
        std::vector<u32> indices;
        Math::Vec3f translation{ 0.0f, 0.0f, 0.0f };
        Math::Vec3f pivot{ 0.0f, 0.0f, 0.0f };
        Math::Vec3f hingeAxis{ 1.0f, 0.0f, 0.0f };
        i32 materialIndex{ 0 };
    };

    std::vector<SubmeshData> submeshes;

    // 1. Fuselage
    {
        SubmeshData sub{};
        sub.nodeName = "Fuselage";
        sub.materialIndex = 0; // White / Blue livery

        // Main cabin body
        AddBox(sub.vertices, sub.indices,
            { -0.55f, -0.45f, -1.8f },
            { +0.55f, +0.65f, +0.8f },
            colAviationWhite);

        // Cabin windshield glass (sloped front)
        AddQuad(sub.vertices, sub.indices,
            { -0.50f, +0.65f, +0.78f },
            { +0.50f, +0.65f, +0.78f },
            { +0.45f, +0.25f, +1.25f },
            { -0.45f, +0.25f, +1.25f },
            colGlassTint);

        // Engine Cowl / Nose
        AddBox(sub.vertices, sub.indices,
            { -0.45f, -0.38f, +0.8f },
            { +0.45f, +0.25f, +2.1f },
            colAviationWhite);

        // Livery side stripes (Blue & Gold)
        AddQuad(sub.vertices, sub.indices,
            { +0.552f, +0.05f, -1.8f }, { +0.552f, +0.05f, +0.8f },
            { +0.552f, +0.15f, +0.8f }, { +0.552f, +0.15f, -1.8f },
            colAeroBlue);
        AddQuad(sub.vertices, sub.indices,
            { -0.552f, +0.05f, +0.8f }, { -0.552f, +0.05f, -1.8f },
            { -0.552f, +0.15f, -1.8f }, { -0.552f, +0.15f, +0.8f },
            colAeroBlue);

        // Rear fuselage tapered boom
        AddBox(sub.vertices, sub.indices,
            { -0.35f, -0.25f, -4.4f },
            { +0.35f, +0.40f, -1.8f },
            colAviationWhite);

        submeshes.push_back(std::move(sub));
    }

    // 2. Wings (Main fixed high-wing + stabilizers)
    {
        SubmeshData sub{};
        sub.nodeName = "Wings";
        sub.materialIndex = 0;

        // Center Wing Section (over cabin)
        AddBox(sub.vertices, sub.indices,
            { -0.60f, +0.65f, -0.8f },
            { +0.60f, +0.82f, +0.6f },
            colAviationWhite);

        // Left Main Wing (Fixed portion up to aileron & flap bays)
        AddBox(sub.vertices, sub.indices,
            { -5.40f, +0.68f, -0.35f },
            { -0.60f, +0.82f, +0.60f },
            colAviationWhite);

        // Right Main Wing (Fixed portion)
        AddBox(sub.vertices, sub.indices,
            { +0.60f, +0.68f, -0.35f },
            { +5.40f, +0.82f, +0.60f },
            colAviationWhite);

        // Wingtips
        AddBox(sub.vertices, sub.indices,
            { -5.50f, +0.67f, -0.40f },
            { -5.40f, +0.83f, +0.60f },
            colAeroBlue);
        AddBox(sub.vertices, sub.indices,
            { +5.40f, +0.67f, -0.40f },
            { +5.50f, +0.83f, +0.60f },
            colAeroBlue);

        // Wing Struts (left & right)
        AddBox(sub.vertices, sub.indices,
            { -2.25f, +0.10f, +0.15f },
            { -2.15f, +0.70f, +0.22f },
            colChromeStrut);
        AddBox(sub.vertices, sub.indices,
            { +2.15f, +0.10f, +0.15f },
            { +2.25f, +0.70f, +0.22f },
            colChromeStrut);

        // Horizontal Stabilizer (Fixed forward section)
        AddBox(sub.vertices, sub.indices,
            { -1.75f, +0.32f, -4.45f },
            { +1.75f, +0.40f, -3.80f },
            colAviationWhite);

        // Vertical Fin (Fixed swept fin)
        AddBox(sub.vertices, sub.indices,
            { -0.08f, +0.38f, -4.50f },
            { +0.08f, +1.95f, -3.90f },
            colAeroBlue);

        submeshes.push_back(std::move(sub));
    }

    // 3. Aileron_L (Articulated Left Wing Trailing Edge)
    {
        SubmeshData sub{};
        sub.nodeName = "Aileron_L";
        sub.pivot = { -4.0f, +0.75f, -0.35f };
        sub.hingeAxis = { 1.0f, 0.0f, 0.0f };
        sub.materialIndex = 0;

        AddBox(sub.vertices, sub.indices,
            { -5.35f, +0.71f, -0.75f },
            { -2.80f, +0.79f, -0.35f },
            colAviationWhite);

        submeshes.push_back(std::move(sub));
    }

    // 4. Aileron_R (Articulated Right Wing Trailing Edge)
    {
        SubmeshData sub{};
        sub.nodeName = "Aileron_R";
        sub.pivot = { +4.0f, +0.75f, -0.35f };
        sub.hingeAxis = { 1.0f, 0.0f, 0.0f };
        sub.materialIndex = 0;

        AddBox(sub.vertices, sub.indices,
            { +2.80f, +0.71f, -0.75f },
            { +5.35f, +0.79f, -0.35f },
            colAviationWhite);

        submeshes.push_back(std::move(sub));
    }

    // 5. Flap_L (Articulated Left Wing Inboard Trailing Edge)
    {
        SubmeshData sub{};
        sub.nodeName = "Flap_L";
        sub.pivot = { -1.7f, +0.75f, -0.35f };
        sub.hingeAxis = { 1.0f, 0.0f, 0.0f };
        sub.materialIndex = 0;

        AddBox(sub.vertices, sub.indices,
            { -2.75f, +0.70f, -0.80f },
            { -0.65f, +0.80f, -0.35f },
            colAviationWhite);

        submeshes.push_back(std::move(sub));
    }

    // 6. Flap_R (Articulated Right Wing Inboard Trailing Edge)
    {
        SubmeshData sub{};
        sub.nodeName = "Flap_R";
        sub.pivot = { +1.7f, +0.75f, -0.35f };
        sub.hingeAxis = { 1.0f, 0.0f, 0.0f };
        sub.materialIndex = 0;

        AddBox(sub.vertices, sub.indices,
            { +0.65f, +0.70f, -0.80f },
            { +2.75f, +0.80f, -0.35f },
            colAviationWhite);

        submeshes.push_back(std::move(sub));
    }

    // 7. Elevator (Articulated Tail Pitch Control)
    {
        SubmeshData sub{};
        sub.nodeName = "Elevator";
        sub.pivot = { 0.0f, +0.36f, -4.45f };
        sub.hingeAxis = { 1.0f, 0.0f, 0.0f }; // Pitch axis
        sub.materialIndex = 0;

        AddBox(sub.vertices, sub.indices,
            { -1.72f, +0.33f, -4.92f },
            { +1.72f, +0.39f, -4.45f },
            colAviationWhite);

        submeshes.push_back(std::move(sub));
    }

    // 8. Rudder (Articulated Tail Yaw Control)
    {
        SubmeshData sub{};
        sub.nodeName = "Rudder";
        sub.pivot = { 0.0f, +0.38f, -4.50f };
        sub.hingeAxis = { 0.0f, 1.0f, 0.0f }; // Yaw axis
        sub.materialIndex = 0;

        AddBox(sub.vertices, sub.indices,
            { -0.06f, +0.40f, -4.95f },
            { +0.06f, +1.92f, -4.50f },
            colAeroBlue);

        submeshes.push_back(std::move(sub));
    }

    // 9. Propeller (Articulated Continuous Spinner + Blades)
    {
        SubmeshData sub{};
        sub.nodeName = "Propeller";
        sub.pivot = { 0.0f, -0.05f, +2.15f };
        sub.hingeAxis = { 0.0f, 0.0f, 1.0f }; // Roll / thrust axis
        sub.materialIndex = 1; // Black / Yellow tips

        // Spinner Cone
        AddBox(sub.vertices, sub.indices,
            { -0.16f, -0.21f, +2.10f },
            { +0.16f, +0.11f, +2.35f },
            colChromeStrut);

        // Blade 1 (Upper)
        AddBox(sub.vertices, sub.indices,
            { -0.06f, +0.11f, +2.18f },
            { +0.06f, +0.82f, +2.22f },
            colPropBlack);
        // Blade 1 Yellow Tip
        AddBox(sub.vertices, sub.indices,
            { -0.06f, +0.82f, +2.18f },
            { +0.06f, +0.95f, +2.22f },
            colSafetyYellow);

        // Blade 2 (Lower)
        AddBox(sub.vertices, sub.indices,
            { -0.06f, -0.92f, +2.18f },
            { +0.06f, -0.21f, +2.22f },
            colPropBlack);
        // Blade 2 Yellow Tip
        AddBox(sub.vertices, sub.indices,
            { -0.06f, -1.05f, +2.18f },
            { +0.06f, -0.92f, +2.22f },
            colSafetyYellow);

        submeshes.push_back(std::move(sub));
    }

    // 10. LandingGear (Tricycle Gear, Struts & Tires)
    {
        SubmeshData sub{};
        sub.nodeName = "LandingGear";
        sub.materialIndex = 2; // Tires / Chrome

        // Nose gear strut
        AddBox(sub.vertices, sub.indices,
            { -0.04f, -0.80f, +1.40f },
            { +0.04f, -0.40f, +1.48f },
            colChromeStrut);
        // Nose wheel tire
        AddCylinder(sub.vertices, sub.indices, { 0.0f, -0.80f, +1.44f }, 0.16f, 0.12f, 12, colRubberTire, 0);

        // Left main gear spring steel strut
        AddBox(sub.vertices, sub.indices,
            { -1.15f, -0.80f, -0.22f },
            { -0.50f, -0.40f, -0.16f },
            colAviationWhite);
        // Left main wheel tire
        AddCylinder(sub.vertices, sub.indices, { -1.15f, -0.80f, -0.19f }, 0.22f, 0.15f, 14, colRubberTire, 0);

        // Right main gear spring steel strut
        AddBox(sub.vertices, sub.indices,
            { +0.50f, -0.80f, -0.22f },
            { +1.15f, -0.40f, -0.16f },
            colAviationWhite);
        // Right main wheel tire
        AddCylinder(sub.vertices, sub.indices, { +1.15f, -0.80f, -0.19f }, 0.22f, 0.15f, 14, colRubberTire, 0);

        submeshes.push_back(std::move(sub));
    }

    // 11. Cockpit Interior (Dashboard & Seats)
    {
        SubmeshData sub{};
        sub.nodeName = "Cockpit";
        sub.materialIndex = 3; // Dark cockpit matte

        // Instrument Panel Glare Shield
        AddBox(sub.vertices, sub.indices,
            { -0.48f, +0.10f, +0.45f },
            { +0.48f, +0.32f, +0.80f },
            colDarkCockpit);

        // Pilot Seat (Left)
        AddBox(sub.vertices, sub.indices,
            { -0.42f, -0.35f, -0.40f },
            { -0.08f, +0.20f, -0.10f },
            colDarkCockpit);

        // Copilot Seat (Right)
        AddBox(sub.vertices, sub.indices,
            { +0.08f, -0.35f, -0.40f },
            { +0.42f, +0.20f, -0.10f },
            colDarkCockpit);

        submeshes.push_back(std::move(sub));
    }

    // =========================================================================
    // Pack Binary Buffer Chunk (BIN)
    // =========================================================================
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

    // =========================================================================
    // Build JSON Manifest Chunk
    // =========================================================================
    Core::JsonValue jsonRoot(std::unordered_map<std::string, Core::JsonValue>{});

    // Asset
    {
        std::unordered_map<std::string, Core::JsonValue> asset;
        asset["version"] = Core::JsonValue("2.0");
        asset["generator"] = Core::JsonValue("LokYS Native Aircraft Model Generator");
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
                Core::JsonValue(col.x), Core::JsonValue(col.y), Core::JsonValue(col.z), Core::JsonValue(col.w)
            };
            pbr["baseColorFactor"] = Core::JsonValue(std::move(baseCol));
            pbr["metallicFactor"] = Core::JsonValue(met);
            pbr["roughnessFactor"] = Core::JsonValue(rough);
            m["pbrMetallicRoughness"] = Core::JsonValue(std::move(pbr));
            materials.push_back(Core::JsonValue(std::move(m)));
        };

        AddMat("WhiteLivery", colAviationWhite, 0.15f, 0.35f);
        AddMat("PropellerMaterial", colPropBlack, 0.60f, 0.40f);
        AddMat("LandingGearMaterial", colChromeStrut, 0.80f, 0.20f);
        AddMat("CockpitMaterial", colDarkCockpit, 0.05f, 0.85f);

        jsonRoot["materials"] = Core::JsonValue(std::move(materials));
    }

    // BufferViews
    {
        std::vector<Core::JsonValue> bvArr;
        for (const auto& bv : bufferViews) {
            std::unordered_map<std::string, Core::JsonValue> b;
            b["buffer"] = Core::JsonValue(0);
            b["byteOffset"] = Core::JsonValue(static_cast<u64>(bv.byteOffset));
            b["byteLength"] = Core::JsonValue(static_cast<u64>(bv.byteLength));
            if (bv.target != 0) {
                b["target"] = Core::JsonValue(bv.target);
            }
            bvArr.push_back(Core::JsonValue(std::move(b)));
        }
        jsonRoot["bufferViews"] = Core::JsonValue(std::move(bvArr));
    }

    // Accessors
    {
        std::vector<Core::JsonValue> accArr;
        for (const auto& acc : accessors) {
            std::unordered_map<std::string, Core::JsonValue> a;
            a["bufferView"] = Core::JsonValue(static_cast<int>(acc.bufferViewIndex));
            a["byteOffset"] = Core::JsonValue(static_cast<u64>(acc.byteOffset));
            a["componentType"] = Core::JsonValue(static_cast<u32>(acc.compType));
            a["count"] = Core::JsonValue(static_cast<u64>(acc.count));
            a["type"] = Core::JsonValue(acc.type);

            if (!acc.minVals.empty()) {
                std::vector<Core::JsonValue> minArr;
                for (f32 v : acc.minVals) minArr.push_back(Core::JsonValue(v));
                a["min"] = Core::JsonValue(std::move(minArr));
            }
            if (!acc.maxVals.empty()) {
                std::vector<Core::JsonValue> maxArr;
                for (f32 v : acc.maxVals) maxArr.push_back(Core::JsonValue(v));
                a["max"] = Core::JsonValue(std::move(maxArr));
            }

            accArr.push_back(Core::JsonValue(std::move(a)));
        }
        jsonRoot["accessors"] = Core::JsonValue(std::move(accArr));
    }

    // Meshes
    {
        std::vector<Core::JsonValue> meshesArr;
        for (size_t i = 0; i < submeshes.size(); ++i) {
            const auto& sub = submeshes[i];
            const auto& meta = meshPrimMetas[i];

            std::unordered_map<std::string, Core::JsonValue> mesh;
            mesh["name"] = Core::JsonValue(sub.nodeName + "_Mesh");

            std::vector<Core::JsonValue> prims;
            std::unordered_map<std::string, Core::JsonValue> prim;

            std::unordered_map<std::string, Core::JsonValue> attrs;
            attrs["POSITION"] = Core::JsonValue(static_cast<int>(meta.posAccessorIdx));
            attrs["NORMAL"] = Core::JsonValue(static_cast<int>(meta.normAccessorIdx));
            attrs["COLOR_0"] = Core::JsonValue(static_cast<int>(meta.colAccessorIdx));
            prim["attributes"] = Core::JsonValue(std::move(attrs));

            prim["indices"] = Core::JsonValue(static_cast<int>(meta.indAccessorIdx));
            prim["material"] = Core::JsonValue(meta.materialIdx);

            prims.push_back(Core::JsonValue(std::move(prim)));
            mesh["primitives"] = Core::JsonValue(std::move(prims));

            meshesArr.push_back(Core::JsonValue(std::move(mesh)));
        }
        jsonRoot["meshes"] = Core::JsonValue(std::move(meshesArr));
    }

    // Nodes (1 root node + 11 child articulated nodes)
    {
        std::vector<Core::JsonValue> nodesArr;

        // Node 0: Root "Cessna172_Root"
        std::unordered_map<std::string, Core::JsonValue> rootNode;
        rootNode["name"] = Core::JsonValue("Cessna172_Root");
        std::vector<Core::JsonValue> rootChildren;
        for (size_t i = 0; i < submeshes.size(); ++i) {
            rootChildren.push_back(Core::JsonValue(static_cast<int>(i + 1)));
        }
        rootNode["children"] = Core::JsonValue(std::move(rootChildren));
        nodesArr.push_back(Core::JsonValue(std::move(rootNode)));

        // Nodes 1..N: Submesh articulated parts
        for (size_t i = 0; i < submeshes.size(); ++i) {
            const auto& sub = submeshes[i];
            std::unordered_map<std::string, Core::JsonValue> node;
            node["name"] = Core::JsonValue(sub.nodeName);
            node["mesh"] = Core::JsonValue(static_cast<int>(i));

            if (sub.translation.x != 0.0f || sub.translation.y != 0.0f || sub.translation.z != 0.0f) {
                std::vector<Core::JsonValue> t{
                    Core::JsonValue(sub.translation.x), Core::JsonValue(sub.translation.y), Core::JsonValue(sub.translation.z)
                };
                node["translation"] = Core::JsonValue(std::move(t));
            }

            nodesArr.push_back(Core::JsonValue(std::move(node)));
        }

        jsonRoot["nodes"] = Core::JsonValue(std::move(nodesArr));
    }

    // Scenes & Default Scene
    {
        std::vector<Core::JsonValue> scenesArr;
        std::unordered_map<std::string, Core::JsonValue> scene;
        scene["name"] = Core::JsonValue("DefaultScene");
        std::vector<Core::JsonValue> sceneNodes{ Core::JsonValue(0) }; // Root node 0
        scene["nodes"] = Core::JsonValue(std::move(sceneNodes));
        scenesArr.push_back(Core::JsonValue(std::move(scene)));

        jsonRoot["scenes"] = Core::JsonValue(std::move(scenesArr));
        jsonRoot["scene"] = Core::JsonValue(0);
    }

    // Buffers
    {
        std::vector<Core::JsonValue> buffersArr;
        std::unordered_map<std::string, Core::JsonValue> b;
        b["byteLength"] = Core::JsonValue(static_cast<u64>(binBuffer.size()));
        buffersArr.push_back(Core::JsonValue(std::move(b)));
        jsonRoot["buffers"] = Core::JsonValue(std::move(buffersArr));
    }

    std::string jsonStr = jsonRoot.Serialize(false);
    // Pad JSON string with spaces (0x20) to 4-byte boundary
    while ((jsonStr.length() % 4) != 0) {
        jsonStr.push_back(' ');
    }

    // =========================================================================
    // Construct GLB Container
    // =========================================================================
    u32 jsonChunkLen = static_cast<u32>(jsonStr.length());
    u32 binChunkLen = static_cast<u32>(binBuffer.size());
    u32 totalLength = 12 + 8 + jsonChunkLen + 8 + binChunkLen;

    std::vector<u8> glb(totalLength);
    size_t outOffset = 0;

    // 1. Header (12 bytes)
    u32 magic = 0x46546C67; // "glTF"
    u32 version = 2;
    std::memcpy(glb.data() + 0, &magic, 4);
    std::memcpy(glb.data() + 4, &version, 4);
    std::memcpy(glb.data() + 8, &totalLength, 4);
    outOffset += 12;

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

} // namespace LokYS::Graphics
