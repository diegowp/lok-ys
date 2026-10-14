#include "LokYS/graphics/GltfModel.hpp"
#include <fstream>
#include <iostream>
#include <cmath>
#include <cstring>
#include <algorithm>

namespace LokYS::Graphics {

// =========================================================================
// GltfPrimitive Implementation
// =========================================================================

bool GltfPrimitive::CreateGPUResources(ID3D12Device* device) {
    if (!device || vertices.empty() || indices.empty()) {
        return false;
    }

    vertexCount = static_cast<u32>(vertices.size());
    indexCount = static_cast<u32>(indices.size());

    const u64 vbSize = vertices.size() * sizeof(Vertex3D);
    const u64 ibSize = indices.size() * sizeof(u32);

    D3D12_HEAP_PROPERTIES uploadHeap{};
    uploadHeap.Type = D3D12_HEAP_TYPE_UPLOAD;
    uploadHeap.CreationNodeMask = 1;
    uploadHeap.VisibleNodeMask = 1;

    // 1. Vertex Buffer
    D3D12_RESOURCE_DESC vbDesc{};
    vbDesc.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER;
    vbDesc.Width = vbSize;
    vbDesc.Height = 1;
    vbDesc.DepthOrArraySize = 1;
    vbDesc.MipLevels = 1;
    vbDesc.SampleDesc.Count = 1;
    vbDesc.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;

    if (FAILED(device->CreateCommittedResource(
            &uploadHeap,
            D3D12_HEAP_FLAG_NONE,
            &vbDesc,
            D3D12_RESOURCE_STATE_GENERIC_READ,
            nullptr,
            IID_PPV_ARGS(&vertexBuffer)))) {
        return false;
    }

    void* vbData = nullptr;
    D3D12_RANGE readRange{ 0, 0 };
    if (FAILED(vertexBuffer->Map(0, &readRange, &vbData))) {
        return false;
    }
    std::memcpy(vbData, vertices.data(), vbSize);
    vertexBuffer->Unmap(0, nullptr);

    vbView.BufferLocation = vertexBuffer->GetGPUVirtualAddress();
    vbView.StrideInBytes = sizeof(Vertex3D);
    vbView.SizeInBytes = static_cast<UINT>(vbSize);

    // 2. Index Buffer
    D3D12_RESOURCE_DESC ibDesc{};
    ibDesc.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER;
    ibDesc.Width = ibSize;
    ibDesc.Height = 1;
    ibDesc.DepthOrArraySize = 1;
    ibDesc.MipLevels = 1;
    ibDesc.SampleDesc.Count = 1;
    ibDesc.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;

    if (FAILED(device->CreateCommittedResource(
            &uploadHeap,
            D3D12_HEAP_FLAG_NONE,
            &ibDesc,
            D3D12_RESOURCE_STATE_GENERIC_READ,
            nullptr,
            IID_PPV_ARGS(&indexBuffer)))) {
        return false;
    }

    void* ibData = nullptr;
    if (FAILED(indexBuffer->Map(0, &readRange, &ibData))) {
        return false;
    }
    std::memcpy(ibData, indices.data(), ibSize);
    indexBuffer->Unmap(0, nullptr);

    ibView.BufferLocation = indexBuffer->GetGPUVirtualAddress();
    ibView.Format = DXGI_FORMAT_R32_UINT;
    ibView.SizeInBytes = static_cast<UINT>(ibSize);

    return true;
}

void GltfPrimitive::Draw(ID3D12GraphicsCommandList* cmdList) const noexcept {
    if (!cmdList || indexCount == 0 || !vertexBuffer || !indexBuffer) {
        return;
    }

    cmdList->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
    cmdList->IASetVertexBuffers(0, 1, &vbView);
    cmdList->IASetIndexBuffer(&ibView);
    cmdList->DrawIndexedInstanced(indexCount, 1, 0, 0, 0);
}

// =========================================================================
// GltfModel Implementation
// =========================================================================

bool GltfModel::LoadFromGlbFile(const std::wstring& filePath, std::string* outError) {
    std::ifstream file(filePath, std::ios::binary | std::ios::ate);
    if (!file.is_open()) {
        if (outError) *outError = "Failed to open GLB file on disk";
        return false;
    }

    std::streamsize fileSize = file.tellg();
    if (fileSize < 12) {
        if (outError) *outError = "GLB file too small to contain header";
        return false;
    }

    file.seekg(0, std::ios::beg);
    std::vector<u8> buffer(static_cast<size_t>(fileSize));
    if (!file.read(reinterpret_cast<char*>(buffer.data()), fileSize)) {
        if (outError) *outError = "Failed to read GLB file contents";
        return false;
    }

    return LoadFromGlbMemory(buffer.data(), buffer.size(), outError);
}

bool GltfModel::LoadFromGlbFile(const std::string& filePath, std::string* outError) {
    std::wstring wpath(filePath.begin(), filePath.end());
    return LoadFromGlbFile(wpath, outError);
}

bool GltfModel::LoadFromGlbMemory(const void* data, size_t size, std::string* outError) {
    if (!data || size < 12) {
        if (outError) *outError = "Invalid memory buffer for GLB";
        return false;
    }

    return ParseGlb(reinterpret_cast<const u8*>(data), size, outError);
}

bool GltfModel::ParseGlb(const u8* data, size_t size, std::string* outError) {
    // 1. Validate Header (12 bytes)
    u32 magic = 0;
    u32 version = 0;
    u32 length = 0;

    std::memcpy(&magic, data + 0, 4);
    std::memcpy(&version, data + 4, 4);
    std::memcpy(&length, data + 8, 4);

    // 0x46546C67 = "glTF" in little-endian
    if (magic != 0x46546C67) {
        if (outError) *outError = "Invalid GLB magic header (expected 'glTF')";
        return false;
    }

    if (version != 2) {
        if (outError) *outError = "Unsupported glTF version (expected 2)";
        return false;
    }

    if (length > size) {
        if (outError) *outError = "GLB declared length exceeds available buffer size";
        return false;
    }

    // 2. Parse Chunks
    size_t offset = 12;

    std::string_view jsonText;
    const u8* binData = nullptr;
    size_t binSize = 0;

    while (offset + 8 <= length) {
        u32 chunkLength = 0;
        u32 chunkType = 0;
        std::memcpy(&chunkLength, data + offset, 4);
        std::memcpy(&chunkType, data + offset + 4, 4);
        offset += 8;

        if (offset + chunkLength > length) {
            if (outError) *outError = "GLB chunk boundary extends beyond file length";
            return false;
        }

        // 0x4E4F534A = "JSON"
        if (chunkType == 0x4E4F534A) {
            jsonText = std::string_view(reinterpret_cast<const char*>(data + offset), chunkLength);
        }
        // 0x004E4942 = "BIN\0"
        else if (chunkType == 0x004E4942) {
            binData = data + offset;
            binSize = chunkLength;
        }

        offset += chunkLength;
        // Chunks are 4-byte aligned
        offset = (offset + 3) & ~size_t(3);
    }

    if (jsonText.empty()) {
        if (outError) *outError = "No JSON chunk found in GLB";
        return false;
    }

    return ParseJson(jsonText, binData, binSize, outError);
}

bool GltfModel::ParseJson(
    std::string_view jsonText,
    const u8* binData,
    size_t binSize,
    std::string* outError)
{
    Core::JsonValue root = Core::JsonValue::Parse(jsonText, outError);
    if (root.IsNull()) {
        return false;
    }

    m_materials.clear();
    m_bufferViews.clear();
    m_accessors.clear();
    m_meshes.clear();
    m_nodes.clear();
    m_rootNodes.clear();
    m_nodeNameMap.clear();

    // 1. Materials
    if (root.HasKey("materials")) {
        const auto& matArray = root["materials"].AsArray();
        for (const auto& matVal : matArray) {
            GltfMaterial mat{};
            mat.name = matVal["name"].AsString();
            if (matVal.HasKey("pbrMetallicRoughness")) {
                const auto& pbr = matVal["pbrMetallicRoughness"];
                if (pbr.HasKey("baseColorFactor")) {
                    const auto& colArr = pbr["baseColorFactor"].AsArray();
                    if (colArr.size() >= 4) {
                        mat.baseColorFactor = Math::Vec4f(
                            colArr[0].AsFloat(1.0f),
                            colArr[1].AsFloat(1.0f),
                            colArr[2].AsFloat(1.0f),
                            colArr[3].AsFloat(1.0f));
                    }
                }
                mat.metallicFactor = pbr["metallicFactor"].AsFloat(0.0f);
                mat.roughnessFactor = pbr["roughnessFactor"].AsFloat(0.5f);
            }
            mat.doubleSided = matVal["doubleSided"].AsBool(false);
            m_materials.push_back(mat);
        }
    }

    // 2. BufferViews
    if (root.HasKey("bufferViews")) {
        const auto& bvArray = root["bufferViews"].AsArray();
        for (const auto& bvVal : bvArray) {
            GltfBufferView bv{};
            bv.bufferIndex = bvVal["buffer"].AsU32(0);
            bv.byteOffset = bvVal["byteOffset"].AsSize(0);
            bv.byteLength = bvVal["byteLength"].AsSize(0);
            bv.byteStride = bvVal["byteStride"].AsSize(0);
            bv.target = bvVal["target"].AsU32(0);
            m_bufferViews.push_back(bv);
        }
    }

    // 3. Accessors
    if (root.HasKey("accessors")) {
        const auto& accArray = root["accessors"].AsArray();
        for (const auto& accVal : accArray) {
            GltfAccessor acc{};
            acc.bufferViewIndex = accVal["bufferView"].AsInt(-1);
            acc.byteOffset = accVal["byteOffset"].AsSize(0);
            acc.componentType = static_cast<GltfComponentType>(accVal["componentType"].AsU32(5126));
            acc.count = accVal["count"].AsSize(0);

            std::string typeStr = accVal["type"].AsString("SCALAR");
            if (typeStr == "SCALAR") acc.type = GltfAccessorType::Scalar;
            else if (typeStr == "VEC2") acc.type = GltfAccessorType::Vec2;
            else if (typeStr == "VEC3") acc.type = GltfAccessorType::Vec3;
            else if (typeStr == "VEC4") acc.type = GltfAccessorType::Vec4;
            else if (typeStr == "MAT2") acc.type = GltfAccessorType::Mat2;
            else if (typeStr == "MAT3") acc.type = GltfAccessorType::Mat3;
            else if (typeStr == "MAT4") acc.type = GltfAccessorType::Mat4;
            else acc.type = GltfAccessorType::Unknown;

            if (accVal.HasKey("min")) {
                for (const auto& minItem : accVal["min"].AsArray()) {
                    acc.minValues.push_back(minItem.AsFloat());
                }
            }
            if (accVal.HasKey("max")) {
                for (const auto& maxItem : accVal["max"].AsArray()) {
                    acc.maxValues.push_back(maxItem.AsFloat());
                }
            }

            m_accessors.push_back(acc);
        }
    }

    // 4. Meshes
    if (root.HasKey("meshes")) {
        const auto& meshArray = root["meshes"].AsArray();
        for (const auto& meshVal : meshArray) {
            GltfMesh mesh{};
            mesh.name = meshVal["name"].AsString();

            if (meshVal.HasKey("primitives")) {
                const auto& primArray = meshVal["primitives"].AsArray();
                for (const auto& primVal : primArray) {
                    GltfPrimitive prim{};
                    prim.materialIndex = primVal["material"].AsInt(-1);

                    const auto& attrs = primVal["attributes"];
                    i32 posAccIdx = attrs["POSITION"].AsInt(-1);
                    i32 normAccIdx = attrs["NORMAL"].AsInt(-1);
                    i32 colAccIdx = attrs["COLOR_0"].AsInt(-1);
                    i32 indAccIdx = primVal["indices"].AsInt(-1);

                    if (posAccIdx >= 0 && posAccIdx < static_cast<i32>(m_accessors.size())) {
                        const auto& posAcc = m_accessors[static_cast<size_t>(posAccIdx)];
                        const GltfAccessor* normAcc = (normAccIdx >= 0 && normAccIdx < static_cast<i32>(m_accessors.size()))
                            ? &m_accessors[static_cast<size_t>(normAccIdx)] : nullptr;
                        const GltfAccessor* colAcc = (colAccIdx >= 0 && colAccIdx < static_cast<i32>(m_accessors.size()))
                            ? &m_accessors[static_cast<size_t>(colAccIdx)] : nullptr;
                        const GltfAccessor* indAcc = (indAccIdx >= 0 && indAccIdx < static_cast<i32>(m_accessors.size()))
                            ? &m_accessors[static_cast<size_t>(indAccIdx)] : nullptr;

                        ExtractPrimitiveData(prim, posAcc, normAcc, colAcc, indAcc, binData, binSize);
                        mesh.primitives.push_back(std::move(prim));
                    }
                }
            }
            m_meshes.push_back(std::move(mesh));
        }
    }

    // 5. Nodes
    if (root.HasKey("nodes")) {
        const auto& nodeArray = root["nodes"].AsArray();
        m_nodes.resize(nodeArray.size());

        for (size_t i = 0; i < nodeArray.size(); ++i) {
            const auto& nodeVal = nodeArray[i];
            auto& node = m_nodes[i];
            node.name = nodeVal["name"].AsString();
            node.meshIndex = nodeVal["mesh"].AsInt(-1);

            if (nodeVal.HasKey("translation")) {
                const auto& t = nodeVal["translation"].AsArray();
                if (t.size() >= 3) {
                    node.restTranslation = Math::Vec3f(t[0].AsFloat(), t[1].AsFloat(), t[2].AsFloat());
                }
            }
            if (nodeVal.HasKey("rotation")) {
                const auto& r = nodeVal["rotation"].AsArray();
                if (r.size() >= 4) {
                    // glTF quaternion order: [x, y, z, w]
                    node.restRotation = Math::Quat4f(
                        r[3].AsFloat(1.0f), // w
                        r[0].AsFloat(0.0f), // x
                        r[1].AsFloat(0.0f), // y
                        r[2].AsFloat(0.0f)  // z
                    ).Normalized();
                }
            }
            if (nodeVal.HasKey("scale")) {
                const auto& s = nodeVal["scale"].AsArray();
                if (s.size() >= 3) {
                    node.restScale = Math::Vec3f(s[0].AsFloat(1.0f), s[1].AsFloat(1.0f), s[2].AsFloat(1.0f));
                }
            }

            node.translation = node.restTranslation;
            node.rotation = node.restRotation;
            node.scale = node.restScale;

            if (nodeVal.HasKey("children")) {
                for (const auto& childIdx : nodeVal["children"].AsArray()) {
                    u32 c = childIdx.AsU32();
                    node.children.push_back(c);
                    if (c < m_nodes.size()) {
                        m_nodes[c].parentIndex = static_cast<i32>(i);
                    }
                }
            }

            if (!node.name.empty()) {
                m_nodeNameMap[node.name] = static_cast<u32>(i);
            }
        }
    }

    // 6. Scenes
    if (root.HasKey("scenes")) {
        i32 defaultScene = root["scene"].AsInt(0);
        const auto& scenes = root["scenes"].AsArray();
        if (defaultScene >= 0 && defaultScene < static_cast<i32>(scenes.size())) {
            const auto& sceneObj = scenes[static_cast<size_t>(defaultScene)];
            if (sceneObj.HasKey("nodes")) {
                for (const auto& rootNodeVal : sceneObj["nodes"].AsArray()) {
                    m_rootNodes.push_back(rootNodeVal.AsU32());
                }
            }
        }
    }

    // If no explicit scene roots were declared, find nodes without parents
    if (m_rootNodes.empty()) {
        for (size_t i = 0; i < m_nodes.size(); ++i) {
            if (m_nodes[i].parentIndex < 0) {
                m_rootNodes.push_back(static_cast<u32>(i));
            }
        }
    }

    AutoDetectArticulations();
    UpdateForwardKinematics();

    m_isLoaded = true;
    return true;
}

void GltfModel::ExtractPrimitiveData(
    GltfPrimitive& prim,
    const GltfAccessor& posAcc,
    const GltfAccessor* normAcc,
    const GltfAccessor* colAcc,
    const GltfAccessor* indAcc,
    const u8* binData,
    size_t binSize)
{
    if (!binData || binSize == 0 || posAcc.bufferViewIndex < 0) {
        return;
    }

    const auto& posBv = m_bufferViews[static_cast<size_t>(posAcc.bufferViewIndex)];
    size_t posStride = (posBv.byteStride > 0) ? posBv.byteStride : sizeof(f32) * 3;
    const u8* posBase = binData + posBv.byteOffset + posAcc.byteOffset;

    const u8* normBase = nullptr;
    size_t normStride = sizeof(f32) * 3;
    if (normAcc && normAcc->bufferViewIndex >= 0) {
        const auto& normBv = m_bufferViews[static_cast<size_t>(normAcc->bufferViewIndex)];
        normStride = (normBv.byteStride > 0) ? normBv.byteStride : sizeof(f32) * 3;
        normBase = binData + normBv.byteOffset + normAcc->byteOffset;
    }

    const u8* colBase = nullptr;
    size_t colStride = sizeof(f32) * 4;
    if (colAcc && colAcc->bufferViewIndex >= 0) {
        const auto& colBv = m_bufferViews[static_cast<size_t>(colAcc->bufferViewIndex)];
        colStride = (colBv.byteStride > 0) ? colBv.byteStride : sizeof(f32) * 4;
        colBase = binData + colBv.byteOffset + colAcc->byteOffset;
    }

    // Material default color tint fallback
    Math::Vec4f defaultTint{ 1.0f, 1.0f, 1.0f, 1.0f };
    if (prim.materialIndex >= 0 && prim.materialIndex < static_cast<i32>(m_materials.size())) {
        defaultTint = m_materials[static_cast<size_t>(prim.materialIndex)].baseColorFactor;
    }

    prim.vertices.resize(posAcc.count);
    for (size_t i = 0; i < posAcc.count; ++i) {
        auto& vert = prim.vertices[i];

        // 1. Position
        const f32* pPos = reinterpret_cast<const f32*>(posBase + i * posStride);
        vert.position = Math::Vec3f(pPos[0], pPos[1], pPos[2]);

        // 2. Normal
        if (normBase) {
            const f32* pNorm = reinterpret_cast<const f32*>(normBase + i * normStride);
            vert.normal = Math::Vec3f(pNorm[0], pNorm[1], pNorm[2]).Normalized();
        } else {
            vert.normal = Math::Vec3f(0.0f, 1.0f, 0.0f);
        }

        // 3. Color
        if (colBase) {
            const f32* pCol = reinterpret_cast<const f32*>(colBase + i * colStride);
            vert.color = Math::Vec4f(pCol[0], pCol[1], pCol[2], (colAcc->type == GltfAccessorType::Vec4) ? pCol[3] : 1.0f);
        } else {
            vert.color = defaultTint;
        }
    }

    // 4. Indices
    if (indAcc && indAcc->bufferViewIndex >= 0) {
        const auto& indBv = m_bufferViews[static_cast<size_t>(indAcc->bufferViewIndex)];
        const u8* indBase = binData + indBv.byteOffset + indAcc->byteOffset;
        prim.indices.resize(indAcc->count);

        if (indAcc->componentType == GltfComponentType::UnsignedShort) {
            const u16* pInd16 = reinterpret_cast<const u16*>(indBase);
            for (size_t i = 0; i < indAcc->count; ++i) {
                prim.indices[i] = static_cast<u32>(pInd16[i]);
            }
        } else if (indAcc->componentType == GltfComponentType::UnsignedInt) {
            const u32* pInd32 = reinterpret_cast<const u32*>(indBase);
            for (size_t i = 0; i < indAcc->count; ++i) {
                prim.indices[i] = pInd32[i];
            }
        } else if (indAcc->componentType == GltfComponentType::UnsignedByte) {
            const u8* pInd8 = reinterpret_cast<const u8*>(indBase);
            for (size_t i = 0; i < indAcc->count; ++i) {
                prim.indices[i] = static_cast<u32>(pInd8[i]);
            }
        }
    } else {
        // Sequential indices if not indexed
        prim.indices.resize(posAcc.count);
        for (size_t i = 0; i < posAcc.count; ++i) {
            prim.indices[i] = static_cast<u32>(i);
        }
    }
}

void GltfModel::AutoDetectArticulations() noexcept {
    m_elevatorNodeIdx = FindNodeIndex("Elevator");
    if (m_elevatorNodeIdx < 0) m_elevatorNodeIdx = FindNodeIndex("elevator");

    m_aileronLeftNodeIdx = FindNodeIndex("Aileron_L");
    if (m_aileronLeftNodeIdx < 0) m_aileronLeftNodeIdx = FindNodeIndex("aileron_l");
    if (m_aileronLeftNodeIdx < 0) m_aileronLeftNodeIdx = FindNodeIndex("aileron_left");

    m_aileronRightNodeIdx = FindNodeIndex("Aileron_R");
    if (m_aileronRightNodeIdx < 0) m_aileronRightNodeIdx = FindNodeIndex("aileron_r");
    if (m_aileronRightNodeIdx < 0) m_aileronRightNodeIdx = FindNodeIndex("aileron_right");

    m_rudderNodeIdx = FindNodeIndex("Rudder");
    if (m_rudderNodeIdx < 0) m_rudderNodeIdx = FindNodeIndex("rudder");

    m_flapLeftNodeIdx = FindNodeIndex("Flap_L");
    if (m_flapLeftNodeIdx < 0) m_flapLeftNodeIdx = FindNodeIndex("flap_l");

    m_flapRightNodeIdx = FindNodeIndex("Flap_R");
    if (m_flapRightNodeIdx < 0) m_flapRightNodeIdx = FindNodeIndex("flap_r");

    m_propellerNodeIdx = FindNodeIndex("Propeller");
    if (m_propellerNodeIdx < 0) m_propellerNodeIdx = FindNodeIndex("propeller");
    if (m_propellerNodeIdx < 0) m_propellerNodeIdx = FindNodeIndex("prop");
    if (m_propellerNodeIdx < 0) m_propellerNodeIdx = FindNodeIndex("Propeller_L");

    m_propellerRightNodeIdx = FindNodeIndex("Propeller_R");
    if (m_propellerRightNodeIdx < 0) m_propellerRightNodeIdx = FindNodeIndex("propeller_r");

    m_gearNoseNodeIdx = FindNodeIndex("Gear_Nose");
    if (m_gearNoseNodeIdx < 0) m_gearNoseNodeIdx = FindNodeIndex("gear_nose");
    m_gearLeftNodeIdx = FindNodeIndex("Gear_L");
    if (m_gearLeftNodeIdx < 0) m_gearLeftNodeIdx = FindNodeIndex("gear_l");
    m_gearRightNodeIdx = FindNodeIndex("Gear_R");
    if (m_gearRightNodeIdx < 0) m_gearRightNodeIdx = FindNodeIndex("gear_r");

    // Configure articulation hinge axes and pivot points
    if (m_elevatorNodeIdx >= 0) {
        auto& node = m_nodes[static_cast<size_t>(m_elevatorNodeIdx)];
        node.articulationType = ArticulationType::Elevator;
        node.hingeAxis = Math::Vec3f(1.0f, 0.0f, 0.0f); // Pitch axis (+X)
    }

    if (m_aileronLeftNodeIdx >= 0) {
        auto& node = m_nodes[static_cast<size_t>(m_aileronLeftNodeIdx)];
        node.articulationType = ArticulationType::AileronLeft;
        node.hingeAxis = Math::Vec3f(1.0f, 0.0f, 0.0f); // Wing trailing edge axis
    }

    if (m_aileronRightNodeIdx >= 0) {
        auto& node = m_nodes[static_cast<size_t>(m_aileronRightNodeIdx)];
        node.articulationType = ArticulationType::AileronRight;
        node.hingeAxis = Math::Vec3f(1.0f, 0.0f, 0.0f);
    }

    if (m_rudderNodeIdx >= 0) {
        auto& node = m_nodes[static_cast<size_t>(m_rudderNodeIdx)];
        node.articulationType = ArticulationType::Rudder;
        node.hingeAxis = Math::Vec3f(0.0f, 1.0f, 0.0f); // Yaw axis (+Y)
    }

    if (m_flapLeftNodeIdx >= 0) {
        auto& node = m_nodes[static_cast<size_t>(m_flapLeftNodeIdx)];
        node.articulationType = ArticulationType::FlapLeft;
        node.hingeAxis = Math::Vec3f(1.0f, 0.0f, 0.0f);
    }

    if (m_flapRightNodeIdx >= 0) {
        auto& node = m_nodes[static_cast<size_t>(m_flapRightNodeIdx)];
        node.articulationType = ArticulationType::FlapRight;
        node.hingeAxis = Math::Vec3f(1.0f, 0.0f, 0.0f);
    }

    if (m_propellerNodeIdx >= 0) {
        auto& node = m_nodes[static_cast<size_t>(m_propellerNodeIdx)];
        node.articulationType = ArticulationType::Propeller;
        node.hingeAxis = Math::Vec3f(0.0f, 0.0f, 1.0f); // Thrust / Roll axis (+Z in graphics)
    }

    if (m_propellerRightNodeIdx >= 0) {
        auto& node = m_nodes[static_cast<size_t>(m_propellerRightNodeIdx)];
        node.articulationType = ArticulationType::Propeller;
        node.hingeAxis = Math::Vec3f(0.0f, 0.0f, 1.0f);
    }
}

i32 GltfModel::FindNodeIndex(const std::string& name) const noexcept {
    auto it = m_nodeNameMap.find(name);
    if (it != m_nodeNameMap.end()) {
        return static_cast<i32>(it->second);
    }
    for (size_t i = 0; i < m_nodes.size(); ++i) {
        if (m_nodes[i].name == name) {
            return static_cast<i32>(i);
        }
    }
    return -1;
}

void GltfModel::SetNodeDeflection(size_t nodeIndex, f32 deflectionRad) noexcept {
    if (nodeIndex < m_nodes.size()) {
        m_nodes[nodeIndex].currentDeflectionRad = deflectionRad;
    }
}

void GltfModel::SetNodeRotation(size_t nodeIndex, const Math::Quat4f& rot) noexcept {
    if (nodeIndex < m_nodes.size()) {
        m_nodes[nodeIndex].rotation = rot;
    }
}

void GltfModel::SetNodeTranslation(size_t nodeIndex, const Math::Vec3f& trans) noexcept {
    if (nodeIndex < m_nodes.size()) {
        m_nodes[nodeIndex].translation = trans;
    }
}

void GltfModel::UpdateForwardKinematics() noexcept {
    // 1. Compute local matrix for each node
    for (auto& node : m_nodes) {
        node.ComputeLocalTransform();
    }

    // 2. Hierarchical propagation from roots
    auto PropagateTransforms = [&](auto& self, u32 nodeIdx, const Math::Mat4f& parentWorld) -> void {
        auto& node = m_nodes[nodeIdx];
        node.worldTransform = node.localTransform * parentWorld;

        for (u32 childIdx : node.children) {
            if (childIdx < m_nodes.size()) {
                self(self, childIdx, node.worldTransform);
            }
        }
    };

    for (u32 rootIdx : m_rootNodes) {
        if (rootIdx < m_nodes.size()) {
            PropagateTransforms(PropagateTransforms, rootIdx, Math::Mat4f::Identity());
        }
    }
}

void GltfModel::ApplyAircraftControls(
    f32 pitchNorm,
    f32 rollNorm,
    f32 yawNorm,
    f32 flapsNorm,
    f32 propAngleRad,
    bool gearExtended) noexcept
{
    // Elevator deflection: +28 deg up (-pitchNorm), -26 deg down
    if (m_elevatorNodeIdx >= 0) {
        constexpr f32 maxElevatorRad = 28.0f * (3.14159265f / 180.0f);
        m_nodes[static_cast<size_t>(m_elevatorNodeIdx)].currentDeflectionRad = -pitchNorm * maxElevatorRad;
    }

    // Ailerons: 20 deg up, 15 deg down
    if (m_aileronLeftNodeIdx >= 0) {
        constexpr f32 maxAileronRad = 20.0f * (3.14159265f / 180.0f);
        m_nodes[static_cast<size_t>(m_aileronLeftNodeIdx)].currentDeflectionRad = rollNorm * maxAileronRad;
    }
    if (m_aileronRightNodeIdx >= 0) {
        constexpr f32 maxAileronRad = 20.0f * (3.14159265f / 180.0f);
        m_nodes[static_cast<size_t>(m_aileronRightNodeIdx)].currentDeflectionRad = -rollNorm * maxAileronRad;
    }

    // Rudder: +/- 16 deg
    if (m_rudderNodeIdx >= 0) {
        constexpr f32 maxRudderRad = 16.0f * (3.14159265f / 180.0f);
        m_nodes[static_cast<size_t>(m_rudderNodeIdx)].currentDeflectionRad = -yawNorm * maxRudderRad;
    }

    // Flaps: 0 to 30 deg downwards
    if (m_flapLeftNodeIdx >= 0) {
        constexpr f32 maxFlapRad = 30.0f * (3.14159265f / 180.0f);
        m_nodes[static_cast<size_t>(m_flapLeftNodeIdx)].currentDeflectionRad = flapsNorm * maxFlapRad;
    }
    if (m_flapRightNodeIdx >= 0) {
        constexpr f32 maxFlapRad = 30.0f * (3.14159265f / 180.0f);
        m_nodes[static_cast<size_t>(m_flapRightNodeIdx)].currentDeflectionRad = flapsNorm * maxFlapRad;
    }

    // Propeller continuous rotation (single or twin)
    if (m_propellerNodeIdx >= 0) {
        m_nodes[static_cast<size_t>(m_propellerNodeIdx)].currentDeflectionRad = propAngleRad;
    }
    if (m_propellerRightNodeIdx >= 0) {
        m_nodes[static_cast<size_t>(m_propellerRightNodeIdx)].currentDeflectionRad = propAngleRad;
    }

    // Retractable Landing Gear animation (translate up into wheel bays when gear is retracted)
    f32 gearTranslateY = gearExtended ? 0.0f : 0.85f;
    if (m_gearNoseNodeIdx >= 0) {
        m_nodes[static_cast<size_t>(m_gearNoseNodeIdx)].translation.y = gearTranslateY;
    }
    if (m_gearLeftNodeIdx >= 0) {
        m_nodes[static_cast<size_t>(m_gearLeftNodeIdx)].translation.y = gearTranslateY;
    }
    if (m_gearRightNodeIdx >= 0) {
        m_nodes[static_cast<size_t>(m_gearRightNodeIdx)].translation.y = gearTranslateY;
    }

    UpdateForwardKinematics();
}

void GltfModel::ApplyAircraftControls(
    f32 pitchNorm,
    f32 rollNorm,
    f32 yawNorm,
    f32 flapsNorm,
    f32 propAngleRad) noexcept
{
    ApplyAircraftControls(pitchNorm, rollNorm, yawNorm, flapsNorm, propAngleRad, true);
}

bool GltfModel::CreateGPUResources(ID3D12Device* device) {
    if (!device) return false;

    bool success = true;
    for (auto& mesh : m_meshes) {
        for (auto& prim : mesh.primitives) {
            if (!prim.CreateGPUResources(device)) {
                success = false;
            }
        }
    }
    return success;
}

void GltfModel::Draw(
    ID3D12GraphicsCommandList* cmdList,
    const Math::Mat4f& rootTransform,
    const Math::Mat4f& viewProjMatrix,
    UINT rootConstantsParamIndex) const noexcept
{
    if (!cmdList || !m_isLoaded) return;

    for (const auto& node : m_nodes) {
        if (node.meshIndex >= 0 && node.meshIndex < static_cast<i32>(m_meshes.size())) {
            const auto& mesh = m_meshes[static_cast<size_t>(node.meshIndex)];

            // Model Matrix = Node World Matrix * Aircraft Root Transform
            Math::Mat4f model = node.worldTransform * rootTransform;
            Math::Mat4f mvp = model * viewProjMatrix;

            // Update constant buffer MVP (first 16 floats of root constants)
            cmdList->SetGraphicsRoot32BitConstants(rootConstantsParamIndex, 16, &mvp, 0);

            for (const auto& prim : mesh.primitives) {
                prim.Draw(cmdList);
            }
        }
    }
}

size_t GltfModel::GetTotalVertexCount() const noexcept {
    size_t count = 0;
    for (const auto& mesh : m_meshes) {
        for (const auto& prim : mesh.primitives) {
            count += prim.vertices.size();
        }
    }
    return count;
}

size_t GltfModel::GetTotalTriangleCount() const noexcept {
    size_t count = 0;
    for (const auto& mesh : m_meshes) {
        for (const auto& prim : mesh.primitives) {
            count += prim.indices.size() / 3;
        }
    }
    return count;
}

} // namespace LokYS::Graphics
