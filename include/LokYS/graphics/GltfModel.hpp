#pragma once

#include "LokYS/graphics/GltfTypes.hpp"
#include "LokYS/core/JsonParser.hpp"
#include <string>
#include <vector>
#include <span>

namespace LokYS::Graphics {

class GltfModel {
public:
    GltfModel() = default;
    ~GltfModel() = default;

    // Load binary glTF (.glb) from file or memory buffer
    [[nodiscard]] bool LoadFromGlbFile(const std::wstring& filePath, std::string* outError = nullptr);
    [[nodiscard]] bool LoadFromGlbFile(const std::string& filePath, std::string* outError = nullptr);
    [[nodiscard]] bool LoadFromGlbMemory(const void* data, size_t size, std::string* outError = nullptr);

    // Create D3D12 vertex and index buffers for all primitives
    [[nodiscard]] bool CreateGPUResources(ID3D12Device* device);

    // Render all meshes in the model with active forward kinematics
    void Draw(
        ID3D12GraphicsCommandList* cmdList,
        const Math::Mat4f& rootTransform,
        const Math::Mat4f& viewProjMatrix,
        UINT rootConstantsParamIndex = 0) const noexcept;

    // Forward Kinematics & Articulation
    void UpdateForwardKinematics() noexcept;
    void SetNodeDeflection(size_t nodeIndex, f32 deflectionRad) noexcept;
    void SetNodeRotation(size_t nodeIndex, const Math::Quat4f& rot) noexcept;
    void SetNodeTranslation(size_t nodeIndex, const Math::Vec3f& trans) noexcept;

    [[nodiscard]] i32 FindNodeIndex(const std::string& name) const noexcept;

    // Aircraft control surface articulation:
    // pitch: elevator [-1.0, 1.0] -> [-28 deg, +26 deg]
    // roll: ailerons [-1.0, 1.0] -> [-20 deg, +15 deg] (differential)
    // yaw: rudder [-1.0, 1.0] -> [-16 deg, +16 deg]
    // flaps: [0.0, 1.0] -> [0 deg, 30 deg]
    // propAngleRad: continuous rotation angle
    void ApplyAircraftControls(
        f32 pitchNorm,
        f32 rollNorm,
        f32 yawNorm,
        f32 flapsNorm,
        f32 propAngleRad) noexcept;

    // Model inspection
    [[nodiscard]] bool IsLoaded() const noexcept { return m_isLoaded; }
    [[nodiscard]] const std::vector<GltfNode>& GetNodes() const noexcept { return m_nodes; }
    [[nodiscard]] std::vector<GltfNode>& GetNodes() noexcept { return m_nodes; }
    [[nodiscard]] const std::vector<GltfMesh>& GetMeshes() const noexcept { return m_meshes; }
    [[nodiscard]] const std::vector<GltfMaterial>& GetMaterials() const noexcept { return m_materials; }

    [[nodiscard]] size_t GetTotalVertexCount() const noexcept;
    [[nodiscard]] size_t GetTotalTriangleCount() const noexcept;

    // Node classification helper
    void AutoDetectArticulations() noexcept;

private:
    [[nodiscard]] bool ParseGlb(const u8* data, size_t size, std::string* outError);
    [[nodiscard]] bool ParseJson(std::string_view jsonText, const u8* binData, size_t binSize, std::string* outError);

    void ExtractPrimitiveData(
        GltfPrimitive& prim,
        const GltfAccessor& posAcc,
        const GltfAccessor* normAcc,
        const GltfAccessor* colAcc,
        const GltfAccessor* indAcc,
        const u8* binData,
        size_t binSize);

    std::vector<GltfNode> m_nodes;
    std::vector<GltfMesh> m_meshes;
    std::vector<GltfMaterial> m_materials;
    std::vector<GltfBufferView> m_bufferViews;
    std::vector<GltfAccessor> m_accessors;
    std::vector<u32> m_rootNodes;

    // Fast node lookup map
    std::unordered_map<std::string, u32> m_nodeNameMap;

    // Cached control node indices
    i32 m_elevatorNodeIdx{ -1 };
    i32 m_aileronLeftNodeIdx{ -1 };
    i32 m_aileronRightNodeIdx{ -1 };
    i32 m_rudderNodeIdx{ -1 };
    i32 m_flapLeftNodeIdx{ -1 };
    i32 m_flapRightNodeIdx{ -1 };
    i32 m_propellerNodeIdx{ -1 };

    bool m_isLoaded{ false };
};

} // namespace LokYS::Graphics
