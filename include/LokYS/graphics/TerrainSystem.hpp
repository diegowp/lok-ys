#pragma once

#include "LokYS/core/Types.hpp"
#include "LokYS/graphics/Mesh.hpp"
#include "LokYS/math/Vector3.hpp"
#include "LokYS/math/Matrix4x4.hpp"
#include <vector>

namespace LokYS::Graphics {

struct TerrainChunk {
    Mesh mesh;
    Math::Vec3f minBoundsGfx{ 0.0f, 0.0f, 0.0f };
    Math::Vec3f maxBoundsGfx{ 0.0f, 0.0f, 0.0f };
    u32 lodLevel{ 0 };
    bool isVisible{ true };
};

class TerrainSystem {
public:
    TerrainSystem() = default;
    ~TerrainSystem() = default;

    TerrainSystem(const TerrainSystem&) = delete;
    TerrainSystem& operator=(const TerrainSystem&) = delete;

    /// @brief Initializes multi-LOD terrain meshes and heightfield structures.
    [[nodiscard]] bool Initialize(ID3D12Device* device);
    void Shutdown();

    /// @brief Evaluates elevation height in graphics coordinates (meters above MSL).
    /// Flat around airport runway (x in [-350, 350], z in [-1600, 1600]), undulating hills elsewhere.
    [[nodiscard]] static f32 SampleElevation(f32 xGfx, f32 zGfx) noexcept;

    /// @brief Evaluates analytical surface normal at given horizontal position.
    [[nodiscard]] static Math::Vec3f SampleNormal(f32 xGfx, f32 zGfx) noexcept;

    /// @brief Draws visible terrain chunks subject to camera frustum culling.
    void Draw(ID3D12GraphicsCommandList* cmdList, const Math::Frustum& cameraFrustum, const Math::Vec3f& cameraPosGfx) noexcept;

    [[nodiscard]] u32 GetRenderedChunkCount() const noexcept { return m_renderedChunks; }
    [[nodiscard]] u32 GetCulledChunkCount() const noexcept { return m_culledChunks; }
    [[nodiscard]] size_t GetTotalChunkCount() const noexcept { return m_chunks.size(); }

private:
    std::vector<TerrainChunk> m_chunks;
    u32 m_renderedChunks{ 0 };
    u32 m_culledChunks{ 0 };
    bool m_initialized{ false };
};

} // namespace LokYS::Graphics
