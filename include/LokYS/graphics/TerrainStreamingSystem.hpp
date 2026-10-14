#pragma once

#include "LokYS/core/Types.hpp"
#include "LokYS/graphics/Mesh.hpp"
#include "LokYS/math/Vector3.hpp"
#include "LokYS/math/Matrix4x4.hpp"
#include "LokYS/scenery/AirportDatabase.hpp"
#include <vector>
#include <memory>

namespace LokYS::Graphics {

struct StreamingTerrainTile {
    Mesh mesh;
    Math::Vec3f minBoundsGfx{ 0.0f, 0.0f, 0.0f };
    Math::Vec3f maxBoundsGfx{ 0.0f, 0.0f, 0.0f };
    Math::Vec3f centerGfx{ 0.0f, 0.0f, 0.0f };
    u32 lodLevel{ 0 };
    bool isVisible{ true };
    i32 tileX{ 0 };
    i32 tileZ{ 0 };
};

class TerrainStreamingSystem {
public:
    TerrainStreamingSystem();
    ~TerrainStreamingSystem() { Shutdown(); }

    TerrainStreamingSystem(const TerrainStreamingSystem&) = delete;
    TerrainStreamingSystem& operator=(const TerrainStreamingSystem&) = delete;

    [[nodiscard]] bool Initialize(ID3D12Device* device);
    void Shutdown();

    /// @brief Evaluates elevation height in graphics coordinates (meters above MSL) across 64km regional world.
    [[nodiscard]] f32 SampleElevation(f32 xGfx, f32 zGfx) const noexcept;

    /// @brief Evaluates analytical surface normal at given horizontal position.
    [[nodiscard]] Math::Vec3f SampleNormal(f32 xGfx, f32 zGfx) const noexcept;

    /// @brief Updates active streaming tile LODs around camera position and draws visible tiles.
    void Draw(
        ID3D12GraphicsCommandList* cmdList,
        const Math::Frustum& cameraFrustum,
        const Math::Vec3f& cameraPosGfx) noexcept;

    void UpdateLODs(f32 camX, f32 camZ) noexcept;
    [[nodiscard]] u32 GetLOD0Count() const noexcept;
    [[nodiscard]] u32 GetLOD1Count() const noexcept;

    [[nodiscard]] constexpr f32 GetWorldSizeMeters() const noexcept { return WORLD_HALF_SIZE * 2.0f; }
    [[nodiscard]] constexpr f32 GetTileSizeMeters() const noexcept { return TILE_SIZE; }

    [[nodiscard]] const Scenery::AirportDatabase& GetAirportDatabase() const noexcept { return m_airportDb; }
    [[nodiscard]] u32 GetRenderedTileCount() const noexcept { return m_renderedTiles; }
    [[nodiscard]] u32 GetCulledTileCount() const noexcept { return m_culledTiles; }
    [[nodiscard]] size_t GetTotalTileCount() const noexcept { return m_tiles.size(); }

private:
    void BuildAllTiles(ID3D12Device* device);
    [[nodiscard]] Mesh BuildTileMesh(
        ID3D12Device* device,
        f32 minX, f32 maxX,
        f32 minZ, f32 maxZ,
        u32 divisions,
        f32& outMinY, f32& outMaxY) const;

    Scenery::AirportDatabase m_airportDb;
    std::vector<StreamingTerrainTile> m_tiles;

    u32 m_renderedTiles{ 0 };
    u32 m_culledTiles{ 0 };
    bool m_initialized{ false };

    static constexpr f32 WORLD_HALF_SIZE = 32000.0f; // 64 km x 64 km world
    static constexpr int TILE_GRID_DIM = 8;          // 8x8 = 64 regional tiles
    static constexpr f32 TILE_SIZE = 8000.0f;        // 8000m x 8000m per tile
};

} // namespace LokYS::Graphics
