#include "LokYS/graphics/TerrainStreamingSystem.hpp"
#include <cmath>
#include <algorithm>

namespace LokYS::Graphics {

TerrainStreamingSystem::TerrainStreamingSystem() {
    BuildAllTiles(nullptr);
}

bool TerrainStreamingSystem::Initialize(ID3D12Device* device) {
    if (!device) return false;

    BuildAllTiles(device);
    m_initialized = true;
    return true;
}

void TerrainStreamingSystem::Shutdown() {
    for (auto& tile : m_tiles) {
        tile.mesh.Shutdown();
    }
    m_tiles.clear();
    m_initialized = false;
}

f32 TerrainStreamingSystem::SampleElevation(f32 xGfx, f32 zGfx) const noexcept {
    // 1. Check multi-airport plateau flattening
    f32 flattenWeight = 0.0f;
    f32 airportElev = m_airportDb.SampleAirportElevationOffset(xGfx, zGfx, flattenWeight);
    if (flattenWeight >= 0.999f) {
        return airportElev;
    }

    // 2. Base rolling regional hills (harmonic frequencies)
    f32 h0 = 55.0f * std::sin(xGfx * 0.00018f) * std::cos(zGfx * 0.00015f);
    f32 h1 = 30.0f * std::cos(xGfx * 0.00042f + zGfx * 0.00035f);
    f32 h2 = 14.0f * std::sin(xGfx * 0.0011f);
    f32 hBase = h0 + h1 + h2 + 45.0f; // Baseline ~45m MSL

    // 3. Mountain ridges in the North/North-East quadrant
    // Mesa Highlands is located at Z_gfx = 14000m, X_gfx = -8000m
    f32 dxMtn = xGfx - (-8000.0f);
    f32 dzMtn = zGfx - (14000.0f);
    f32 distMtn = std::sqrt(dxMtn * dxMtn + dzMtn * dzMtn);

    f32 hMtn = 0.0f;
    if (distMtn < 16000.0f) {
        f32 frac = (16000.0f - distMtn) / 16000.0f;
        // Mountain ridge spine rising to +550m
        f32 ridge = 350.0f * frac * frac + 80.0f * std::sin(xGfx * 0.0008f) * std::cos(zGfx * 0.0006f);
        hMtn = std::max(0.0f, ridge);
    }

    f32 hRegional = std::max(2.0f, hBase + hMtn);

    // 4. Smooth blend between regional terrain and airport elevation
    return (1.0f - flattenWeight) * hRegional + flattenWeight * airportElev;
}

Math::Vec3f TerrainStreamingSystem::SampleNormal(f32 xGfx, f32 zGfx) const noexcept {
    constexpr f32 eps = 2.0f;
    f32 hL = SampleElevation(xGfx - eps, zGfx);
    f32 hR = SampleElevation(xGfx + eps, zGfx);
    f32 hD = SampleElevation(xGfx, zGfx - eps);
    f32 hU = SampleElevation(xGfx, zGfx + eps);

    Math::Vec3f n{ hL - hR, 2.0f * eps, hD - hU };
    return n.Normalized();
}

Mesh TerrainStreamingSystem::BuildTileMesh(
    ID3D12Device* device,
    f32 minX, f32 maxX,
    f32 minZ, f32 maxZ,
    u32 divisions,
    f32& outMinY, f32& outMaxY) const
{
    std::vector<Vertex3D> vertices;
    std::vector<u32> indices;

    u32 numVertsX = divisions + 1;
    u32 numVertsZ = divisions + 1;
    vertices.reserve(numVertsX * numVertsZ);

    f32 stepX = (maxX - minX) / static_cast<f32>(divisions);
    f32 stepZ = (maxZ - minZ) / static_cast<f32>(divisions);

    outMinY = 1e9f;
    outMaxY = -1e9f;

    for (u32 iz = 0; iz < numVertsZ; ++iz) {
        f32 z = minZ + static_cast<f32>(iz) * stepZ;
        for (u32 ix = 0; ix < numVertsX; ++ix) {
            f32 x = minX + static_cast<f32>(ix) * stepX;
            f32 y = SampleElevation(x, z);

            outMinY = std::min(outMinY, y);
            outMaxY = std::max(outMaxY, y);

            Math::Vec3f norm = SampleNormal(x, z);

            // Natural elevation-dependent coloring
            Math::Vec4f col;
            if (y < 80.0f) {
                // Lowland lush green meadows
                col = { 0.22f, 0.48f, 0.18f, 1.0f };
            } else if (y < 250.0f) {
                // Temperate forest / hills olive
                col = { 0.32f, 0.52f, 0.22f, 1.0f };
            } else if (y < 500.0f) {
                // Mountain mesa and highlands dry plateau
                col = { 0.48f, 0.44f, 0.32f, 1.0f };
            } else {
                // Alpine granite rock and scree
                col = { 0.55f, 0.53f, 0.50f, 1.0f };
            }

            vertices.push_back({ { x, y, z }, norm, col });
        }
    }

    // Build triangular index grid
    for (u32 iz = 0; iz < divisions; ++iz) {
        for (u32 ix = 0; ix < divisions; ++ix) {
            u32 i0 = iz * numVertsX + ix;
            u32 i1 = i0 + 1;
            u32 i2 = (iz + 1) * numVertsX + ix;
            u32 i3 = i2 + 1;

            indices.push_back(i0);
            indices.push_back(i2);
            indices.push_back(i1);

            indices.push_back(i1);
            indices.push_back(i2);
            indices.push_back(i3);
        }
    }

    if (!device) {
        return Mesh{};
    }

    return Mesh::CreateFromVertices(device, vertices, indices);
}

void TerrainStreamingSystem::BuildAllTiles(ID3D12Device* device) {
    m_tiles.clear();
    m_tiles.reserve(TILE_GRID_DIM * TILE_GRID_DIM);

    for (int tz = 0; tz < TILE_GRID_DIM; ++tz) {
        f32 minZ = -WORLD_HALF_SIZE + static_cast<f32>(tz) * TILE_SIZE;
        f32 maxZ = minZ + TILE_SIZE;

        for (int tx = 0; tx < TILE_GRID_DIM; ++tx) {
            f32 minX = -WORLD_HALF_SIZE + static_cast<f32>(tx) * TILE_SIZE;
            f32 maxX = minX + TILE_SIZE;

            StreamingTerrainTile tile{};
            tile.tileX = tx;
            tile.tileZ = tz;
            tile.centerGfx = Math::Vec3f((minX + maxX) * 0.5f, 0.0f, (minZ + maxZ) * 0.5f);

            // Near airfield tiles receive higher geometric division (24x24), outer tiles receive 12x12
            // Center tile is at tx=4, tz=4 (contains KLOK at 0,0)
            bool isAirportTile = (std::abs(tx - 4) <= 1 && std::abs(tz - 4) <= 1) ||
                                 (tx == 3 && tz == 5); // KMES mesa tile

            u32 divisions = isAirportTile ? 24 : 12;
            tile.lodLevel = isAirportTile ? 0 : 1;

            f32 minY = 0.0f;
            f32 maxY = 0.0f;
            tile.mesh = BuildTileMesh(device, minX, maxX, minZ, maxZ, divisions, minY, maxY);

            tile.centerGfx.y = (minY + maxY) * 0.5f;
            tile.minBoundsGfx = Math::Vec3f(minX, minY - 2.0f, minZ);
            tile.maxBoundsGfx = Math::Vec3f(maxX, maxY + 2.0f, maxZ);

            m_tiles.push_back(std::move(tile));
        }
    }
}

void TerrainStreamingSystem::Draw(
    ID3D12GraphicsCommandList* cmdList,
    const Math::Frustum& cameraFrustum,
    const Math::Vec3f& cameraPosGfx) noexcept
{
    if (!cmdList || !m_initialized) return;

    m_renderedTiles = 0;
    m_culledTiles = 0;

    for (auto& tile : m_tiles) {
        // Distance check: Skip tiles farther than 36 km
        f32 dx = tile.centerGfx.x - cameraPosGfx.x;
        f32 dz = tile.centerGfx.z - cameraPosGfx.z;
        f32 distSq = dx * dx + dz * dz;
        if (distSq > (36000.0f * 36000.0f)) {
            tile.isVisible = false;
            m_culledTiles++;
            continue;
        }

        // Camera Frustum AABB Culling
        if (!cameraFrustum.IntersectsAABB(tile.minBoundsGfx, tile.maxBoundsGfx)) {
            tile.isVisible = false;
            m_culledTiles++;
            continue;
        }

        tile.isVisible = true;
        tile.mesh.Draw(cmdList);
        m_renderedTiles++;
    }
}

void TerrainStreamingSystem::UpdateLODs(f32 camX, f32 camZ) noexcept {
    for (auto& tile : m_tiles) {
        f32 dx = tile.centerGfx.x - camX;
        f32 dz = tile.centerGfx.z - camZ;
        f32 dist = std::sqrt(dx * dx + dz * dz);
        tile.lodLevel = (dist <= 14000.0f) ? 0 : 1;
    }
}

u32 TerrainStreamingSystem::GetLOD0Count() const noexcept {
    u32 count = 0;
    for (const auto& tile : m_tiles) {
        if (tile.lodLevel == 0) count++;
    }
    return count;
}

u32 TerrainStreamingSystem::GetLOD1Count() const noexcept {
    u32 count = 0;
    for (const auto& tile : m_tiles) {
        if (tile.lodLevel == 1) count++;
    }
    return count;
}

} // namespace LokYS::Graphics
