#include "LokYS/graphics/TerrainSystem.hpp"
#include <algorithm>
#include <cmath>

namespace LokYS::Graphics {

f32 TerrainSystem::SampleElevation(f32 xGfx, f32 zGfx) noexcept {
    // Distance from the flat airport runway reservation box
    f32 dx = std::max(0.0f, std::abs(xGfx) - 300.0f);
    f32 dz = std::max(0.0f, std::abs(zGfx) - 1400.0f);
    f32 distOutside = std::sqrt(dx * dx + dz * dz);

    // Flat reservation mesa at y = 0
    if (distOutside <= 200.0f) {
        return 0.0f;
    }

    // Smooth cubic Hermite blend from airport mesa to regional hills
    f32 weight = 1.0f;
    if (distOutside < 1600.0f) {
        f32 s = (distOutside - 200.0f) / 1400.0f;
        weight = s * s * (3.0f - 2.0f * s);
    }

    // Multi-harmonic regional terrain elevation (positive rolling terrain base)
    f32 h1 = 90.0f * std::sin(0.00030f * xGfx) * std::cos(0.00035f * zGfx);
    f32 h2 = 45.0f * std::cos(0.00075f * xGfx + 0.4f) * std::sin(0.00060f * zGfx);
    f32 h3 = 25.0f * std::sin(0.00160f * xGfx + 1.2f) * std::cos(0.00130f * zGfx);
    f32 localH = 80.0f + h1 + h2 + h3;

    // Distant mountain ridges
    f32 mtnH = 0.0f;
    if (distOutside > 3500.0f) {
        f32 mtnWeight = std::clamp((distOutside - 3500.0f) / 5000.0f, 0.0f, 1.0f);
        f32 m1 = 280.0f * std::sin(0.00012f * xGfx + 0.9f) * std::cos(0.00014f * zGfx + 0.3f);
        f32 m2 = 120.0f * std::cos(0.00022f * xGfx) * std::cos(0.00020f * zGfx);
        mtnH = mtnWeight * (200.0f + m1 + m2);
    }

    f32 totalElev = (localH + mtnH) * weight;
    return std::max(0.0f, totalElev);
}

Math::Vec3f TerrainSystem::SampleNormal(f32 xGfx, f32 zGfx) noexcept {
    constexpr f32 EPS = 2.0f;
    f32 hL = SampleElevation(xGfx - EPS, zGfx);
    f32 hR = SampleElevation(xGfx + EPS, zGfx);
    f32 hD = SampleElevation(xGfx, zGfx - EPS);
    f32 hU = SampleElevation(xGfx, zGfx + EPS);

    f32 nx = -(hR - hL) / (2.0f * EPS);
    f32 nz = -(hU - hD) / (2.0f * EPS);

    Math::Vec3f n{ nx, 1.0f, nz };
    return n.Normalized();
}

static Math::Vec4f ComputeTerrainColor(f32 y, int ix, int iz) noexcept {
    // Lowland meadow grass (two checker shades)
    const Math::Vec4f grassGreen1{ 0.28f, 0.52f, 0.24f, 1.0f };
    const Math::Vec4f grassGreen2{ 0.24f, 0.47f, 0.21f, 1.0f };
    const Math::Vec4f earthOlive{ 0.38f, 0.44f, 0.25f, 1.0f };
    const Math::Vec4f rockGray{ 0.48f, 0.46f, 0.44f, 1.0f };
    const Math::Vec4f snowWhite{ 0.88f, 0.90f, 0.92f, 1.0f };

    Math::Vec4f baseColor = ((ix + iz) % 2 == 0) ? grassGreen1 : grassGreen2;

    if (y < 60.0f) {
        return baseColor;
    } else if (y < 180.0f) {
        f32 t = (y - 60.0f) / 120.0f;
        return baseColor * (1.0f - t) + earthOlive * t;
    } else if (y < 350.0f) {
        f32 t = (y - 180.0f) / 170.0f;
        return earthOlive * (1.0f - t) + rockGray * t;
    } else {
        f32 t = std::clamp((y - 350.0f) / 150.0f, 0.0f, 1.0f);
        return rockGray * (1.0f - t) + snowWhite * t;
    }
}

static TerrainChunk BuildChunk(
    ID3D12Device* device,
    f32 x0, f32 x1,
    f32 z0, f32 z1,
    int subdivisions,
    u32 lodLevel)
{
    TerrainChunk chunk;
    chunk.lodLevel = lodLevel;

    std::vector<Vertex3D> vertices;
    std::vector<u32> indices;

    f32 stepX = (x1 - x0) / static_cast<f32>(subdivisions);
    f32 stepZ = (z1 - z0) / static_cast<f32>(subdivisions);

    f32 minY =  1e9f;
    f32 maxY = -1e9f;

    // Grid vertices
    for (int iz = 0; iz <= subdivisions; ++iz) {
        f32 z = z0 + static_cast<f32>(iz) * stepZ;
        for (int ix = 0; ix <= subdivisions; ++ix) {
            f32 x = x0 + static_cast<f32>(ix) * stepX;
            f32 y = TerrainSystem::SampleElevation(x, z);

            minY = std::min(minY, y);
            maxY = std::max(maxY, y);

            Math::Vec3f normal = TerrainSystem::SampleNormal(x, z);
            Math::Vec4f color = ComputeTerrainColor(y, ix, iz);

            vertices.push_back({ { x, y, z }, normal, color });
        }
    }

    // Grid triangle indices
    int stride = subdivisions + 1;
    for (int iz = 0; iz < subdivisions; ++iz) {
        for (int ix = 0; ix < subdivisions; ++ix) {
            u32 topLeft     = static_cast<u32>(iz * stride + ix);
            u32 topRight    = topLeft + 1;
            u32 bottomLeft  = static_cast<u32>((iz + 1) * stride + ix);
            u32 bottomRight = bottomLeft + 1;

            indices.push_back(topLeft);
            indices.push_back(bottomLeft);
            indices.push_back(topRight);

            indices.push_back(topRight);
            indices.push_back(bottomLeft);
            indices.push_back(bottomRight);
        }
    }

    chunk.minBoundsGfx = { x0, minY, z0 };
    chunk.maxBoundsGfx = { x1, maxY, z1 };

    static_cast<void>(chunk.mesh.Create(device, vertices, indices));
    return chunk;
}

bool TerrainSystem::Initialize(ID3D12Device* device) {
    if (!device) {
        return false;
    }

    m_chunks.clear();

    // LOD 0: Airport core and immediate surroundings: 4 chunks of 2000m x 2000m each
    // Spans [-2000, 2000] in X and Z
    constexpr f32 CORE_HALF = 2000.0f;
    constexpr int CORE_SUBDIV = 32;
    for (int iz = 0; iz < 2; ++iz) {
        for (int ix = 0; ix < 2; ++ix) {
            f32 x0 = -CORE_HALF + static_cast<f32>(ix) * CORE_HALF;
            f32 x1 = x0 + CORE_HALF;
            f32 z0 = -CORE_HALF + static_cast<f32>(iz) * CORE_HALF;
            f32 z1 = z0 + CORE_HALF;
            m_chunks.push_back(BuildChunk(device, x0, x1, z0, z1, CORE_SUBDIV, 0));
        }
    }

    // LOD 1: Mid Regional Ring: 12 chunks of 3000m x 3000m covering [-8000, 8000] around core
    constexpr f32 MID_STEP = 4000.0f;
    constexpr int MID_SUBDIV = 24;
    for (f32 z = -8000.0f; z < 8000.0f; z += MID_STEP) {
        for (f32 x = -8000.0f; x < 8000.0f; x += MID_STEP) {
            // Skip the inner LOD 0 region [-2000, 2000]
            if (x >= -2000.0f && x + MID_STEP <= 2000.0f &&
                z >= -2000.0f && z + MID_STEP <= 2000.0f) {
                continue;
            }
            m_chunks.push_back(BuildChunk(device, x, x + MID_STEP, z, z + MID_STEP, MID_SUBDIV, 1));
        }
    }

    // LOD 2: Distant Mountains Ring: 12 chunks of 10000m x 10000m covering [-20000, 20000]
    constexpr f32 FAR_STEP = 10000.0f;
    constexpr int FAR_SUBDIV = 20;
    for (f32 z = -20000.0f; z < 20000.0f; z += FAR_STEP) {
        for (f32 x = -20000.0f; x < 20000.0f; x += FAR_STEP) {
            // Skip inner region [-8000, 8000]
            if (x >= -8000.0f && x + FAR_STEP <= 8000.0f &&
                z >= -8000.0f && z + FAR_STEP <= 8000.0f) {
                continue;
            }
            m_chunks.push_back(BuildChunk(device, x, x + FAR_STEP, z, z + FAR_STEP, FAR_SUBDIV, 2));
        }
    }

    m_initialized = true;
    return true;
}

void TerrainSystem::Shutdown() {
    for (auto& chunk : m_chunks) {
        chunk.mesh.Destroy();
    }
    m_chunks.clear();
    m_initialized = false;
}

void TerrainSystem::Draw(
    ID3D12GraphicsCommandList* cmdList,
    const Math::Frustum& cameraFrustum,
    const Math::Vec3f& cameraPosGfx) noexcept
{
    if (!cmdList || !m_initialized) {
        return;
    }

    m_renderedChunks = 0;
    m_culledChunks = 0;

    for (const auto& chunk : m_chunks) {
        // Compute camera-relative chunk bounds to test against camera-relative frustum
        Math::Vec3f minRel = chunk.minBoundsGfx - cameraPosGfx;
        Math::Vec3f maxRel = chunk.maxBoundsGfx - cameraPosGfx;

        if (cameraFrustum.IntersectsAABB(minRel, maxRel)) {
            chunk.mesh.Draw(cmdList);
            m_renderedChunks++;
        } else {
            m_culledChunks++;
        }
    }
}

} // namespace LokYS::Graphics
