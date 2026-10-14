#include "LokYS/graphics/ApproachLightingSystem.hpp"
#include <cmath>
#include <algorithm>

namespace LokYS::Graphics {

namespace {

void AddLightBox(
    std::vector<Vertex3D>& verts,
    std::vector<u32>& inds,
    const Math::Vec3f& center,
    f32 halfSize,
    const Math::Vec4f& color)
{
    f32 hs = halfSize;
    Math::Vec3f p[8] = {
        { center.x - hs, center.y - hs, center.z - hs },
        { center.x + hs, center.y - hs, center.z - hs },
        { center.x + hs, center.y + hs, center.z - hs },
        { center.x - hs, center.y + hs, center.z - hs },
        { center.x - hs, center.y - hs, center.z + hs },
        { center.x + hs, center.y - hs, center.z + hs },
        { center.x + hs, center.y + hs, center.z + hs },
        { center.x - hs, center.y + hs, center.z + hs },
    };

    // 6 faces (24 vertices with normals)
    auto AddFace = [&](int i0, int i1, int i2, int i3, const Math::Vec3f& n) {
        u32 fBase = static_cast<u32>(verts.size());
        verts.push_back({ p[i0], n, color });
        verts.push_back({ p[i1], n, color });
        verts.push_back({ p[i2], n, color });
        verts.push_back({ p[i3], n, color });

        inds.push_back(fBase + 0);
        inds.push_back(fBase + 1);
        inds.push_back(fBase + 2);
        inds.push_back(fBase + 0);
        inds.push_back(fBase + 2);
        inds.push_back(fBase + 3);
    };

    AddFace(0, 3, 2, 1, { 0.0f, 0.0f, -1.0f }); // Front
    AddFace(5, 6, 7, 4, { 0.0f, 0.0f, +1.0f }); // Back
    AddFace(4, 7, 3, 0, { -1.0f, 0.0f, 0.0f }); // Left
    AddFace(1, 2, 6, 5, { +1.0f, 0.0f, 0.0f }); // Right
    AddFace(3, 7, 6, 2, { 0.0f, +1.0f, 0.0f }); // Top
    AddFace(4, 0, 1, 5, { 0.0f, -1.0f, 0.0f }); // Bottom
}

} // namespace

bool ApproachLightingSystem::Initialize(ID3D12Device* device) {
    if (!device) return false;

    BuildStaticBarsMesh(device);
    BuildDynamicStrobesMeshes(device);

    m_initialized = true;
    return true;
}

void ApproachLightingSystem::Shutdown() {
    m_staticBarsMesh.Shutdown();
    for (auto& m : m_rabbitStrobes) {
        m.Shutdown();
    }
    m_reilStrobesMesh.Shutdown();
    m_initialized = false;
}

void ApproachLightingSystem::BuildStaticBarsMesh(ID3D12Device* device) {
    std::vector<Vertex3D> verts;
    std::vector<u32> inds;

    Math::Vec4f colWhite{ 1.0f, 1.0f, 0.95f, 1.0f };
    Math::Vec4f colGreen{ 0.1f, 1.0f, 0.25f, 1.0f };
    Math::Vec4f colRed{ 1.0f, 0.12f, 0.12f, 1.0f };

    // 1. KLOK Runway 36 Approach Lighting System (ALSF-1 720m array)
    // Runway threshold is at Z_gfx = -800m. Approach path extends South to Z = -1520m.
    // 24 light bar stations spaced every 30m
    for (int st = 1; st <= 24; ++st) {
        f32 zPos = -800.0f - static_cast<f32>(st * 30);
        f32 yPos = 0.4f; // Elevated 40cm above ground

        // Centerline 5-lamp bar (width 4m)
        for (int lamp = -2; lamp <= 2; ++lamp) {
            f32 xPos = static_cast<f32>(lamp) * 0.9f;
            AddLightBox(verts, inds, { xPos, yPos, zPos }, 0.25f, colWhite);
        }

        // 100-ft (30m) Crossbars at 300m (st = 10, Z = -1100m)
        if (st == 10) {
            for (int cb = -15; cb <= 15; ++cb) {
                if (std::abs(cb) <= 2) continue; // Skip centerline overlap
                f32 xPos = static_cast<f32>(cb) * 1.0f;
                AddLightBox(verts, inds, { xPos, yPos, zPos }, 0.25f, colWhite);
            }
        }

        // 150m Crossbar (st = 5, Z = -950m)
        if (st == 5) {
            for (int cb = -8; cb <= 8; ++cb) {
                if (std::abs(cb) <= 2) continue;
                f32 xPos = static_cast<f32>(cb) * 1.0f;
                AddLightBox(verts, inds, { xPos, yPos, zPos }, 0.25f, colWhite);
            }
        }

        // Red side termination wing bars at st = 1 and st = 2 (Z = -830m and -860m)
        if (st <= 2) {
            for (int wb = -12; wb <= -4; ++wb) {
                AddLightBox(verts, inds, { static_cast<f32>(wb) * 1.0f, yPos, zPos }, 0.25f, colRed);
            }
            for (int wb = 4; wb <= 12; ++wb) {
                AddLightBox(verts, inds, { static_cast<f32>(wb) * 1.0f, yPos, zPos }, 0.25f, colRed);
            }
        }
    }

    // 2. Green Runway 36 Threshold Bar (at Z = -800m spanning 45m runway width)
    for (int tb = -22; tb <= 22; ++tb) {
        f32 xPos = static_cast<f32>(tb) * 1.0f;
        AddLightBox(verts, inds, { xPos, 0.35f, -800.0f }, 0.28f, colGreen);
    }

    // 3. KMES Mesa Highlands Runway 09 Threshold & End lights
    // Center at Z_gfx = 14000m, X_gfx = -8000m, Y_gfx = 350.0m. Runway heading East (090).
    // Runway 09 threshold at X_gfx = -8700m (700m West of center)
    for (int tb = -15; tb <= 15; ++tb) {
        f32 zOffset = static_cast<f32>(tb) * 1.0f;
        AddLightBox(verts, inds, { -8700.0f, 350.35f, 14000.0f + zOffset }, 0.28f, colGreen);
        AddLightBox(verts, inds, { -7300.0f, 350.35f, 14000.0f + zOffset }, 0.28f, colRed);
    }

    m_staticBarsMesh = Mesh::CreateFromVertices(device, verts, inds);
}

void ApproachLightingSystem::BuildDynamicStrobesMeshes(ID3D12Device* device) {
    if (!device) return;

    Math::Vec4f colStrobe{ 1.0f, 1.0f, 1.0f, 1.0f }; // Pure high-intensity white strobe

    // 1. Prebuild each of the 21 "The Rabbit" sequenced strobe station meshes
    for (int i = 0; i < 21; ++i) {
        std::vector<Vertex3D> verts;
        std::vector<u32> inds;
        f32 zStrobe = -1520.0f + static_cast<f32>(i * 30);
        AddLightBox(verts, inds, { 0.0f, 0.75f, zStrobe }, 0.65f, colStrobe);
        m_rabbitStrobes[i] = Mesh::CreateFromVertices(device, verts, inds);
    }

    // 2. Prebuild REIL synchronized threshold strobe mesh (KLOK + KMES corners)
    {
        std::vector<Vertex3D> verts;
        std::vector<u32> inds;
        // KLOK threshold corners
        AddLightBox(verts, inds, { -25.0f, 0.60f, -800.0f }, 0.50f, colStrobe);
        AddLightBox(verts, inds, { +25.0f, 0.60f, -800.0f }, 0.50f, colStrobe);
        // KMES threshold corners
        AddLightBox(verts, inds, { -8700.0f, 350.60f, 13982.0f }, 0.50f, colStrobe);
        AddLightBox(verts, inds, { -8700.0f, 350.60f, 14018.0f }, 0.50f, colStrobe);
        m_reilStrobesMesh = Mesh::CreateFromVertices(device, verts, inds);
    }
}

void ApproachLightingSystem::Update(f64 /*dt*/, f64 simTime, ID3D12Device* /*device*/) noexcept {
    m_simTime = simTime;

    // "The Rabbit" sequenced strobe cycle: 1.0 second period
    f64 cycleFrac = std::fmod(simTime, 1.0);
    if (cycleFrac < 0.0) cycleFrac += 1.0;

    // 21 strobes fire sequentially over the first 0.70 seconds (33ms each), followed by 0.30s dark pause
    if (cycleFrac < 0.70) {
        m_status.activeRabbitIndex = static_cast<int>((cycleFrac / 0.70) * 21.0);
        m_status.activeRabbitIndex = std::clamp(m_status.activeRabbitIndex, 0, 20);
        m_status.rabbitProgress = static_cast<f32>(cycleFrac / 0.70);
    } else {
        m_status.activeRabbitIndex = -1; // Dark pause before next cycle
        m_status.rabbitProgress = 0.0f;
    }

    // REIL flashes at 1.0 Hz (100ms flash duration at start of second)
    m_status.reilFlashed = (cycleFrac < 0.10);
}

void ApproachLightingSystem::DrawStaticBars(ID3D12GraphicsCommandList* cmdList) const noexcept {
    if (cmdList && m_initialized) {
        m_staticBarsMesh.Draw(cmdList);
    }
}

void ApproachLightingSystem::DrawDynamicStrobes(ID3D12GraphicsCommandList* cmdList) const noexcept {
    if (!cmdList || !m_initialized) return;

    if (m_status.activeRabbitIndex >= 0 && m_status.activeRabbitIndex < 21) {
        m_rabbitStrobes[m_status.activeRabbitIndex].Draw(cmdList);
    }
    if (m_status.reilFlashed) {
        m_reilStrobesMesh.Draw(cmdList);
    }
}

} // namespace LokYS::Graphics
