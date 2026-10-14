#include "LokYS/graphics/ScenerySystem.hpp"
#include <cmath>
#include <vector>
#include <algorithm>

namespace LokYS::Graphics {

namespace {

constexpr f32 PI = 3.1415926535f;

void AddQuad(
    std::vector<Vertex3D>& v,
    std::vector<u32>& idx,
    const Math::Vec3f& p0,
    const Math::Vec3f& p1,
    const Math::Vec3f& p2,
    const Math::Vec3f& p3,
    const Math::Vec3f& normal,
    const Math::Vec4f& color)
{
    u32 base = static_cast<u32>(v.size());
    v.push_back({ p0, normal, color });
    v.push_back({ p1, normal, color });
    v.push_back({ p2, normal, color });
    v.push_back({ p3, normal, color });

    idx.push_back(base + 0);
    idx.push_back(base + 1);
    idx.push_back(base + 2);

    idx.push_back(base + 0);
    idx.push_back(base + 2);
    idx.push_back(base + 3);
}

void AddBox(
    std::vector<Vertex3D>& v,
    std::vector<u32>& idx,
    const Math::Vec3f& center,
    const Math::Vec3f& halfSize,
    const Math::Vec4f& color)
{
    f32 cx = center.x;
    f32 cy = center.y;
    f32 cz = center.z;
    f32 hx = halfSize.x;
    f32 hy = halfSize.y;
    f32 hz = halfSize.z;

    // +Y (Top)
    AddQuad(v, idx,
        { cx - hx, cy + hy, cz - hz }, { cx - hx, cy + hy, cz + hz },
        { cx + hx, cy + hy, cz + hz }, { cx + hx, cy + hy, cz - hz },
        { 0.0f, 1.0f, 0.0f }, color);

    // -Y (Bottom)
    AddQuad(v, idx,
        { cx - hx, cy - hy, cz + hz }, { cx - hx, cy - hy, cz - hz },
        { cx + hx, cy - hy, cz - hz }, { cx + hx, cy - hy, cz + hz },
        { 0.0f, -1.0f, 0.0f }, color * 0.7f);

    // +Z (North / Front)
    AddQuad(v, idx,
        { cx - hx, cy - hy, cz + hz }, { cx + hx, cy - hy, cz + hz },
        { cx + hx, cy + hy, cz + hz }, { cx - hx, cy + hy, cz + hz },
        { 0.0f, 0.0f, 1.0f }, color * 0.9f);

    // -Z (South / Back)
    AddQuad(v, idx,
        { cx + hx, cy - hy, cz - hz }, { cx - hx, cy - hy, cz - hz },
        { cx - hx, cy + hy, cz - hz }, { cx + hx, cy + hy, cz - hz },
        { 0.0f, 0.0f, -1.0f }, color * 0.8f);

    // +X (East / Right)
    AddQuad(v, idx,
        { cx + hx, cy - hy, cz + hz }, { cx + hx, cy - hy, cz - hz },
        { cx + hx, cy + hy, cz - hz }, { cx + hx, cy + hy, cz + hz },
        { 1.0f, 0.0f, 0.0f }, color * 0.85f);

    // -X (West / Left)
    AddQuad(v, idx,
        { cx - hx, cy - hy, cz - hz }, { cx - hx, cy - hy, cz + hz },
        { cx - hx, cy + hy, cz + hz }, { cx - hx, cy + hy, cz - hz },
        { -1.0f, 0.0f, 0.0f }, color * 0.85f);
}

} // namespace

bool ScenerySystem::Initialize(ID3D12Device* device) {
    if (!device) {
        return false;
    }

    BuildBuildings(device);
    BuildRunwayLights(device);
    RebuildPapiMesh(device);
    RebuildWindsockMesh(device);

    m_initialized = true;
    return true;
}

void ScenerySystem::Shutdown() {
    m_windsockMesh.Destroy();
    m_papiMesh.Destroy();
    m_edgeLightsMesh.Destroy();
    m_buildingsMesh.Destroy();
    m_initialized = false;
}

void ScenerySystem::BuildBuildings(ID3D12Device* device) {
    std::vector<Vertex3D> v;
    std::vector<u32> idx;

    const Math::Vec4f concreteGray{ 0.72f, 0.72f, 0.74f, 1.0f };
    const Math::Vec4f darkGlass{ 0.15f, 0.35f, 0.30f, 1.0f };
    const Math::Vec4f roofWhite{ 0.90f, 0.90f, 0.92f, 1.0f };
    const Math::Vec4f hangarBlue{ 0.28f, 0.38f, 0.52f, 1.0f };
    const Math::Vec4f metalSilver{ 0.76f, 0.77f, 0.79f, 1.0f };
    const Math::Vec4f terminalTan{ 0.74f, 0.70f, 0.62f, 1.0f };
    const Math::Vec4f redObstruction{ 0.95f, 0.15f, 0.15f, 1.0f };

    // 1. Air Traffic Control (ATC) Tower at (120, 0, -200)
    // Tower Shaft: 10m x 10m base, 32m height
    AddBox(v, idx, { 120.0f, 16.0f, -200.0f }, { 5.0f, 16.0f, 5.0f }, concreteGray);
    // Observation Glass Cab: 14m x 14m, 6m height at y = 32..38
    AddBox(v, idx, { 120.0f, 35.0f, -200.0f }, { 7.0f, 3.0f, 7.0f }, darkGlass);
    // Cab Roof
    AddBox(v, idx, { 120.0f, 38.5f, -200.0f }, { 8.0f, 0.5f, 8.0f }, roofWhite);
    // Antenna Mast with Red Beacon
    AddBox(v, idx, { 120.0f, 43.0f, -200.0f }, { 0.3f, 4.0f, 0.3f }, metalSilver);
    AddBox(v, idx, { 120.0f, 47.3f, -200.0f }, { 0.5f, 0.3f, 0.5f }, redObstruction);

    // 2. Main Maintenance Hangar at (160, 0, -450): 60m x 40m, 14m height
    AddBox(v, idx, { 160.0f, 7.0f, -450.0f }, { 30.0f, 7.0f, 20.0f }, hangarBlue);
    AddBox(v, idx, { 160.0f, 14.5f, -450.0f }, { 31.0f, 0.5f, 21.0f }, roofWhite);

    // 3. Secondary T-Hangars at (160, 0, -280) and (160, 0, -620)
    AddBox(v, idx, { 160.0f, 4.5f, -280.0f }, { 18.0f, 4.5f, 12.0f }, metalSilver);
    AddBox(v, idx, { 160.0f, 9.2f, -280.0f }, { 19.0f, 0.2f, 13.0f }, roofWhite);

    AddBox(v, idx, { 160.0f, 4.5f, -620.0f }, { 18.0f, 4.5f, 12.0f }, metalSilver);
    AddBox(v, idx, { 160.0f, 9.2f, -620.0f }, { 19.0f, 0.2f, 13.0f }, roofWhite);

    // 4. Passenger Terminal at (120, 0, 50): 45m x 24m, 10m height
    AddBox(v, idx, { 120.0f, 5.0f, 50.0f }, { 22.5f, 5.0f, 12.0f }, terminalTan);
    AddBox(v, idx, { 120.0f, 10.3f, 50.0f }, { 23.5f, 0.3f, 13.0f }, roofWhite);

    static_cast<void>(m_buildingsMesh.Create(device, v, idx));
}

void ScenerySystem::BuildRunwayLights(ID3D12Device* device) {
    std::vector<Vertex3D> v;
    std::vector<u32> idx;

    const Math::Vec4f fixtureYellow{ 0.85f, 0.75f, 0.15f, 1.0f };
    const Math::Vec4f lightWhite{ 1.0f, 1.0f, 1.0f, 1.0f };
    const Math::Vec4f lightYellow{ 1.0f, 0.88f, 0.20f, 1.0f };
    const Math::Vec4f lightGreen{ 0.15f, 1.0f, 0.25f, 1.0f };
    const Math::Vec4f lightRed{ 1.0f, 0.10f, 0.10f, 1.0f };

    // Runway 36 Edge Lights: X = +/-23.5m, Z from -1000m to +1000m every 50m
    constexpr f32 EDGE_X = 23.5f;
    for (f32 z = -1000.0f; z <= 1000.0f; z += 50.0f) {
        // Caution zone in last 600m (Z > 400m)
        Math::Vec4f bulbCol = (z > 400.0f) ? lightYellow : lightWhite;

        // West / Left edge
        AddBox(v, idx, { -EDGE_X, 0.15f, z }, { 0.20f, 0.15f, 0.20f }, fixtureYellow);
        AddBox(v, idx, { -EDGE_X, 0.45f, z }, { 0.15f, 0.15f, 0.15f }, bulbCol);

        // East / Right edge
        AddBox(v, idx, {  EDGE_X, 0.15f, z }, { 0.20f, 0.15f, 0.20f }, fixtureYellow);
        AddBox(v, idx, {  EDGE_X, 0.45f, z }, { 0.15f, 0.15f, 0.15f }, bulbCol);
    }

    // South Threshold Green Light Bar at Z = -1000m (8 lights across runway width)
    for (int b = 0; b < 8; ++b) {
        f32 x = -20.0f + static_cast<f32>(b) * (40.0f / 7.0f);
        AddBox(v, idx, { x, 0.15f, -1001.0f }, { 0.20f, 0.15f, 0.20f }, fixtureYellow);
        AddBox(v, idx, { x, 0.45f, -1001.0f }, { 0.18f, 0.15f, 0.18f }, lightGreen);
    }

    // North Runway End Red Light Bar at Z = +1000m (8 lights across runway width)
    for (int b = 0; b < 8; ++b) {
        f32 x = -20.0f + static_cast<f32>(b) * (40.0f / 7.0f);
        AddBox(v, idx, { x, 0.15f, 1001.0f }, { 0.20f, 0.15f, 0.20f }, fixtureYellow);
        AddBox(v, idx, { x, 0.45f, 1001.0f }, { 0.18f, 0.15f, 0.18f }, lightRed);
    }

    static_cast<void>(m_edgeLightsMesh.Create(device, v, idx));
}

PapiStatus ScenerySystem::ComputePapiStatus(const Math::Vec3f& eyeGfx) noexcept {
    PapiStatus status{};

    // PAPI location: X = -35.0m, Z = -700.0m (300m past South threshold)
    f32 dx = eyeGfx.x - (-35.0f);
    f32 dz = eyeGfx.z - (-700.0f);
    f32 dy = eyeGfx.y - 0.5f;

    // Approaching from South towards Runway 36 (dz < 0)
    if (dz < -10.0f) {
        f32 dist = std::sqrt(dx * dx + dz * dz);
        f32 angleDeg = std::atan2(dy, dist) * (180.0f / PI);
        status.approachAngleDeg = angleDeg;
        status.inApproachSector = true;

        // Standard 3-degree glidepath thresholds (ascending order)
        status.lights[0] = (angleDeg >= 2.50f) ? PapiColor::White : PapiColor::Red;
        status.lights[1] = (angleDeg >= 2.83f) ? PapiColor::White : PapiColor::Red;
        status.lights[2] = (angleDeg >= 3.17f) ? PapiColor::White : PapiColor::Red;
        status.lights[3] = (angleDeg >= 3.50f) ? PapiColor::White : PapiColor::Red;
    } else {
        status.inApproachSector = false;
        status.approachAngleDeg = 0.0f;
        status.lights.fill(PapiColor::Red);
    }

    return status;
}

void ScenerySystem::RebuildPapiMesh(ID3D12Device* device) {
    std::vector<Vertex3D> v;
    std::vector<u32> idx;

    const Math::Vec4f housingColor{ 0.22f, 0.22f, 0.24f, 1.0f };
    const Math::Vec4f whiteBeam{ 1.0f, 1.0f, 1.0f, 1.0f };
    const Math::Vec4f redBeam{ 1.0f, 0.08f, 0.08f, 1.0f };

    // 4 PAPI light housings spaced 9m apart: X = -48.5, -39.5, -30.5, -21.5 at Z = -700
    constexpr f32 PAPI_Z = -700.0f;
    constexpr f32 PAPI_X_START = -48.5f;
    constexpr f32 PAPI_SPACING = 9.0f;

    for (int i = 0; i < 4; ++i) {
        f32 x = PAPI_X_START + static_cast<f32>(i) * PAPI_SPACING;

        // Housing box: 1.6m wide, 0.8m high, 1.2m deep
        AddBox(v, idx, { x, 0.4f, PAPI_Z }, { 0.8f, 0.4f, 0.6f }, housingColor);

        // Front face lens (facing South / -Z)
        Math::Vec4f lensColor = (m_papiStatus.lights[i] == PapiColor::White) ? whiteBeam : redBeam;
        AddQuad(v, idx,
            { x + 0.6f, 0.15f, PAPI_Z - 0.61f },
            { x - 0.6f, 0.15f, PAPI_Z - 0.61f },
            { x - 0.6f, 0.65f, PAPI_Z - 0.61f },
            { x + 0.6f, 0.65f, PAPI_Z - 0.61f },
            { 0.0f, 0.0f, -1.0f },
            lensColor);
    }

    m_papiMesh.Destroy();
    static_cast<void>(m_papiMesh.Create(device, v, idx));
}

void ScenerySystem::RebuildWindsockMesh(ID3D12Device* device) {
    std::vector<Vertex3D> v;
    std::vector<u32> idx;

    const Math::Vec4f poleColor{ 0.75f, 0.75f, 0.78f, 1.0f };
    const Math::Vec4f sockOrange{ 1.0f, 0.42f, 0.08f, 1.0f };
    const Math::Vec4f sockWhite{ 0.95f, 0.95f, 0.98f, 1.0f };

    // Pole at (-50, 0, -650), height 6.0m
    const Math::Vec3f polePos{ -50.0f, 3.0f, -650.0f };
    AddBox(v, idx, polePos, { 0.15f, 3.0f, 0.15f }, poleColor);

    // Swivel mount top ring at y = 6.0m
    AddBox(v, idx, { -50.0f, 6.1f, -650.0f }, { 0.4f, 0.1f, 0.4f }, poleColor);

    // Windsock fabric cone pointing downwind
    // Yaw angle: m_windsockYawRad (0 = pointing North +Z)
    // Droop angle: m_windsockDroop (0 = horizontal, positive = drooping down)
    f32 cosY = std::cos(m_windsockYawRad);
    f32 sinY = std::sin(m_windsockYawRad);

    f32 droopCos = std::cos(m_windsockDroop);
    f32 droopSin = std::sin(m_windsockDroop);

    // 4 sections of the cone (striped orange and white)
    constexpr int NUM_SEGS = 4;
    constexpr f32 SEG_LEN = 0.75f;
    f32 radius = 0.40f;

    Math::Vec3f prevCenter{ -50.0f, 6.0f, -650.0f };

    for (int s = 0; s < NUM_SEGS; ++s) {
        f32 nextRadius = radius * 0.75f;

        // Step along cone axis: (sinY, -droopSin, cosY) * SEG_LEN
        Math::Vec3f stepDir{ sinY * droopCos, -droopSin, cosY * droopCos };
        Math::Vec3f nextCenter = prevCenter + stepDir * SEG_LEN;

        Math::Vec4f segCol = (s % 2 == 0) ? sockOrange : sockWhite;

        // Simplified 8-sided cone segment approximated with bounding quad slices
        AddBox(v, idx, (prevCenter + nextCenter) * 0.5f,
            { radius * 0.8f, radius * 0.8f, SEG_LEN * 0.5f }, segCol);

        prevCenter = nextCenter;
        radius = nextRadius;
    }

    m_windsockMesh.Destroy();
    static_cast<void>(m_windsockMesh.Create(device, v, idx));
}

void ScenerySystem::Update(
    const Math::Vec3f& cameraPosGfx,
    f64 windSpeedMps,
    f64 windDirectionDeg,
    ID3D12Device* device) noexcept
{
    // 1. Update PAPI light states
    PapiStatus newPapi = ComputePapiStatus(cameraPosGfx);
    bool papiChanged = false;
    for (size_t i = 0; i < 4; ++i) {
        if (newPapi.lights[i] != m_prevPapiColors[i]) {
            papiChanged = true;
            break;
        }
    }

    if (papiChanged) {
        m_papiStatus = newPapi;
        m_prevPapiColors = newPapi.lights;
        if (device) {
            RebuildPapiMesh(device);
        }
    }

    // 2. Update Windsock orientation
    // Wind blows FROM windDirectionDeg (0 = North). Windsock points downwind (FROM + 180).
    // In graphics space: X is East, Z is North.
    // Angle in radians: 0 points along +Z (North).
    f64 downwindDeg = windDirectionDeg + 180.0;
    if (downwindDeg >= 360.0) {
        downwindDeg -= 360.0;
    }

    f32 targetYaw = static_cast<f32>(downwindDeg * (PI / 180.0));
    // Wind speed droop: 0 m/s -> droop 50 deg; >= 7.5 m/s (15 kt) -> droop 5 deg (near horizontal)
    f32 speedFactor = std::clamp(static_cast<f32>(windSpeedMps) / 7.5f, 0.0f, 1.0f);
    f32 targetDroop = (1.0f - speedFactor) * 0.85f + 0.08f;

    bool windChanged = (std::abs(windSpeedMps - m_prevWindSpeed) > 0.5) ||
                       (std::abs(windDirectionDeg - m_prevWindDir) > 2.0);

    if (windChanged) {
        m_windsockYawRad = targetYaw;
        m_windsockDroop = targetDroop;
        m_prevWindSpeed = windSpeedMps;
        m_prevWindDir = windDirectionDeg;
        if (device) {
            RebuildWindsockMesh(device);
        }
    }
}

void ScenerySystem::DrawBuildings(ID3D12GraphicsCommandList* cmdList) const noexcept {
    if (cmdList && m_initialized) {
        m_buildingsMesh.Draw(cmdList);
    }
}

void ScenerySystem::DrawRunwayLights(ID3D12GraphicsCommandList* cmdList) const noexcept {
    if (cmdList && m_initialized) {
        m_edgeLightsMesh.Draw(cmdList);
    }
}

void ScenerySystem::DrawPapi(ID3D12GraphicsCommandList* cmdList) const noexcept {
    if (cmdList && m_initialized) {
        m_papiMesh.Draw(cmdList);
    }
}

void ScenerySystem::DrawWindsock(ID3D12GraphicsCommandList* cmdList) const noexcept {
    if (cmdList && m_initialized) {
        m_windsockMesh.Draw(cmdList);
    }
}

} // namespace LokYS::Graphics
