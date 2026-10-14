#include "LokYS/graphics/Mesh.hpp"
#include <cstring>
#include <cmath>

namespace LokYS::Graphics {

bool Mesh::Create(
    ID3D12Device* device,
    const std::vector<Vertex3D>& vertices,
    const std::vector<u32>& indices)
{
    if (!device || vertices.empty() || indices.empty()) {
        return false;
    }

    m_vertexCount = static_cast<u32>(vertices.size());
    m_indexCount = static_cast<u32>(indices.size());

    const u64 vbSize = vertices.size() * sizeof(Vertex3D);
    const u64 ibSize = indices.size() * sizeof(u32);

    D3D12_HEAP_PROPERTIES uploadHeap{};
    uploadHeap.Type = D3D12_HEAP_TYPE_UPLOAD;
    uploadHeap.CPUPageProperty = D3D12_CPU_PAGE_PROPERTY_UNKNOWN;
    uploadHeap.MemoryPoolPreference = D3D12_MEMORY_POOL_UNKNOWN;
    uploadHeap.CreationNodeMask = 1;
    uploadHeap.VisibleNodeMask = 1;

    // 1. Create Vertex Buffer in Upload Heap
    D3D12_RESOURCE_DESC vbDesc{};
    vbDesc.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER;
    vbDesc.Width = vbSize;
    vbDesc.Height = 1;
    vbDesc.DepthOrArraySize = 1;
    vbDesc.MipLevels = 1;
    vbDesc.Format = DXGI_FORMAT_UNKNOWN;
    vbDesc.SampleDesc.Count = 1;
    vbDesc.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;

    if (FAILED(device->CreateCommittedResource(
            &uploadHeap,
            D3D12_HEAP_FLAG_NONE,
            &vbDesc,
            D3D12_RESOURCE_STATE_GENERIC_READ,
            nullptr,
            IID_PPV_ARGS(&m_vertexBuffer)))) {
        return false;
    }

    void* vbData = nullptr;
    D3D12_RANGE readRange{ 0, 0 };
    if (FAILED(m_vertexBuffer->Map(0, &readRange, &vbData))) {
        return false;
    }
    std::memcpy(vbData, vertices.data(), vbSize);
    m_vertexBuffer->Unmap(0, nullptr);

    m_vbView.BufferLocation = m_vertexBuffer->GetGPUVirtualAddress();
    m_vbView.StrideInBytes = sizeof(Vertex3D);
    m_vbView.SizeInBytes = static_cast<UINT>(vbSize);

    // 2. Create Index Buffer in Upload Heap
    D3D12_RESOURCE_DESC ibDesc{};
    ibDesc.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER;
    ibDesc.Width = ibSize;
    ibDesc.Height = 1;
    ibDesc.DepthOrArraySize = 1;
    ibDesc.MipLevels = 1;
    ibDesc.Format = DXGI_FORMAT_UNKNOWN;
    ibDesc.SampleDesc.Count = 1;
    ibDesc.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;

    if (FAILED(device->CreateCommittedResource(
            &uploadHeap,
            D3D12_HEAP_FLAG_NONE,
            &ibDesc,
            D3D12_RESOURCE_STATE_GENERIC_READ,
            nullptr,
            IID_PPV_ARGS(&m_indexBuffer)))) {
        return false;
    }

    void* ibData = nullptr;
    if (FAILED(m_indexBuffer->Map(0, &readRange, &ibData))) {
        return false;
    }
    std::memcpy(ibData, indices.data(), ibSize);
    m_indexBuffer->Unmap(0, nullptr);

    m_ibView.BufferLocation = m_indexBuffer->GetGPUVirtualAddress();
    m_ibView.Format = DXGI_FORMAT_R32_UINT;
    m_ibView.SizeInBytes = static_cast<UINT>(ibSize);

    return true;
}

void Mesh::Draw(ID3D12GraphicsCommandList* cmdList) const noexcept {
    if (!cmdList || m_indexCount == 0) {
        return;
    }
    cmdList->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
    cmdList->IASetVertexBuffers(0, 1, &m_vbView);
    cmdList->IASetIndexBuffer(&m_ibView);
    cmdList->DrawIndexedInstanced(m_indexCount, 1, 0, 0, 0);
}

// Helper to push a colored quad (2 triangles)
static void AddQuad(
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

// Helper to push a 3D box
static void AddBox(
    std::vector<Vertex3D>& v,
    std::vector<u32>& idx,
    const Math::Vec3f& center,
    const Math::Vec3f& halfSize,
    const Math::Vec4f& color)
{
    f32 hx = halfSize.x;
    f32 hy = halfSize.y;
    f32 hz = halfSize.z;
    f32 cx = center.x;
    f32 cy = center.y;
    f32 cz = center.z;

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

    // +Z (Front)
    AddQuad(v, idx,
        { cx - hx, cy - hy, cz + hz }, { cx + hx, cy - hy, cz + hz },
        { cx + hx, cy + hy, cz + hz }, { cx - hx, cy + hy, cz + hz },
        { 0.0f, 0.0f, 1.0f }, color * 0.9f);

    // -Z (Back)
    AddQuad(v, idx,
        { cx + hx, cy - hy, cz - hz }, { cx - hx, cy - hy, cz - hz },
        { cx - hx, cy + hy, cz - hz }, { cx + hx, cy + hy, cz - hz },
        { 0.0f, 0.0f, -1.0f }, color * 0.8f);

    // +X (Right)
    AddQuad(v, idx,
        { cx + hx, cy - hy, cz + hz }, { cx + hx, cy - hy, cz - hz },
        { cx + hx, cy + hy, cz - hz }, { cx + hx, cy + hy, cz + hz },
        { 1.0f, 0.0f, 0.0f }, color * 0.85f);

    // -X (Left)
    AddQuad(v, idx,
        { cx - hx, cy - hy, cz - hz }, { cx - hx, cy - hy, cz + hz },
        { cx - hx, cy + hy, cz + hz }, { cx - hx, cy + hy, cz - hz },
        { -1.0f, 0.0f, 0.0f }, color * 0.85f);
}

Mesh Mesh::CreateTerrain(ID3D12Device* device) {
    std::vector<Vertex3D> v;
    std::vector<u32> idx;

    // Large grid 16 km x 16 km with checker grass shades
    constexpr int GRID_CELLS = 32;
    constexpr f32 GRID_SIZE = 16000.0f;
    constexpr f32 CELL_SIZE = GRID_SIZE / static_cast<f32>(GRID_CELLS);
    constexpr f32 HALF_GRID = GRID_SIZE * 0.5f;

    const Math::Vec4f grassGreen1{ 0.28f, 0.52f, 0.24f, 1.0f };
    const Math::Vec4f grassGreen2{ 0.25f, 0.48f, 0.22f, 1.0f };

    for (int iz = 0; iz < GRID_CELLS; ++iz) {
        for (int ix = 0; ix < GRID_CELLS; ++ix) {
            f32 x0 = -HALF_GRID + ix * CELL_SIZE;
            f32 x1 = x0 + CELL_SIZE;
            f32 z0 = -HALF_GRID + iz * CELL_SIZE;
            f32 z1 = z0 + CELL_SIZE;

            Math::Vec4f col = ((ix + iz) % 2 == 0) ? grassGreen1 : grassGreen2;

            AddQuad(v, idx,
                { x0, 0.0f, z0 },
                { x0, 0.0f, z1 },
                { x1, 0.0f, z1 },
                { x1, 0.0f, z0 },
                { 0.0f, 1.0f, 0.0f },
                col);
        }
    }

    Mesh mesh;
    static_cast<void>(mesh.Create(device, v, idx));
    return mesh;
}


Mesh Mesh::CreateRunway(ID3D12Device* device) {
    std::vector<Vertex3D> v;
    std::vector<u32> idx;

    // Runway dimensions: Length 2000m (-1000m to +1000m along Z), Width 45m (-22.5m to +22.5m along X)
    constexpr f32 RWY_HALF_LEN = 1000.0f;
    constexpr f32 RWY_HALF_WIDTH = 22.5f;
    constexpr f32 RWY_Y = 0.02f; // Slight offset to prevent z-fighting with ground

    const Math::Vec4f asphaltColor{ 0.18f, 0.18f, 0.20f, 1.0f };
    const Math::Vec4f stripeWhite{ 0.96f, 0.96f, 0.96f, 1.0f };
    const Math::Vec4f edgeYellow{ 0.90f, 0.80f, 0.15f, 1.0f };

    // 1. Main Asphalt Surface
    AddQuad(v, idx,
        { -RWY_HALF_WIDTH, RWY_Y, -RWY_HALF_LEN },
        { -RWY_HALF_WIDTH, RWY_Y,  RWY_HALF_LEN },
        {  RWY_HALF_WIDTH, RWY_Y,  RWY_HALF_LEN },
        {  RWY_HALF_WIDTH, RWY_Y, -RWY_HALF_LEN },
        { 0.0f, 1.0f, 0.0f },
        asphaltColor);

    // 2. Yellow Edge Lines
    constexpr f32 EDGE_WIDTH = 0.8f;
    constexpr f32 MARK_Y = RWY_Y + 0.01f;

    // Left edge line
    AddQuad(v, idx,
        { -RWY_HALF_WIDTH, MARK_Y, -RWY_HALF_LEN },
        { -RWY_HALF_WIDTH, MARK_Y,  RWY_HALF_LEN },
        { -RWY_HALF_WIDTH + EDGE_WIDTH, MARK_Y,  RWY_HALF_LEN },
        { -RWY_HALF_WIDTH + EDGE_WIDTH, MARK_Y, -RWY_HALF_LEN },
        { 0.0f, 1.0f, 0.0f },
        edgeYellow);

    // Right edge line
    AddQuad(v, idx,
        { RWY_HALF_WIDTH - EDGE_WIDTH, MARK_Y, -RWY_HALF_LEN },
        { RWY_HALF_WIDTH - EDGE_WIDTH, MARK_Y,  RWY_HALF_LEN },
        { RWY_HALF_WIDTH, MARK_Y,  RWY_HALF_LEN },
        { RWY_HALF_WIDTH, MARK_Y, -RWY_HALF_LEN },
        { 0.0f, 1.0f, 0.0f },
        edgeYellow);

    // 3. Dashed White Centerlines (every 50m: 30m stripe, 20m gap)
    constexpr f32 CENTER_STRIPE_HALF_W = 0.9f;
    for (f32 z = -RWY_HALF_LEN + 60.0f; z < RWY_HALF_LEN - 60.0f; z += 50.0f) {
        AddQuad(v, idx,
            { -CENTER_STRIPE_HALF_W, MARK_Y, z },
            { -CENTER_STRIPE_HALF_W, MARK_Y, z + 30.0f },
            {  CENTER_STRIPE_HALF_W, MARK_Y, z + 30.0f },
            {  CENTER_STRIPE_HALF_W, MARK_Y, z },
            { 0.0f, 1.0f, 0.0f },
            stripeWhite);
    }

    // 4. Threshold Bars (Piano Keys) at both ends
    constexpr int NUM_BARS = 8;
    constexpr f32 BAR_WIDTH = 1.8f;
    constexpr f32 BAR_LEN = 30.0f;

    auto AddThresholdBars = [&](f32 zStart, f32 zEnd) {
        f32 spacing = (RWY_HALF_WIDTH * 2.0f - 8.0f) / static_cast<f32>(NUM_BARS);
        f32 startX = -RWY_HALF_WIDTH + 4.0f;
        for (int b = 0; b < NUM_BARS; ++b) {
            f32 x0 = startX + b * spacing;
            f32 x1 = x0 + BAR_WIDTH;
            AddQuad(v, idx,
                { x0, MARK_Y, zStart },
                { x0, MARK_Y, zEnd },
                { x1, MARK_Y, zEnd },
                { x1, MARK_Y, zStart },
                { 0.0f, 1.0f, 0.0f },
                stripeWhite);
        }
    };

    AddThresholdBars(-RWY_HALF_LEN + 5.0f, -RWY_HALF_LEN + 5.0f + BAR_LEN);
    AddThresholdBars(RWY_HALF_LEN - 5.0f - BAR_LEN, RWY_HALF_LEN - 5.0f);

    Mesh mesh;
    static_cast<void>(mesh.Create(device, v, idx));
    return mesh;
}


Mesh Mesh::CreateCessna172(ID3D12Device* device) {
    std::vector<Vertex3D> v;
    std::vector<u32> idx;

    // Aircraft Colors
    const Math::Vec4f whiteBody{ 0.94f, 0.94f, 0.95f, 1.0f };
    const Math::Vec4f blueStripe{ 0.12f, 0.35f, 0.75f, 1.0f };
    const Math::Vec4f darkGlass{ 0.12f, 0.18f, 0.28f, 1.0f };
    const Math::Vec4f blackTire{ 0.15f, 0.15f, 0.15f, 1.0f };
    const Math::Vec4f propBlade{ 0.20f, 0.20f, 0.22f, 1.0f };
    const Math::Vec4f propTip{ 0.95f, 0.85f, 0.10f, 1.0f };

    // 1. Cabin Fuselage (Main Cabin Body)
    // Local coords: +Z Forward (Nose), +X Right (Right Wing), +Y Up
    AddBox(v, idx,
        { 0.0f, 0.0f, 0.0f },
        { 0.55f, 0.55f, 1.6f },
        whiteBody);

    // Cheatline blue stripe across cabin sides
    AddBox(v, idx,
        { 0.0f, -0.05f, 0.0f },
        { 0.56f, 0.10f, 1.62f },
        blueStripe);

    // 2. Windshield (Front Cockpit Window)
    AddQuad(v, idx,
        { -0.50f, 0.15f, 1.4f },
        { -0.45f, 0.55f, 0.8f },
        {  0.45f, 0.55f, 0.8f },
        {  0.50f, 0.15f, 1.4f },
        { 0.0f, 0.707f, 0.707f },
        darkGlass);

    // Side windows (Left and Right)
    AddQuad(v, idx,
        { -0.56f, 0.15f, 0.8f },
        { -0.56f, 0.15f, -0.4f },
        { -0.56f, 0.50f, -0.4f },
        { -0.56f, 0.50f, 0.8f },
        { -1.0f, 0.0f, 0.0f },
        darkGlass);

    AddQuad(v, idx,
        { 0.56f, 0.15f, -0.4f },
        { 0.56f, 0.15f, 0.8f },
        { 0.56f, 0.50f, 0.8f },
        { 0.56f, 0.50f, -0.4f },
        { 1.0f, 0.0f, 0.0f },
        darkGlass);

    // 3. Engine Cowling / Nose
    AddBox(v, idx,
        { 0.0f, -0.05f, 2.0f },
        { 0.45f, 0.40f, 0.45f },
        whiteBody);

    // Propeller Spinner
    AddBox(v, idx,
        { 0.0f, -0.05f, 2.50f },
        { 0.15f, 0.15f, 0.12f },
        blueStripe);

    // Propeller Blades (2-blade fixed pitch)
    AddBox(v, idx,
        { 0.0f, -0.05f, 2.45f },
        { 0.04f, 0.85f, 0.02f },
        propBlade);

    // Propeller Yellow Tips
    AddBox(v, idx,
        { 0.0f, 0.75f, 2.45f },
        { 0.045f, 0.10f, 0.025f },
        propTip);
    AddBox(v, idx,
        { 0.0f, -0.85f, 2.45f },
        { 0.045f, 0.10f, 0.025f },
        propTip);

    // 4. Tail Cone (Tapering back from cabin to empennage)
    AddBox(v, idx,
        { 0.0f, 0.05f, -2.6f },
        { 0.30f, 0.35f, 1.1f },
        whiteBody);
    AddBox(v, idx,
        { 0.0f, 0.15f, -4.2f },
        { 0.18f, 0.25f, 0.8f },
        whiteBody);

    // 5. High Wings (Wingspan 11.0m, Chord 1.5m)
    // Mounted at top of cabin Y = +0.55m
    AddBox(v, idx,
        { 0.0f, 0.58f, 0.2f },
        { 5.50f, 0.08f, 0.75f },
        whiteBody);

    // Wing Leading Edge Blue Trim
    AddBox(v, idx,
        { 0.0f, 0.58f, 0.95f },
        { 5.51f, 0.07f, 0.05f },
        blueStripe);

    // Wing Struts (Left and Right)
    // Left strut
    AddBox(v, idx,
        { -1.8f, 0.05f, 0.2f },
        { 0.04f, 0.45f, 0.06f },
        whiteBody);
    // Right strut
    AddBox(v, idx,
        { 1.8f, 0.05f, 0.2f },
        { 0.04f, 0.45f, 0.06f },
        whiteBody);

    // 6. Horizontal Stabilizer & Elevators (Empennage)
    AddBox(v, idx,
        { 0.0f, 0.35f, -4.7f },
        { 1.70f, 0.04f, 0.45f },
        whiteBody);

    // 7. Vertical Stabilizer & Rudder (Swept Fin)
    AddBox(v, idx,
        { 0.0f, 1.05f, -4.5f },
        { 0.06f, 0.65f, 0.65f },
        blueStripe);

    // 8. Tricycle Landing Gear
    // Nose Gear Strut & Wheel
    AddBox(v, idx,
        { 0.0f, -0.65f, 1.8f },
        { 0.04f, 0.25f, 0.04f },
        whiteBody);
    AddBox(v, idx,
        { 0.0f, -0.90f, 1.8f },
        { 0.08f, 0.14f, 0.14f },
        blackTire);

    // Main Gear Struts & Wheels (Left and Right)
    // Left main gear
    AddBox(v, idx,
        { -0.95f, -0.55f, -0.2f },
        { 0.25f, 0.30f, 0.05f },
        whiteBody);
    AddBox(v, idx,
        { -1.15f, -0.90f, -0.2f },
        { 0.10f, 0.18f, 0.18f },
        blackTire);

    // Right main gear
    AddBox(v, idx,
        { 0.95f, -0.55f, -0.2f },
        { 0.25f, 0.30f, 0.05f },
        whiteBody);
    AddBox(v, idx,
        { 1.15f, -0.90f, -0.2f },
        { 0.10f, 0.18f, 0.18f },
        blackTire);

    Mesh mesh;
    static_cast<void>(mesh.Create(device, v, idx));
    return mesh;
}


} // namespace LokYS::Graphics
