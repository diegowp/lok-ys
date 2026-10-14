#include "LokYS/graphics/AvionicsRenderer.hpp"
#include <d3dcompiler.h>
#include <cmath>
#include <algorithm>

namespace LokYS::Graphics {

namespace {

// Embedded HLSL shader for 2D avionics instrument panel rendering
const char* g_avionicsShaderSource = R"(
cbuffer PanelConstants : register(b0) {
    float2 u_screenSize;
    float2 u_panelPos;
    float  u_panelScale;
    float  u_opacity;
    float2 u_padding;
};

struct VSInput {
    float3 position : POSITION;
    float3 normal   : NORMAL;
    float4 color    : COLOR;
};

struct PSInput {
    float4 position : SV_POSITION;
    float4 color    : COLOR;
};

PSInput VSMain(VSInput input) {
    PSInput output;
    float2 pixelPos = u_panelPos + input.position.xy * u_panelScale;
    float xNdc = (pixelPos.x / u_screenSize.x) * 2.0f - 1.0f;
    float yNdc = 1.0f - (pixelPos.y / u_screenSize.y) * 2.0f;
    output.position = float4(xNdc, yNdc, 0.0f, 1.0f);
    output.color = float4(input.color.rgb, input.color.a * u_opacity);
    return output;
}

float4 PSMain(PSInput input) : SV_TARGET {
    return input.color;
}
)";

constexpr f32 PI = 3.14159265358979323846f;
constexpr f32 DEG_TO_RAD = PI / 180.0f;

} // anonymous namespace

bool AvionicsRenderer::Initialize(ID3D12Device* device) {
    if (!device) return false;

    if (!CreateRootSignature(device)) return false;
    if (!CreatePipelineState(device)) return false;
    if (!CreateBuffers(device)) return false;

    m_vertices.reserve(MaxVertices);
    m_indices.reserve(MaxIndices);
    m_initialized = true;
    return true;
}

void AvionicsRenderer::Shutdown() {
    if (m_vertexBuffer && m_mappedVertices) {
        m_vertexBuffer->Unmap(0, nullptr);
        m_mappedVertices = nullptr;
    }
    if (m_indexBuffer && m_mappedIndices) {
        m_indexBuffer->Unmap(0, nullptr);
        m_mappedIndices = nullptr;
    }
    m_vertexBuffer.Reset();
    m_indexBuffer.Reset();
    m_pipelineState.Reset();
    m_rootSignature.Reset();
    m_initialized = false;
}

bool AvionicsRenderer::CreateRootSignature(ID3D12Device* device) {
    D3D12_ROOT_PARAMETER rootParam{};
    rootParam.ParameterType = D3D12_ROOT_PARAMETER_TYPE_32BIT_CONSTANTS;
    rootParam.Constants.ShaderRegister = 0;
    rootParam.Constants.RegisterSpace = 0;
    rootParam.Constants.Num32BitValues = 8; // 8 floats (32 bytes)
    rootParam.ShaderVisibility = D3D12_SHADER_VISIBILITY_VERTEX;

    D3D12_ROOT_SIGNATURE_DESC rootSigDesc{};
    rootSigDesc.NumParameters = 1;
    rootSigDesc.pParameters = &rootParam;
    rootSigDesc.NumStaticSamplers = 0;
    rootSigDesc.pStaticSamplers = nullptr;
    rootSigDesc.Flags = D3D12_ROOT_SIGNATURE_FLAG_ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT;

    Microsoft::WRL::ComPtr<ID3DBlob> signatureBlob;
    Microsoft::WRL::ComPtr<ID3DBlob> errorBlob;

    if (FAILED(D3D12SerializeRootSignature(&rootSigDesc, D3D_ROOT_SIGNATURE_VERSION_1, &signatureBlob, &errorBlob))) {
        return false;
    }

    if (FAILED(device->CreateRootSignature(0, signatureBlob->GetBufferPointer(), signatureBlob->GetBufferSize(), IID_PPV_ARGS(&m_rootSignature)))) {
        return false;
    }

    return true;
}

bool AvionicsRenderer::CreatePipelineState(ID3D12Device* device) {
    Microsoft::WRL::ComPtr<ID3DBlob> vsBlob;
    Microsoft::WRL::ComPtr<ID3DBlob> psBlob;
    Microsoft::WRL::ComPtr<ID3DBlob> errorBlob;

    UINT compileFlags = 0;
#ifdef _DEBUG
    compileFlags = D3DCOMPILE_DEBUG | D3DCOMPILE_SKIP_OPTIMIZATION;
#else
    compileFlags = D3DCOMPILE_OPTIMIZATION_LEVEL3;
#endif

    if (FAILED(D3DCompile(g_avionicsShaderSource, strlen(g_avionicsShaderSource), "AvionicsShader", nullptr, nullptr, "VSMain", "vs_5_0", compileFlags, 0, &vsBlob, &errorBlob))) {
        return false;
    }

    if (FAILED(D3DCompile(g_avionicsShaderSource, strlen(g_avionicsShaderSource), "AvionicsShader", nullptr, nullptr, "PSMain", "ps_5_0", compileFlags, 0, &psBlob, &errorBlob))) {
        return false;
    }

    D3D12_INPUT_ELEMENT_DESC inputLayout[] = {
        { "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT,    0, 0,  D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 },
        { "NORMAL",   0, DXGI_FORMAT_R32G32B32_FLOAT,    0, 12, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 },
        { "COLOR",    0, DXGI_FORMAT_R32G32B32A32_FLOAT, 0, 24, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 }
    };

    D3D12_GRAPHICS_PIPELINE_STATE_DESC psoDesc{};
    psoDesc.pRootSignature = m_rootSignature.Get();
    psoDesc.VS = { vsBlob->GetBufferPointer(), vsBlob->GetBufferSize() };
    psoDesc.PS = { psBlob->GetBufferPointer(), psBlob->GetBufferSize() };

    // Alpha blending enabled
    D3D12_RENDER_TARGET_BLEND_DESC blendDesc{};
    blendDesc.BlendEnable = TRUE;
    blendDesc.LogicOpEnable = FALSE;
    blendDesc.SrcBlend = D3D12_BLEND_SRC_ALPHA;
    blendDesc.DestBlend = D3D12_BLEND_INV_SRC_ALPHA;
    blendDesc.BlendOp = D3D12_BLEND_OP_ADD;
    blendDesc.SrcBlendAlpha = D3D12_BLEND_ONE;
    blendDesc.DestBlendAlpha = D3D12_BLEND_INV_SRC_ALPHA;
    blendDesc.BlendOpAlpha = D3D12_BLEND_OP_ADD;
    blendDesc.RenderTargetWriteMask = D3D12_COLOR_WRITE_ENABLE_ALL;

    psoDesc.BlendState.RenderTarget[0] = blendDesc;
    psoDesc.SampleMask = UINT_MAX;

    // Rasterizer state: Solid, Cull None
    psoDesc.RasterizerState.FillMode = D3D12_FILL_MODE_SOLID;
    psoDesc.RasterizerState.CullMode = D3D12_CULL_MODE_NONE;
    psoDesc.RasterizerState.DepthClipEnable = TRUE;

    // Depth-stencil state: Depth test DISABLED for overlay
    psoDesc.DepthStencilState.DepthEnable = FALSE;
    psoDesc.DepthStencilState.DepthWriteMask = D3D12_DEPTH_WRITE_MASK_ZERO;

    psoDesc.InputLayout = { inputLayout, _countof(inputLayout) };
    psoDesc.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
    psoDesc.NumRenderTargets = 1;
    psoDesc.RTVFormats[0] = DXGI_FORMAT_R8G8B8A8_UNORM;
    psoDesc.DSVFormat = DXGI_FORMAT_D32_FLOAT;
    psoDesc.SampleDesc.Count = 1;

    if (FAILED(device->CreateGraphicsPipelineState(&psoDesc, IID_PPV_ARGS(&m_pipelineState)))) {
        return false;
    }

    return true;
}

bool AvionicsRenderer::CreateBuffers(ID3D12Device* device) {
    const size_t vbSize = MaxVertices * sizeof(Vertex3D);
    const size_t ibSize = MaxIndices * sizeof(u32);

    D3D12_HEAP_PROPERTIES uploadHeap{};
    uploadHeap.Type = D3D12_HEAP_TYPE_UPLOAD;

    D3D12_RESOURCE_DESC bufferDesc{};
    bufferDesc.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER;
    bufferDesc.Alignment = 0;
    bufferDesc.Width = vbSize;
    bufferDesc.Height = 1;
    bufferDesc.DepthOrArraySize = 1;
    bufferDesc.MipLevels = 1;
    bufferDesc.Format = DXGI_FORMAT_UNKNOWN;
    bufferDesc.SampleDesc.Count = 1;
    bufferDesc.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;

    if (FAILED(device->CreateCommittedResource(
            &uploadHeap,
            D3D12_HEAP_FLAG_NONE,
            &bufferDesc,
            D3D12_RESOURCE_STATE_GENERIC_READ,
            nullptr,
            IID_PPV_ARGS(&m_vertexBuffer)))) {
        return false;
    }

    bufferDesc.Width = ibSize;
    if (FAILED(device->CreateCommittedResource(
            &uploadHeap,
            D3D12_HEAP_FLAG_NONE,
            &bufferDesc,
            D3D12_RESOURCE_STATE_GENERIC_READ,
            nullptr,
            IID_PPV_ARGS(&m_indexBuffer)))) {
        return false;
    }

    // Keep upload buffers persistently mapped for fast zero-allocation per-frame uploads
    D3D12_RANGE readRange{ 0, 0 };
    if (FAILED(m_vertexBuffer->Map(0, &readRange, reinterpret_cast<void**>(&m_mappedVertices)))) {
        return false;
    }
    if (FAILED(m_indexBuffer->Map(0, &readRange, reinterpret_cast<void**>(&m_mappedIndices)))) {
        return false;
    }

    m_vbView.BufferLocation = m_vertexBuffer->GetGPUVirtualAddress();
    m_vbView.StrideInBytes = sizeof(Vertex3D);
    m_vbView.SizeInBytes = static_cast<UINT>(vbSize);

    m_ibView.BufferLocation = m_indexBuffer->GetGPUVirtualAddress();
    m_ibView.Format = DXGI_FORMAT_R32_UINT;
    m_ibView.SizeInBytes = static_cast<UINT>(ibSize);

    return true;
}

void AvionicsRenderer::Render(
    ID3D12GraphicsCommandList* cmdList,
    u32 screenWidth,
    u32 screenHeight,
    const Avionics::SixPackState& state,
    bool visible,
    f32 panelScale) noexcept
{
    if (!cmdList || !m_initialized || !visible || screenWidth == 0 || screenHeight == 0) {
        return;
    }

    m_vertices.clear();
    m_indices.clear();

    // Position panel in lower portion of the screen (cockpit dashboard overlay)
    f32 panelCenterX = 0.0f;
    f32 panelCenterY = 0.0f;
    f32 dialRadius = 52.0f;

    BuildPanelGeometry(panelCenterX, panelCenterY, dialRadius, state);

    if (m_vertices.empty() || m_indices.empty()) {
        return;
    }

    // Clamp to buffer capacities
    size_t copyVertCount = std::min(m_vertices.size(), MaxVertices);
    size_t copyIndexCount = std::min(m_indices.size(), MaxIndices);

    memcpy(m_mappedVertices, m_vertices.data(), copyVertCount * sizeof(Vertex3D));
    memcpy(m_mappedIndices, m_indices.data(), copyIndexCount * sizeof(u32));

    cmdList->SetGraphicsRootSignature(m_rootSignature.Get());
    cmdList->SetPipelineState(m_pipelineState.Get());

    // Screen placement: Bottom-center of viewport
    f32 screenX = static_cast<f32>(screenWidth) * 0.5f;
    f32 screenY = static_cast<f32>(screenHeight) - 135.0f * panelScale;

    struct Constants {
        f32 screenSize[2];
        f32 panelPos[2];
        f32 panelScale;
        f32 opacity;
        f32 padding[2];
    } constants;

    constants.screenSize[0] = static_cast<f32>(screenWidth);
    constants.screenSize[1] = static_cast<f32>(screenHeight);
    constants.panelPos[0] = screenX;
    constants.panelPos[1] = screenY;
    constants.panelScale = panelScale;
    constants.opacity = m_opacity;
    constants.padding[0] = 0.0f;
    constants.padding[1] = 0.0f;

    cmdList->SetGraphicsRoot32BitConstants(0, 8, &constants, 0);

    cmdList->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
    cmdList->IASetVertexBuffers(0, 1, &m_vbView);
    cmdList->IASetIndexBuffer(&m_ibView);

    cmdList->DrawIndexedInstanced(static_cast<UINT>(copyIndexCount), 1, 0, 0, 0);
}

void AvionicsRenderer::BuildPanelGeometry(
    f32 centerX,
    f32 centerY,
    f32 dialRadius,
    const Avionics::SixPackState& state)
{
    // Grid spacing between dial centers
    f32 spacingX = dialRadius * 2.22f; // ~115.4 px
    f32 spacingY = dialRadius * 2.22f; // ~115.4 px

    // Background panel housing plate (dark cockpit glare shield & panel frame)
    f32 panelHalfW = spacingX * 1.55f + 16.0f;
    f32 panelHalfH = spacingY * 1.05f + 14.0f;
    Math::Vec4f panelColor{ 0.10f, 0.10f, 0.11f, 0.94f };
    Math::Vec4f panelBorder{ 0.25f, 0.25f, 0.27f, 0.95f };

    // Outer backing plate
    AddQuad(
        { centerX - panelHalfW, centerY - panelHalfH, 0.0f },
        { centerX + panelHalfW, centerY - panelHalfH, 0.0f },
        { centerX + panelHalfW, centerY + panelHalfH, 0.0f },
        { centerX - panelHalfW, centerY + panelHalfH, 0.0f },
        panelColor);

    // Bevel borders
    AddLine(centerX - panelHalfW, centerY - panelHalfH, centerX + panelHalfW, centerY - panelHalfH, 2.5f, panelBorder);
    AddLine(centerX - panelHalfW, centerY + panelHalfH, centerX + panelHalfW, centerY + panelHalfH, 2.5f, panelBorder);
    AddLine(centerX - panelHalfW, centerY - panelHalfH, centerX - panelHalfW, centerY + panelHalfH, 2.5f, panelBorder);
    AddLine(centerX + panelHalfW, centerY - panelHalfH, centerX + panelHalfW, centerY + panelHalfH, 2.5f, panelBorder);

    // Instrument center coordinates:
    // Top Row (y = -spacingY * 0.5)
    f32 row0Y = centerY - spacingY * 0.5f;
    f32 col0X = centerX - spacingX;      // Airspeed
    f32 col1X = centerX;                 // Attitude
    f32 col2X = centerX + spacingX;      // Altimeter

    // Bottom Row (y = +spacingY * 0.5)
    f32 row1Y = centerY + spacingY * 0.5f;
    f32 col3X = centerX - spacingX;      // Turn Coordinator
    f32 col4X = centerX;                 // Heading
    f32 col5X = centerX + spacingX;      // Vertical Speed

    // 1. Airspeed Indicator (ASI)
    DrawInstrumentHousing(col0X, row0Y, dialRadius);
    DrawAirspeedIndicator(col0X, row0Y, dialRadius, state.airspeed);

    // 2. Attitude Indicator (AI)
    DrawInstrumentHousing(col1X, row0Y, dialRadius);
    DrawAttitudeIndicator(col1X, row0Y, dialRadius, state.attitude);

    // 3. Altimeter (ALT)
    DrawInstrumentHousing(col2X, row0Y, dialRadius);
    DrawAltimeter(col2X, row0Y, dialRadius, state.altimeter);

    // 4. Turn Coordinator (TC)
    DrawInstrumentHousing(col3X, row1Y, dialRadius);
    DrawTurnCoordinator(col3X, row1Y, dialRadius, state.turnCoordinator);

    // 5. Heading Indicator (DG)
    DrawInstrumentHousing(col4X, row1Y, dialRadius);
    DrawHeadingIndicator(col4X, row1Y, dialRadius, state.heading);

    // 6. Vertical Speed Indicator (VSI)
    DrawInstrumentHousing(col5X, row1Y, dialRadius);
    DrawVerticalSpeedIndicator(col5X, row1Y, dialRadius, state.verticalSpeed);
}

void AvionicsRenderer::DrawInstrumentHousing(f32 cx, f32 cy, f32 radius) {
    // Outer black bezel
    AddCircleFill(cx, cy, radius + 4.0f, 32, { 0.15f, 0.15f, 0.16f, 1.0f });
    // Metallic bezel rim
    AddRing(cx, cy, radius + 2.0f, radius + 4.0f, 32, { 0.35f, 0.35f, 0.38f, 1.0f });
    // Inner black dial face
    AddCircleFill(cx, cy, radius, 32, { 0.05f, 0.05f, 0.05f, 1.0f });

    // 4 Corner mounting screws
    f32 screwDist = radius + 2.5f;
    Math::Vec4f screwColor{ 0.45f, 0.45f, 0.48f, 1.0f };
    AddCircleFill(cx - screwDist * 0.7f, cy - screwDist * 0.7f, 1.8f, 8, screwColor);
    AddCircleFill(cx + screwDist * 0.7f, cy - screwDist * 0.7f, 1.8f, 8, screwColor);
    AddCircleFill(cx - screwDist * 0.7f, cy + screwDist * 0.7f, 1.8f, 8, screwColor);
    AddCircleFill(cx + screwDist * 0.7f, cy + screwDist * 0.7f, 1.8f, 8, screwColor);
}

void AvionicsRenderer::DrawAirspeedIndicator(f32 cx, f32 cy, f32 r, const Avionics::AirspeedIndicatorState& asi) {
    // Cessna 172 Airspeed color arcs
    // 40 kts = 20 deg, 85 kts = 110 deg, 129 kts = 198 deg, 163 kts = 266 deg
    f32 arcR_in = r * 0.78f;
    f32 arcR_out = r * 0.88f;

    // White Arc: Flap Operating Range (40 - 85 kts)
    AddArcRing(cx, cy, arcR_in, arcR_out, 20.0f, 110.0f, 16, { 1.0f, 1.0f, 1.0f, 0.85f });
    // Green Arc: Normal Operating Range (48 - 129 kts)
    AddArcRing(cx, cy, arcR_in, arcR_out, 36.0f, 198.0f, 24, { 0.15f, 0.85f, 0.25f, 0.90f });
    // Yellow Arc: Caution Range (129 - 163 kts)
    AddArcRing(cx, cy, arcR_in, arcR_out, 198.0f, 266.0f, 16, { 0.95f, 0.85f, 0.10f, 0.90f });
    // Red Radial Line: Never Exceed Vne (163 kts)
    AddTick(cx, cy, r * 0.95f, r * 0.25f, 266.0f, 2.5f, { 0.95f, 0.15f, 0.15f, 1.0f });

    // Speed Ticks: Every 10 kts (small) and 20 kts (major) from 40 to 200 kts
    for (int kts = 40; kts <= 200; kts += 10) {
        f32 deg = 20.0f + ((kts - 40.0f) / 160.0f) * 320.0f;
        bool isMajor = (kts % 20 == 0);
        f32 tickLen = isMajor ? (r * 0.18f) : (r * 0.10f);
        f32 thick = isMajor ? 1.8f : 1.0f;
        AddTick(cx, cy, r * 0.92f, tickLen, deg, thick, { 0.95f, 0.95f, 0.95f, 0.9f });
    }

    // Text label simulated markers: "AIRSPEED" banner at top
    AddLine(cx - 10.0f, cy - r * 0.40f, cx + 10.0f, cy - r * 0.40f, 1.5f, { 0.7f, 0.7f, 0.7f, 0.8f });
    AddLine(cx - 8.0f, cy + r * 0.35f, cx + 8.0f, cy + r * 0.35f, 1.2f, { 0.6f, 0.6f, 0.6f, 0.7f });

    // Active Needle
    AddNeedle(cx, cy, r * 0.82f, 2.6f, static_cast<f32>(asi.needleAngleDeg), { 1.0f, 1.0f, 1.0f, 1.0f }, 10.0f);
    // Center cap (orange hub)
    AddCircleFill(cx, cy, 3.8f, 12, { 0.95f, 0.45f, 0.10f, 1.0f });
    AddCircleFill(cx, cy, 2.0f, 8, { 0.10f, 0.10f, 0.10f, 1.0f });
}

void AvionicsRenderer::DrawAttitudeIndicator(f32 cx, f32 cy, f32 r, const Avionics::AttitudeIndicatorState& ai) {
    // Artificial horizon sphere disk: sky blue top, earth brown bottom
    f32 horizonR = r * 0.88f;

    // Pitch shift: 20 deg pitch shifts horizon by 0.5 * radius
    f32 pitchOffset = static_cast<f32>(ai.pitchDisplacementNorm) * (horizonR * 0.50f);
    f32 rollRad = static_cast<f32>(ai.rollAngleRad);

    // Earth (brown) lower hemisphere
    AddCircleFill(cx, cy, horizonR, 32, { 0.45f, 0.28f, 0.12f, 1.0f });

    // Sky (blue) upper hemisphere tilted by roll and translated by pitch
    // Represented by an arc or polygon dividing line
    f32 cosR = std::cos(-rollRad);
    f32 sinR = std::sin(-rollRad);

    // Normal to horizon pointing toward sky
    f32 nx = -sinR;
    f32 ny = -cosR;

    // Horizon center point
    f32 hx = cx + nx * pitchOffset;
    f32 hy = cy + ny * pitchOffset;

    // Draw sky half-disk
    for (int i = 0; i < 24; ++i) {
        f32 a0 = (-rollRad) + (static_cast<f32>(i) / 24.0f) * PI;
        f32 a1 = (-rollRad) + (static_cast<f32>(i + 1) / 24.0f) * PI;
        Math::Vec3f p0{ hx, hy, 0.0f };
        Math::Vec3f p1{ cx + std::cos(a0) * horizonR, cy - std::sin(a0) * horizonR, 0.0f };
        Math::Vec3f p2{ cx + std::cos(a1) * horizonR, cy - std::sin(a1) * horizonR, 0.0f };
        AddTriangle(p0, p1, p2, { 0.18f, 0.48f, 0.88f, 1.0f });
    }

    // Horizon line (white dividing line)
    f32 hx0 = hx - cosR * horizonR;
    f32 hy0 = hy + sinR * horizonR;
    f32 hx1 = hx + cosR * horizonR;
    f32 hy1 = hy - sinR * horizonR;
    AddLine(hx0, hy0, hx1, hy1, 2.5f, { 1.0f, 1.0f, 1.0f, 1.0f });

    // Pitch ladder rungs: +/- 10 deg, +/- 20 deg
    for (int p = -20; p <= 20; p += 10) {
        if (p == 0) continue;
        f32 rungOffset = pitchOffset - (static_cast<f32>(p) / 20.0f) * (horizonR * 0.50f);
        f32 rx = cx + nx * rungOffset;
        f32 ry = cy + ny * rungOffset;
        f32 rungW = (std::abs(p) == 10) ? (r * 0.22f) : (r * 0.15f);
        AddLine(rx - cosR * rungW, ry + sinR * rungW, rx + cosR * rungW, ry - sinR * rungW, 1.8f, { 1.0f, 1.0f, 1.0f, 0.9f });
    }

    // Outer bezel masking ring
    AddRing(cx, cy, horizonR, r * 1.02f, 32, { 0.12f, 0.12f, 0.14f, 1.0f });

    // Roll angle indicator ticks at top of dial: 0, +/-10, +/-20, +/-30, +/-60 deg
    const f32 rollTicks[] = { 0.0f, 10.0f, 20.0f, 30.0f, 45.0f, 60.0f, -10.0f, -20.0f, -30.0f, -45.0f, -60.0f };
    for (f32 rt : rollTicks) {
        f32 tickLen = (std::abs(rt) == 30.0f || rt == 0.0f) ? (r * 0.12f) : (r * 0.07f);
        AddTick(cx, cy, r * 0.88f, tickLen, rt, 1.8f, { 1.0f, 1.0f, 1.0f, 0.9f });
    }

    // Yellow miniature aircraft reference symbol (fixed in center of instrument)
    Math::Vec4f yellowSymbol{ 1.0f, 0.82f, 0.05f, 1.0f };
    // Center dot
    AddCircleFill(cx, cy, 3.0f, 12, yellowSymbol);
    // Left wing bar
    AddLine(cx - r * 0.40f, cy, cx - r * 0.12f, cy, 3.0f, yellowSymbol);
    AddLine(cx - r * 0.12f, cy, cx - r * 0.12f, cy + 3.5f, 2.5f, yellowSymbol);
    // Right wing bar
    AddLine(cx + r * 0.12f, cy, cx + r * 0.40f, cy, 3.0f, yellowSymbol);
    AddLine(cx + r * 0.12f, cy, cx + r * 0.12f, cy + 3.5f, 2.5f, yellowSymbol);
}

void AvionicsRenderer::DrawAltimeter(f32 cx, f32 cy, f32 r, const Avionics::AltimeterState& alt) {
    // Altimeter dial markings: 0 to 9 numbers around periphery
    for (int i = 0; i < 10; ++i) {
        f32 deg = static_cast<f32>(i) * 36.0f;
        // Major 100-ft tick
        AddTick(cx, cy, r * 0.92f, r * 0.16f, deg, 2.0f, { 1.0f, 1.0f, 1.0f, 0.95f });

        // Intermediate ticks (every 20 ft)
        for (int j = 1; j < 5; ++j) {
            f32 subDeg = deg + j * (36.0f / 5.0f);
            AddTick(cx, cy, r * 0.92f, r * 0.09f, subDeg, 1.0f, { 0.9f, 0.9f, 0.9f, 0.75f });
        }
    }

    // Kollsman window at 3 o'clock (barometric setting display bar)
    f32 kwX = cx + r * 0.42f;
    f32 kwY = cy;
    AddQuad(
        { kwX - 9.0f, kwY - 6.0f, 0.0f },
        { kwX + 9.0f, kwY - 6.0f, 0.0f },
        { kwX + 9.0f, kwY + 6.0f, 0.0f },
        { kwX - 9.0f, kwY + 6.0f, 0.0f },
        { 0.18f, 0.18f, 0.20f, 1.0f });
    AddLine(kwX - 9.0f, kwY - 6.0f, kwX + 9.0f, kwY - 6.0f, 1.0f, { 0.6f, 0.6f, 0.6f, 0.8f });
    AddLine(kwX - 9.0f, kwY + 6.0f, kwX + 9.0f, kwY + 6.0f, 1.0f, { 0.6f, 0.6f, 0.6f, 0.8f });

    // 10,000 ft pointer: Thin needle with triangular tip
    f32 a10000 = static_cast<f32>(alt.needle10000FtAngleDeg);
    AddNeedle(cx, cy, r * 0.90f, 1.5f, a10000, { 1.0f, 1.0f, 1.0f, 0.85f }, 6.0f);
    // Triangular tip
    f32 rad10k = a10000 * DEG_TO_RAD;
    f32 tipX = cx + std::sin(rad10k) * (r * 0.86f);
    f32 tipY = cy - std::cos(rad10k) * (r * 0.86f);
    AddCircleFill(tipX, tipY, 3.0f, 6, { 1.0f, 1.0f, 1.0f, 1.0f });

    // 1,000 ft pointer: Short and broad needle
    AddNeedle(cx, cy, r * 0.55f, 4.0f, static_cast<f32>(alt.needle1000FtAngleDeg), { 0.95f, 0.95f, 0.95f, 1.0f }, 8.0f);

    // 100 ft pointer: Long needle with tip reaching outer scale
    AddNeedle(cx, cy, r * 0.85f, 2.5f, static_cast<f32>(alt.needle100FtAngleDeg), { 1.0f, 1.0f, 1.0f, 1.0f }, 10.0f);

    // Center pivot cap
    AddCircleFill(cx, cy, 3.8f, 12, { 0.25f, 0.25f, 0.28f, 1.0f });
    AddCircleFill(cx, cy, 1.8f, 8, { 0.05f, 0.05f, 0.05f, 1.0f });
}

void AvionicsRenderer::DrawTurnCoordinator(f32 cx, f32 cy, f32 r, const Avionics::TurnCoordinatorState& tc) {
    // Top section: Miniature aircraft silhouette indicating rate of turn
    f32 tcY = cy - r * 0.12f;

    // Standard rate turn indices ("L" and "R" ticks)
    // Left index (-20 deg bank mark)
    AddTick(cx, tcY, r * 0.65f, r * 0.14f, -20.0f, 2.5f, { 1.0f, 1.0f, 1.0f, 0.9f });
    // Right index (+20 deg bank mark)
    AddTick(cx, tcY, r * 0.65f, r * 0.14f, 20.0f, 2.5f, { 1.0f, 1.0f, 1.0f, 0.9f });
    // Level wing reference marks
    AddLine(cx - r * 0.60f, tcY, cx - r * 0.35f, tcY, 2.0f, { 1.0f, 1.0f, 1.0f, 0.8f });
    AddLine(cx + r * 0.35f, tcY, cx + r * 0.60f, tcY, 2.0f, { 1.0f, 1.0f, 1.0f, 0.8f });

    // Miniature aircraft banking by symbolRollDeg
    f32 rollRad = static_cast<f32>(tc.symbolRollDeg) * DEG_TO_RAD;
    f32 cR = std::cos(rollRad);
    f32 sR = std::sin(rollRad);

    Math::Vec4f planeColor{ 1.0f, 1.0f, 1.0f, 1.0f };
    // Fuselage center circle
    AddCircleFill(cx, tcY, 3.5f, 12, planeColor);
    // Vertical fin
    AddLine(cx, tcY, cx - sR * (r * 0.22f), tcY - cR * (r * 0.22f), 2.5f, planeColor);
    // Left wing
    AddLine(cx, tcY, cx - cR * (r * 0.38f), tcY - sR * (r * 0.38f), 3.0f, planeColor);
    // Right wing
    AddLine(cx, tcY, cx + cR * (r * 0.38f), tcY + sR * (r * 0.38f), 3.0f, planeColor);

    // "2 MIN" turn label line
    AddLine(cx - 8.0f, cy + r * 0.15f, cx + 8.0f, cy + r * 0.15f, 1.2f, { 0.6f, 0.6f, 0.6f, 0.8f });

    // Bottom section: Inclinometer ("ball in the cage")
    f32 tubeY = cy + r * 0.48f;
    f32 tubeW = r * 0.46f;
    f32 tubeH = r * 0.14f;

    // Curved fluid-filled glass tube background
    AddQuad(
        { cx - tubeW, tubeY - tubeH, 0.0f },
        { cx + tubeW, tubeY - tubeH, 0.0f },
        { cx + tubeW, tubeY + tubeH, 0.0f },
        { cx - tubeW, tubeY + tubeH, 0.0f },
        { 0.18f, 0.22f, 0.24f, 0.95f });

    // Dual white reference index wires
    f32 wireDist = 5.0f;
    AddLine(cx - wireDist, tubeY - tubeH, cx - wireDist, tubeY + tubeH, 1.5f, { 1.0f, 1.0f, 1.0f, 0.9f });
    AddLine(cx + wireDist, tubeY - tubeH, cx + wireDist, tubeY + tubeH, 1.5f, { 1.0f, 1.0f, 1.0f, 0.9f });

    // Black agate ball displaced by slipSkidBallPosition
    f32 ballOffset = static_cast<f32>(tc.slipSkidBallPosition) * (tubeW - 6.0f);
    f32 ballX = cx + ballOffset;
    AddCircleFill(ballX, tubeY, 4.2f, 12, { 0.02f, 0.02f, 0.02f, 1.0f });
    AddCircleFill(ballX - 1.0f, tubeY - 1.0f, 1.5f, 8, { 0.4f, 0.4f, 0.4f, 0.8f }); // specular highlight
}

void AvionicsRenderer::DrawHeadingIndicator(f32 cx, f32 cy, f32 r, const Avionics::HeadingIndicatorState& hdg) {
    // Rotating compass card
    f32 cardDeg = static_cast<f32>(hdg.compassCardAngleDeg);

    // Outer azimuth ring with tick marks every 5 and 10 degrees
    for (int deg = 0; deg < 360; deg += 5) {
        f32 a = cardDeg + static_cast<f32>(deg);
        bool isMajor = (deg % 30 == 0);
        bool isMedium = (deg % 10 == 0);
        f32 tickLen = isMajor ? (r * 0.16f) : (isMedium ? (r * 0.11f) : (r * 0.06f));
        f32 thick = isMajor ? 2.0f : 1.0f;
        AddTick(cx, cy, r * 0.92f, tickLen, a, thick, { 1.0f, 1.0f, 1.0f, 0.9f });
    }

    // Cardinal Points (N, E, S, W) triangular markers on rotating card
    // North (0 deg)
    f32 nRad = (cardDeg + 0.0f) * DEG_TO_RAD;
    f32 nX = cx + std::sin(nRad) * (r * 0.76f);
    f32 nY = cy - std::cos(nRad) * (r * 0.76f);
    AddCircleFill(nX, nY, 3.0f, 3, { 0.95f, 0.85f, 0.10f, 1.0f }); // Yellow triangle for North

    // Fixed aircraft reference silhouette in center
    Math::Vec4f orangeRef{ 0.95f, 0.50f, 0.10f, 1.0f };
    AddCircleFill(cx, cy, 3.5f, 12, orangeRef);
    AddLine(cx, cy - r * 0.25f, cx, cy + r * 0.20f, 2.5f, orangeRef); // fuselage
    AddLine(cx - r * 0.24f, cy, cx + r * 0.24f, cy, 3.0f, orangeRef); // wings
    AddLine(cx - r * 0.10f, cy + r * 0.16f, cx + r * 0.10f, cy + r * 0.16f, 2.0f, orangeRef); // tail

    // Top lubber line (fixed reference at 12 o'clock)
    AddTriangle(
        { cx - 4.0f, cy - r * 0.96f, 0.0f },
        { cx + 4.0f, cy - r * 0.96f, 0.0f },
        { cx, cy - r * 0.86f, 0.0f },
        { 1.0f, 0.82f, 0.05f, 1.0f });

    // Pilot-settable Heading Bug (orange notch on compass card)
    f32 bugAngle = cardDeg + static_cast<f32>(hdg.headingBugDeg);
    f32 bRad = bugAngle * DEG_TO_RAD;
    f32 bx = cx + std::sin(bRad) * (r * 0.90f);
    f32 by = cy - std::cos(bRad) * (r * 0.90f);
    AddCircleFill(bx, by, 3.5f, 3, { 1.0f, 0.45f, 0.05f, 1.0f });
}

void AvionicsRenderer::DrawVerticalSpeedIndicator(f32 cx, f32 cy, f32 r, const Avionics::VerticalSpeedIndicatorState& vsi) {
    // VSI dial face: 0 at 9 o'clock horizontal (angle = 270 deg or -90 deg from 12 o'clock)
    // Positive climb sweeps UPWARD (from 270 to 360/0 to 80 deg)
    // Negative descent sweeps DOWNWARD (from 270 to 180 to 100 deg)

    // Horizontal zero mark at 9 o'clock
    AddTick(cx, cy, r * 0.92f, r * 0.20f, 270.0f, 2.5f, { 1.0f, 1.0f, 1.0f, 1.0f });

    // Climb Ticks (0, 500, 1000, 1500, 2000 fpm)
    // +2000 fpm maps to +170 deg rotation from 9 o'clock (angle 270 + 170 = 440 = 80 deg)
    const f32 fpmSteps[] = { 500.0f, 1000.0f, 1500.0f, 2000.0f };
    for (f32 fpm : fpmSteps) {
        f32 frac = fpm / 2000.0f;
        f32 degUp = 270.0f + frac * 170.0f;
        f32 degDn = 270.0f - frac * 170.0f;

        // Up tick
        AddTick(cx, cy, r * 0.92f, r * 0.16f, degUp, 2.0f, { 1.0f, 1.0f, 1.0f, 0.9f });
        // Down tick
        AddTick(cx, cy, r * 0.92f, r * 0.16f, degDn, 2.0f, { 1.0f, 1.0f, 1.0f, 0.9f });

        // Intermediate ticks (every 100 fpm)
        for (int sub = 1; sub < 5; ++sub) {
            f32 subFrac = (fpm - 500.0f + sub * 100.0f) / 2000.0f;
            f32 subUp = 270.0f + subFrac * 170.0f;
            f32 subDn = 270.0f - subFrac * 170.0f;
            AddTick(cx, cy, r * 0.92f, r * 0.08f, subUp, 1.0f, { 0.8f, 0.8f, 0.8f, 0.7f });
            AddTick(cx, cy, r * 0.92f, r * 0.08f, subDn, 1.0f, { 0.8f, 0.8f, 0.8f, 0.7f });
        }
    }

    // "UP" and "DN" text indicators
    AddLine(cx - r * 0.35f, cy - r * 0.35f, cx - r * 0.20f, cy - r * 0.35f, 1.5f, { 0.7f, 0.7f, 0.7f, 0.8f });
    AddLine(cx - r * 0.35f, cy + r * 0.35f, cx - r * 0.20f, cy + r * 0.35f, 1.5f, { 0.7f, 0.7f, 0.7f, 0.8f });

    // Active Needle: 270 deg (9 o'clock) + needleAngleDeg
    f32 needleDeg = 270.0f + static_cast<f32>(vsi.needleAngleDeg);
    AddNeedle(cx, cy, r * 0.84f, 2.6f, needleDeg, { 1.0f, 1.0f, 1.0f, 1.0f }, 9.0f);

    // Center pivot cap
    AddCircleFill(cx, cy, 3.8f, 12, { 0.95f, 0.45f, 0.10f, 1.0f });
    AddCircleFill(cx, cy, 2.0f, 8, { 0.10f, 0.10f, 0.10f, 1.0f });
}

// Vector primitive drawing helpers
void AvionicsRenderer::AddQuad(
    const Math::Vec3f& p0,
    const Math::Vec3f& p1,
    const Math::Vec3f& p2,
    const Math::Vec3f& p3,
    const Math::Vec4f& col)
{
    u32 base = static_cast<u32>(m_vertices.size());
    Math::Vec3f normal{ 0.0f, 0.0f, 1.0f };

    m_vertices.push_back({ p0, normal, col });
    m_vertices.push_back({ p1, normal, col });
    m_vertices.push_back({ p2, normal, col });
    m_vertices.push_back({ p3, normal, col });

    m_indices.push_back(base + 0);
    m_indices.push_back(base + 1);
    m_indices.push_back(base + 2);

    m_indices.push_back(base + 0);
    m_indices.push_back(base + 2);
    m_indices.push_back(base + 3);
}

void AvionicsRenderer::AddTriangle(
    const Math::Vec3f& p0,
    const Math::Vec3f& p1,
    const Math::Vec3f& p2,
    const Math::Vec4f& col)
{
    u32 base = static_cast<u32>(m_vertices.size());
    Math::Vec3f normal{ 0.0f, 0.0f, 1.0f };

    m_vertices.push_back({ p0, normal, col });
    m_vertices.push_back({ p1, normal, col });
    m_vertices.push_back({ p2, normal, col });

    m_indices.push_back(base + 0);
    m_indices.push_back(base + 1);
    m_indices.push_back(base + 2);
}

void AvionicsRenderer::AddCircleFill(f32 cx, f32 cy, f32 radius, int segments, const Math::Vec4f& col) {
    if (segments < 3) segments = 3;
    u32 centerIdx = static_cast<u32>(m_vertices.size());
    Math::Vec3f normal{ 0.0f, 0.0f, 1.0f };

    m_vertices.push_back({ { cx, cy, 0.0f }, normal, col });

    for (int i = 0; i < segments; ++i) {
        f32 angle = (static_cast<f32>(i) / segments) * (2.0f * PI);
        f32 x = cx + std::cos(angle) * radius;
        f32 y = cy + std::sin(angle) * radius;
        m_vertices.push_back({ { x, y, 0.0f }, normal, col });
    }

    for (int i = 0; i < segments; ++i) {
        u32 i0 = centerIdx;
        u32 i1 = centerIdx + 1 + i;
        u32 i2 = centerIdx + 1 + ((i + 1) % segments);
        m_indices.push_back(i0);
        m_indices.push_back(i1);
        m_indices.push_back(i2);
    }
}

void AvionicsRenderer::AddRing(f32 cx, f32 cy, f32 innerR, f32 outerR, int segments, const Math::Vec4f& col) {
    if (segments < 3) segments = 3;
    for (int i = 0; i < segments; ++i) {
        f32 a0 = (static_cast<f32>(i) / segments) * (2.0f * PI);
        f32 a1 = (static_cast<f32>(i + 1) / segments) * (2.0f * PI);

        Math::Vec3f p0{ cx + std::cos(a0) * innerR, cy + std::sin(a0) * innerR, 0.0f };
        Math::Vec3f p1{ cx + std::cos(a0) * outerR, cy + std::sin(a0) * outerR, 0.0f };
        Math::Vec3f p2{ cx + std::cos(a1) * outerR, cy + std::sin(a1) * outerR, 0.0f };
        Math::Vec3f p3{ cx + std::cos(a1) * innerR, cy + std::sin(a1) * innerR, 0.0f };

        AddQuad(p0, p1, p2, p3, col);
    }
}

void AvionicsRenderer::AddArcRing(
    f32 cx, f32 cy,
    f32 innerR, f32 outerR,
    f32 startDeg, f32 endDeg,
    int segments,
    const Math::Vec4f& col)
{
    if (segments < 2) segments = 2;
    f32 span = endDeg - startDeg;
    if (span < 0.0f) span += 360.0f;

    for (int i = 0; i < segments; ++i) {
        f32 a0 = (startDeg + (static_cast<f32>(i) / segments) * span) * DEG_TO_RAD;
        f32 a1 = (startDeg + (static_cast<f32>(i + 1) / segments) * span) * DEG_TO_RAD;

        // 0 deg points UP (y is negative in screen coordinates)
        f32 c0 = std::sin(a0), s0 = -std::cos(a0);
        f32 c1 = std::sin(a1), s1 = -std::cos(a1);

        Math::Vec3f p0{ cx + c0 * innerR, cy + s0 * innerR, 0.0f };
        Math::Vec3f p1{ cx + c0 * outerR, cy + s0 * outerR, 0.0f };
        Math::Vec3f p2{ cx + c1 * outerR, cy + s1 * outerR, 0.0f };
        Math::Vec3f p3{ cx + c1 * innerR, cy + s1 * innerR, 0.0f };

        AddQuad(p0, p1, p2, p3, col);
    }
}

void AvionicsRenderer::AddLine(f32 x0, f32 y0, f32 x1, f32 y1, f32 thickness, const Math::Vec4f& col) {
    f32 dx = x1 - x0;
    f32 dy = y1 - y0;
    f32 len = std::sqrt(dx * dx + dy * dy);
    if (len < 1e-4f) return;

    f32 nx = (-dy / len) * (thickness * 0.5f);
    f32 ny = (dx / len) * (thickness * 0.5f);

    Math::Vec3f p0{ x0 + nx, y0 + ny, 0.0f };
    Math::Vec3f p1{ x1 + nx, y1 + ny, 0.0f };
    Math::Vec3f p2{ x1 - nx, y1 - ny, 0.0f };
    Math::Vec3f p3{ x0 - nx, y0 - ny, 0.0f };

    AddQuad(p0, p1, p2, p3, col);
}

void AvionicsRenderer::AddTick(f32 cx, f32 cy, f32 radius, f32 length, f32 angleDeg, f32 thickness, const Math::Vec4f& col) {
    f32 rad = angleDeg * DEG_TO_RAD;
    f32 dx = std::sin(rad);
    f32 dy = -std::cos(rad);

    f32 x0 = cx + dx * radius;
    f32 y0 = cy + dy * radius;
    f32 x1 = cx + dx * (radius - length);
    f32 y1 = cy + dy * (radius - length);

    AddLine(x0, y0, x1, y1, thickness, col);
}

void AvionicsRenderer::AddNeedle(
    f32 cx, f32 cy,
    f32 length, f32 baseW,
    f32 angleDeg,
    const Math::Vec4f& col,
    f32 counterLen)
{
    f32 rad = angleDeg * DEG_TO_RAD;
    f32 dx = std::sin(rad);
    f32 dy = -std::cos(rad);
    f32 px = -dy; // perpendicular
    f32 py = dx;

    f32 halfW = baseW * 0.5f;

    // Tip
    Math::Vec3f tip{ cx + dx * length, cy + dy * length, 0.0f };
    // Right base
    Math::Vec3f rBase{ cx + px * halfW, cy + py * halfW, 0.0f };
    // Left base
    Math::Vec3f lBase{ cx - px * halfW, cy - py * halfW, 0.0f };
    // Counterweight tail
    Math::Vec3f tail{ cx - dx * counterLen, cy - dy * counterLen, 0.0f };

    AddTriangle(tip, rBase, lBase, col);
    AddTriangle(rBase, tail, lBase, col);
}

} // namespace LokYS::Graphics
