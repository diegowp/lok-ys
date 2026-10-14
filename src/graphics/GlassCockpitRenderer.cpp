#include "LokYS/graphics/GlassCockpitRenderer.hpp"
#include <d3dcompiler.h>
#include <cmath>
#include <algorithm>
#include <sstream>
#include <iomanip>

namespace LokYS::Graphics {

namespace {

const char* g_glassCockpitShaderSource = R"(
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

static bool ClipLineSegmentToRect(
    f32& x0, f32& y0, f32& x1, f32& y1,
    f32 minX, f32 maxX, f32 minY, f32 maxY)
{
    auto outCode = [&](f32 x, f32 y) -> int {
        int code = 0;
        if (x < minX) code |= 1;
        else if (x > maxX) code |= 2;
        if (y < minY) code |= 4;
        else if (y > maxY) code |= 8;
        return code;
    };

    int c0 = outCode(x0, y0);
    int c1 = outCode(x1, y1);

    for (int iter = 0; iter < 4; ++iter) {
        if ((c0 | c1) == 0) {
            return true;
        }
        if ((c0 & c1) != 0) {
            return false;
        }
        int cOut = (c0 != 0) ? c0 : c1;
        f32 x = 0.0f, y = 0.0f;
        if (cOut & 8) {
            x = x0 + (x1 - x0) * (maxY - y0) / (y1 - y0 + 1e-9f);
            y = maxY;
        } else if (cOut & 4) {
            x = x0 + (x1 - x0) * (minY - y0) / (y1 - y0 + 1e-9f);
            y = minY;
        } else if (cOut & 2) {
            y = y0 + (y1 - y0) * (maxX - x0) / (x1 - x0 + 1e-9f);
            x = maxX;
        } else if (cOut & 1) {
            y = y0 + (y1 - y0) * (minX - x0) / (x1 - x0 + 1e-9f);
            x = minX;
        }

        if (cOut == c0) {
            x0 = x;
            y0 = y;
            c0 = outCode(x0, y0);
        } else {
            x1 = x;
            y1 = y;
            c1 = outCode(x1, y1);
        }
    }
    return (c0 | c1) == 0;
}

} // anonymous namespace

bool GlassCockpitRenderer::Initialize(ID3D12Device* device) {
    if (!device) return false;

    if (!CreateRootSignature(device)) return false;
    if (!CreatePipelineState(device)) return false;
    if (!CreateBuffers(device)) return false;

    m_vertices.reserve(MaxVertices);
    m_indices.reserve(MaxIndices);
    m_initialized = true;
    return true;
}

void GlassCockpitRenderer::Shutdown() {
    if (m_vertexBuffer) {
        m_vertexBuffer->Unmap(0, nullptr);
        m_mappedVertices = nullptr;
    }
    if (m_indexBuffer) {
        m_indexBuffer->Unmap(0, nullptr);
        m_mappedIndices = nullptr;
    }
    m_vertexBuffer.Reset();
    m_indexBuffer.Reset();
    m_pipelineState.Reset();
    m_rootSignature.Reset();
    m_vertices.clear();
    m_indices.clear();
    m_initialized = false;
}

bool GlassCockpitRenderer::CreateRootSignature(ID3D12Device* device) {
    D3D12_ROOT_PARAMETER rootParam{};
    rootParam.ParameterType = D3D12_ROOT_PARAMETER_TYPE_32BIT_CONSTANTS;
    rootParam.Constants.ShaderRegister = 0;
    rootParam.Constants.RegisterSpace = 0;
    rootParam.Constants.Num32BitValues = 8;
    rootParam.ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;

    D3D12_ROOT_SIGNATURE_DESC rsDesc{};
    rsDesc.NumParameters = 1;
    rsDesc.pParameters = &rootParam;
    rsDesc.Flags = D3D12_ROOT_SIGNATURE_FLAG_ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT;

    Microsoft::WRL::ComPtr<ID3DBlob> signatureBlob;
    Microsoft::WRL::ComPtr<ID3DBlob> errorBlob;
    HRESULT hr = D3D12SerializeRootSignature(&rsDesc, D3D_ROOT_SIGNATURE_VERSION_1, &signatureBlob, &errorBlob);
    if (FAILED(hr)) return false;

    hr = device->CreateRootSignature(0, signatureBlob->GetBufferPointer(), signatureBlob->GetBufferSize(), IID_PPV_ARGS(&m_rootSignature));
    return SUCCEEDED(hr);
}

bool GlassCockpitRenderer::CreatePipelineState(ID3D12Device* device) {
    Microsoft::WRL::ComPtr<ID3DBlob> vsBlob;
    Microsoft::WRL::ComPtr<ID3DBlob> psBlob;
    Microsoft::WRL::ComPtr<ID3DBlob> errorBlob;

    UINT compileFlags = 0;
#if defined(_DEBUG)
    compileFlags |= D3DCOMPILE_DEBUG | D3DCOMPILE_SKIP_OPTIMIZATION;
#else
    compileFlags |= D3DCOMPILE_OPTIMIZATION_LEVEL3;
#endif

    HRESULT hr = D3DCompile(
        g_glassCockpitShaderSource, strlen(g_glassCockpitShaderSource),
        "GlassCockpitVS", nullptr, nullptr, "VSMain", "vs_5_0", compileFlags, 0, &vsBlob, &errorBlob);
    if (FAILED(hr)) return false;

    hr = D3DCompile(
        g_glassCockpitShaderSource, strlen(g_glassCockpitShaderSource),
        "GlassCockpitPS", nullptr, nullptr, "PSMain", "ps_5_0", compileFlags, 0, &psBlob, &errorBlob);
    if (FAILED(hr)) return false;

    D3D12_INPUT_ELEMENT_DESC inputElements[] = {
        { "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT,    0, offsetof(Vertex3D, position), D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 },
        { "NORMAL",   0, DXGI_FORMAT_R32G32B32_FLOAT,    0, offsetof(Vertex3D, normal),   D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 },
        { "COLOR",    0, DXGI_FORMAT_R32G32B32A32_FLOAT, 0, offsetof(Vertex3D, color),    D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 }
    };

    D3D12_GRAPHICS_PIPELINE_STATE_DESC psoDesc{};
    psoDesc.pRootSignature = m_rootSignature.Get();
    psoDesc.VS = { vsBlob->GetBufferPointer(), vsBlob->GetBufferSize() };
    psoDesc.PS = { psBlob->GetBufferPointer(), psBlob->GetBufferSize() };
    psoDesc.InputLayout = { inputElements, _countof(inputElements) };
    psoDesc.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
    psoDesc.NumRenderTargets = 1;
    psoDesc.RTVFormats[0] = DXGI_FORMAT_R8G8B8A8_UNORM;
    psoDesc.SampleDesc.Count = 1;
    psoDesc.SampleMask = UINT_MAX;

    // Alpha blending
    D3D12_BLEND_DESC blendDesc{};
    blendDesc.AlphaToCoverageEnable = FALSE;
    blendDesc.IndependentBlendEnable = FALSE;
    blendDesc.RenderTarget[0].BlendEnable = TRUE;
    blendDesc.RenderTarget[0].SrcBlend = D3D12_BLEND_SRC_ALPHA;
    blendDesc.RenderTarget[0].DestBlend = D3D12_BLEND_INV_SRC_ALPHA;
    blendDesc.RenderTarget[0].BlendOp = D3D12_BLEND_OP_ADD;
    blendDesc.RenderTarget[0].SrcBlendAlpha = D3D12_BLEND_ONE;
    blendDesc.RenderTarget[0].DestBlendAlpha = D3D12_BLEND_ZERO;
    blendDesc.RenderTarget[0].BlendOpAlpha = D3D12_BLEND_OP_ADD;
    blendDesc.RenderTarget[0].RenderTargetWriteMask = D3D12_COLOR_WRITE_ENABLE_ALL;
    psoDesc.BlendState = blendDesc;

    // Rasterizer: no culling
    D3D12_RASTERIZER_DESC rastDesc{};
    rastDesc.FillMode = D3D12_FILL_MODE_SOLID;
    rastDesc.CullMode = D3D12_CULL_MODE_NONE;
    psoDesc.RasterizerState = rastDesc;

    // Depth-stencil: disabled for 2D overlay
    D3D12_DEPTH_STENCIL_DESC dsDesc{};
    dsDesc.DepthEnable = FALSE;
    dsDesc.StencilEnable = FALSE;
    psoDesc.DepthStencilState = dsDesc;

    hr = device->CreateGraphicsPipelineState(&psoDesc, IID_PPV_ARGS(&m_pipelineState));
    return SUCCEEDED(hr);
}

bool GlassCockpitRenderer::CreateBuffers(ID3D12Device* device) {
    size_t vbSize = MaxVertices * sizeof(Vertex3D);
    size_t ibSize = MaxIndices * sizeof(u32);

    D3D12_HEAP_PROPERTIES heapProps{};
    heapProps.Type = D3D12_HEAP_TYPE_UPLOAD;

    D3D12_RESOURCE_DESC vbDesc{};
    vbDesc.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER;
    vbDesc.Width = vbSize;
    vbDesc.Height = 1;
    vbDesc.DepthOrArraySize = 1;
    vbDesc.MipLevels = 1;
    vbDesc.SampleDesc.Count = 1;
    vbDesc.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;

    HRESULT hr = device->CreateCommittedResource(
        &heapProps, D3D12_HEAP_FLAG_NONE, &vbDesc,
        D3D12_RESOURCE_STATE_GENERIC_READ, nullptr, IID_PPV_ARGS(&m_vertexBuffer));
    if (FAILED(hr)) return false;

    D3D12_RANGE readRange{ 0, 0 };
    hr = m_vertexBuffer->Map(0, &readRange, reinterpret_cast<void**>(&m_mappedVertices));
    if (FAILED(hr)) return false;

    m_vbView.BufferLocation = m_vertexBuffer->GetGPUVirtualAddress();
    m_vbView.SizeInBytes = static_cast<UINT>(vbSize);
    m_vbView.StrideInBytes = sizeof(Vertex3D);

    D3D12_RESOURCE_DESC ibDesc = vbDesc;
    ibDesc.Width = ibSize;

    hr = device->CreateCommittedResource(
        &heapProps, D3D12_HEAP_FLAG_NONE, &ibDesc,
        D3D12_RESOURCE_STATE_GENERIC_READ, nullptr, IID_PPV_ARGS(&m_indexBuffer));
    if (FAILED(hr)) return false;

    hr = m_indexBuffer->Map(0, &readRange, reinterpret_cast<void**>(&m_mappedIndices));
    if (FAILED(hr)) return false;

    m_ibView.BufferLocation = m_indexBuffer->GetGPUVirtualAddress();
    m_ibView.SizeInBytes = static_cast<UINT>(ibSize);
    m_ibView.Format = DXGI_FORMAT_R32_UINT;

    return true;
}

void GlassCockpitRenderer::Render(
    ID3D12GraphicsCommandList* cmdList,
    u32 screenWidth,
    u32 screenHeight,
    const Avionics::SixPackState& avionics,
    const Navigation::NavReceiverState& nav1,
    const Autopilot::AutopilotStatus& apStatus,
    bool visible,
    f32 panelScale) noexcept
{
    // Legacy pass-through with explicit scale override
    RenderPfd(
        cmdList,
        screenWidth,
        screenHeight,
        avionics,
        nav1,
        apStatus,
        visible,
        PfdDockPosition::BottomCenter,
        panelScale);
}

void GlassCockpitRenderer::RenderPfd(
    ID3D12GraphicsCommandList* cmdList,
    u32 screenWidth,
    u32 screenHeight,
    const Avionics::SixPackState& avionics,
    const Navigation::NavReceiverState& nav1,
    const Autopilot::AutopilotStatus& apStatus,
    bool visible,
    PfdDockPosition dockPos,
    f32 scaleOverride) noexcept
{
    if (!m_initialized || !visible || !cmdList || screenWidth == 0 || screenHeight == 0) return;

    m_vertices.clear();
    m_indices.clear();

    // Standard PFD virtual aspect: width 640 x height 480
    f32 pfdW = 640.0f;
    f32 pfdH = 480.0f;

    f32 panelScale = scaleOverride;
    if (panelScale <= 0.0f) {
        if (dockPos == PfdDockPosition::CornerMini) {
            // Unobtrusive corner mini display: ~26% of screen height
            panelScale = (static_cast<f32>(screenHeight) * 0.26f) / pfdH;
            panelScale = std::clamp(panelScale, 0.30f, 0.65f);
        } else {
            // Full PFD instrument dock: ~38% of screen height (leaving >60% windshield)
            panelScale = (static_cast<f32>(screenHeight) * 0.38f) / pfdH;
            panelScale = std::clamp(panelScale, 0.45f, 1.10f);
        }
    }

    f32 pfdX = 0.0f;
    f32 pfdY = 0.0f;

    switch (dockPos) {
    case PfdDockPosition::BottomLeft:
        // Pilot seat PFD (Captain's side): anchored to bottom-left with margin
        pfdX = 24.0f;
        pfdY = static_cast<f32>(screenHeight) - (pfdH * panelScale + 18.0f);
        break;

    case PfdDockPosition::BottomCenter:
        // Center dashboard: centered horizontally, docked to bottom
        pfdX = (static_cast<f32>(screenWidth) - pfdW * panelScale) * 0.5f;
        pfdY = static_cast<f32>(screenHeight) - (pfdH * panelScale + 18.0f);
        break;

    case PfdDockPosition::CornerMini:
        // Compact corner display: docked tightly in bottom-left
        pfdX = 16.0f;
        pfdY = static_cast<f32>(screenHeight) - (pfdH * panelScale + 14.0f);
        break;

    case PfdDockPosition::Center:
        // IFR test / instrument focus mode: centered
        pfdX = (static_cast<f32>(screenWidth) - pfdW * panelScale) * 0.5f;
        pfdY = (static_cast<f32>(screenHeight) - pfdH * panelScale) * 0.5f;
        break;
    }

    if (pfdY < 10.0f) pfdY = 10.0f;

    BuildPfdGeometry(0.0f, 0.0f, pfdW, pfdH, avionics, nav1, apStatus);

    if (m_indices.empty()) return;

    size_t vertCount = std::min(m_vertices.size(), MaxVertices);
    size_t idxCount = std::min(m_indices.size(), MaxIndices);

    memcpy(m_mappedVertices, m_vertices.data(), vertCount * sizeof(Vertex3D));
    memcpy(m_mappedIndices, m_indices.data(), idxCount * sizeof(u32));

    cmdList->SetGraphicsRootSignature(m_rootSignature.Get());
    cmdList->SetPipelineState(m_pipelineState.Get());

    struct {
        float screenSize[2];
        float panelPos[2];
        float panelScale;
        float opacity;
        float padding[2];
    } constants{};

    constants.screenSize[0] = static_cast<float>(screenWidth);
    constants.screenSize[1] = static_cast<float>(screenHeight);
    constants.panelPos[0] = pfdX;
    constants.panelPos[1] = pfdY;
    constants.panelScale = panelScale;
    constants.opacity = m_opacity;

    cmdList->SetGraphicsRoot32BitConstants(0, 8, &constants, 0);

    cmdList->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
    cmdList->IASetVertexBuffers(0, 1, &m_vbView);
    cmdList->IASetIndexBuffer(&m_ibView);
    cmdList->DrawIndexedInstanced(static_cast<UINT>(idxCount), 1, 0, 0, 0);
}

void GlassCockpitRenderer::RenderTransparentHud(
    ID3D12GraphicsCommandList* cmdList,
    u32 screenWidth,
    u32 screenHeight,
    const Avionics::SixPackState& avionics,
    const Navigation::NavReceiverState& nav1,
    const Autopilot::AutopilotStatus& apStatus,
    const Math::Vec3d& velocityBody,
    bool visible) noexcept
{
    if (!m_initialized || !visible || !cmdList || screenWidth == 0 || screenHeight == 0) return;

    m_vertices.clear();
    m_indices.clear();

    BuildTransparentHudGeometry(screenWidth, screenHeight, avionics, nav1, apStatus, velocityBody);

    if (m_indices.empty()) return;

    size_t vertCount = std::min(m_vertices.size(), MaxVertices);
    size_t idxCount = std::min(m_indices.size(), MaxIndices);

    memcpy(m_mappedVertices, m_vertices.data(), vertCount * sizeof(Vertex3D));
    memcpy(m_mappedIndices, m_indices.data(), idxCount * sizeof(u32));

    cmdList->SetGraphicsRootSignature(m_rootSignature.Get());
    cmdList->SetPipelineState(m_pipelineState.Get());

    struct {
        float screenSize[2];
        float panelPos[2];
        float panelScale;
        float opacity;
        float padding[2];
    } constants{};

    constants.screenSize[0] = static_cast<float>(screenWidth);
    constants.screenSize[1] = static_cast<float>(screenHeight);
    constants.panelPos[0] = 0.0f;
    constants.panelPos[1] = 0.0f;
    constants.panelScale = 1.0f;
    constants.opacity = m_opacity;

    cmdList->SetGraphicsRoot32BitConstants(0, 8, &constants, 0);

    cmdList->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
    cmdList->IASetVertexBuffers(0, 1, &m_vbView);
    cmdList->IASetIndexBuffer(&m_ibView);
    cmdList->DrawIndexedInstanced(static_cast<UINT>(idxCount), 1, 0, 0, 0);
}

void GlassCockpitRenderer::AddClippedConvexPoly(
    const std::vector<Math::Vec3f>& poly,
    f32 minX, f32 maxX, f32 minY, f32 maxY,
    const Math::Vec4f& col)
{
    if (poly.size() < 3) return;

    struct P2 { f32 x, y; };
    std::vector<P2> current;
    current.reserve(poly.size() + 4);
    for (const auto& v : poly) {
        current.push_back({ v.x, v.y });
    }

    auto clipEdge = [](const std::vector<P2>& in, auto isInside, auto intersect) {
        std::vector<P2> out;
        if (in.empty()) return out;
        P2 s = in.back();
        for (const auto& p : in) {
            bool pIn = isInside(p);
            bool sIn = isInside(s);
            if (pIn) {
                if (!sIn) {
                    out.push_back(intersect(s, p));
                }
                out.push_back(p);
            } else if (sIn) {
                out.push_back(intersect(s, p));
            }
            s = p;
        }
        return out;
    };

    // 1. Left plane: x >= minX
    current = clipEdge(current,
        [minX](const P2& p) { return p.x >= minX; },
        [minX](const P2& a, const P2& b) -> P2 {
            f32 dx = b.x - a.x;
            f32 t = (std::abs(dx) > 1e-6f) ? (minX - a.x) / dx : 0.0f;
            return { minX, a.y + t * (b.y - a.y) };
        });

    // 2. Right plane: x <= maxX
    current = clipEdge(current,
        [maxX](const P2& p) { return p.x <= maxX; },
        [maxX](const P2& a, const P2& b) -> P2 {
            f32 dx = b.x - a.x;
            f32 t = (std::abs(dx) > 1e-6f) ? (maxX - a.x) / dx : 0.0f;
            return { maxX, a.y + t * (b.y - a.y) };
        });

    // 3. Top plane: y >= minY
    current = clipEdge(current,
        [minY](const P2& p) { return p.y >= minY; },
        [minY](const P2& a, const P2& b) -> P2 {
            f32 dy = b.y - a.y;
            f32 t = (std::abs(dy) > 1e-6f) ? (minY - a.y) / dy : 0.0f;
            return { a.x + t * (b.x - a.x), minY };
        });

    // 4. Bottom plane: y <= maxY
    current = clipEdge(current,
        [maxY](const P2& p) { return p.y <= maxY; },
        [maxY](const P2& a, const P2& b) -> P2 {
            f32 dy = b.y - a.y;
            f32 t = (std::abs(dy) > 1e-6f) ? (maxY - a.y) / dy : 0.0f;
            return { a.x + t * (b.x - a.x), maxY };
        });

    if (current.size() < 3) return;

    for (size_t i = 1; i + 1 < current.size(); ++i) {
        AddTriangle(
            { current[0].x, current[0].y, 0.0f },
            { current[i].x, current[i].y, 0.0f },
            { current[i + 1].x, current[i + 1].y, 0.0f },
            col);
    }
}

void GlassCockpitRenderer::BuildTransparentHudGeometry(
    u32 screenWidth,
    u32 screenHeight,
    const Avionics::SixPackState& avi,
    const Navigation::NavReceiverState& nav1,
    const Autopilot::AutopilotStatus& ap,
    const Math::Vec3d& velocityBody)
{
    f32 sw = static_cast<f32>(screenWidth);
    f32 sh = static_cast<f32>(screenHeight);
    f32 cx = sw * 0.5f;
    f32 cy = sh * 0.5f;

    Math::Vec4f colHudGreen{ 0.05f, 0.98f, 0.35f, 0.95f };
    Math::Vec4f colHudCyan{ 0.0f, 0.88f, 1.0f, 0.95f };
    Math::Vec4f colHudAmber{ 1.0f, 0.75f, 0.05f, 0.95f };
    Math::Vec4f colHudMagenta{ 1.0f, 0.15f, 0.95f, 0.95f };

    f32 pitchDeg = static_cast<f32>(avi.attitude.pitchDeg);
    f32 rollDeg = static_cast<f32>(avi.attitude.rollDeg);
    f32 iasKnots = static_cast<f32>(avi.airspeed.indicatedAirspeedKnots);
    f32 altFeet = static_cast<f32>(avi.altimeter.indicatedAltitudeFeet);
    f32 vsFpm = static_cast<f32>(avi.verticalSpeed.verticalSpeedFpm);
    f32 hdgDeg = static_cast<f32>(avi.heading.indicatedHeadingDeg);

    // 1. Center Aircraft Boresight / Waterline Symbol (Fuselage reference)
    AddLine(cx - 30.0f, cy, cx - 10.0f, cy, 2.0f, colHudGreen);
    AddLine(cx - 10.0f, cy, cx, cy - 6.0f, 2.0f, colHudGreen);
    AddLine(cx, cy - 6.0f, cx + 10.0f, cy, 2.0f, colHudGreen);
    AddLine(cx + 10.0f, cy, cx + 30.0f, cy, 2.0f, colHudGreen);

    // 2. Flight Path Marker (Velocity Vector)
    f32 speedMps = static_cast<f32>(std::sqrt(velocityBody.x * velocityBody.x + velocityBody.y * velocityBody.y + velocityBody.z * velocityBody.z));
    f32 pxPerDeg = 4.2f;
    f32 fpmCx = cx;
    f32 fpmCy = cy;
    if (speedMps > 10.0f) {
        f32 alphaRad = static_cast<f32>(std::atan2(velocityBody.z, velocityBody.x));
        f32 betaRad = static_cast<f32>(std::asin(std::clamp(velocityBody.y / speedMps, -1.0, 1.0)));
        f32 alphaDeg = alphaRad * (180.0f / PI);
        f32 betaDeg = betaRad * (180.0f / PI);
        fpmCx = cx + std::clamp(betaDeg * pxPerDeg, -120.0f, 120.0f);
        fpmCy = cy + std::clamp(alphaDeg * pxPerDeg, -120.0f, 120.0f);
    }
    AddRing(fpmCx, fpmCy, 5.0f, 6.5f, 16, colHudGreen);
    AddLine(fpmCx - 14.0f, fpmCy, fpmCx - 6.5f, fpmCy, 2.0f, colHudGreen);
    AddLine(fpmCx + 6.5f, fpmCy, fpmCx + 14.0f, fpmCy, 2.0f, colHudGreen);
    AddLine(fpmCx, fpmCy - 6.5f, fpmCx, fpmCy - 12.0f, 2.0f, colHudGreen);

    // 3. Collimated Horizon & Pitch Ladder
    f32 rollRad = rollDeg * DEG_TO_RAD;
    f32 cosR = std::cos(-rollRad);
    f32 sinR = std::sin(-rollRad);
    f32 pitchOffset = pitchDeg * pxPerDeg;

    auto XformHud = [&](f32 lx, f32 ly) -> Math::Vec3f {
        f32 rx = lx * cosR - ly * sinR;
        f32 ry = lx * sinR + ly * cosR;
        return { cx + rx, cy + pitchOffset + ry, 0.0f };
    };

    Math::Vec3f hzL0 = XformHud(-150.0f, 0.0f);
    Math::Vec3f hzL1 = XformHud(-35.0f, 0.0f);
    Math::Vec3f hzR0 = XformHud(+35.0f, 0.0f);
    Math::Vec3f hzR1 = XformHud(+150.0f, 0.0f);
    AddLine(hzL0.x, hzL0.y, hzL1.x, hzL1.y, 2.0f, colHudGreen);
    AddLine(hzR0.x, hzR0.y, hzR1.x, hzR1.y, 2.0f, colHudGreen);

    for (int p = -20; p <= 20; p += 5) {
        if (p == 0) continue;
        f32 rungY = -static_cast<f32>(p) * pxPerDeg;
        f32 halfSpan = (std::abs(p) % 10 == 0) ? 36.0f : 20.0f;
        f32 gap = 12.0f;

        Math::Vec3f rL0 = XformHud(-halfSpan, rungY);
        Math::Vec3f rL1 = XformHud(-gap, rungY);
        AddLine(rL0.x, rL0.y, rL1.x, rL1.y, 1.8f, colHudGreen);

        Math::Vec3f rR0 = XformHud(+gap, rungY);
        Math::Vec3f rR1 = XformHud(+halfSpan, rungY);
        AddLine(rR0.x, rR0.y, rR1.x, rR1.y, 1.8f, colHudGreen);

        f32 tipLen = (p > 0) ? +5.0f : -5.0f;
        Math::Vec3f tL = XformHud(-halfSpan, rungY + tipLen);
        Math::Vec3f tR = XformHud(+halfSpan, rungY + tipLen);
        AddLine(rL0.x, rL0.y, tL.x, tL.y, 1.8f, colHudGreen);
        AddLine(rR0.x, rR0.y, tR.x, tR.y, 1.8f, colHudGreen);

        if (std::abs(p) % 10 == 0) {
            std::string pStr = std::to_string(std::abs(p));
            Math::Vec3f lblPos = XformHud(-halfSpan - 16.0f, rungY - 4.0f);
            DrawString(pStr, lblPos.x, lblPos.y, 9.0f, colHudGreen);
        }
    }

    // 4. Roll arc & pointer
    f32 rollRadius = 160.0f;
    f32 rollCenterY = cy + 20.0f;
    AddArcRing(cx, rollCenterY, rollRadius - 1.0f, rollRadius + 1.0f, 230.0f, 310.0f, 24, colHudGreen);
    int hudRollTicks[] = { -45, -30, -20, -10, 0, 10, 20, 30, 45 };
    for (int deg : hudRollTicks) {
        f32 angleRad = (90.0f + static_cast<f32>(deg)) * DEG_TO_RAD;
        f32 len = (deg == 0 || std::abs(deg) == 30) ? 8.0f : 4.0f;
        f32 x0 = cx + (rollRadius - len) * std::cos(angleRad);
        f32 y0 = rollCenterY - (rollRadius - len) * std::sin(angleRad);
        f32 x1 = cx + rollRadius * std::cos(angleRad);
        f32 y1 = rollCenterY - rollRadius * std::sin(angleRad);
        AddLine(x0, y0, x1, y1, 1.5f, colHudGreen);
    }
    f32 ptrAngleRad = (90.0f - rollDeg) * DEG_TO_RAD;
    f32 ptrTipX = cx + (rollRadius - 2.0f) * std::cos(ptrAngleRad);
    f32 ptrTipY = rollCenterY - (rollRadius - 2.0f) * std::sin(ptrAngleRad);
    f32 ptrBaseX0 = cx + (rollRadius - 8.0f) * std::cos(ptrAngleRad - 0.04f);
    f32 ptrBaseY0 = rollCenterY - (rollRadius - 8.0f) * std::sin(ptrAngleRad - 0.04f);
    f32 ptrBaseX1 = cx + (rollRadius - 8.0f) * std::cos(ptrAngleRad + 0.04f);
    f32 ptrBaseY1 = rollCenterY - (rollRadius - 8.0f) * std::sin(ptrAngleRad + 0.04f);
    AddTriangle({ ptrTipX, ptrTipY, 0.0f }, { ptrBaseX0, ptrBaseY0, 0.0f }, { ptrBaseX1, ptrBaseY1, 0.0f }, colHudGreen);

    // 5. Left Airspeed Readout
    f32 asX = cx - 210.0f;
    f32 asY = cy;
    AddLine(asX - 42.0f, asY - 14.0f, asX + 10.0f, asY - 14.0f, 2.0f, colHudGreen);
    AddLine(asX + 10.0f, asY - 14.0f, asX + 22.0f, asY, 2.0f, colHudGreen);
    AddLine(asX + 22.0f, asY, asX + 10.0f, asY + 14.0f, 2.0f, colHudGreen);
    AddLine(asX + 10.0f, asY + 14.0f, asX - 42.0f, asY + 14.0f, 2.0f, colHudGreen);
    AddLine(asX - 42.0f, asY + 14.0f, asX - 42.0f, asY - 14.0f, 2.0f, colHudGreen);

    std::stringstream asSs;
    asSs << static_cast<int>(iasKnots);
    DrawString(asSs.str(), asX - 34.0f, asY - 6.0f, 13.0f, colHudGreen, 2.0f);
    DrawString("KT", asX - 32.0f, asY - 26.0f, 9.0f, colHudCyan);

    for (int k = -30; k <= 30; k += 10) {
        if (k == 0) continue;
        f32 tickY = asY - static_cast<f32>(k) * 2.2f;
        AddLine(asX + 22.0f, tickY, asX + 30.0f, tickY, 1.5f, colHudGreen);
        std::stringstream ssK;
        ssK << static_cast<int>(iasKnots) + k;
        DrawString(ssK.str(), asX + 34.0f, tickY - 4.0f, 8.0f, colHudGreen);
    }

    // 6. Right Altitude Readout
    f32 altX = cx + 210.0f;
    f32 altY = cy;
    AddLine(altX - 10.0f, altY - 14.0f, altX + 46.0f, altY - 14.0f, 2.0f, colHudGreen);
    AddLine(altX + 46.0f, altY - 14.0f, altX + 46.0f, altY + 14.0f, 2.0f, colHudGreen);
    AddLine(altX + 46.0f, altY + 14.0f, altX - 10.0f, altY + 14.0f, 2.0f, colHudGreen);
    AddLine(altX - 10.0f, altY + 14.0f, altX - 22.0f, altY, 2.0f, colHudGreen);
    AddLine(altX - 22.0f, altY, altX - 10.0f, altY - 14.0f, 2.0f, colHudGreen);

    std::stringstream altSs;
    altSs << static_cast<int>(altFeet);
    DrawString(altSs.str(), altX - 6.0f, altY - 6.0f, 13.0f, colHudGreen, 2.0f);
    DrawString("FT", altX + 20.0f, altY - 26.0f, 9.0f, colHudCyan);

    for (int a = -300; a <= 300; a += 100) {
        if (a == 0) continue;
        f32 tickY = altY - static_cast<f32>(a) * 0.22f;
        AddLine(altX - 30.0f, tickY, altX - 22.0f, tickY, 1.5f, colHudGreen);
        std::stringstream ssA;
        ssA << static_cast<int>(altFeet) + a;
        DrawString(ssA.str(), altX - 60.0f, tickY - 4.0f, 8.0f, colHudGreen);
    }

    if (ap.verticalMode == Autopilot::VerticalMode::ALT) {
        std::stringstream tAltSs;
        tAltSs << "SEL " << static_cast<int>(ap.targets.targetAltitudeFt);
        DrawString(tAltSs.str(), altX - 10.0f, altY - 42.0f, 9.0f, colHudCyan);
    }

    std::stringstream vsSs;
    vsSs << (vsFpm >= 0 ? "+" : "") << static_cast<int>(vsFpm) << " FPM";
    DrawString(vsSs.str(), altX - 10.0f, altY + 22.0f, 9.0f, colHudGreen);

    // 7. Heading Tape at Bottom
    f32 hdgY = sh - 45.0f;
    f32 hdgW = 280.0f;
    AddLine(cx - hdgW * 0.5f, hdgY, cx + hdgW * 0.5f, hdgY, 1.8f, colHudGreen);
    AddLine(cx - 20.0f, hdgY + 2.0f, cx + 20.0f, hdgY + 2.0f, 1.8f, colHudGreen);
    AddLine(cx + 20.0f, hdgY + 2.0f, cx + 20.0f, hdgY + 22.0f, 1.8f, colHudGreen);
    AddLine(cx + 20.0f, hdgY + 22.0f, cx - 20.0f, hdgY + 22.0f, 1.8f, colHudGreen);
    AddLine(cx - 20.0f, hdgY + 22.0f, cx - 20.0f, hdgY + 2.0f, 1.8f, colHudGreen);
    std::stringstream hdgBoxSs;
    hdgBoxSs << std::setw(3) << std::setfill('0') << static_cast<int>(hdgDeg);
    DrawString(hdgBoxSs.str(), cx - 14.0f, hdgY + 6.0f, 12.0f, colHudGreen, 2.0f);

    f32 pxPerHdgDeg = 3.6f;
    for (int d = -30; d <= 30; d += 5) {
        if (d == 0) continue;
        f32 tickX = cx + static_cast<f32>(d) * pxPerHdgDeg;
        f32 tickLen = (d % 10 == 0) ? 8.0f : 4.0f;
        AddLine(tickX, hdgY - tickLen, tickX, hdgY, 1.5f, colHudGreen);
        if (d % 10 == 0) {
            int lblDeg = (static_cast<int>(hdgDeg) + d + 360) % 360;
            std::stringstream ssD;
            ssD << lblDeg / 10;
            DrawString(ssD.str(), tickX - 5.0f, hdgY - 20.0f, 9.0f, colHudGreen);
        }
    }

    // Heading Bug Diamond
    f32 bugDelta = static_cast<f32>(ap.targets.targetHeadingDeg - hdgDeg);
    while (bugDelta > 180.0f) bugDelta -= 360.0f;
    while (bugDelta < -180.0f) bugDelta += 360.0f;
    if (std::abs(bugDelta) <= 35.0f) {
        f32 bugX = cx + bugDelta * pxPerHdgDeg;
        AddLine(bugX, hdgY - 10.0f, bugX - 5.0f, hdgY - 5.0f, 1.8f, colHudCyan);
        AddLine(bugX - 5.0f, hdgY - 5.0f, bugX, hdgY, 1.8f, colHudCyan);
        AddLine(bugX, hdgY, bugX + 5.0f, hdgY - 5.0f, 1.8f, colHudCyan);
        AddLine(bugX + 5.0f, hdgY - 5.0f, bugX, hdgY - 10.0f, 1.8f, colHudCyan);
    }

    // 8. Flight Director Steering Cue
    if (ap.flightDirectorActive) {
        f32 fdPitchOffset = (pitchDeg - static_cast<f32>(ap.fdCues.commandPitchDeg)) * pxPerDeg;
        f32 fdRollOffset = (rollDeg - static_cast<f32>(ap.fdCues.commandRollDeg)) * 1.8f;
        f32 fdBarLen = 50.0f;
        AddLine(cx - fdBarLen, cy + fdPitchOffset, cx + fdBarLen, cy + fdPitchOffset, 2.5f, colHudMagenta);
        AddLine(cx + fdRollOffset, cy - 35.0f, cx + fdRollOffset, cy + 35.0f, 2.5f, colHudMagenta);
    }

    // 9. Autopilot Top Banner
    f32 bannerY = 24.0f;
    std::string apText = ap.masterEngaged ? "AP" : (ap.flightDirectorActive ? "FD" : "");
    if (!apText.empty()) {
        DrawString(apText, cx - 120.0f, bannerY, 13.0f, colHudGreen, 2.0f);
    }
    DrawString(LateralModeToString(ap.lateralMode), cx - 60.0f, bannerY, 11.0f, colHudGreen);
    DrawString(VerticalModeToString(ap.verticalMode), cx + 20.0f, bannerY, 11.0f, colHudGreen);

    if (!nav1.stationIdent.empty()) {
        std::stringstream navSs;
        navSs << "NAV1 " << nav1.stationIdent << " " << std::fixed << std::setprecision(2) << nav1.activeFrequencyMhz;
        DrawString(navSs.str(), 25.0f, bannerY, 10.0f, colHudCyan);
    }
    if (nav1.dme.hasSignal) {
        std::stringstream dmeSs;
        dmeSs << "DME " << std::fixed << std::setprecision(1) << nav1.dme.slantRangeNm << " NM";
        DrawString(dmeSs.str(), 25.0f, bannerY + 16.0f, 10.0f, colHudCyan);
    }
}

void GlassCockpitRenderer::BuildPfdGeometry(
    f32 x, f32 y, f32 w, f32 h,
    const Avionics::SixPackState& avi,
    const Navigation::NavReceiverState& nav1,
    const Autopilot::AutopilotStatus& ap)
{
    // 1. Dark bezel & housing
    DrawBezelAndHousing(x, y, w, h);

    // 2. Top Nav/Comm frequencies header
    f32 headerH = 24.0f;
    DrawTopFrequenciesHeader(x + 10.0f, y + 8.0f, w - 20.0f, nav1);

    // 3. FMA (Flight Mode Annunciator) Banner
    f32 fmaY = y + 8.0f + headerH + 4.0f;
    f32 fmaH = 34.0f;
    DrawFmaBanner(x + 10.0f, fmaY, w - 20.0f, fmaH, ap);

    // 4. Center Electronic Attitude Director Indicator (EADI)
    f32 adiW = 280.0f;
    f32 adiH = 220.0f;
    f32 adiCx = x + w * 0.5f;
    f32 adiCy = fmaY + fmaH + 8.0f + (adiH * 0.5f);
    DrawAttitudeDirectorIndicator(
        adiCx, adiCy, adiW, adiH,
        static_cast<f32>(avi.attitude.pitchDeg),
        static_cast<f32>(avi.attitude.rollDeg),
        static_cast<f32>(avi.turnCoordinator.slipSkidBallPosition),
        ap.fdCues);

    // 5. Left Airspeed Tape
    f32 tapeW = 75.0f;
    f32 tapeH = 220.0f;
    f32 asX = adiCx - (adiW * 0.5f) - tapeW - 6.0f;
    f32 asY = adiCy - (tapeH * 0.5f);
    DrawAirspeedTape(
        asX, asY, tapeW, tapeH,
        static_cast<f32>(avi.airspeed.indicatedAirspeedKnots),
        static_cast<f32>(avi.verticalSpeed.verticalSpeedFpm));

    // 6. Right Altimeter Tape
    f32 altX = adiCx + (adiW * 0.5f) + 6.0f;
    f32 altY = asY;
    DrawAltimeterTape(
        altX, altY, tapeW, tapeH,
        static_cast<f32>(avi.altimeter.indicatedAltitudeFeet),
        static_cast<f32>(ap.targets.targetAltitudeFt),
        static_cast<f32>(avi.altimeter.kollsmanInHg));

    // 7. Far Right Vertical Speed Tape
    f32 vsW = 38.0f;
    f32 vsX = altX + tapeW + 4.0f;
    f32 vsY = asY + 10.0f;
    f32 vsH = tapeH - 20.0f;
    DrawVerticalSpeedTape(
        vsX, vsY, vsW, vsH,
        static_cast<f32>(avi.verticalSpeed.verticalSpeedFpm),
        static_cast<f32>(ap.targets.targetVerticalSpeedFpm),
        ap.verticalMode == Autopilot::VerticalMode::VS);

    // 8. Bottom Horizontal Situation Indicator (HSI) Compass
    f32 hsiRadius = 85.0f;
    f32 hsiCx = adiCx;
    f32 hsiCy = adiCy + (adiH * 0.5f) + 12.0f + hsiRadius;
    if (hsiCy + hsiRadius > y + h - 10.0f) {
        hsiRadius = (y + h - 10.0f) - (adiCy + (adiH * 0.5f) + 12.0f);
        hsiCy = adiCy + (adiH * 0.5f) + 12.0f + hsiRadius;
    }
    DrawHsiCompass(
        hsiCx, hsiCy, hsiRadius,
        static_cast<f32>(avi.heading.indicatedHeadingDeg),
        static_cast<f32>(ap.targets.targetHeadingDeg),
        nav1);
}

void GlassCockpitRenderer::DrawBezelAndHousing(f32 x, f32 y, f32 w, f32 h) {
    Math::Vec4f colOuterBezel{ 0.10f, 0.11f, 0.12f, 0.98f };
    Math::Vec4f colInnerHousing{ 0.04f, 0.05f, 0.06f, 0.96f };
    Math::Vec4f colBorder{ 0.35f, 0.38f, 0.42f, 1.0f };

    // Outer bezel frame
    AddQuad(
        { x, y, 0.0f },
        { x + w, y, 0.0f },
        { x + w, y + h, 0.0f },
        { x, y + h, 0.0f },
        colOuterBezel);

    // Inner instrument panel
    f32 b = 4.0f;
    AddQuad(
        { x + b, y + b, 0.0f },
        { x + w - b, y + b, 0.0f },
        { x + w - b, y + h - b, 0.0f },
        { x + b, y + h - b, 0.0f },
        colInnerHousing);

    // Bezel border line
    AddLine(x, y, x + w, y, 1.5f, colBorder);
    AddLine(x + w, y, x + w, y + h, 1.5f, colBorder);
    AddLine(x + w, y + h, x, y + h, 1.5f, colBorder);
    AddLine(x, y + h, x, y, 1.5f, colBorder);
}

void GlassCockpitRenderer::DrawTopFrequenciesHeader(f32 x, f32 y, f32 w, const Navigation::NavReceiverState& nav1) {
    Math::Vec4f colBoxBg{ 0.02f, 0.02f, 0.03f, 0.9f };
    Math::Vec4f colBorder{ 0.25f, 0.28f, 0.32f, 1.0f };
    Math::Vec4f colCyan{ 0.0f, 0.9f, 1.0f, 1.0f };
    Math::Vec4f colWhite{ 0.95f, 0.95f, 0.95f, 1.0f };
    Math::Vec4f colGreen{ 0.2f, 0.95f, 0.3f, 1.0f };

    AddQuad(
        { x, y, 0.0f },
        { x + w, y, 0.0f },
        { x + w, y + 22.0f, 0.0f },
        { x, y + 22.0f, 0.0f },
        colBoxBg);
    AddLine(x, y + 22.0f, x + w, y + 22.0f, 1.0f, colBorder);

    // NAV1 Active & Standby Frequencies
    std::stringstream navSs;
    navSs << "NAV1 " << std::fixed << std::setprecision(2) << nav1.activeFrequencyMhz;
    if (!nav1.stationIdent.empty()) {
        navSs << " " << nav1.stationIdent;
    }
    DrawString(navSs.str(), x + 6.0f, y + 6.0f, 10.0f, colCyan);

    std::stringstream stbySs;
    stbySs << "[" << std::fixed << std::setprecision(2) << nav1.standbyFrequencyMhz << "]";
    DrawString(stbySs.str(), x + 160.0f, y + 6.0f, 10.0f, colWhite);

    // DME Distance and Speed
    if (nav1.dme.hasSignal) {
        std::stringstream dmeSs;
        dmeSs << "DME " << std::fixed << std::setprecision(1) << nav1.dme.slantRangeNm << "NM "
              << static_cast<int>(nav1.dme.groundSpeedKnots) << "KT";
        DrawString(dmeSs.str(), x + 240.0f, y + 6.0f, 10.0f, colGreen);
    }

    // Transponder
    DrawString("XPDR 1200 ALT", x + w - 110.0f, y + 6.0f, 10.0f, colCyan);
}

void GlassCockpitRenderer::DrawFmaBanner(f32 x, f32 y, f32 w, f32 h, const Autopilot::AutopilotStatus& ap) {
    Math::Vec4f colFmaBg{ 0.03f, 0.04f, 0.05f, 0.95f };
    Math::Vec4f colBorder{ 0.30f, 0.34f, 0.38f, 1.0f };
    Math::Vec4f colGreen{ 0.1f, 0.95f, 0.2f, 1.0f };
    Math::Vec4f colAmber{ 1.0f, 0.75f, 0.0f, 1.0f };
    Math::Vec4f colWhite{ 0.95f, 0.95f, 0.95f, 1.0f };

    AddQuad(
        { x, y, 0.0f },
        { x + w, y, 0.0f },
        { x + w, y + h, 0.0f },
        { x, y + h, 0.0f },
        colFmaBg);

    AddLine(x, y, x + w, y, 1.0f, colBorder);
    AddLine(x, y + h, x + w, y + h, 1.0f, colBorder);

    f32 boxW = w / 3.0f;
    AddLine(x + boxW, y, x + boxW, y + h, 1.0f, colBorder);
    AddLine(x + boxW * 2.0f, y, x + boxW * 2.0f, y + h, 1.0f, colBorder);

    // 1. Lateral Mode Box (Left)
    std::string latStr = LateralModeToString(ap.lateralMode);
    DrawString(latStr, x + 16.0f, y + 8.0f, 14.0f, colGreen, 2.0f);
    if (ap.lateralArmed != Autopilot::LateralArmedMode::None) {
        std::string armedStr = (ap.lateralArmed == Autopilot::LateralArmedMode::LOC) ? "ARM LOC" : "ARM NAV";
        DrawString(armedStr, x + 80.0f, y + 10.0f, 9.0f, colWhite);
    }

    // 2. Autopilot Master & FD Box (Center)
    f32 centerBoxX = x + boxW;
    if (ap.masterEngaged) {
        DrawString("AP", centerBoxX + 24.0f, y + 8.0f, 16.0f, colGreen, 2.5f);
    } else if (ap.disconnectWarning) {
        DrawString("AP DISC", centerBoxX + 10.0f, y + 8.0f, 14.0f, colAmber, 2.0f);
    } else {
        DrawString("--", centerBoxX + 32.0f, y + 8.0f, 14.0f, colWhite);
    }

    if (ap.flightDirectorActive) {
        DrawString("FD", centerBoxX + 80.0f, y + 8.0f, 14.0f, colGreen, 2.0f);
    }
    if (ap.yawDamperActive) {
        DrawString("YD", centerBoxX + 115.0f, y + 8.0f, 12.0f, colGreen);
    }

    // 3. Vertical Mode Box (Right)
    f32 rightBoxX = x + boxW * 2.0f;
    std::stringstream vertSs;
    vertSs << VerticalModeToString(ap.verticalMode);
    if (ap.verticalMode == Autopilot::VerticalMode::ALT) {
        vertSs << " " << static_cast<int>(ap.targets.targetAltitudeFt);
    } else if (ap.verticalMode == Autopilot::VerticalMode::VS) {
        vertSs << " " << (ap.targets.targetVerticalSpeedFpm >= 0 ? "+" : "")
               << static_cast<int>(ap.targets.targetVerticalSpeedFpm);
    }
    DrawString(vertSs.str(), rightBoxX + 16.0f, y + 8.0f, 13.0f, colGreen, 2.0f);
    if (ap.verticalArmed == Autopilot::VerticalArmedMode::GS) {
        DrawString("ARM GS", rightBoxX + 120.0f, y + 10.0f, 9.0f, colWhite);
    }
}

void GlassCockpitRenderer::DrawAttitudeDirectorIndicator(
    f32 cx, f32 cy, f32 w, f32 h,
    f32 pitchDeg, f32 rollDeg, f32 slipBall,
    const Autopilot::FlightDirectorCues& fd)
{
    Math::Vec4f colSky{ 0.0f, 0.45f, 0.85f, m_eadiBgOpacity };
    Math::Vec4f colEarth{ 0.40f, 0.22f, 0.08f, m_eadiBgOpacity };
    Math::Vec4f colHorizon{ 1.0f, 1.0f, 1.0f, 1.0f };
    Math::Vec4f colYellow{ 1.0f, 0.82f, 0.0f, 1.0f };
    Math::Vec4f colMagenta{ 1.0f, 0.1f, 0.95f, 1.0f };
    Math::Vec4f colApertureBorder{ 0.35f, 0.38f, 0.42f, 1.0f };

    f32 halfW = w * 0.5f;
    f32 halfH = h * 0.5f;
    f32 minX = cx - halfW;
    f32 maxX = cx + halfW;
    f32 minY = cy - halfH;
    f32 maxY = cy + halfH;

    // Pitch displacement: pixels per degree
    f32 pxPerDeg = 3.6f;
    f32 pitchOffset = pitchDeg * pxPerDeg;

    // Roll angle in radians
    f32 rollRad = rollDeg * DEG_TO_RAD;
    f32 cosR = std::cos(-rollRad);
    f32 sinR = std::sin(-rollRad);

    // Transform local point (lx, ly) around horizon center (cx, cy + pitchOffset)
    auto Xform = [&](f32 lx, f32 ly) -> Math::Vec3f {
        f32 rx = lx * cosR - ly * sinR;
        f32 ry = lx * sinR + ly * cosR;
        return { cx + rx, cy + pitchOffset + ry, 0.0f };
    };

    // Draw Sky (upper half) and Earth (lower half) strictly clipped to ADI window
    f32 ext = std::sqrt(w * w + h * h);
    Math::Vec3f pSkyTL = Xform(-ext, -ext);
    Math::Vec3f pSkyTR = Xform(+ext, -ext);
    Math::Vec3f pSkyBR = Xform(+ext, 0.0f);
    Math::Vec3f pSkyBL = Xform(-ext, 0.0f);
    std::vector<Math::Vec3f> skyPoly = { pSkyTL, pSkyTR, pSkyBR, pSkyBL };
    AddClippedConvexPoly(skyPoly, minX, maxX, minY, maxY, colSky);

    Math::Vec3f pEarthTL = Xform(-ext, 0.0f);
    Math::Vec3f pEarthTR = Xform(+ext, 0.0f);
    Math::Vec3f pEarthBR = Xform(+ext, +ext);
    Math::Vec3f pEarthBL = Xform(-ext, +ext);
    std::vector<Math::Vec3f> earthPoly = { pEarthTL, pEarthTR, pEarthBR, pEarthBL };
    AddClippedConvexPoly(earthPoly, minX, maxX, minY, maxY, colEarth);

    auto AddClippedLine = [&](f32 lx0, f32 ly0, f32 lx1, f32 ly1, f32 th, const Math::Vec4f& col) {
        f32 rx0 = lx0, ry0 = ly0, rx1 = lx1, ry1 = ly1;
        if (ClipLineSegmentToRect(rx0, ry0, rx1, ry1, minX, maxX, minY, maxY)) {
            AddLine(rx0, ry0, rx1, ry1, th, col);
        }
    };

    // Horizon line
    Math::Vec3f h0 = Xform(-halfW * 0.95f, 0.0f);
    Math::Vec3f h1 = Xform(+halfW * 0.95f, 0.0f);
    AddClippedLine(h0.x, h0.y, h1.x, h1.y, 2.5f, colHorizon);

    // Pitch Ladder rungs every 5 and 10 degrees
    for (int p = -25; p <= 25; p += 5) {
        if (p == 0) continue;
        f32 rungY = -static_cast<f32>(p) * pxPerDeg;
        f32 rungHalfW = (std::abs(p) % 10 == 0) ? 26.0f : 14.0f;

        Math::Vec3f r0 = Xform(-rungHalfW, rungY);
        Math::Vec3f r1 = Xform(+rungHalfW, rungY);
        AddClippedLine(r0.x, r0.y, r1.x, r1.y, 1.5f, colHorizon);

        // Downward or upward tips
        f32 tipLen = (p > 0) ? +4.0f : -4.0f;
        Math::Vec3f t0 = Xform(-rungHalfW, rungY + tipLen);
        Math::Vec3f t1 = Xform(+rungHalfW, rungY + tipLen);
        AddClippedLine(r0.x, r0.y, t0.x, t0.y, 1.5f, colHorizon);
        AddClippedLine(r1.x, r1.y, t1.x, t1.y, 1.5f, colHorizon);

        // Pitch numeric labels on 10 and 20 degree lines
        if (std::abs(p) % 10 == 0) {
            std::string pStr = std::to_string(std::abs(p));
            Math::Vec3f textPos = Xform(-rungHalfW - 14.0f, rungY - 4.0f);
            if (textPos.x >= minX + 2.0f && textPos.x <= maxX - 16.0f &&
                textPos.y >= minY + 2.0f && textPos.y <= maxY - 12.0f) {
                DrawString(pStr, textPos.x, textPos.y, 8.0f, colHorizon);
            }
        }
    }

    // Crisp aperture boundary around EADI
    AddLine(minX, minY, maxX, minY, 1.5f, colApertureBorder);
    AddLine(maxX, minY, maxX, maxY, 1.5f, colApertureBorder);
    AddLine(maxX, maxY, minX, maxY, 1.5f, colApertureBorder);
    AddLine(minX, maxY, minX, minY, 1.5f, colApertureBorder);

    // Roll scale arc and ticks
    f32 rollRadius = halfW * 0.75f;
    f32 rollCenterY = cy + 25.0f;
    AddArcRing(cx, rollCenterY, rollRadius - 1.0f, rollRadius + 1.0f, 210.0f, 330.0f, 32, colHorizon);

    int rollTicks[] = { -60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60 };
    for (int deg : rollTicks) {
        f32 angleRad = (90.0f + static_cast<f32>(deg)) * DEG_TO_RAD;
        f32 len = (deg == 0 || std::abs(deg) == 30 || std::abs(deg) == 60) ? 8.0f : 4.0f;
        f32 x0 = cx + (rollRadius - len) * std::cos(angleRad);
        f32 y0 = rollCenterY - (rollRadius - len) * std::sin(angleRad);
        f32 x1 = cx + rollRadius * std::cos(angleRad);
        f32 y1 = rollCenterY - rollRadius * std::sin(angleRad);
        AddLine(x0, y0, x1, y1, 1.5f, colHorizon);
    }

    // Roll Sky Pointer triangle
    f32 ptrAngleRad = (90.0f - rollDeg) * DEG_TO_RAD;
    f32 ptrTipX = cx + (rollRadius - 2.0f) * std::cos(ptrAngleRad);
    f32 ptrTipY = rollCenterY - (rollRadius - 2.0f) * std::sin(ptrAngleRad);
    f32 ptrBaseX0 = cx + (rollRadius - 10.0f) * std::cos(ptrAngleRad - 0.05f);
    f32 ptrBaseY0 = rollCenterY - (rollRadius - 10.0f) * std::sin(ptrAngleRad - 0.05f);
    f32 ptrBaseX1 = cx + (rollRadius - 10.0f) * std::cos(ptrAngleRad + 0.05f);
    f32 ptrBaseY1 = rollCenterY - (rollRadius - 10.0f) * std::sin(ptrAngleRad + 0.05f);
    AddTriangle({ ptrTipX, ptrTipY, 0.0f }, { ptrBaseX0, ptrBaseY0, 0.0f }, { ptrBaseX1, ptrBaseY1, 0.0f }, colHorizon);

    // Slip/skid indicator trapezoid under roll pointer
    f32 slipOffset = std::clamp(slipBall * 15.0f, -18.0f, 18.0f);
    AddQuad(
        { ptrTipX + slipOffset - 6.0f, ptrTipY + 4.0f, 0.0f },
        { ptrTipX + slipOffset + 6.0f, ptrTipY + 4.0f, 0.0f },
        { ptrTipX + slipOffset + 6.0f, ptrTipY + 8.0f, 0.0f },
        { ptrTipX + slipOffset - 6.0f, ptrTipY + 8.0f, 0.0f },
        colHorizon);

    // Flight Director (FD) Command Crossbars
    if (fd.active) {
        f32 fdPitchOffset = (pitchDeg - static_cast<f32>(fd.commandPitchDeg)) * pxPerDeg;
        f32 fdRollOffset = (rollDeg - static_cast<f32>(fd.commandRollDeg)) * 1.5f;

        // Magenta horizontal pitch command bar
        f32 fdBarW = 70.0f;
        AddLine(cx - fdBarW, cy + fdPitchOffset, cx + fdBarW, cy + fdPitchOffset, 3.0f, colMagenta);

        // Magenta vertical roll command bar
        f32 fdBarH = 50.0f;
        AddLine(cx + fdRollOffset, cy - fdBarH, cx + fdRollOffset, cy + fdBarH, 3.0f, colMagenta);
    }

    // Miniature Aircraft Reference Symbol (Yellow center pip & wings)
    f32 pipR = 3.0f;
    AddCircleFill(cx, cy, pipR, 12, colYellow);
    AddLine(cx - 40.0f, cy, cx - 12.0f, cy, 3.5f, colYellow);
    AddLine(cx - 12.0f, cy, cx - 12.0f, cy + 8.0f, 3.5f, colYellow);
    AddLine(cx + 12.0f, cy, cx + 40.0f, cy, 3.5f, colYellow);
    AddLine(cx + 12.0f, cy, cx + 12.0f, cy + 8.0f, 3.5f, colYellow);
}

void GlassCockpitRenderer::DrawAirspeedTape(
    f32 x, f32 y, f32 w, f32 h,
    f32 iasKnots, f32 verticalSpeedFpm)
{
    Math::Vec4f colTapeBg{ 0.05f, 0.06f, 0.07f, 0.88f };
    Math::Vec4f colBorder{ 0.35f, 0.38f, 0.42f, 1.0f };
    Math::Vec4f colWhite{ 0.95f, 0.95f, 0.95f, 1.0f };
    Math::Vec4f colGreen{ 0.1f, 0.85f, 0.2f, 1.0f };
    Math::Vec4f colYellow{ 1.0f, 0.85f, 0.1f, 1.0f };
    Math::Vec4f colRed{ 0.95f, 0.15f, 0.15f, 1.0f };
    Math::Vec4f colMagenta{ 1.0f, 0.1f, 0.95f, 1.0f };

    // Tape background & border
    AddQuad({ x, y, 0.0f }, { x + w, y, 0.0f }, { x + w, y + h, 0.0f }, { x, y + h, 0.0f }, colTapeBg);
    AddLine(x + w, y, x + w, y + h, 1.5f, colBorder);

    f32 cy = y + h * 0.5f;
    f32 pxPerKnot = 2.4f;

    // Colored speed bands (Cessna 172 airspeed polars)
    // White band: Vso (40) to Vfe (85)
    // Green band: Vs (48) to Vno (129)
    // Yellow band: Vno (129) to Vne (163)
    // Red line: Vne (163)
    auto KnotToY = [&](f32 kts) -> f32 {
        return cy - (kts - iasKnots) * pxPerKnot;
    };

    f32 bandX = x + w - 8.0f;
    f32 bandW = 5.0f;

    f32 yVso = KnotToY(40.0f);
    f32 yVfe = KnotToY(85.0f);
    AddQuad({ bandX - 4.0f, yVfe, 0.0f }, { bandX - 4.0f + 3.0f, yVfe, 0.0f }, { bandX - 4.0f + 3.0f, yVso, 0.0f }, { bandX - 4.0f, yVso, 0.0f }, colWhite);

    f32 yVs = KnotToY(48.0f);
    f32 yVno = KnotToY(129.0f);
    AddQuad({ bandX, yVno, 0.0f }, { bandX + bandW, yVno, 0.0f }, { bandX + bandW, yVs, 0.0f }, { bandX, yVs, 0.0f }, colGreen);

    f32 yVne = KnotToY(163.0f);
    AddQuad({ bandX, yVne, 0.0f }, { bandX + bandW, yVne, 0.0f }, { bandX + bandW, yVno, 0.0f }, { bandX, yVno, 0.0f }, colYellow);
    AddLine(bandX - 2.0f, yVne, bandX + bandW + 2.0f, yVne, 2.5f, colRed);

    // Moving tick marks and numeric labels every 10 and 20 knots
    int minKts = static_cast<int>(iasKnots - (h * 0.5f) / pxPerKnot) / 10 * 10;
    int maxKts = static_cast<int>(iasKnots + (h * 0.5f) / pxPerKnot) / 10 * 10 + 10;

    for (int k = minKts; k <= maxKts; k += 10) {
        if (k < 0) continue;
        f32 ty = KnotToY(static_cast<f32>(k));
        if (ty < y || ty > y + h) continue;

        f32 tickLen = (k % 20 == 0) ? 14.0f : 8.0f;
        AddLine(x + w - 10.0f - tickLen, ty, x + w - 10.0f, ty, 1.5f, colWhite);

        if (k % 20 == 0) {
            std::string kStr = std::to_string(k);
            DrawString(kStr, x + 6.0f, ty - 5.0f, 10.0f, colWhite);
        }
    }

    // 6-second airspeed trend vector (approximated from vertical speed & pitch or accel)
    f32 accelKtsPerSec = verticalSpeedFpm * 0.0005f; // trend projection
    f32 trendY = KnotToY(iasKnots + accelKtsPerSec * 6.0f);
    trendY = std::clamp(trendY, y + 6.0f, y + h - 6.0f);
    AddLine(x + w - 2.0f, cy, x + w - 2.0f, trendY, 3.0f, colMagenta);

    // Digital rolling readout box with pointer notch
    f32 boxH = 26.0f;
    f32 boxW = 56.0f;
    f32 boxX = x + w - boxW - 2.0f;
    f32 boxY = cy - boxH * 0.5f;

    AddQuad(
        { boxX, boxY, 0.0f },
        { boxX + boxW, boxY, 0.0f },
        { boxX + boxW, boxY + boxH, 0.0f },
        { boxX, boxY + boxH, 0.0f },
        { 0.0f, 0.0f, 0.0f, 0.98f });

    AddLine(boxX, boxY, boxX + boxW, boxY, 1.5f, colGreen);
    AddLine(boxX + boxW, boxY, boxX + boxW, boxY + boxH, 1.5f, colGreen);
    AddLine(boxX + boxW, boxY + boxH, boxX, boxY + boxH, 1.5f, colGreen);
    AddLine(boxX, boxY + boxH, boxX, boxY, 1.5f, colGreen);

    // Pointer notch pointing right
    AddTriangle(
        { boxX + boxW + 4.0f, cy, 0.0f },
        { boxX + boxW, cy - 5.0f, 0.0f },
        { boxX + boxW, cy + 5.0f, 0.0f },
        colGreen);

    std::string iasStr = std::to_string(static_cast<int>(iasKnots));
    DrawString(iasStr, boxX + 8.0f, boxY + 6.0f, 14.0f, colWhite, 2.0f);
}

void GlassCockpitRenderer::DrawAltimeterTape(
    f32 x, f32 y, f32 w, f32 h,
    f32 altFeet, f32 targetAltFeet, f32 kollsmanInHg)
{
    Math::Vec4f colTapeBg{ 0.05f, 0.06f, 0.07f, 0.88f };
    Math::Vec4f colBorder{ 0.35f, 0.38f, 0.42f, 1.0f };
    Math::Vec4f colWhite{ 0.95f, 0.95f, 0.95f, 1.0f };
    Math::Vec4f colGreen{ 0.1f, 0.85f, 0.2f, 1.0f };
    Math::Vec4f colCyan{ 0.0f, 0.9f, 1.0f, 1.0f };

    // Tape background & border
    AddQuad({ x, y, 0.0f }, { x + w, y, 0.0f }, { x + w, y + h, 0.0f }, { x, y + h, 0.0f }, colTapeBg);
    AddLine(x, y, x, y + h, 1.5f, colBorder);

    f32 cy = y + h * 0.5f;
    f32 pxPerFoot = 0.22f;

    auto AltToY = [&](f32 alt) -> f32 {
        return cy - (alt - altFeet) * pxPerFoot;
    };

    // Selected Target Altitude Bug on tape
    f32 targetY = AltToY(targetAltFeet);
    if (targetY >= y && targetY <= y + h) {
        AddTriangle(
            { x + 2.0f, targetY, 0.0f },
            { x + 10.0f, targetY - 6.0f, 0.0f },
            { x + 10.0f, targetY + 6.0f, 0.0f },
            colCyan);
    }

    // Target altitude box above tape
    std::stringstream tgtSs;
    tgtSs << static_cast<int>(targetAltFeet);
    AddQuad({ x, y - 18.0f, 0.0f }, { x + w, y - 18.0f, 0.0f }, { x + w, y - 2.0f, 0.0f }, { x, y - 2.0f, 0.0f }, { 0.0f, 0.0f, 0.0f, 0.9f });
    DrawString(tgtSs.str(), x + 14.0f, y - 16.0f, 12.0f, colCyan, 1.8f);

    // Moving tick marks every 100 ft, numbers every 500 ft
    int minAlt = static_cast<int>(altFeet - (h * 0.5f) / pxPerFoot) / 100 * 100;
    int maxAlt = static_cast<int>(altFeet + (h * 0.5f) / pxPerFoot) / 100 * 100 + 100;

    for (int a = minAlt; a <= maxAlt; a += 100) {
        if (a < 0) continue;
        f32 ty = AltToY(static_cast<f32>(a));
        if (ty < y || ty > y + h) continue;

        f32 tickLen = (a % 500 == 0) ? 14.0f : 8.0f;
        AddLine(x + 2.0f, ty, x + 2.0f + tickLen, ty, 1.5f, colWhite);

        if (a % 200 == 0) {
            std::string aStr = std::to_string(a);
            DrawString(aStr, x + 20.0f, ty - 5.0f, 10.0f, colWhite);
        }
    }

    // Digital rolling readout drum box
    f32 boxH = 28.0f;
    f32 boxW = 62.0f;
    f32 boxX = x + 2.0f;
    f32 boxY = cy - boxH * 0.5f;

    AddQuad(
        { boxX, boxY, 0.0f },
        { boxX + boxW, boxY, 0.0f },
        { boxX + boxW, boxY + boxH, 0.0f },
        { boxX, boxY + boxH, 0.0f },
        { 0.0f, 0.0f, 0.0f, 0.98f });

    AddLine(boxX, boxY, boxX + boxW, boxY, 1.5f, colGreen);
    AddLine(boxX + boxW, boxY, boxX + boxW, boxY + boxH, 1.5f, colGreen);
    AddLine(boxX + boxW, boxY + boxH, boxX, boxY + boxH, 1.5f, colGreen);
    AddLine(boxX, boxY + boxH, boxX, boxY, 1.5f, colGreen);

    // Pointer notch pointing left
    AddTriangle(
        { boxX - 4.0f, cy, 0.0f },
        { boxX, cy - 5.0f, 0.0f },
        { boxX, cy + 5.0f, 0.0f },
        colGreen);

    std::string altStr = std::to_string(static_cast<int>(altFeet));
    DrawString(altStr, boxX + 6.0f, boxY + 7.0f, 14.0f, colWhite, 2.0f);

    // Barometric setting below tape
    std::stringstream baroSs;
    baroSs << std::fixed << std::setprecision(2) << kollsmanInHg << "IN";
    AddQuad({ x, y + h + 4.0f, 0.0f }, { x + w, y + h + 4.0f, 0.0f }, { x + w, y + h + 18.0f, 0.0f }, { x, y + h + 18.0f, 0.0f }, { 0.0f, 0.0f, 0.0f, 0.85f });
    DrawString(baroSs.str(), x + 6.0f, y + h + 5.0f, 10.0f, colCyan);
}

void GlassCockpitRenderer::DrawVerticalSpeedTape(
    f32 x, f32 y, f32 w, f32 h,
    f32 vsFpm, f32 targetVsFpm, bool vsModeActive)
{
    Math::Vec4f colTapeBg{ 0.04f, 0.05f, 0.06f, 0.85f };
    Math::Vec4f colBorder{ 0.30f, 0.34f, 0.38f, 1.0f };
    Math::Vec4f colWhite{ 0.95f, 0.95f, 0.95f, 1.0f };
    Math::Vec4f colCyan{ 0.0f, 0.9f, 1.0f, 1.0f };

    AddQuad({ x, y, 0.0f }, { x + w, y, 0.0f }, { x + w, y + h, 0.0f }, { x, y + h, 0.0f }, colTapeBg);
    AddLine(x, y, x, y + h, 1.0f, colBorder);

    f32 cy = y + h * 0.5f;
    f32 pxPerFpm = (h * 0.5f) / 2000.0f;

    // Linear scale marks: +/- 2000, 1500, 1000, 500, 0
    int vsTicks[] = { -2000, -1500, -1000, -500, 0, 500, 1000, 1500, 2000 };
    for (int v : vsTicks) {
        f32 ty = cy - static_cast<f32>(v) * pxPerFpm;
        f32 tickLen = (std::abs(v) % 1000 == 0) ? 8.0f : 4.0f;
        AddLine(x + 2.0f, ty, x + 2.0f + tickLen, ty, 1.0f, colWhite);

        if (std::abs(v) == 1000 || std::abs(v) == 2000) {
            std::string s = std::to_string(std::abs(v) / 1000);
            DrawString(s, x + 12.0f, ty - 4.0f, 8.0f, colWhite);
        }
    }

    // Pointer needle
    f32 clampedVs = std::clamp(vsFpm, -2200.0f, 2200.0f);
    f32 needleY = cy - clampedVs * pxPerFpm;
    AddTriangle(
        { x + 2.0f, needleY, 0.0f },
        { x + w - 4.0f, needleY - 4.0f, 0.0f },
        { x + w - 4.0f, needleY + 4.0f, 0.0f },
        colWhite);

    // Target VS bug if active
    if (vsModeActive) {
        f32 tgtY = cy - std::clamp(targetVsFpm, -2000.0f, 2000.0f) * pxPerFpm;
        AddTriangle(
            { x + 1.0f, tgtY, 0.0f },
            { x + 7.0f, tgtY - 4.0f, 0.0f },
            { x + 7.0f, tgtY + 4.0f, 0.0f },
            colCyan);
    }
}

void GlassCockpitRenderer::DrawHsiCompass(
    f32 cx, f32 cy, f32 radius,
    f32 headingDeg, f32 targetHeadingDeg,
    const Navigation::NavReceiverState& nav1)
{
    Math::Vec4f colBg{ 0.03f, 0.04f, 0.05f, 0.92f };
    Math::Vec4f colRing{ 0.35f, 0.38f, 0.42f, 1.0f };
    Math::Vec4f colWhite{ 0.95f, 0.95f, 0.95f, 1.0f };
    Math::Vec4f colCyan{ 0.0f, 0.9f, 1.0f, 1.0f };
    Math::Vec4f colMagenta{ 1.0f, 0.1f, 0.95f, 1.0f };

    AddCircleFill(cx, cy, radius, 32, colBg);
    AddRing(cx, cy, radius - 2.0f, radius, 32, colRing);

    // Compass card ticks every 5 and 10 degrees
    for (int deg = 0; deg < 360; deg += 5) {
        f32 relDeg = static_cast<f32>(deg) - headingDeg;
        f32 angleRad = (90.0f - relDeg) * DEG_TO_RAD;

        f32 cosA = std::cos(angleRad);
        f32 sinA = std::sin(angleRad);

        f32 tickLen = (deg % 30 == 0) ? 9.0f : ((deg % 10 == 0) ? 6.0f : 3.0f);
        f32 x0 = cx + (radius - tickLen) * cosA;
        f32 y0 = cy - (radius - tickLen) * sinA;
        f32 x1 = cx + (radius - 1.0f) * cosA;
        f32 y1 = cy - (radius - 1.0f) * sinA;
        AddLine(x0, y0, x1, y1, 1.5f, colWhite);

        // Cardinal & numeric labels every 30 degrees
        if (deg % 30 == 0) {
            std::string lbl;
            switch (deg) {
            case 0:   lbl = "N"; break;
            case 90:  lbl = "E"; break;
            case 180: lbl = "S"; break;
            case 270: lbl = "W"; break;
            default:  lbl = std::to_string(deg / 10); break;
            }
            f32 lx = cx + (radius - 18.0f) * cosA - 4.0f;
            f32 ly = cy - (radius - 18.0f) * sinA - 4.0f;
            Math::Vec4f colLbl = (deg == 0) ? colCyan : colWhite;
            DrawString(lbl, lx, ly, 9.0f, colLbl);
        }
    }

    // Top Lubber line
    AddLine(cx, cy - radius - 6.0f, cx, cy - radius + 4.0f, 2.5f, colWhite);

    // Digital Heading Readout Box at top of HSI
    std::stringstream hdgSs;
    int curHdgInt = static_cast<int>(headingDeg);
    if (curHdgInt == 0) curHdgInt = 360;
    hdgSs << std::setfill('0') << std::setw(3) << curHdgInt << "\u00B0";
    AddQuad({ cx - 22.0f, cy - radius - 20.0f, 0.0f }, { cx + 22.0f, cy - radius - 20.0f, 0.0f }, { cx + 22.0f, cy - radius - 4.0f, 0.0f }, { cx - 22.0f, cy - radius - 4.0f, 0.0f }, { 0.0f, 0.0f, 0.0f, 0.95f });
    DrawString(hdgSs.str(), cx - 18.0f, cy - radius - 18.0f, 12.0f, colWhite, 1.8f);

    // Cyan Heading Bug cursor
    f32 bugRelDeg = static_cast<f32>(targetHeadingDeg) - headingDeg;
    f32 bugAngleRad = (90.0f - bugRelDeg) * DEG_TO_RAD;
    f32 bx = cx + (radius - 2.0f) * std::cos(bugAngleRad);
    f32 by = cy - (radius - 2.0f) * std::sin(bugAngleRad);
    f32 bcos = std::cos(bugAngleRad);
    f32 bsin = std::sin(bugAngleRad);
    AddTriangle(
        { bx, by, 0.0f },
        { bx - 8.0f * bcos + 6.0f * bsin, by + 8.0f * bsin + 6.0f * bcos, 0.0f },
        { bx - 8.0f * bcos - 6.0f * bsin, by + 8.0f * bsin - 6.0f * bcos, 0.0f },
        colCyan);

    // Magenta Course Pointer & Lateral CDI Deviation (NAV1)
    if (!nav1.cdi.navFlag) {
        f32 obsRelDeg = static_cast<f32>(nav1.cdi.obsDeg) - headingDeg;
        f32 obsAngleRad = (90.0f - obsRelDeg) * DEG_TO_RAD;
        f32 cosObs = std::cos(obsAngleRad);
        f32 sinObs = std::sin(obsAngleRad);

        // Course arrowhead
        f32 tipX = cx + (radius - 12.0f) * cosObs;
        f32 tipY = cy - (radius - 12.0f) * sinObs;
        AddTriangle(
            { tipX, tipY, 0.0f },
            { tipX - 12.0f * cosObs + 6.0f * sinObs, tipY + 12.0f * sinObs + 6.0f * cosObs, 0.0f },
            { tipX - 12.0f * cosObs - 6.0f * sinObs, tipY + 12.0f * sinObs - 6.0f * cosObs, 0.0f },
            colMagenta);

        // CDI Deviation Bar
        f32 cdiOffset = std::clamp(static_cast<f32>(nav1.cdi.courseDeviationDots / 5.0) * 28.0f, -32.0f, 32.0f);
        f32 cdiX0 = cx + cdiOffset * sinObs - 20.0f * cosObs;
        f32 cdiY0 = cy + cdiOffset * cosObs + 20.0f * sinObs;
        f32 cdiX1 = cx + cdiOffset * sinObs + 20.0f * cosObs;
        f32 cdiY1 = cy + cdiOffset * cosObs - 20.0f * sinObs;
        AddLine(cdiX0, cdiY0, cdiX1, cdiY1, 2.5f, colMagenta);
    }
}

// Vector primitive implementations
void GlassCockpitRenderer::AddQuad(
    const Math::Vec3f& p0, const Math::Vec3f& p1,
    const Math::Vec3f& p2, const Math::Vec3f& p3,
    const Math::Vec4f& col)
{
    u32 baseIdx = static_cast<u32>(m_vertices.size());
    Math::Vec3f norm{ 0.0f, 0.0f, 1.0f };

    m_vertices.push_back({ p0, norm, col });
    m_vertices.push_back({ p1, norm, col });
    m_vertices.push_back({ p2, norm, col });
    m_vertices.push_back({ p3, norm, col });

    m_indices.push_back(baseIdx + 0);
    m_indices.push_back(baseIdx + 1);
    m_indices.push_back(baseIdx + 2);

    m_indices.push_back(baseIdx + 0);
    m_indices.push_back(baseIdx + 2);
    m_indices.push_back(baseIdx + 3);
}

void GlassCockpitRenderer::AddTriangle(
    const Math::Vec3f& p0, const Math::Vec3f& p1,
    const Math::Vec3f& p2, const Math::Vec4f& col)
{
    u32 baseIdx = static_cast<u32>(m_vertices.size());
    Math::Vec3f norm{ 0.0f, 0.0f, 1.0f };

    m_vertices.push_back({ p0, norm, col });
    m_vertices.push_back({ p1, norm, col });
    m_vertices.push_back({ p2, norm, col });

    m_indices.push_back(baseIdx + 0);
    m_indices.push_back(baseIdx + 1);
    m_indices.push_back(baseIdx + 2);
}

void GlassCockpitRenderer::AddCircleFill(f32 cx, f32 cy, f32 radius, int segments, const Math::Vec4f& col) {
    u32 centerIdx = static_cast<u32>(m_vertices.size());
    Math::Vec3f norm{ 0.0f, 0.0f, 1.0f };
    m_vertices.push_back({ { cx, cy, 0.0f }, norm, col });

    f32 step = (2.0f * PI) / static_cast<f32>(segments);
    for (int i = 0; i <= segments; ++i) {
        f32 angle = static_cast<f32>(i) * step;
        m_vertices.push_back({ { cx + radius * std::cos(angle), cy + radius * std::sin(angle), 0.0f }, norm, col });
    }

    for (int i = 0; i < segments; ++i) {
        m_indices.push_back(centerIdx);
        m_indices.push_back(centerIdx + 1 + i);
        m_indices.push_back(centerIdx + 2 + i);
    }
}

void GlassCockpitRenderer::AddRing(f32 cx, f32 cy, f32 innerR, f32 outerR, int segments, const Math::Vec4f& col) {
    AddArcRing(cx, cy, innerR, outerR, 0.0f, 360.0f, segments, col);
}

void GlassCockpitRenderer::AddArcRing(
    f32 cx, f32 cy, f32 innerR, f32 outerR,
    f32 startDeg, f32 endDeg, int segments, const Math::Vec4f& col)
{
    f32 startRad = startDeg * DEG_TO_RAD;
    f32 endRad = endDeg * DEG_TO_RAD;
    f32 step = (endRad - startRad) / static_cast<f32>(segments);

    Math::Vec3f norm{ 0.0f, 0.0f, 1.0f };
    u32 baseIdx = static_cast<u32>(m_vertices.size());

    for (int i = 0; i <= segments; ++i) {
        f32 angle = startRad + static_cast<f32>(i) * step;
        f32 c = std::cos(angle);
        f32 s = std::sin(angle);
        m_vertices.push_back({ { cx + innerR * c, cy + innerR * s, 0.0f }, norm, col });
        m_vertices.push_back({ { cx + outerR * c, cy + outerR * s, 0.0f }, norm, col });
    }

    for (int i = 0; i < segments; ++i) {
        u32 i0 = baseIdx + i * 2;
        u32 o0 = i0 + 1;
        u32 i1 = i0 + 2;
        u32 o1 = i0 + 3;

        m_indices.push_back(i0);
        m_indices.push_back(o0);
        m_indices.push_back(o1);

        m_indices.push_back(i0);
        m_indices.push_back(o1);
        m_indices.push_back(i1);
    }
}

void GlassCockpitRenderer::AddLine(f32 x0, f32 y0, f32 x1, f32 y1, f32 thickness, const Math::Vec4f& col) {
    f32 dx = x1 - x0;
    f32 dy = y1 - y0;
    f32 len = std::sqrt(dx * dx + dy * dy);
    if (len < 1e-4f) return;

    f32 nx = -dy / len * (thickness * 0.5f);
    f32 ny = dx / len * (thickness * 0.5f);

    AddQuad(
        { x0 + nx, y0 + ny, 0.0f },
        { x1 + nx, y1 + ny, 0.0f },
        { x1 - nx, y1 - ny, 0.0f },
        { x0 - nx, y0 - ny, 0.0f },
        col);
}

void GlassCockpitRenderer::AddTick(f32 cx, f32 cy, f32 radius, f32 length, f32 angleDeg, f32 thickness, const Math::Vec4f& col) {
    f32 rad = angleDeg * DEG_TO_RAD;
    f32 c = std::cos(rad);
    f32 s = std::sin(rad);

    f32 x0 = cx + (radius - length) * c;
    f32 y0 = cy + (radius - length) * s;
    f32 x1 = cx + radius * c;
    f32 y1 = cy + radius * s;

    AddLine(x0, y0, x1, y1, thickness, col);
}

// Procedural vector stroke font for 0-9, A-Z, symbols
void GlassCockpitRenderer::DrawChar(char c, f32 x, f32 y, f32 size, const Math::Vec4f& col, f32 thickness) {
    f32 w = size * 0.65f;
    f32 h = size;

    auto L = [&](f32 x0, f32 y0, f32 x1, f32 y1) {
        AddLine(x + x0 * w, y + y0 * h, x + x1 * w, y + y1 * h, thickness, col);
    };

    switch (c) {
    case '0': L(0,0, 1,0); L(1,0, 1,1); L(1,1, 0,1); L(0,1, 0,0); L(0,1, 1,0); break;
    case '1': L(0.5f,0, 0.5f,1); L(0.2f,0.2f, 0.5f,0); L(0.2f,1, 0.8f,1); break;
    case '2': L(0,0, 1,0); L(1,0, 1,0.5f); L(1,0.5f, 0,0.5f); L(0,0.5f, 0,1); L(0,1, 1,1); break;
    case '3': L(0,0, 1,0); L(1,0, 1,1); L(0,0.5f, 1,0.5f); L(0,1, 1,1); break;
    case '4': L(0,0, 0,0.5f); L(0,0.5f, 1,0.5f); L(1,0, 1,1); break;
    case '5': L(1,0, 0,0); L(0,0, 0,0.5f); L(0,0.5f, 1,0.5f); L(1,0.5f, 1,1); L(1,1, 0,1); break;
    case '6': L(1,0, 0,0); L(0,0, 0,1); L(0,1, 1,1); L(1,1, 1,0.5f); L(1,0.5f, 0,0.5f); break;
    case '7': L(0,0, 1,0); L(1,0, 0.3f,1); break;
    case '8': L(0,0, 1,0); L(1,0, 1,1); L(1,1, 0,1); L(0,1, 0,0); L(0,0.5f, 1,0.5f); break;
    case '9': L(1,0.5f, 0,0.5f); L(0,0.5f, 0,0); L(0,0, 1,0); L(1,0, 1,1); L(1,1, 0,1); break;
    case 'A': L(0,1, 0.5f,0); L(0.5f,0, 1,1); L(0.25f,0.55f, 0.75f,0.55f); break;
    case 'B': L(0,0, 0.7f,0); L(0.7f,0, 0.9f,0.25f); L(0.9f,0.25f, 0.7f,0.5f); L(0.7f,0.5f, 0,0.5f); L(0.7f,0.5f, 0.9f,0.75f); L(0.9f,0.75f, 0.7f,1); L(0.7f,1, 0,1); L(0,0, 0,1); break;
    case 'C': L(1,0, 0,0); L(0,0, 0,1); L(0,1, 1,1); break;
    case 'D': L(0,0, 0.7f,0); L(0.7f,0, 1,0.3f); L(1,0.3f, 1,0.7f); L(1,0.7f, 0.7f,1); L(0.7f,1, 0,1); L(0,0, 0,1); break;
    case 'E': L(1,0, 0,0); L(0,0, 0,1); L(0,1, 1,1); L(0,0.5f, 0.8f,0.5f); break;
    case 'F': L(1,0, 0,0); L(0,0, 0,1); L(0,0.5f, 0.8f,0.5f); break;
    case 'G': L(1,0, 0,0); L(0,0, 0,1); L(0,1, 1,1); L(1,1, 1,0.5f); L(1,0.5f, 0.5f,0.5f); break;
    case 'H': L(0,0, 0,1); L(1,0, 1,1); L(0,0.5f, 1,0.5f); break;
    case 'I': L(0.2f,0, 0.8f,0); L(0.5f,0, 0.5f,1); L(0.2f,1, 0.8f,1); break;
    case 'K': L(0,0, 0,1); L(1,0, 0,0.5f); L(0,0.5f, 1,1); break;
    case 'L': L(0,0, 0,1); L(0,1, 1,1); break;
    case 'M': L(0,1, 0,0); L(0,0, 0.5f,0.5f); L(0.5f,0.5f, 1,0); L(1,0, 1,1); break;
    case 'N': L(0,1, 0,0); L(0,0, 1,1); L(1,1, 1,0); break;
    case 'O': L(0,0, 1,0); L(1,0, 1,1); L(1,1, 0,1); L(0,1, 0,0); break;
    case 'P': L(0,1, 0,0); L(0,0, 1,0); L(1,0, 1,0.5f); L(1,0.5f, 0,0.5f); break;
    case 'R': L(0,1, 0,0); L(0,0, 1,0); L(1,0, 1,0.5f); L(1,0.5f, 0,0.5f); L(0,0.5f, 1,1); break;
    case 'S': L(1,0, 0,0); L(0,0, 0,0.5f); L(0,0.5f, 1,0.5f); L(1,0.5f, 1,1); L(1,1, 0,1); break;
    case 'T': L(0,0, 1,0); L(0.5f,0, 0.5f,1); break;
    case 'U': L(0,0, 0,1); L(0,1, 1,1); L(1,1, 1,0); break;
    case 'V': L(0,0, 0.5f,1); L(0.5f,1, 1,0); break;
    case 'W': L(0,0, 0.25f,1); L(0.25f,1, 0.5f,0.5f); L(0.5f,0.5f, 0.75f,1); L(0.75f,1, 1,0); break;
    case 'X': L(0,0, 1,1); L(1,0, 0,1); break;
    case 'Y': L(0,0, 0.5f,0.5f); L(1,0, 0.5f,0.5f); L(0.5f,0.5f, 0.5f,1); break;
    case '-': L(0.1f,0.5f, 0.9f,0.5f); break;
    case '+': L(0.1f,0.5f, 0.9f,0.5f); L(0.5f,0.1f, 0.5f,0.9f); break;
    case '.': L(0.4f,0.9f, 0.6f,0.9f); break;
    case ':': L(0.5f,0.25f, 0.5f,0.35f); L(0.5f,0.65f, 0.5f,0.75f); break;
    case '[': L(0.7f,0, 0.3f,0); L(0.3f,0, 0.3f,1); L(0.3f,1, 0.7f,1); break;
    case ']': L(0.3f,0, 0.7f,0); L(0.7f,0, 0.7f,1); L(0.7f,1, 0.3f,1); break;
    case '!': L(0.5f,0, 0.5f,0.7f); L(0.5f,0.85f, 0.5f,1.0f); break;
    default: break;
    }
}

void GlassCockpitRenderer::DrawString(const std::string& str, f32 x, f32 y, f32 size, const Math::Vec4f& col, f32 thickness) {
    f32 curX = x;
    f32 charW = size * 0.65f;
    f32 spacing = size * 0.25f;

    for (char c : str) {
        if (c == ' ') {
            curX += charW * 0.8f;
        } else {
            DrawChar(c, curX, y, size, col, thickness);
            curX += charW + spacing;
        }
    }
}

} // namespace LokYS::Graphics
