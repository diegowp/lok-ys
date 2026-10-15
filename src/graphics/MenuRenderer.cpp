#include "LokYS/graphics/MenuRenderer.hpp"
#include <d3dcompiler.h>
#include <cmath>
#include <algorithm>
#include <iomanip>
#include <sstream>

namespace LokYS::Graphics {

namespace {

const char* g_menuShaderSource = R"(
cbuffer MenuConstants : register(b0) {
    float2 u_screenSize;
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
    float xNdc = (input.position.x / u_screenSize.x) * 2.0f - 1.0f;
    float yNdc = 1.0f - (input.position.y / u_screenSize.y) * 2.0f;
    output.position = float4(xNdc, yNdc, 0.0f, 1.0f);
    output.color = input.color;
    return output;
}

float4 PSMain(PSInput input) : SV_TARGET {
    return input.color;
}
)";

} // namespace

bool MenuRenderer::Initialize(ID3D12Device* device) {
    if (!device) return false;

    if (!CreateRootSignature(device)) return false;
    if (!CreatePipelineState(device)) return false;
    if (!CreateBuffers(device)) return false;

    m_vertices.reserve(MaxVertices);
    m_indices.reserve(MaxIndices);
    m_initialized = true;
    return true;
}

void MenuRenderer::Shutdown() {
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

bool MenuRenderer::CreateRootSignature(ID3D12Device* device) {
    D3D12_ROOT_PARAMETER rootParam{};
    rootParam.ParameterType = D3D12_ROOT_PARAMETER_TYPE_32BIT_CONSTANTS;
    rootParam.Constants.ShaderRegister = 0;
    rootParam.Constants.RegisterSpace = 0;
    rootParam.Constants.Num32BitValues = 4;
    rootParam.ShaderVisibility = D3D12_SHADER_VISIBILITY_VERTEX;

    D3D12_ROOT_SIGNATURE_DESC rootDesc{};
    rootDesc.NumParameters = 1;
    rootDesc.pParameters = &rootParam;
    rootDesc.Flags = D3D12_ROOT_SIGNATURE_FLAG_ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT;

    Microsoft::WRL::ComPtr<ID3DBlob> sigBlob, errBlob;
    HRESULT hr = D3D12SerializeRootSignature(&rootDesc, D3D_ROOT_SIGNATURE_VERSION_1, &sigBlob, &errBlob);
    if (FAILED(hr)) return false;

    hr = device->CreateRootSignature(0, sigBlob->GetBufferPointer(), sigBlob->GetBufferSize(), IID_PPV_ARGS(&m_rootSignature));
    return SUCCEEDED(hr);
}

bool MenuRenderer::CreatePipelineState(ID3D12Device* device) {
    Microsoft::WRL::ComPtr<ID3DBlob> vsBlob, psBlob, errBlob;

    HRESULT hr = D3DCompile(
        g_menuShaderSource, strlen(g_menuShaderSource),
        "MenuVS", nullptr, nullptr,
        "VSMain", "vs_5_0",
        D3DCOMPILE_OPTIMIZATION_LEVEL3, 0,
        &vsBlob, &errBlob);
    if (FAILED(hr)) return false;

    hr = D3DCompile(
        g_menuShaderSource, strlen(g_menuShaderSource),
        "MenuPS", nullptr, nullptr,
        "PSMain", "ps_5_0",
        D3DCOMPILE_OPTIMIZATION_LEVEL3, 0,
        &psBlob, &errBlob);
    if (FAILED(hr)) return false;

    D3D12_INPUT_ELEMENT_DESC inputLayout[] = {
        { "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT,    0, offsetof(Vertex3D, position), D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 },
        { "NORMAL",   0, DXGI_FORMAT_R32G32B32_FLOAT,    0, offsetof(Vertex3D, normal),   D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 },
        { "COLOR",    0, DXGI_FORMAT_R32G32B32A32_FLOAT, 0, offsetof(Vertex3D, color),    D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 },
    };

    D3D12_GRAPHICS_PIPELINE_STATE_DESC psoDesc{};
    psoDesc.pRootSignature = m_rootSignature.Get();
    psoDesc.VS = { vsBlob->GetBufferPointer(), vsBlob->GetBufferSize() };
    psoDesc.PS = { psBlob->GetBufferPointer(), psBlob->GetBufferSize() };
    psoDesc.InputLayout = { inputLayout, _countof(inputLayout) };
    psoDesc.RasterizerState.FillMode = D3D12_FILL_MODE_SOLID;
    psoDesc.RasterizerState.CullMode = D3D12_CULL_MODE_NONE;

    // Alpha blending
    psoDesc.BlendState.RenderTarget[0].BlendEnable = TRUE;
    psoDesc.BlendState.RenderTarget[0].SrcBlend = D3D12_BLEND_SRC_ALPHA;
    psoDesc.BlendState.RenderTarget[0].DestBlend = D3D12_BLEND_INV_SRC_ALPHA;
    psoDesc.BlendState.RenderTarget[0].BlendOp = D3D12_BLEND_OP_ADD;
    psoDesc.BlendState.RenderTarget[0].SrcBlendAlpha = D3D12_BLEND_ONE;
    psoDesc.BlendState.RenderTarget[0].DestBlendAlpha = D3D12_BLEND_INV_SRC_ALPHA;
    psoDesc.BlendState.RenderTarget[0].BlendOpAlpha = D3D12_BLEND_OP_ADD;
    psoDesc.BlendState.RenderTarget[0].RenderTargetWriteMask = D3D12_COLOR_WRITE_ENABLE_ALL;

    psoDesc.DepthStencilState.DepthEnable = FALSE;
    psoDesc.SampleMask = UINT_MAX;
    psoDesc.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
    psoDesc.NumRenderTargets = 1;
    psoDesc.RTVFormats[0] = DXGI_FORMAT_R8G8B8A8_UNORM;
    psoDesc.SampleDesc.Count = 1;

    hr = device->CreateGraphicsPipelineState(&psoDesc, IID_PPV_ARGS(&m_pipelineState));
    return SUCCEEDED(hr);
}

bool MenuRenderer::CreateBuffers(ID3D12Device* device) {
    D3D12_HEAP_PROPERTIES heapProps{};
    heapProps.Type = D3D12_HEAP_TYPE_UPLOAD;

    D3D12_RESOURCE_DESC vbDesc{};
    vbDesc.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER;
    vbDesc.Width = MaxVertices * sizeof(Vertex3D);
    vbDesc.Height = 1;
    vbDesc.DepthOrArraySize = 1;
    vbDesc.MipLevels = 1;
    vbDesc.Format = DXGI_FORMAT_UNKNOWN;
    vbDesc.SampleDesc.Count = 1;
    vbDesc.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;

    HRESULT hr = device->CreateCommittedResource(
        &heapProps, D3D12_HEAP_FLAG_NONE,
        &vbDesc, D3D12_RESOURCE_STATE_GENERIC_READ,
        nullptr, IID_PPV_ARGS(&m_vertexBuffer));
    if (FAILED(hr)) return false;

    D3D12_RESOURCE_DESC ibDesc = vbDesc;
    ibDesc.Width = MaxIndices * sizeof(u32);

    hr = device->CreateCommittedResource(
        &heapProps, D3D12_HEAP_FLAG_NONE,
        &ibDesc, D3D12_RESOURCE_STATE_GENERIC_READ,
        nullptr, IID_PPV_ARGS(&m_indexBuffer));
    if (FAILED(hr)) return false;

    D3D12_RANGE readRange{ 0, 0 };
    hr = m_vertexBuffer->Map(0, &readRange, reinterpret_cast<void**>(&m_mappedVertices));
    if (FAILED(hr)) return false;

    hr = m_indexBuffer->Map(0, &readRange, reinterpret_cast<void**>(&m_mappedIndices));
    if (FAILED(hr)) return false;

    m_vbView.BufferLocation = m_vertexBuffer->GetGPUVirtualAddress();
    m_vbView.SizeInBytes = static_cast<UINT>(MaxVertices * sizeof(Vertex3D));
    m_vbView.StrideInBytes = sizeof(Vertex3D);

    m_ibView.BufferLocation = m_indexBuffer->GetGPUVirtualAddress();
    m_ibView.SizeInBytes = static_cast<UINT>(MaxIndices * sizeof(u32));
    m_ibView.Format = DXGI_FORMAT_R32_UINT;

    return true;
}

void MenuRenderer::UpdateMouse(f32 mouseX, f32 mouseY, bool leftDown, bool leftClicked) noexcept {
    m_mouseX = mouseX;
    m_mouseY = mouseY;
    m_leftDown = leftDown;
    m_leftClicked = leftClicked;
}

void MenuRenderer::OnKeyDown([[maybe_unused]] u32 vkey, [[maybe_unused]] bool shift, [[maybe_unused]] bool ctrl, [[maybe_unused]] bool alt) noexcept {
    if (m_listeningAction.has_value()) {
        // Listening for rebind
        if (vkey == VK_ESCAPE) {
            m_listeningAction.reset();
            return;
        }
        // Save rebind
        // (Action map will be updated by caller via GetListeningAction())
        return;
    }

    if (vkey == VK_RETURN) {
        m_startFlightTriggered = true;
        m_resumeFlightTriggered = true;
    } else if (vkey == VK_ESCAPE) {
        m_resumeFlightTriggered = true;
        m_exitToMenuTriggered = true;
    } else if (vkey == 'R') {
        m_restartFlightTriggered = true;
        m_toggleSpawnModeTriggered = true;
    } else if (vkey == 'O') {
        m_settingsTriggered = true;
    } else if (vkey == 'M' || vkey == VK_BACK) {
        m_exitToMenuTriggered = true;
    } else if (vkey == 'V') {
        m_cycleAircraftTriggered = true;
    } else if (vkey == VK_TAB) {
        m_cycleAirportTriggered = true;
    } else if (vkey == 'W') {
        m_cycleWeatherTriggered = true;
    }
}

void MenuRenderer::AddQuad(
    const Math::Vec3f& p0, const Math::Vec3f& p1,
    const Math::Vec3f& p2, const Math::Vec3f& p3,
    const Math::Vec4f& col)
{
    if (m_vertices.size() + 4 > MaxVertices || m_indices.size() + 6 > MaxIndices) return;

    u32 baseIdx = static_cast<u32>(m_vertices.size());
    Math::Vec3f norm{ 0.0f, 0.0f, -1.0f };

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

void MenuRenderer::AddLine(f32 x0, f32 y0, f32 x1, f32 y1, f32 thickness, const Math::Vec4f& col) {
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

void MenuRenderer::DrawChar(char c, f32 x, f32 y, f32 size, const Math::Vec4f& col, f32 thickness) {
    f32 w = size * 0.60f;
    f32 h = size;

    auto line = [&](f32 lx0, f32 ly0, f32 lx1, f32 ly1) {
        AddLine(x + lx0 * w, y + ly0 * h, x + lx1 * w, y + ly1 * h, thickness, col);
    };

    switch (c) {
    case 'A': line(0,1, 0.5f,0); line(0.5f,0, 1,1); line(0.2f,0.6f, 0.8f,0.6f); break;
    case 'B': line(0,0, 0,1); line(0,0, 0.7f,0); line(0.7f,0, 0.7f,0.5f); line(0.7f,0.5f, 0,0.5f); line(0,0.5f, 0.8f,0.5f); line(0.8f,0.5f, 0.8f,1); line(0.8f,1, 0,1); break;
    case 'C': line(1,0, 0,0); line(0,0, 0,1); line(0,1, 1,1); break;
    case 'D': line(0,0, 0,1); line(0,0, 0.6f,0); line(0.6f,0, 1,0.5f); line(1,0.5f, 0.6f,1); line(0.6f,1, 0,1); break;
    case 'E': line(1,0, 0,0); line(0,0, 0,1); line(0,1, 1,1); line(0,0.5f, 0.7f,0.5f); break;
    case 'F': line(1,0, 0,0); line(0,0, 0,1); line(0,0.5f, 0.7f,0.5f); break;
    case 'G': line(1,0, 0,0); line(0,0, 0,1); line(0,1, 1,1); line(1,1, 1,0.5f); line(1,0.5f, 0.5f,0.5f); break;
    case 'H': line(0,0, 0,1); line(1,0, 1,1); line(0,0.5f, 1,0.5f); break;
    case 'I': line(0.2f,0, 0.8f,0); line(0.5f,0, 0.5f,1); line(0.2f,1, 0.8f,1); break;
    case 'J': line(0.8f,0, 0.8f,0.8f); line(0.8f,0.8f, 0.5f,1); line(0.5f,1, 0.2f,0.8f); break;
    case 'K': line(0,0, 0,1); line(1,0, 0,0.5f); line(0,0.5f, 1,1); break;
    case 'L': line(0,0, 0,1); line(0,1, 1,1); break;
    case 'M': line(0,1, 0,0); line(0,0, 0.5f,0.6f); line(0.5f,0.6f, 1,0); line(1,0, 1,1); break;
    case 'N': line(0,1, 0,0); line(0,0, 1,1); line(1,1, 1,0); break;
    case 'O': line(0,0, 1,0); line(1,0, 1,1); line(1,1, 0,1); line(0,1, 0,0); break;
    case 'P': line(0,1, 0,0); line(0,0, 1,0); line(1,0, 1,0.5f); line(1,0.5f, 0,0.5f); break;
    case 'Q': line(0,0, 1,0); line(1,0, 1,1); line(1,1, 0,1); line(0,1, 0,0); line(0.6f,0.6f, 1,1); break;
    case 'R': line(0,1, 0,0); line(0,0, 1,0); line(1,0, 1,0.5f); line(1,0.5f, 0,0.5f); line(0,0.5f, 1,1); break;
    case 'S': line(1,0, 0,0); line(0,0, 0,0.5f); line(0,0.5f, 1,0.5f); line(1,0.5f, 1,1); line(1,1, 0,1); break;
    case 'T': line(0,0, 1,0); line(0.5f,0, 0.5f,1); break;
    case 'U': line(0,0, 0,1); line(0,1, 1,1); line(1,1, 1,0); break;
    case 'V': line(0,0, 0.5f,1); line(0.5f,1, 1,0); break;
    case 'W': line(0,0, 0.2f,1); line(0.2f,1, 0.5f,0.4f); line(0.5f,0.4f, 0.8f,1); line(0.8f,1, 1,0); break;
    case 'X': line(0,0, 1,1); line(1,0, 0,1); break;
    case 'Y': line(0,0, 0.5f,0.5f); line(1,0, 0.5f,0.5f); line(0.5f,0.5f, 0.5f,1); break;
    case 'Z': line(0,0, 1,0); line(1,0, 0,1); line(0,1, 1,1); break;
    case '0': line(0,0, 1,0); line(1,0, 1,1); line(1,1, 0,1); line(0,1, 0,0); line(1,0, 0,1); break;
    case '1': line(0.2f,0.2f, 0.5f,0); line(0.5f,0, 0.5f,1); line(0.2f,1, 0.8f,1); break;
    case '2': line(0,0, 1,0); line(1,0, 1,0.5f); line(1,0.5f, 0,0.5f); line(0,0.5f, 0,1); line(0,1, 1,1); break;
    case '3': line(0,0, 1,0); line(1,0, 1,1); line(0,1, 1,1); line(0.3f,0.5f, 1,0.5f); break;
    case '4': line(0,0, 0,0.5f); line(0,0.5f, 1,0.5f); line(0.8f,0, 0.8f,1); break;
    case '5': line(1,0, 0,0); line(0,0, 0,0.5f); line(0,0.5f, 1,0.5f); line(1,0.5f, 1,1); line(1,1, 0,1); break;
    case '6': line(1,0, 0,0); line(0,0, 0,1); line(0,1, 1,1); line(1,1, 1,0.5f); line(1,0.5f, 0,0.5f); break;
    case '7': line(0,0, 1,0); line(1,0, 0.3f,1); break;
    case '8': line(0,0, 1,0); line(1,0, 1,1); line(1,1, 0,1); line(0,1, 0,0); line(0,0.5f, 1,0.5f); break;
    case '9': line(1,0.5f, 0,0.5f); line(0,0.5f, 0,0); line(0,0, 1,0); line(1,0, 1,1); line(1,1, 0,1); break;
    case '-': line(0.2f,0.5f, 0.8f,0.5f); break;
    case '+': line(0.2f,0.5f, 0.8f,0.5f); line(0.5f,0.2f, 0.5f,0.8f); break;
    case '/': line(0,1, 1,0); break;
    case '.': line(0.4f,0.9f, 0.6f,0.9f); break;
    case ':': line(0.45f,0.3f, 0.55f,0.3f); line(0.45f,0.7f, 0.55f,0.7f); break;
    case '[': line(0.7f,0, 0.3f,0); line(0.3f,0, 0.3f,1); line(0.3f,1, 0.7f,1); break;
    case ']': line(0.3f,0, 0.7f,0); line(0.7f,0, 0.7f,1); line(0.7f,1, 0.3f,1); break;
    case '(': line(0.7f,0, 0.3f,0.5f); line(0.3f,0.5f, 0.7f,1); break;
    case ')': line(0.3f,0, 0.7f,0.5f); line(0.7f,0.5f, 0.3f,1); break;
    case '<': line(0.8f,0.1f, 0.2f,0.5f); line(0.2f,0.5f, 0.8f,0.9f); break;
    case '>': line(0.2f,0.1f, 0.8f,0.5f); line(0.8f,0.5f, 0.2f,0.9f); break;
    default: break;
    }
}

void MenuRenderer::DrawString(const std::string& str, f32 x, f32 y, f32 size, const Math::Vec4f& col, f32 thickness) {
    f32 cx = x;
    f32 charW = size * 0.60f;
    f32 spacing = charW + size * 0.22f;

    for (char c : str) {
        if (c == ' ') {
            cx += spacing;
            continue;
        }
        DrawChar(static_cast<char>(std::toupper(static_cast<unsigned char>(c))), cx, y, size, col, thickness);
        cx += spacing;
    }
}

bool MenuRenderer::DrawButton(
    f32 x, f32 y, f32 w, f32 h,
    const std::string& label,
    const Math::Vec4f& baseColor,
    const Math::Vec4f& hoverColor,
    const Math::Vec4f& textColor,
    f32 fontSize)
{
    bool hovered = (m_mouseX >= x && m_mouseX <= x + w && m_mouseY >= y && m_mouseY <= y + h);
    Math::Vec4f col = hovered ? hoverColor : baseColor;

    AddQuad(
        { x, y, 0.0f },
        { x + w, y, 0.0f },
        { x + w, y + h, 0.0f },
        { x, y + h, 0.0f },
        col);

    Math::Vec4f borderCol = hovered ? Math::Vec4f{ 0.3f, 0.9f, 1.0f, 1.0f } : Math::Vec4f{ 0.3f, 0.45f, 0.65f, 0.8f };
    AddLine(x, y, x + w, y, hovered ? 2.0f : 1.0f, borderCol);
    AddLine(x + w, y, x + w, y + h, hovered ? 2.0f : 1.0f, borderCol);
    AddLine(x + w, y + h, x, y + h, hovered ? 2.0f : 1.0f, borderCol);
    AddLine(x, y + h, x, y, hovered ? 2.0f : 1.0f, borderCol);

    f32 textW = static_cast<f32>(label.length()) * (fontSize * 0.82f);
    f32 tx = x + (w - textW) * 0.5f;
    f32 ty = y + (h - fontSize) * 0.5f;
    DrawString(label, tx, ty, fontSize, textColor, hovered ? 2.0f : 1.6f);

    return hovered && m_leftClicked;
}

void MenuRenderer::FlushBuffer(ID3D12GraphicsCommandList* cmdList, u32 screenWidth, u32 screenHeight) {
    if (m_indices.empty() || !cmdList) return;

    size_t vertCount = std::min(m_vertices.size(), MaxVertices);
    size_t idxCount = std::min(m_indices.size(), MaxIndices);
    memcpy(m_mappedVertices, m_vertices.data(), vertCount * sizeof(Vertex3D));
    memcpy(m_mappedIndices, m_indices.data(), idxCount * sizeof(u32));

    cmdList->SetGraphicsRootSignature(m_rootSignature.Get());
    cmdList->SetPipelineState(m_pipelineState.Get());

    struct {
        float screenSize[2];
        float padding[2];
    } constants{};
    constants.screenSize[0] = static_cast<float>(screenWidth);
    constants.screenSize[1] = static_cast<float>(screenHeight);

    cmdList->SetGraphicsRoot32BitConstants(0, 4, &constants, 0);
    cmdList->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
    cmdList->IASetVertexBuffers(0, 1, &m_vbView);
    cmdList->IASetIndexBuffer(&m_ibView);
    cmdList->DrawIndexedInstanced(static_cast<UINT>(idxCount), 1, 0, 0, 0);
}

void MenuRenderer::RenderPreFlight(
    ID3D12GraphicsCommandList* cmdList,
    u32 screenWidth,
    u32 screenHeight,
    const std::wstring& aircraftName,
    const std::string& airportName,
    const std::string& airportIcao,
    const std::string& runwayIdent,
    f64 runwayLengthM,
    f64 elevationM,
    const std::string& weatherPresetName,
    const std::string& spawnModeName,
    f64 fuelPercentage) noexcept
{
    if (!m_initialized || !cmdList || screenWidth == 0 || screenHeight == 0) return;

    m_vertices.clear();
    m_indices.clear();

    f32 sw = static_cast<f32>(screenWidth);
    f32 sh = static_cast<f32>(screenHeight);

    // Deep cockpit vignette / backdrop
    AddQuad(
        { 0.0f, 0.0f, 0.0f },
        { sw, 0.0f, 0.0f },
        { sw, sh, 0.0f },
        { 0.0f, sh, 0.0f },
        { 0.02f, 0.04f, 0.07f, 0.85f });

    f32 cardW = 780.0f;
    f32 cardH = 480.0f;
    f32 cardX = (sw - cardW) * 0.5f;
    f32 cardY = (sh - cardH) * 0.5f;

    // Card background
    AddQuad(
        { cardX, cardY, 0.0f },
        { cardX + cardW, cardY, 0.0f },
        { cardX + cardW, cardY + cardH, 0.0f },
        { cardX, cardY + cardH, 0.0f },
        { 0.06f, 0.09f, 0.14f, 0.98f });

    Math::Vec4f colCyan{ 0.15f, 0.85f, 1.0f, 1.0f };
    Math::Vec4f colAmber{ 1.0f, 0.75f, 0.1f, 1.0f };
    Math::Vec4f colBorder{ 0.25f, 0.35f, 0.50f, 0.8f };
    Math::Vec4f colWhite{ 0.95f, 0.95f, 0.98f, 1.0f };
    Math::Vec4f colGray{ 0.55f, 0.65f, 0.75f, 1.0f };
    Math::Vec4f colGreen{ 0.18f, 0.95f, 0.40f, 1.0f };

    // Outer framing lines
    AddLine(cardX, cardY, cardX + cardW, cardY, 2.0f, colCyan);
    AddLine(cardX + cardW, cardY, cardX + cardW, cardY + cardH, 2.0f, colBorder);
    AddLine(cardX + cardW, cardY + cardH, cardX, cardY + cardH, 2.0f, colBorder);
    AddLine(cardX, cardY + cardH, cardX, cardY, 2.0f, colBorder);

    // Header banner
    DrawString("LOKYS FLIGHT SIMULATOR", cardX + 32.0f, cardY + 22.0f, 18.0f, colCyan, 2.2f);
    DrawString("MISSION DISPATCH & PRE-FLIGHT SETUP", cardX + 32.0f, cardY + 46.0f, 11.0f, colGray);
    AddLine(cardX + 32.0f, cardY + 66.0f, cardX + cardW - 32.0f, cardY + 66.0f, 1.0f, colBorder);

    f32 rowY = cardY + 84.0f;
    f32 rowH = 68.0f;

    // Row 1: Aircraft Selection
    std::string narrowAc;
    for (wchar_t wc : aircraftName) narrowAc += (wc < 128) ? static_cast<char>(wc) : '?';

    DrawString("AIRCRAFT MODEL", cardX + 32.0f, rowY, 10.0f, colGray);
    DrawString(narrowAc, cardX + 32.0f, rowY + 16.0f, 15.0f, colGreen, 2.0f);
    if (DrawButton(cardX + cardW - 140.0f, rowY + 8.0f, 108.0f, 28.0f, "[V] CYCLE", { 0.10f, 0.15f, 0.24f, 0.9f }, { 0.20f, 0.40f, 0.70f, 1.0f }, colAmber, 11.0f)) {
        m_cycleAircraftTriggered = true;
    }
    AddLine(cardX + 32.0f, rowY + 48.0f, cardX + cardW - 32.0f, rowY + 48.0f, 1.0f, { 0.15f, 0.22f, 0.32f, 0.5f });

    // Row 2: Airfield & Runway
    rowY += rowH;
    std::ostringstream aptStream;
    aptStream << airportName << " (" << airportIcao << ") - RWY " << runwayIdent
              << " | " << static_cast<int>(runwayLengthM) << "M | " << static_cast<int>(elevationM) << "M ELEV";
    DrawString("AIRFIELD & RUNWAY", cardX + 32.0f, rowY, 10.0f, colGray);
    DrawString(aptStream.str(), cardX + 32.0f, rowY + 16.0f, 13.0f, colWhite, 1.8f);
    if (DrawButton(cardX + cardW - 140.0f, rowY + 8.0f, 108.0f, 28.0f, "[TAB] CYCLE", { 0.10f, 0.15f, 0.24f, 0.9f }, { 0.20f, 0.40f, 0.70f, 1.0f }, colAmber, 11.0f)) {
        m_cycleAirportTriggered = true;
    }
    AddLine(cardX + 32.0f, rowY + 48.0f, cardX + cardW - 32.0f, rowY + 48.0f, 1.0f, { 0.15f, 0.22f, 0.32f, 0.5f });

    // Row 3: Weather Presets
    rowY += rowH;
    DrawString("ATMOSPHERE & WEATHER PRESET", cardX + 32.0f, rowY, 10.0f, colGray);
    DrawString(weatherPresetName, cardX + 32.0f, rowY + 16.0f, 13.0f, colWhite, 1.8f);
    if (DrawButton(cardX + cardW - 140.0f, rowY + 8.0f, 108.0f, 28.0f, "[W] CYCLE", { 0.10f, 0.15f, 0.24f, 0.9f }, { 0.20f, 0.40f, 0.70f, 1.0f }, colAmber, 11.0f)) {
        m_cycleWeatherTriggered = true;
    }
    AddLine(cardX + 32.0f, rowY + 48.0f, cardX + cardW - 32.0f, rowY + 48.0f, 1.0f, { 0.15f, 0.22f, 0.32f, 0.5f });

    // Row 4: Spawn Mode & Initial Fuel
    rowY += rowH;
    std::string fuelStr = "FUEL: " + std::to_string(static_cast<int>(fuelPercentage * 100.0)) + "%";
    DrawString("SPAWN PROFILE & DISPATCH", cardX + 32.0f, rowY, 10.0f, colGray);
    DrawString(spawnModeName + "  -  " + fuelStr, cardX + 32.0f, rowY + 16.0f, 13.0f, colWhite, 1.8f);
    if (DrawButton(cardX + cardW - 140.0f, rowY + 8.0f, 108.0f, 28.0f, "[R] TOGGLE", { 0.10f, 0.15f, 0.24f, 0.9f }, { 0.20f, 0.40f, 0.70f, 1.0f }, colAmber, 11.0f)) {
        m_toggleSpawnModeTriggered = true;
    }
    AddLine(cardX + 32.0f, rowY + 48.0f, cardX + cardW - 32.0f, rowY + 48.0f, 1.0f, { 0.15f, 0.22f, 0.32f, 0.5f });

    // Bottom Action Buttons
    f32 btnY = cardY + cardH - 64.0f;
    if (DrawButton(cardX + 32.0f, btnY, 160.0f, 40.0f, "SETTINGS [O]", { 0.10f, 0.15f, 0.24f, 0.9f }, { 0.20f, 0.35f, 0.55f, 1.0f }, colWhite, 12.0f)) {
        m_settingsTriggered = true;
    }

    if (DrawButton(cardX + cardW - 280.0f, btnY, 248.0f, 40.0f, "FLY NOW [ENTER]", { 0.08f, 0.35f, 0.65f, 0.95f }, { 0.15f, 0.55f, 0.95f, 1.0f }, colWhite, 13.0f)) {
        m_startFlightTriggered = true;
    }

    FlushBuffer(cmdList, screenWidth, screenHeight);
}

void MenuRenderer::RenderPause(
    ID3D12GraphicsCommandList* cmdList,
    u32 screenWidth,
    u32 screenHeight) noexcept
{
    if (!m_initialized || !cmdList || screenWidth == 0 || screenHeight == 0) return;

    m_vertices.clear();
    m_indices.clear();

    f32 sw = static_cast<f32>(screenWidth);
    f32 sh = static_cast<f32>(screenHeight);

    // Dimmed overlay
    AddQuad(
        { 0.0f, 0.0f, 0.0f },
        { sw, 0.0f, 0.0f },
        { sw, sh, 0.0f },
        { 0.0f, sh, 0.0f },
        { 0.01f, 0.02f, 0.04f, 0.70f });

    f32 cardW = 460.0f;
    f32 cardH = 280.0f;
    f32 cardX = (sw - cardW) * 0.5f;
    f32 cardY = (sh - cardH) * 0.5f;

    AddQuad(
        { cardX, cardY, 0.0f },
        { cardX + cardW, cardY, 0.0f },
        { cardX + cardW, cardY + cardH, 0.0f },
        { cardX, cardY + cardH, 0.0f },
        { 0.06f, 0.08f, 0.12f, 0.98f });

    Math::Vec4f colAmber{ 1.0f, 0.75f, 0.1f, 1.0f };
    Math::Vec4f colBorder{ 0.30f, 0.40f, 0.55f, 1.0f };
    Math::Vec4f colWhite{ 0.95f, 0.95f, 0.98f, 1.0f };

    AddLine(cardX, cardY, cardX + cardW, cardY, 2.0f, colAmber);
    AddLine(cardX + cardW, cardY, cardX + cardW, cardY + cardH, 2.0f, colBorder);
    AddLine(cardX + cardW, cardY + cardH, cardX, cardY + cardH, 2.0f, colBorder);
    AddLine(cardX, cardY + cardH, cardX, cardY, 2.0f, colBorder);

    DrawString("SIMULATION PAUSED", cardX + 115.0f, cardY + 22.0f, 17.0f, colAmber, 2.5f);
    AddLine(cardX + 24.0f, cardY + 50.0f, cardX + cardW - 24.0f, cardY + 50.0f, 1.0f, colBorder);

    f32 btnY = cardY + 68.0f;
    f32 btnH = 38.0f;
    f32 btnSpacing = 48.0f;

    if (DrawButton(cardX + 40.0f, btnY, cardW - 80.0f, btnH, "RESUME FLIGHT [ESC / ENTER]", { 0.10f, 0.16f, 0.25f, 0.95f }, { 0.15f, 0.45f, 0.75f, 1.0f }, colWhite, 13.0f)) {
        m_resumeFlightTriggered = true;
    }
    btnY += btnSpacing;

    if (DrawButton(cardX + 40.0f, btnY, cardW - 80.0f, btnH, "RESTART FLIGHT [R]", { 0.10f, 0.16f, 0.25f, 0.95f }, { 0.15f, 0.45f, 0.75f, 1.0f }, colWhite, 13.0f)) {
        m_restartFlightTriggered = true;
    }
    btnY += btnSpacing;

    if (DrawButton(cardX + 40.0f, btnY, cardW - 80.0f, btnH, "CONTROLS & SETTINGS [O]", { 0.10f, 0.16f, 0.25f, 0.95f }, { 0.15f, 0.45f, 0.75f, 1.0f }, colWhite, 13.0f)) {
        m_settingsTriggered = true;
    }
    btnY += btnSpacing;

    if (DrawButton(cardX + 40.0f, btnY, cardW - 80.0f, btnH, "EXIT TO MAIN MENU [M]", { 0.15f, 0.08f, 0.10f, 0.95f }, { 0.45f, 0.15f, 0.20f, 1.0f }, { 1.0f, 0.5f, 0.5f, 1.0f }, 13.0f)) {
        m_exitToMenuTriggered = true;
    }

    FlushBuffer(cmdList, screenWidth, screenHeight);
}

void MenuRenderer::RenderSettings(
    ID3D12GraphicsCommandList* cmdList,
    u32 screenWidth,
    u32 screenHeight,
    const Input::SimulatorConfig& config) noexcept
{
    if (!m_initialized || !cmdList || screenWidth == 0 || screenHeight == 0) return;

    m_vertices.clear();
    m_indices.clear();

    f32 sw = static_cast<f32>(screenWidth);
    f32 sh = static_cast<f32>(screenHeight);

    // Dark backdrop
    AddQuad(
        { 0.0f, 0.0f, 0.0f },
        { sw, 0.0f, 0.0f },
        { sw, sh, 0.0f },
        { 0.0f, sh, 0.0f },
        { 0.02f, 0.04f, 0.07f, 0.90f });

    f32 cardW = 860.0f;
    f32 cardH = 540.0f;
    f32 cardX = (sw - cardW) * 0.5f;
    f32 cardY = (sh - cardH) * 0.5f;

    AddQuad(
        { cardX, cardY, 0.0f },
        { cardX + cardW, cardY, 0.0f },
        { cardX + cardW, cardY + cardH, 0.0f },
        { cardX, cardY + cardH, 0.0f },
        { 0.05f, 0.08f, 0.12f, 0.98f });

    Math::Vec4f colCyan{ 0.15f, 0.85f, 1.0f, 1.0f };
    Math::Vec4f colBorder{ 0.25f, 0.35f, 0.50f, 0.8f };
    Math::Vec4f colWhite{ 0.95f, 0.95f, 0.98f, 1.0f };
    Math::Vec4f colGray{ 0.55f, 0.65f, 0.75f, 1.0f };
    Math::Vec4f colAmber{ 1.0f, 0.75f, 0.1f, 1.0f };
    Math::Vec4f colGreen{ 0.18f, 0.95f, 0.40f, 1.0f };

    AddLine(cardX, cardY, cardX + cardW, cardY, 2.0f, colCyan);
    AddLine(cardX + cardW, cardY, cardX + cardW, cardY + cardH, 2.0f, colBorder);
    AddLine(cardX + cardW, cardY + cardH, cardX, cardY + cardH, 2.0f, colBorder);
    AddLine(cardX, cardY + cardH, cardX, cardY, 2.0f, colBorder);

    DrawString("GLOBAL SETTINGS & CONTROL REMAPPING", cardX + 32.0f, cardY + 22.0f, 17.0f, colCyan, 2.2f);
    DrawString("CONFIGURE INPUT ACTION BINDINGS & CALIBRATION", cardX + 32.0f, cardY + 46.0f, 10.0f, colGray);
    AddLine(cardX + 32.0f, cardY + 64.0f, cardX + cardW - 32.0f, cardY + 64.0f, 1.0f, colBorder);

    // Audio & Display quick controls
    f32 topY = cardY + 76.0f;
    std::string volStr = "VOLUME: " + std::to_string(static_cast<int>(config.masterVolume * 100.0f)) + "%";
    DrawString(volStr, cardX + 32.0f, topY + 6.0f, 12.0f, colWhite);
    if (DrawButton(cardX + 170.0f, topY, 40.0f, 26.0f, "-", { 0.12f, 0.18f, 0.28f, 0.9f }, { 0.20f, 0.40f, 0.70f, 1.0f }, colWhite, 12.0f)) {
        m_volume = std::max(0.0f, config.masterVolume - 0.05f);
        m_saveConfigTriggered = true;
    }
    if (DrawButton(cardX + 218.0f, topY, 40.0f, 26.0f, "+", { 0.12f, 0.18f, 0.28f, 0.9f }, { 0.20f, 0.40f, 0.70f, 1.0f }, colWhite, 12.0f)) {
        m_volume = std::min(1.0f, config.masterVolume + 0.05f);
        m_saveConfigTriggered = true;
    }

    const char* dispNames[] = { "PFD GLASS", "TRANSPARENT HUD", "ANALOG SIX-PACK", "OFF" };
    u32 safeDispMode = config.cockpitDisplayMode % 4;
    std::string dispStr = std::string("INSTRUMENTS: ") + dispNames[safeDispMode];
    DrawString(dispStr, cardX + 340.0f, topY + 6.0f, 12.0f, colWhite);
    if (DrawButton(cardX + 660.0f, topY, 90.0f, 26.0f, "CYCLE [H]", { 0.12f, 0.18f, 0.28f, 0.9f }, { 0.20f, 0.40f, 0.70f, 1.0f }, colAmber, 11.0f)) {
        m_displayMode = (config.cockpitDisplayMode + 1) % 4;
        m_saveConfigTriggered = true;
    }

    AddLine(cardX + 32.0f, topY + 36.0f, cardX + cardW - 32.0f, topY + 36.0f, 1.0f, colBorder);

    // Bindings list header
    f32 listY = topY + 44.0f;
    DrawString("ACTION", cardX + 40.0f, listY, 11.0f, colGray);
    DrawString("DESCRIPTION", cardX + 240.0f, listY, 11.0f, colGray);
    DrawString("CURRENT BINDING", cardX + 580.0f, listY, 11.0f, colGray);
    listY += 20.0f;

    // Render 7 action rows
    u32 maxActions = static_cast<u32>(Input::InputAction::Count);
    for (u32 i = 0; i < 7 && (i + m_settingsScrollOffset) < maxActions; ++i) {
        auto action = static_cast<Input::InputAction>(i + m_settingsScrollOffset);
        const char* name = Input::ActionMap::GetActionName(action);
        const char* desc = Input::ActionMap::GetActionDescription(action);
        Input::KeyBinding kb = config.actionMap.GetBinding(action);
        std::string bindStr = kb.ToString();

        bool isListening = (m_listeningAction.has_value() && *m_listeningAction == action);
        Math::Vec4f rowCol = (i % 2 == 0) ? Math::Vec4f{ 0.08f, 0.11f, 0.17f, 0.8f } : Math::Vec4f{ 0.06f, 0.09f, 0.14f, 0.8f };

        AddQuad(
            { cardX + 32.0f, listY, 0.0f },
            { cardX + cardW - 32.0f, listY, 0.0f },
            { cardX + cardW - 32.0f, listY + 32.0f, 0.0f },
            { cardX + 32.0f, listY + 32.0f, 0.0f },
            rowCol);

        DrawString(name, cardX + 40.0f, listY + 8.0f, 12.0f, colWhite, 1.8f);
        DrawString(desc, cardX + 240.0f, listY + 9.0f, 10.0f, colGray);

        if (isListening) {
            DrawString("[PRESS ANY KEY...]", cardX + 580.0f, listY + 8.0f, 12.0f, colAmber, 2.0f);
        } else {
            if (DrawButton(cardX + 580.0f, listY + 3.0f, 170.0f, 26.0f, bindStr, { 0.12f, 0.18f, 0.28f, 0.9f }, { 0.25f, 0.50f, 0.80f, 1.0f }, colCyan, 11.0f)) {
                m_listeningAction = action;
            }
        }

        listY += 36.0f;
    }

    // Bottom action row
    f32 botY = cardY + cardH - 56.0f;
    if (DrawButton(cardX + 32.0f, botY, 180.0f, 36.0f, "RESET DEFAULTS", { 0.18f, 0.10f, 0.12f, 0.95f }, { 0.45f, 0.20f, 0.25f, 1.0f }, { 1.0f, 0.6f, 0.6f, 1.0f }, 11.0f)) {
        m_resetBindingsTriggered = true;
    }

    if (DrawButton(cardX + 230.0f, botY, 140.0f, 36.0f, "MORE ACTIONS", { 0.10f, 0.15f, 0.24f, 0.9f }, { 0.20f, 0.35f, 0.55f, 1.0f }, colWhite, 11.0f)) {
        m_settingsScrollOffset = (m_settingsScrollOffset + 7) % maxActions;
    }

    if (DrawButton(cardX + cardW - 220.0f, botY, 188.0f, 36.0f, "SAVE & BACK [ESC]", { 0.08f, 0.35f, 0.65f, 0.95f }, { 0.15f, 0.55f, 0.95f, 1.0f }, colWhite, 12.0f)) {
        m_saveConfigTriggered = true;
        m_exitToMenuTriggered = true;
    }

    FlushBuffer(cmdList, screenWidth, screenHeight);
}

} // namespace LokYS::Graphics
