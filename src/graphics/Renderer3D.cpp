#include "LokYS/graphics/Renderer3D.hpp"
#include "LokYS/math/CoordinateFrames.hpp"
#include <d3dcompiler.h>
#include <iostream>

namespace LokYS::Graphics {

static const char* g_shaderSource = R"(
cbuffer SceneConstants : register(b0)
{
    row_major float4x4 gWorldViewProj;
    float4 gSunDirection;
    float4 gSunColor;
    float4 gAmbientColor;
};

struct VSInput
{
    float3 position : POSITION;
    float3 normal   : NORMAL;
    float4 color    : COLOR;
};

struct PSInput
{
    float4 position : SV_POSITION;
    float3 normal   : NORMAL;
    float4 color    : COLOR;
};

PSInput VSMain(VSInput input)
{
    PSInput output;
    output.position = mul(float4(input.position, 1.0f), gWorldViewProj);
    output.normal = input.normal;
    output.color = input.color;
    return output;
}

float4 PSMain(PSInput input) : SV_TARGET
{
    float3 n = normalize(input.normal);
    float3 l = normalize(gSunDirection.xyz);
    float NdotL = max(dot(n, l), 0.0f);
    float3 lighting = gAmbientColor.rgb + gSunColor.rgb * NdotL;
    return float4(input.color.rgb * lighting, input.color.a);
}
)";

bool Renderer3D::Initialize(ID3D12Device* device) {
    if (!device) {
        return false;
    }

    if (!CreateRootSignature(device)) {
        return false;
    }

    if (!CreatePipelineState(device)) {
        return false;
    }

    m_initialized = true;
    return true;
}

void Renderer3D::Shutdown() {
    m_pipelineState.Reset();
    m_rootSignature.Reset();
    m_initialized = false;
}

bool Renderer3D::CreateRootSignature(ID3D12Device* device) {
    D3D12_ROOT_PARAMETER rootParam{};
    rootParam.ParameterType = D3D12_ROOT_PARAMETER_TYPE_32BIT_CONSTANTS;
    rootParam.ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;
    rootParam.Constants.ShaderRegister = 0;
    rootParam.Constants.RegisterSpace = 0;
    rootParam.Constants.Num32BitValues = 28; // 16 (MVP) + 4 (SunDir) + 4 (SunColor) + 4 (Ambient)

    D3D12_ROOT_SIGNATURE_DESC rootSigDesc{};
    rootSigDesc.NumParameters = 1;
    rootSigDesc.pParameters = &rootParam;
    rootSigDesc.NumStaticSamplers = 0;
    rootSigDesc.pStaticSamplers = nullptr;
    rootSigDesc.Flags = D3D12_ROOT_SIGNATURE_FLAG_ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT;

    Microsoft::WRL::ComPtr<ID3DBlob> signatureBlob;
    Microsoft::WRL::ComPtr<ID3DBlob> errorBlob;

    if (FAILED(D3D12SerializeRootSignature(
            &rootSigDesc,
            D3D_ROOT_SIGNATURE_VERSION_1,
            &signatureBlob,
            &errorBlob))) {
        return false;
    }

    if (FAILED(device->CreateRootSignature(
            0,
            signatureBlob->GetBufferPointer(),
            signatureBlob->GetBufferSize(),
            IID_PPV_ARGS(&m_rootSignature)))) {
        return false;
    }

    return true;
}

bool Renderer3D::CreatePipelineState(ID3D12Device* device) {
    Microsoft::WRL::ComPtr<ID3DBlob> vsBlob;
    Microsoft::WRL::ComPtr<ID3DBlob> psBlob;
    Microsoft::WRL::ComPtr<ID3DBlob> errorBlob;

    UINT compileFlags = 0;
#ifdef _DEBUG
    compileFlags = D3DCOMPILE_DEBUG | D3DCOMPILE_SKIP_OPTIMIZATION;
#else
    compileFlags = D3DCOMPILE_OPTIMIZATION_LEVEL3;
#endif

    if (FAILED(D3DCompile(
            g_shaderSource,
            strlen(g_shaderSource),
            "Shader",
            nullptr,
            nullptr,
            "VSMain",
            "vs_5_0",
            compileFlags,
            0,
            &vsBlob,
            &errorBlob))) {
        return false;
    }

    if (FAILED(D3DCompile(
            g_shaderSource,
            strlen(g_shaderSource),
            "Shader",
            nullptr,
            nullptr,
            "PSMain",
            "ps_5_0",
            compileFlags,
            0,
            &psBlob,
            &errorBlob))) {
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

    // Blend state: Opaque
    psoDesc.BlendState.AlphaToCoverageEnable = FALSE;
    psoDesc.BlendState.IndependentBlendEnable = FALSE;
    psoDesc.BlendState.RenderTarget[0].BlendEnable = FALSE;
    psoDesc.BlendState.RenderTarget[0].LogicOpEnable = FALSE;
    psoDesc.BlendState.RenderTarget[0].RenderTargetWriteMask = D3D12_COLOR_WRITE_ENABLE_ALL;

    psoDesc.SampleMask = UINT_MAX;

    // Rasterizer state: Solid, Cull None (handles thin wing double sides)
    psoDesc.RasterizerState.FillMode = D3D12_FILL_MODE_SOLID;
    psoDesc.RasterizerState.CullMode = D3D12_CULL_MODE_NONE;
    psoDesc.RasterizerState.FrontCounterClockwise = FALSE;
    psoDesc.RasterizerState.DepthClipEnable = TRUE;

    // Depth-stencil state: Depth test & write enabled
    psoDesc.DepthStencilState.DepthEnable = TRUE;
    psoDesc.DepthStencilState.DepthWriteMask = D3D12_DEPTH_WRITE_MASK_ALL;
    psoDesc.DepthStencilState.DepthFunc = D3D12_COMPARISON_FUNC_LESS_EQUAL;
    psoDesc.DepthStencilState.StencilEnable = FALSE;

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

void Renderer3D::Render(
    ID3D12GraphicsCommandList* cmdList,
    const Camera& camera,
    const Mesh& terrain,
    const Mesh& runway,
    const Mesh& aircraft,
    const Math::Vec3d& aircraftPosNed,
    const Math::Quat4d& aircraftAttitude) const noexcept
{
    if (!cmdList || !m_initialized) {
        return;
    }

    cmdList->SetGraphicsRootSignature(m_rootSignature.Get());
    cmdList->SetPipelineState(m_pipelineState.Get());

    const Math::Mat4f& viewProj = camera.GetViewProjMatrix();

    // Constant buffer packet structure (28 floats)
    struct RootConstants {
        Math::Mat4f mvp;
        Math::Vec4f sunDir;
        Math::Vec4f sunColor;
        Math::Vec4f ambientColor;
    } constants;

    constants.sunDir = Math::Vec4f(m_lighting.sunDirection.x, m_lighting.sunDirection.y, m_lighting.sunDirection.z, 0.0f);
    constants.sunColor = m_lighting.sunColor;
    constants.ambientColor = m_lighting.ambientColor;

    // 1. Draw Terrain & Runway (World Origin at NED [0, 0, 0])
    // Camera-relative translation for world origin: P_rel = NedToGraphics(0 - camPosNed)
    const Math::Vec3f originRelGfx = Math::CoordinateFrames::NedToGraphics(-camera.GetCameraPosNed());
    const Math::Mat4f worldModel = Math::Mat4f::Translation(originRelGfx);

    constants.mvp = worldModel * viewProj;
    cmdList->SetGraphicsRoot32BitConstants(0, 28, &constants, 0);

    terrain.Draw(cmdList);
    runway.Draw(cmdList);

    // 2. Draw Aircraft
    // Camera-relative position for aircraft:
    const Math::Vec3f aircraftRelGfx = Math::CoordinateFrames::NedToGraphics(aircraftPosNed - camera.GetCameraPosNed());

    // Aircraft attitude in graphics space:
    // Local forward (+Z): rotated by attitude
    const Math::Vec3d fwdNed = aircraftAttitude.Rotate(Math::Vec3d(1.0, 0.0, 0.0));
    const Math::Vec3d rightNed = aircraftAttitude.Rotate(Math::Vec3d(0.0, 1.0, 0.0));
    const Math::Vec3d upNed = aircraftAttitude.Rotate(Math::Vec3d(0.0, 0.0, -1.0));

    const Math::Vec3f fwdGfx = Math::CoordinateFrames::NedToGraphics(fwdNed);
    const Math::Vec3f rightGfx = Math::CoordinateFrames::NedToGraphics(rightNed);
    const Math::Vec3f upGfx = Math::CoordinateFrames::NedToGraphics(upNed);

    // Aircraft rotation matrix (Row-major vector post-multiply: v * R)
    // Row 0 = Right, Row 1 = Up, Row 2 = Forward
    const Math::Mat4f rotModel{
        rightGfx.x, rightGfx.y, rightGfx.z, 0.0f,
        upGfx.x,    upGfx.y,    upGfx.z,    0.0f,
        fwdGfx.x,   fwdGfx.y,   fwdGfx.z,   0.0f,
        0.0f,       0.0f,       0.0f,       1.0f
    };

    const Math::Mat4f transModel = Math::Mat4f::Translation(aircraftRelGfx);
    const Math::Mat4f aircraftModel = rotModel * transModel;

    constants.mvp = aircraftModel * viewProj;
    cmdList->SetGraphicsRoot32BitConstants(0, 28, &constants, 0);

    aircraft.Draw(cmdList);
}

void Renderer3D::Render(
    ID3D12GraphicsCommandList* cmdList,
    const Camera& camera,
    const Mesh& terrain,
    const Mesh& runway,
    const GltfModel& aircraftModel,
    const Math::Vec3d& aircraftPosNed,
    const Math::Quat4d& aircraftAttitude) const noexcept
{
    if (!cmdList || !m_initialized) {
        return;
    }

    cmdList->SetGraphicsRootSignature(m_rootSignature.Get());
    cmdList->SetPipelineState(m_pipelineState.Get());

    const Math::Mat4f& viewProj = camera.GetViewProjMatrix();

    struct RootConstants {
        Math::Mat4f mvp;
        Math::Vec4f sunDir;
        Math::Vec4f sunColor;
        Math::Vec4f ambientColor;
    } constants;

    constants.sunDir = Math::Vec4f(m_lighting.sunDirection.x, m_lighting.sunDirection.y, m_lighting.sunDirection.z, 0.0f);
    constants.sunColor = m_lighting.sunColor;
    constants.ambientColor = m_lighting.ambientColor;

    // 1. Draw Terrain & Runway
    const Math::Vec3f originRelGfx = Math::CoordinateFrames::NedToGraphics(-camera.GetCameraPosNed());
    const Math::Mat4f worldModel = Math::Mat4f::Translation(originRelGfx);

    constants.mvp = worldModel * viewProj;
    cmdList->SetGraphicsRoot32BitConstants(0, 28, &constants, 0);

    terrain.Draw(cmdList);
    runway.Draw(cmdList);

    // 2. Draw Articulated glTF Aircraft Model
    const Math::Vec3f aircraftRelGfx = Math::CoordinateFrames::NedToGraphics(aircraftPosNed - camera.GetCameraPosNed());

    const Math::Vec3d fwdNed = aircraftAttitude.Rotate(Math::Vec3d(1.0, 0.0, 0.0));
    const Math::Vec3d rightNed = aircraftAttitude.Rotate(Math::Vec3d(0.0, 1.0, 0.0));
    const Math::Vec3d upNed = aircraftAttitude.Rotate(Math::Vec3d(0.0, 0.0, -1.0));

    const Math::Vec3f fwdGfx = Math::CoordinateFrames::NedToGraphics(fwdNed);
    const Math::Vec3f rightGfx = Math::CoordinateFrames::NedToGraphics(rightNed);
    const Math::Vec3f upGfx = Math::CoordinateFrames::NedToGraphics(upNed);

    const Math::Mat4f rotModel{
        rightGfx.x, rightGfx.y, rightGfx.z, 0.0f,
        upGfx.x,    upGfx.y,    upGfx.z,    0.0f,
        fwdGfx.x,   fwdGfx.y,   fwdGfx.z,   0.0f,
        0.0f,       0.0f,       0.0f,       1.0f
    };

    const Math::Mat4f transModel = Math::Mat4f::Translation(aircraftRelGfx);
    const Math::Mat4f aircraftRootTransform = rotModel * transModel;

    // Prime lighting constants in shader
    cmdList->SetGraphicsRoot32BitConstants(0, 28, &constants, 0);

    aircraftModel.Draw(cmdList, aircraftRootTransform, viewProj, 0);
}

} // namespace LokYS::Graphics
