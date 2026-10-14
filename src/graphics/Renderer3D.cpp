#include "LokYS/graphics/Renderer3D.hpp"
#include "LokYS/graphics/ShadowMap.hpp"
#include "LokYS/graphics/TerrainSystem.hpp"
#include "LokYS/graphics/ScenerySystem.hpp"
#include "LokYS/network/RemoteAircraft.hpp"
#include "LokYS/math/CoordinateFrames.hpp"
#include <d3dcompiler.h>
#include <algorithm>
#include <iostream>

namespace LokYS::Graphics {

static const char* g_shaderSource = R"(
cbuffer SceneConstants : register(b0)
{
    row_major float4x4 gWorldViewProj;
    row_major float4x4 gLightViewProj;
    float4 gSunDirection;      // xyz: Sun direction towards ground, w: shadow bias
    float4 gSunColor;          // rgb: Sun color, a: shadow strength (0..1)
    float4 gAmbientColor;      // rgb: Ambient color, a: unused
    float4 gFogParams;         // x: fogStart, y: fogEnd, z: fogDensity, w: fogHeightDecay
    float4 gFogColor;          // rgb: Sky/fog color, a: fogEnabled (0.0 or 1.0)
    float4 gCameraPos;         // xyz: camera pos in graphics space, w: unused
};

Texture2D gShadowMap : register(t0);
SamplerComparisonState gShadowSampler : register(s0);

struct VSInput
{
    float3 position : POSITION;
    float3 normal   : NORMAL;
    float4 color    : COLOR;
};

struct PSInput
{
    float4 position     : SV_POSITION;
    float3 normal       : NORMAL;
    float4 color        : COLOR;
    float3 worldPos     : TEXCOORD0;
    float4 shadowPos    : TEXCOORD1;
};

PSInput VSMain(VSInput input)
{
    PSInput output;
    float4 pos = float4(input.position, 1.0f);
    output.position = mul(pos, gWorldViewProj);
    output.normal = input.normal;
    output.color = input.color;
    output.worldPos = input.position;
    output.shadowPos = mul(pos, gLightViewProj);
    return output;
}

float CalculateShadow(float4 shadowPos, float NdotL)
{
    // Perspective / Orthographic divide
    float3 projCoords = shadowPos.xyz / shadowPos.w;
    float2 shadowUV = projCoords.xy * float2(0.5f, -0.5f) + float2(0.5f, 0.5f);
    float currentDepth = projCoords.z;

    // Check outside shadow frustum bounds
    if (shadowUV.x < 0.001f || shadowUV.x > 0.999f ||
        shadowUV.y < 0.001f || shadowUV.y > 0.999f ||
        currentDepth < 0.0f   || currentDepth > 1.0f)
    {
        return 1.0f; // Fully illuminated outside shadow cascade
    }

    // Adaptive slope-scaled depth bias to eliminate shadow acne
    float bias = max(0.0015f * (1.0f - NdotL), 0.0003f);

    // 3x3 Percentage Closer Filter (PCF)
    float shadow = 0.0f;
    float2 texelSize = float2(1.0f / 2048.0f, 1.0f / 2048.0f);

    [unroll]
    for (int y = -1; y <= 1; ++y)
    {
        [unroll]
        for (int x = -1; x <= 1; ++x)
        {
            float pcfDepth = gShadowMap.SampleCmpLevelZero(
                gShadowSampler,
                shadowUV + float2(x, y) * texelSize,
                currentDepth - bias);
            shadow += pcfDepth;
        }
    }

    return shadow / 9.0f;
}

float4 PSMain(PSInput input) : SV_TARGET
{
    float3 n = normalize(input.normal);
    float3 l = normalize(gSunDirection.xyz);
    float NdotL = max(dot(n, l), 0.0f);

    float shadowFactor = 1.0f;
    if (gSunColor.a > 0.01f && NdotL > 0.0f)
    {
        shadowFactor = CalculateShadow(input.shadowPos, NdotL);
        shadowFactor = lerp(1.0f - gSunColor.a, 1.0f, shadowFactor);
    }

    float3 diffuse = gSunColor.rgb * (NdotL * shadowFactor);
    float3 lighting = gAmbientColor.rgb + diffuse;
    float3 litColor = input.color.rgb * lighting;

    // Atmospheric Aerial Fog & Sun Forward Scattering Glow
    if (gFogColor.a > 0.5f)
    {
        float3 viewRay = input.worldPos - gCameraPos.xyz;
        float dist = length(viewRay);
        float3 rayDir = viewRay / max(dist, 1e-4f);

        float heightDecay = max(gFogParams.w, 1e-6f);
        float avgHeight = max(0.0f, (input.worldPos.y + gCameraPos.y) * 0.5f);
        float opticalDepth = dist * gFogParams.z * exp(-avgHeight * heightDecay);
        float fogFactor = saturate(1.0f - exp(-opticalDepth));

        float sunGlow = pow(max(dot(rayDir, l), 0.0f), 12.0f) * 0.35f;
        float3 fogColor = gFogColor.rgb + gSunColor.rgb * sunGlow;

        litColor = lerp(litColor, fogColor, fogFactor);
    }

    return float4(litColor, input.color.a);
}
)";

struct alignas(16) SceneConstantsBuffer {
    Math::Mat4f mvp;
    Math::Mat4f lightMvp;
    Math::Vec4f sunDir;
    Math::Vec4f sunColor;
    Math::Vec4f ambientColor;
    Math::Vec4f fogParams;
    Math::Vec4f fogColor;
    Math::Vec4f cameraPos;
};

static_assert(sizeof(SceneConstantsBuffer) == 56 * sizeof(f32), "SceneConstantsBuffer must be exactly 56 floats");

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
    // Parameter 0: 56 32-bit constants (SceneConstantsBuffer)
    D3D12_ROOT_PARAMETER rootParams[2]{};

    rootParams[0].ParameterType = D3D12_ROOT_PARAMETER_TYPE_32BIT_CONSTANTS;
    rootParams[0].ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;
    rootParams[0].Constants.ShaderRegister = 0;
    rootParams[0].Constants.RegisterSpace = 0;
    rootParams[0].Constants.Num32BitValues = 56;

    // Parameter 1: Descriptor table for Texture2D gShadowMap (t0)
    D3D12_DESCRIPTOR_RANGE srvRange{};
    srvRange.RangeType = D3D12_DESCRIPTOR_RANGE_TYPE_SRV;
    srvRange.NumDescriptors = 1;
    srvRange.BaseShaderRegister = 0;
    srvRange.RegisterSpace = 0;
    srvRange.OffsetInDescriptorsFromTableStart = D3D12_DESCRIPTOR_RANGE_OFFSET_APPEND;

    rootParams[1].ParameterType = D3D12_ROOT_PARAMETER_TYPE_DESCRIPTOR_TABLE;
    rootParams[1].ShaderVisibility = D3D12_SHADER_VISIBILITY_PIXEL;
    rootParams[1].DescriptorTable.NumDescriptorRanges = 1;
    rootParams[1].DescriptorTable.pDescriptorRanges = &srvRange;

    // Static Sampler 0: Comparison Sampler for Shadow Map PCF
    D3D12_STATIC_SAMPLER_DESC samplerDesc{};
    samplerDesc.Filter = D3D12_FILTER_COMPARISON_MIN_MAG_LINEAR_MIP_POINT;
    samplerDesc.AddressU = D3D12_TEXTURE_ADDRESS_MODE_BORDER;
    samplerDesc.AddressV = D3D12_TEXTURE_ADDRESS_MODE_BORDER;
    samplerDesc.AddressW = D3D12_TEXTURE_ADDRESS_MODE_BORDER;
    samplerDesc.MipLODBias = 0.0f;
    samplerDesc.MaxAnisotropy = 1;
    samplerDesc.ComparisonFunc = D3D12_COMPARISON_FUNC_LESS_EQUAL;
    samplerDesc.BorderColor = D3D12_STATIC_BORDER_COLOR_OPAQUE_WHITE;
    samplerDesc.MinLOD = 0.0f;
    samplerDesc.MaxLOD = D3D12_FLOAT32_MAX;
    samplerDesc.ShaderRegister = 0;
    samplerDesc.RegisterSpace = 0;
    samplerDesc.ShaderVisibility = D3D12_SHADER_VISIBILITY_PIXEL;

    D3D12_ROOT_SIGNATURE_DESC rootSigDesc{};
    rootSigDesc.NumParameters = 2;
    rootSigDesc.pParameters = rootParams;
    rootSigDesc.NumStaticSamplers = 1;
    rootSigDesc.pStaticSamplers = &samplerDesc;
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

    psoDesc.BlendState.AlphaToCoverageEnable = FALSE;
    psoDesc.BlendState.IndependentBlendEnable = FALSE;
    psoDesc.BlendState.RenderTarget[0].BlendEnable = FALSE;
    psoDesc.BlendState.RenderTarget[0].LogicOpEnable = FALSE;
    psoDesc.BlendState.RenderTarget[0].RenderTargetWriteMask = D3D12_COLOR_WRITE_ENABLE_ALL;

    psoDesc.SampleMask = UINT_MAX;

    psoDesc.RasterizerState.FillMode = D3D12_FILL_MODE_SOLID;
    psoDesc.RasterizerState.CullMode = D3D12_CULL_MODE_NONE;
    psoDesc.RasterizerState.FrontCounterClockwise = FALSE;
    psoDesc.RasterizerState.DepthClipEnable = TRUE;

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

void Renderer3D::RenderShadowPass(
    ID3D12GraphicsCommandList* cmdList,
    const ShadowMap& shadowMap,
    const GltfModel& aircraftModel,
    const Math::Vec3d& aircraftPosNed,
    const Math::Quat4d& aircraftAttitude) const noexcept
{
    if (!cmdList) {
        return;
    }

    // Shadow pass uses the light View-Projection matrix
    const Math::Mat4f& lightViewProj = shadowMap.GetLightViewProjMatrix();

    // Aircraft world position in graphics space
    const Math::Vec3f aircraftGfx = Math::CoordinateFrames::NedToGraphics(aircraftPosNed);

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

    const Math::Mat4f transModel = Math::Mat4f::Translation(aircraftGfx);
    const Math::Mat4f aircraftRootTransform = rotModel * transModel;

    // Draw aircraft into shadow map
    aircraftModel.Draw(cmdList, aircraftRootTransform, lightViewProj, 0);
}

void Renderer3D::Render(
    ID3D12GraphicsCommandList* cmdList,
    const Camera& camera,
    const ShadowMap* shadowMap,
    TerrainSystem& terrain,
    const Mesh& runway,
    const ScenerySystem& scenery,
    const GltfModel& aircraftModel,
    const Math::Vec3d& aircraftPosNed,
    const Math::Quat4d& aircraftAttitude) const noexcept
{
    if (!cmdList || !m_initialized) {
        return;
    }

    cmdList->SetGraphicsRootSignature(m_rootSignature.Get());
    cmdList->SetPipelineState(m_pipelineState.Get());

    // Bind Shadow Map SRV Descriptor Heap and Table
    if (shadowMap && shadowMap->GetSrvHeap()) {
        ID3D12DescriptorHeap* heaps[] = { shadowMap->GetSrvHeap() };
        cmdList->SetDescriptorHeaps(1, heaps);
        cmdList->SetGraphicsRootDescriptorTable(1, shadowMap->GetSrvGpuHandle());
    }

    const Math::Mat4f& viewProj = camera.GetViewProjMatrix();
    const Math::Frustum frustum = Math::Frustum::FromViewProj(viewProj);

    SceneConstantsBuffer constants{};
    constants.sunDir = Math::Vec4f(m_lighting.sunDirection.x, m_lighting.sunDirection.y, m_lighting.sunDirection.z, 0.0f);
    // Alpha channel of sunColor controls shadow intensity: 0.85 = prominent realistic shadow
    constants.sunColor = Math::Vec4f(m_lighting.sunColor.x, m_lighting.sunColor.y, m_lighting.sunColor.z, shadowMap ? 0.85f : 0.0f);
    constants.ambientColor = m_lighting.ambientColor;
    constants.fogParams = Math::Vec4f(m_fog.startDistance, m_fog.endDistance, m_fog.density, m_fog.heightDecay);
    constants.fogColor = Math::Vec4f(m_fog.color.x, m_fog.color.y, m_fog.color.z, m_fog.enabled ? 1.0f : 0.0f);
    constants.cameraPos = Math::Vec4f(camera.GetCameraPosGfx().x, camera.GetCameraPosGfx().y, camera.GetCameraPosGfx().z, 0.0f);

    // Light View-Projection matrix for shadow sampling
    constants.lightMvp = shadowMap ? shadowMap->GetLightViewProjMatrix() : Math::Mat4f::Identity();

    // 1. Draw Multi-LOD Terrain & Runway & Scenery (Camera-relative world origin)
    const Math::Vec3f originRelGfx = Math::CoordinateFrames::NedToGraphics(-camera.GetCameraPosNed());
    const Math::Mat4f worldModel = Math::Mat4f::Translation(originRelGfx);

    constants.mvp = worldModel * viewProj;
    cmdList->SetGraphicsRoot32BitConstants(0, 56, &constants, 0);

    // Draw Multi-LOD Terrain chunks (with frustum culling)
    terrain.Draw(cmdList, frustum, camera.GetCameraPosGfx());

    // Draw Runway Asphalt and markings
    runway.Draw(cmdList);

    // Draw Airport Scenery (Buildings, Runway Lights, PAPI, Windsock)
    scenery.DrawBuildings(cmdList);
    scenery.DrawRunwayLights(cmdList);
    scenery.DrawPapi(cmdList);
    scenery.DrawWindsock(cmdList);

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

    cmdList->SetGraphicsRoot32BitConstants(0, 56, &constants, 0);
    aircraftModel.Draw(cmdList, aircraftRootTransform, viewProj, 0);
}

void Renderer3D::Render(
    ID3D12GraphicsCommandList* cmdList,
    const Camera& camera,
    const ShadowMap* shadowMap,
    TerrainSystem& terrain,
    const Mesh& runway,
    const ScenerySystem& scenery,
    GltfModel& aircraftModel,
    const Math::Vec3d& aircraftPosNed,
    const Math::Quat4d& aircraftAttitude,
    const std::vector<Network::RemoteAircraft>& remotePeers) const noexcept
{
    // Draw terrain, scenery, runway, and local aircraft
    Render(cmdList, camera, shadowMap, terrain, runway, scenery, aircraftModel, aircraftPosNed, aircraftAttitude);

    if (remotePeers.empty() || !cmdList || !m_initialized) {
        return;
    }

    const Math::Mat4f& viewProj = camera.GetViewProjMatrix();

    // Draw each remote peer aircraft
    for (const auto& peer : remotePeers) {
        const Math::Vec3f peerRelGfx = Math::CoordinateFrames::NedToGraphics(peer.GetPositionNed() - camera.GetCameraPosNed());

        const Math::Vec3d fwdNed = peer.GetAttitude().Rotate(Math::Vec3d(1.0, 0.0, 0.0));
        const Math::Vec3d rightNed = peer.GetAttitude().Rotate(Math::Vec3d(0.0, 1.0, 0.0));
        const Math::Vec3d upNed = peer.GetAttitude().Rotate(Math::Vec3d(0.0, 0.0, -1.0));

        const Math::Vec3f fwdGfx = Math::CoordinateFrames::NedToGraphics(fwdNed);
        const Math::Vec3f rightGfx = Math::CoordinateFrames::NedToGraphics(rightNed);
        const Math::Vec3f upGfx = Math::CoordinateFrames::NedToGraphics(upNed);

        const Math::Mat4f rotModel{
            rightGfx.x, rightGfx.y, rightGfx.z, 0.0f,
            upGfx.x,    upGfx.y,    upGfx.z,    0.0f,
            fwdGfx.x,   fwdGfx.y,   fwdGfx.z,   0.0f,
            0.0f,       0.0f,       0.0f,       1.0f
        };

        const Math::Mat4f transModel = Math::Mat4f::Translation(peerRelGfx);
        const Math::Mat4f peerRootTransform = rotModel * transModel;

        aircraftModel.ApplyAircraftControls(
            peer.GetElevator(),
            peer.GetAileron(),
            peer.GetRudder(),
            peer.GetFlaps(),
            static_cast<float>(peer.GetPropellerAngleRad()));

        aircraftModel.Draw(cmdList, peerRootTransform, viewProj, 0);
    }
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

    SceneConstantsBuffer constants{};
    constants.sunDir = Math::Vec4f(m_lighting.sunDirection.x, m_lighting.sunDirection.y, m_lighting.sunDirection.z, 0.0f);
    constants.sunColor = Math::Vec4f(m_lighting.sunColor.x, m_lighting.sunColor.y, m_lighting.sunColor.z, 0.0f); // No shadows in fallback
    constants.ambientColor = m_lighting.ambientColor;
    constants.fogParams = Math::Vec4f(m_fog.startDistance, m_fog.endDistance, m_fog.density, m_fog.heightDecay);
    constants.fogColor = Math::Vec4f(m_fog.color.x, m_fog.color.y, m_fog.color.z, m_fog.enabled ? 1.0f : 0.0f);
    constants.cameraPos = Math::Vec4f(camera.GetCameraPosGfx().x, camera.GetCameraPosGfx().y, camera.GetCameraPosGfx().z, 0.0f);
    constants.lightMvp = Math::Mat4f::Identity();

    // 1. Draw Terrain & Runway
    const Math::Vec3f originRelGfx = Math::CoordinateFrames::NedToGraphics(-camera.GetCameraPosNed());
    const Math::Mat4f worldModel = Math::Mat4f::Translation(originRelGfx);

    constants.mvp = worldModel * viewProj;
    cmdList->SetGraphicsRoot32BitConstants(0, 56, &constants, 0);

    terrain.Draw(cmdList);
    runway.Draw(cmdList);

    // 2. Draw Aircraft Model
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

    cmdList->SetGraphicsRoot32BitConstants(0, 56, &constants, 0);
    aircraftModel.Draw(cmdList, aircraftRootTransform, viewProj, 0);
}

} // namespace LokYS::Graphics
