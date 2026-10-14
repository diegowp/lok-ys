#pragma once

#include "LokYS/core/Types.hpp"
#include "LokYS/math/Vector3.hpp"
#include "LokYS/math/Matrix4x4.hpp"

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <d3d12.h>
#include <wrl/client.h>

namespace LokYS::Graphics {

class ShadowMap {
public:
    explicit ShadowMap(u32 resolution = 2048);
    ~ShadowMap() = default;

    ShadowMap(const ShadowMap&) = delete;
    ShadowMap& operator=(const ShadowMap&) = delete;
    ShadowMap(ShadowMap&&) noexcept = default;
    ShadowMap& operator=(ShadowMap&&) noexcept = default;

    [[nodiscard]] bool Initialize(ID3D12Device* device);
    void Shutdown();

    /// @brief Computes directional light View-Projection matrix for shadow mapping.
    /// @param sunDir Direction vector pointing towards the ground (light propagation vector).
    /// @param targetPos World center of shadow focus (aircraft center).
    /// @param boxWidth Lateral and vertical span of the orthographic projection frustum.
    /// @param nearZ Near clipping plane along light ray.
    /// @param farZ Far clipping plane along light ray.
    void UpdateLightMatrix(
        const Math::Vec3f& sunDir,
        const Math::Vec3f& targetPos,
        f32 boxWidth = 36.0f,
        f32 boxHeight = 36.0f,
        f32 nearZ = -60.0f,
        f32 farZ = 60.0f) noexcept;

    /// @brief Begins shadow pass: sets depth viewport/scissor, transitions to DEPTH_WRITE, clears depth to 1.0.
    void Begin(ID3D12GraphicsCommandList* cmdList) noexcept;

    /// @brief Ends shadow pass: transitions depth buffer to PIXEL_SHADER_RESOURCE for main forward pass.
    void End(ID3D12GraphicsCommandList* cmdList) noexcept;

    [[nodiscard]] const Math::Mat4f& GetLightViewProjMatrix() const noexcept { return m_lightViewProj; }
    [[nodiscard]] ID3D12DescriptorHeap* GetSrvHeap() const noexcept { return m_srvHeap.Get(); }
    [[nodiscard]] D3D12_GPU_DESCRIPTOR_HANDLE GetSrvGpuHandle() const noexcept {
        return m_srvHeap ? m_srvHeap->GetGPUDescriptorHandleForHeapStart() : D3D12_GPU_DESCRIPTOR_HANDLE{0};
    }
    [[nodiscard]] u32 GetResolution() const noexcept { return m_resolution; }
    [[nodiscard]] ID3D12RootSignature* GetRootSignature() const noexcept { return m_rootSig.Get(); }
    [[nodiscard]] ID3D12PipelineState* GetPipelineState() const noexcept { return m_pso.Get(); }

private:
    bool CreateResources(ID3D12Device* device);
    bool CreatePipelines(ID3D12Device* device);

    u32 m_resolution{ 2048 };

    Microsoft::WRL::ComPtr<ID3D12Resource> m_shadowTexture;
    Microsoft::WRL::ComPtr<ID3D12DescriptorHeap> m_dsvHeap;
    Microsoft::WRL::ComPtr<ID3D12DescriptorHeap> m_srvHeap;

    Microsoft::WRL::ComPtr<ID3D12RootSignature> m_rootSig;
    Microsoft::WRL::ComPtr<ID3D12PipelineState> m_pso;

    D3D12_VIEWPORT m_viewport{};
    D3D12_RECT m_scissorRect{};

    Math::Mat4f m_lightView{ Math::Mat4f::Identity() };
    Math::Mat4f m_lightProj{ Math::Mat4f::Identity() };
    Math::Mat4f m_lightViewProj{ Math::Mat4f::Identity() };

    bool m_initialized{ false };
};

} // namespace LokYS::Graphics
