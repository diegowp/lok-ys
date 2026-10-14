#pragma once

#include "LokYS/core/Types.hpp"
#include <string>
#include <array>

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <d3d12.h>
#include <dxgi1_6.h>
#include <wrl/client.h>

#include "LokYS/graphics/GpuCapabilities.hpp"

namespace LokYS {

class GraphicsDevice {
public:
    static constexpr u32 FRAME_COUNT = 3; // Triple-buffered swapchain

    GraphicsDevice();
    ~GraphicsDevice();

    GraphicsDevice(const GraphicsDevice&) = delete;
    GraphicsDevice& operator=(const GraphicsDevice&) = delete;
    GraphicsDevice(GraphicsDevice&&) noexcept = default;
    GraphicsDevice& operator=(GraphicsDevice&&) noexcept = default;

    [[nodiscard]] bool Initialize(HWND hwnd, u32 width, u32 height);
    void Shutdown();

    void BeginFrame(const f32 clearColor[4]);
    void EndFrame(bool vsync = true);
    void BindMainRenderTarget();
    void OnResize(u32 width, u32 height);

    void WaitForGpu();

    [[nodiscard]] ID3D12Device* GetDevice() const noexcept { return m_device.Get(); }
    [[nodiscard]] ID3D12GraphicsCommandList* GetCommandList() const noexcept { return m_commandList.Get(); }
    [[nodiscard]] u32 GetCurrentFrameIndex() const noexcept { return m_frameIndex; }
    [[nodiscard]] const std::wstring& GetAdapterName() const noexcept { return m_adapterName; }
    [[nodiscard]] u64 GetDedicatedVRAM() const noexcept { return m_dedicatedVRAM; }
    [[nodiscard]] const GpuCapabilitiesReport& GetGpuReport() const noexcept { return m_gpuReport; }
    [[nodiscard]] u32 GetWidth() const noexcept { return m_width; }
    [[nodiscard]] u32 GetHeight() const noexcept { return m_height; }
    [[nodiscard]] f32 GetAspectRatio() const noexcept {
        return static_cast<f32>(m_width) / static_cast<f32>(m_height > 0 ? m_height : 1);
    }
    [[nodiscard]] D3D12_CPU_DESCRIPTOR_HANDLE GetDepthStencilView() const noexcept;

private:
    bool CreateDeviceAndQueue();
    bool CreateSwapChain(HWND hwnd, u32 width, u32 height);
    bool CreateDescriptorHeapsAndRTVs();
    bool CreateDepthStencil(u32 width, u32 height);
    bool CreateCommandObjects();
    bool CreateSyncObjects();

    void MoveToNextFrame();

    Microsoft::WRL::ComPtr<IDXGIFactory4> m_dxgiFactory;
    Microsoft::WRL::ComPtr<IDXGIAdapter1> m_adapter;
    Microsoft::WRL::ComPtr<ID3D12Device> m_device;
    Microsoft::WRL::ComPtr<ID3D12CommandQueue> m_commandQueue;

    Microsoft::WRL::ComPtr<IDXGISwapChain3> m_swapChain;
    Microsoft::WRL::ComPtr<ID3D12DescriptorHeap> m_rtvHeap;
    std::array<Microsoft::WRL::ComPtr<ID3D12Resource>, FRAME_COUNT> m_renderTargets;
    u32 m_rtvDescriptorSize{0};

    Microsoft::WRL::ComPtr<ID3D12DescriptorHeap> m_dsvHeap;
    Microsoft::WRL::ComPtr<ID3D12Resource> m_depthStencil;
    u32 m_dsvDescriptorSize{0};

    std::array<Microsoft::WRL::ComPtr<ID3D12CommandAllocator>, FRAME_COUNT> m_commandAllocators;
    Microsoft::WRL::ComPtr<ID3D12GraphicsCommandList> m_commandList;

    Microsoft::WRL::ComPtr<ID3D12Fence> m_fence;
    std::array<u64, FRAME_COUNT> m_fenceValues{};
    u64 m_currentFenceValue{1};
    HANDLE m_fenceEvent{nullptr};

    u32 m_frameIndex{0};
    u32 m_width{1280};
    u32 m_height{720};
    D3D12_VIEWPORT m_viewport{};
    D3D12_RECT m_scissorRect{};

    std::wstring m_adapterName{};
    u64 m_dedicatedVRAM{0};
    GpuCapabilitiesReport m_gpuReport{};
    bool m_initialized{false};
};


} // namespace LokYS
