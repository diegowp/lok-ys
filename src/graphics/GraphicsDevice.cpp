#include "LokYS/graphics/GraphicsDevice.hpp"
#include <algorithm>

using Microsoft::WRL::ComPtr;

namespace LokYS {

GraphicsDevice::GraphicsDevice() = default;

GraphicsDevice::~GraphicsDevice() {
    Shutdown();
}

bool GraphicsDevice::Initialize(HWND hwnd, u32 width, u32 height) {
    if (m_initialized) {
        return true;
    }

    m_width = width;
    m_height = height;

    m_viewport = { 0.0f, 0.0f, static_cast<f32>(width), static_cast<f32>(height), 0.0f, 1.0f };
    m_scissorRect = { 0, 0, static_cast<LONG>(width), static_cast<LONG>(height) };

    if (!CreateDeviceAndQueue()) {
        return false;
    }

    if (!CreateSwapChain(hwnd, width, height)) {
        return false;
    }

    if (!CreateDescriptorHeapsAndRTVs()) {
        return false;
    }

    if (!CreateDepthStencil(width, height)) {
        return false;
    }

    if (!CreateCommandObjects()) {
        return false;
    }

    if (!CreateSyncObjects()) {
        return false;
    }

    m_gpuReport = GpuCapabilityProber::Probe(m_device.Get(), m_adapter.Get());

    m_initialized = true;
    return true;
}

void GraphicsDevice::Shutdown() {
    if (m_initialized) {
        WaitForGpu();

        for (u32 i = 0; i < FRAME_COUNT; ++i) {
            m_renderTargets[i].Reset();
            m_commandAllocators[i].Reset();
        }

        m_depthStencil.Reset();
        m_dsvHeap.Reset();
        m_commandList.Reset();

        m_rtvHeap.Reset();
        m_swapChain.Reset();
        m_commandQueue.Reset();
        m_device.Reset();
        m_adapter.Reset();
        m_dxgiFactory.Reset();

        if (m_fenceEvent) {
            CloseHandle(m_fenceEvent);
            m_fenceEvent = nullptr;
        }

        m_fence.Reset();
        m_initialized = false;
    }
}

bool GraphicsDevice::CreateDeviceAndQueue() {
    UINT dxgiFlags = 0;
#if defined(_DEBUG)
    ComPtr<ID3D12Debug> debugController;
    if (SUCCEEDED(D3D12GetDebugInterface(IID_PPV_ARGS(&debugController)))) {
        debugController->EnableDebugLayer();
        dxgiFlags |= DXGI_CREATE_FACTORY_DEBUG;
    }
#endif

    HRESULT hr = CreateDXGIFactory2(dxgiFlags, IID_PPV_ARGS(&m_dxgiFactory));
    if (FAILED(hr)) {
        hr = CreateDXGIFactory2(0, IID_PPV_ARGS(&m_dxgiFactory));
        if (FAILED(hr)) {
            return false;
        }
    }

    // High performance adapter enumeration
    ComPtr<IDXGIFactory6> factory6;
    if (SUCCEEDED(m_dxgiFactory.As(&factory6))) {
        for (UINT i = 0;
             SUCCEEDED(factory6->EnumAdapterByGpuPreference(
                 i, DXGI_GPU_PREFERENCE_HIGH_PERFORMANCE, IID_PPV_ARGS(&m_adapter)));
             ++i) {
            DXGI_ADAPTER_DESC1 desc{};
            m_adapter->GetDesc1(&desc);

            if (desc.Flags & DXGI_ADAPTER_FLAG_SOFTWARE) {
                continue;
            }

            if (SUCCEEDED(D3D12CreateDevice(m_adapter.Get(), D3D_FEATURE_LEVEL_11_0, IID_PPV_ARGS(&m_device)))) {
                m_adapterName = desc.Description;
                m_dedicatedVRAM = desc.DedicatedVideoMemory;
                break;
            }
        }
    }

    if (!m_device) {
        for (UINT i = 0; SUCCEEDED(m_dxgiFactory->EnumAdapters1(i, &m_adapter)); ++i) {
            DXGI_ADAPTER_DESC1 desc{};
            m_adapter->GetDesc1(&desc);

            if (desc.Flags & DXGI_ADAPTER_FLAG_SOFTWARE) {
                continue;
            }

            if (SUCCEEDED(D3D12CreateDevice(m_adapter.Get(), D3D_FEATURE_LEVEL_11_0, IID_PPV_ARGS(&m_device)))) {
                m_adapterName = desc.Description;
                m_dedicatedVRAM = desc.DedicatedVideoMemory;
                break;
            }
        }
    }

    if (!m_device) {
        return false;
    }

    // Create Direct Command Queue
    D3D12_COMMAND_QUEUE_DESC queueDesc{};
    queueDesc.Flags = D3D12_COMMAND_QUEUE_FLAG_NONE;
    queueDesc.Type = D3D12_COMMAND_LIST_TYPE_DIRECT;

    if (FAILED(m_device->CreateCommandQueue(&queueDesc, IID_PPV_ARGS(&m_commandQueue)))) {
        return false;
    }

    return true;
}

bool GraphicsDevice::CreateSwapChain(HWND hwnd, u32 width, u32 height) {
    DXGI_SWAP_CHAIN_DESC1 swapChainDesc{};
    swapChainDesc.BufferCount = FRAME_COUNT;
    swapChainDesc.Width = width;
    swapChainDesc.Height = height;
    swapChainDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
    swapChainDesc.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
    swapChainDesc.SwapEffect = DXGI_SWAP_EFFECT_FLIP_DISCARD;
    swapChainDesc.SampleDesc.Count = 1;
    swapChainDesc.SampleDesc.Quality = 0;

    ComPtr<IDXGISwapChain1> swapChain1;
    HRESULT hr = m_dxgiFactory->CreateSwapChainForHwnd(
        m_commandQueue.Get(),
        hwnd,
        &swapChainDesc,
        nullptr,
        nullptr,
        &swapChain1);

    if (FAILED(hr)) {
        return false;
    }

    m_dxgiFactory->MakeWindowAssociation(hwnd, DXGI_MWA_NO_ALT_ENTER);

    if (FAILED(swapChain1.As(&m_swapChain))) {
        return false;
    }

    m_frameIndex = m_swapChain->GetCurrentBackBufferIndex();
    return true;
}

bool GraphicsDevice::CreateDescriptorHeapsAndRTVs() {
    D3D12_DESCRIPTOR_HEAP_DESC rtvHeapDesc{};
    rtvHeapDesc.NumDescriptors = FRAME_COUNT;
    rtvHeapDesc.Type = D3D12_DESCRIPTOR_HEAP_TYPE_RTV;
    rtvHeapDesc.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_NONE;

    if (FAILED(m_device->CreateDescriptorHeap(&rtvHeapDesc, IID_PPV_ARGS(&m_rtvHeap)))) {
        return false;
    }

    m_rtvDescriptorSize = m_device->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_RTV);

    D3D12_CPU_DESCRIPTOR_HANDLE rtvHandle = m_rtvHeap->GetCPUDescriptorHandleForHeapStart();
    for (u32 n = 0; n < FRAME_COUNT; ++n) {
        if (FAILED(m_swapChain->GetBuffer(n, IID_PPV_ARGS(&m_renderTargets[n])))) {
            return false;
        }
        m_device->CreateRenderTargetView(m_renderTargets[n].Get(), nullptr, rtvHandle);
        rtvHandle.ptr += m_rtvDescriptorSize;
    }

    // Create DSV Heap
    D3D12_DESCRIPTOR_HEAP_DESC dsvHeapDesc{};
    dsvHeapDesc.NumDescriptors = 1;
    dsvHeapDesc.Type = D3D12_DESCRIPTOR_HEAP_TYPE_DSV;
    dsvHeapDesc.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_NONE;

    if (FAILED(m_device->CreateDescriptorHeap(&dsvHeapDesc, IID_PPV_ARGS(&m_dsvHeap)))) {
        return false;
    }

    m_dsvDescriptorSize = m_device->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_DSV);

    return true;
}

bool GraphicsDevice::CreateDepthStencil(u32 width, u32 height) {
    D3D12_RESOURCE_DESC depthDesc{};
    depthDesc.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D;
    depthDesc.Alignment = 0;
    depthDesc.Width = width;
    depthDesc.Height = height;
    depthDesc.DepthOrArraySize = 1;
    depthDesc.MipLevels = 1;
    depthDesc.Format = DXGI_FORMAT_D32_FLOAT;
    depthDesc.SampleDesc.Count = 1;
    depthDesc.SampleDesc.Quality = 0;
    depthDesc.Layout = D3D12_TEXTURE_LAYOUT_UNKNOWN;
    depthDesc.Flags = D3D12_RESOURCE_FLAG_ALLOW_DEPTH_STENCIL;

    D3D12_CLEAR_VALUE clearValue{};
    clearValue.Format = DXGI_FORMAT_D32_FLOAT;
    clearValue.DepthStencil.Depth = 1.0f;
    clearValue.DepthStencil.Stencil = 0;

    D3D12_HEAP_PROPERTIES heapProps{};
    heapProps.Type = D3D12_HEAP_TYPE_DEFAULT;
    heapProps.CPUPageProperty = D3D12_CPU_PAGE_PROPERTY_UNKNOWN;
    heapProps.MemoryPoolPreference = D3D12_MEMORY_POOL_UNKNOWN;
    heapProps.CreationNodeMask = 1;
    heapProps.VisibleNodeMask = 1;

    if (FAILED(m_device->CreateCommittedResource(
            &heapProps,
            D3D12_HEAP_FLAG_NONE,
            &depthDesc,
            D3D12_RESOURCE_STATE_DEPTH_WRITE,
            &clearValue,
            IID_PPV_ARGS(&m_depthStencil)))) {
        return false;
    }

    D3D12_DEPTH_STENCIL_VIEW_DESC dsvDesc{};
    dsvDesc.Format = DXGI_FORMAT_D32_FLOAT;
    dsvDesc.ViewDimension = D3D12_DSV_DIMENSION_TEXTURE2D;
    dsvDesc.Flags = D3D12_DSV_FLAG_NONE;

    m_device->CreateDepthStencilView(m_depthStencil.Get(), &dsvDesc, m_dsvHeap->GetCPUDescriptorHandleForHeapStart());
    return true;
}

D3D12_CPU_DESCRIPTOR_HANDLE GraphicsDevice::GetDepthStencilView() const noexcept {
    return m_dsvHeap->GetCPUDescriptorHandleForHeapStart();
}

bool GraphicsDevice::CreateCommandObjects() {
    for (u32 n = 0; n < FRAME_COUNT; ++n) {
        if (FAILED(m_device->CreateCommandAllocator(
                D3D12_COMMAND_LIST_TYPE_DIRECT, IID_PPV_ARGS(&m_commandAllocators[n])))) {
            return false;
        }
    }

    if (FAILED(m_device->CreateCommandList(
            0,
            D3D12_COMMAND_LIST_TYPE_DIRECT,
            m_commandAllocators[m_frameIndex].Get(),
            nullptr,
            IID_PPV_ARGS(&m_commandList)))) {
        return false;
    }

    // Command list starts open; close it until first frame render
    if (FAILED(m_commandList->Close())) {
        return false;
    }

    return true;
}

bool GraphicsDevice::CreateSyncObjects() {
    if (FAILED(m_device->CreateFence(0, D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS(&m_fence)))) {
        return false;
    }

    m_fenceValues.fill(0);
    m_currentFenceValue = 1;

    m_fenceEvent = CreateEventW(nullptr, FALSE, FALSE, nullptr);
    if (!m_fenceEvent) {
        return false;
    }

    return true;
}

void GraphicsDevice::BeginFrame(const f32 clearColor[4]) {
    // Reset allocator and list for current backbuffer
    m_commandAllocators[m_frameIndex]->Reset();
    m_commandList->Reset(m_commandAllocators[m_frameIndex].Get(), nullptr);

    // Transition backbuffer: PRESENT -> RENDER_TARGET
    D3D12_RESOURCE_BARRIER barrier{};
    barrier.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
    barrier.Flags = D3D12_RESOURCE_BARRIER_FLAG_NONE;
    barrier.Transition.pResource = m_renderTargets[m_frameIndex].Get();
    barrier.Transition.StateBefore = D3D12_RESOURCE_STATE_PRESENT;
    barrier.Transition.StateAfter = D3D12_RESOURCE_STATE_RENDER_TARGET;
    barrier.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
    m_commandList->ResourceBarrier(1, &barrier);

    D3D12_CPU_DESCRIPTOR_HANDLE rtvHandle = m_rtvHeap->GetCPUDescriptorHandleForHeapStart();
    rtvHandle.ptr += static_cast<SIZE_T>(m_frameIndex) * m_rtvDescriptorSize;

    D3D12_CPU_DESCRIPTOR_HANDLE dsvHandle = m_dsvHeap->GetCPUDescriptorHandleForHeapStart();

    m_commandList->RSSetViewports(1, &m_viewport);
    m_commandList->RSSetScissorRects(1, &m_scissorRect);
    m_commandList->OMSetRenderTargets(1, &rtvHandle, FALSE, &dsvHandle);
    m_commandList->ClearRenderTargetView(rtvHandle, clearColor, 0, nullptr);
    m_commandList->ClearDepthStencilView(dsvHandle, D3D12_CLEAR_FLAG_DEPTH, 1.0f, 0, 0, nullptr);
}


void GraphicsDevice::EndFrame(bool vsync) {
    // Transition backbuffer: RENDER_TARGET -> PRESENT
    D3D12_RESOURCE_BARRIER barrier{};
    barrier.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
    barrier.Flags = D3D12_RESOURCE_BARRIER_FLAG_NONE;
    barrier.Transition.pResource = m_renderTargets[m_frameIndex].Get();
    barrier.Transition.StateBefore = D3D12_RESOURCE_STATE_RENDER_TARGET;
    barrier.Transition.StateAfter = D3D12_RESOURCE_STATE_PRESENT;
    barrier.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
    m_commandList->ResourceBarrier(1, &barrier);

    m_commandList->Close();

    ID3D12CommandList* ppCommandLists[] = { m_commandList.Get() };
    m_commandQueue->ExecuteCommandLists(1, ppCommandLists);

    m_swapChain->Present(vsync ? 1 : 0, 0);

    MoveToNextFrame();
}

void GraphicsDevice::MoveToNextFrame() {
    const u64 currentFenceValue = m_currentFenceValue;
    m_commandQueue->Signal(m_fence.Get(), currentFenceValue);
    m_fenceValues[m_frameIndex] = currentFenceValue;
    m_currentFenceValue++;

    m_frameIndex = m_swapChain->GetCurrentBackBufferIndex();

    if (m_fence->GetCompletedValue() < m_fenceValues[m_frameIndex]) {
        m_fence->SetEventOnCompletion(m_fenceValues[m_frameIndex], m_fenceEvent);
        WaitForSingleObject(m_fenceEvent, INFINITE);
    }
}

void GraphicsDevice::WaitForGpu() {
    if (m_commandQueue && m_fence && m_fenceEvent) {
        m_commandQueue->Signal(m_fence.Get(), m_currentFenceValue);
        m_fence->SetEventOnCompletion(m_currentFenceValue, m_fenceEvent);
        WaitForSingleObject(m_fenceEvent, INFINITE);
        m_currentFenceValue++;
    }
}

void GraphicsDevice::OnResize(u32 width, u32 height) {
    if (!m_device || !m_swapChain) {
        return;
    }
    if (width == 0 || height == 0) {
        return;
    }
    if (width == m_width && height == m_height) {
        return;
    }

    WaitForGpu();

    for (u32 i = 0; i < FRAME_COUNT; ++i) {
        m_renderTargets[i].Reset();
        m_fenceValues[i] = m_fenceValues[m_frameIndex];
    }

    m_swapChain->ResizeBuffers(FRAME_COUNT, width, height, DXGI_FORMAT_R8G8B8A8_UNORM, 0);
    m_width = width;
    m_height = height;
    m_frameIndex = m_swapChain->GetCurrentBackBufferIndex();

    m_viewport = { 0.0f, 0.0f, static_cast<f32>(width), static_cast<f32>(height), 0.0f, 1.0f };
    m_scissorRect = { 0, 0, static_cast<LONG>(width), static_cast<LONG>(height) };

    D3D12_CPU_DESCRIPTOR_HANDLE rtvHandle = m_rtvHeap->GetCPUDescriptorHandleForHeapStart();
    for (u32 n = 0; n < FRAME_COUNT; ++n) {
        m_swapChain->GetBuffer(n, IID_PPV_ARGS(&m_renderTargets[n]));
        m_device->CreateRenderTargetView(m_renderTargets[n].Get(), nullptr, rtvHandle);
        rtvHandle.ptr += m_rtvDescriptorSize;
    }

    m_depthStencil.Reset();
    static_cast<void>(CreateDepthStencil(width, height));
}


} // namespace LokYS
