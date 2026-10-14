#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif

#include "LokYS/core/EngineContext.hpp"
#include <windows.h>
#include <d3d12.h>
#include <dxgi1_6.h>
#include <wrl/client.h>
#include <iostream>

using Microsoft::WRL::ComPtr;

namespace LokYS {

EngineContext::EngineContext() = default;

EngineContext::~EngineContext() {
    if (m_initialized) {
        Shutdown();
    }
}

bool EngineContext::Initialize() {
    if (m_initialized) {
        return true;
    }

    if (!ProbeSystemHardware()) {
        return false;
    }

    if (!ProbeDirect3D12()) {
        return false;
    }

    m_initialized = true;
    return true;
}

void EngineContext::Shutdown() {
    m_initialized = false;
}

bool EngineContext::ProbeSystemHardware() {
    SYSTEM_INFO sysInfo{};
    GetSystemInfo(&sysInfo);
    m_capabilities.cpuLogicalProcessors = sysInfo.dwNumberOfProcessors;
    m_capabilities.cpuCores = sysInfo.dwNumberOfProcessors; // Conservative baseline
    return true;
}

bool EngineContext::ProbeDirect3D12() {
    UINT dxgiFlags = 0;
#if defined(_DEBUG)
    dxgiFlags |= DXGI_CREATE_FACTORY_DEBUG;
#endif

    ComPtr<IDXGIFactory4> dxgiFactory;
    HRESULT hr = CreateDXGIFactory2(dxgiFlags, IID_PPV_ARGS(&dxgiFactory));
    if (FAILED(hr)) {
        // Fallback without debug flag if debug layer unavailable
        hr = CreateDXGIFactory2(0, IID_PPV_ARGS(&dxgiFactory));
        if (FAILED(hr)) {
            return false;
        }
    }

    ComPtr<IDXGIAdapter1> selectedAdapter;
    ComPtr<IDXGIFactory6> factory6;
    if (SUCCEEDED(dxgiFactory.As(&factory6))) {
        for (UINT i = 0;
             SUCCEEDED(factory6->EnumAdapterByGpuPreference(
                 i, DXGI_GPU_PREFERENCE_HIGH_PERFORMANCE, IID_PPV_ARGS(&selectedAdapter)));
             ++i) {
            DXGI_ADAPTER_DESC1 desc{};
            selectedAdapter->GetDesc1(&desc);

            if (desc.Flags & DXGI_ADAPTER_FLAG_SOFTWARE) {
                continue;
            }

            // Probe D3D12 creation capability with null device pointer
            if (SUCCEEDED(D3D12CreateDevice(selectedAdapter.Get(), D3D_FEATURE_LEVEL_11_0, _uuidof(ID3D12Device), nullptr))) {
                m_capabilities.d3d12Supported = true;
                m_capabilities.adapterName = desc.Description;
                m_capabilities.dedicatedVideoMemory = desc.DedicatedVideoMemory;
                return true;
            }
        }
    }

    // Fallback: standard adapter enumeration
    for (UINT i = 0; SUCCEEDED(dxgiFactory->EnumAdapters1(i, &selectedAdapter)); ++i) {
        DXGI_ADAPTER_DESC1 desc{};
        selectedAdapter->GetDesc1(&desc);

        if (desc.Flags & DXGI_ADAPTER_FLAG_SOFTWARE) {
            continue;
        }

        if (SUCCEEDED(D3D12CreateDevice(selectedAdapter.Get(), D3D_FEATURE_LEVEL_11_0, _uuidof(ID3D12Device), nullptr))) {
            m_capabilities.d3d12Supported = true;
            m_capabilities.adapterName = desc.Description;
            m_capabilities.dedicatedVideoMemory = desc.DedicatedVideoMemory;
            return true;
        }
    }

    return false;
}

} // namespace LokYS
