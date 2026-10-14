#include "LokYS/graphics/GpuCapabilities.hpp"
#include <sstream>
#include <iomanip>
#include <wrl/client.h>

using Microsoft::WRL::ComPtr;

namespace LokYS {

GpuCapabilitiesReport GpuCapabilityProber::Probe(ID3D12Device* device, IDXGIAdapter1* adapter) {
    GpuCapabilitiesReport report{};

    if (adapter) {
        DXGI_ADAPTER_DESC1 desc{};
        if (SUCCEEDED(adapter->GetDesc1(&desc))) {
            report.adapterName = desc.Description;
            report.vendorId = desc.VendorId;
            report.deviceId = desc.DeviceId;
            report.dedicatedVideoMemory = desc.DedicatedVideoMemory;
            report.sharedSystemMemory = desc.SharedSystemMemory;
        }

        ComPtr<IDXGIAdapter3> adapter3;
        if (SUCCEEDED(adapter->QueryInterface(IID_PPV_ARGS(&adapter3)))) {
            DXGI_QUERY_VIDEO_MEMORY_INFO memInfo{};
            if (SUCCEEDED(adapter3->QueryVideoMemoryInfo(0, DXGI_MEMORY_SEGMENT_GROUP_LOCAL, &memInfo))) {
                report.localMemoryBudget = memInfo.Budget;
                report.localMemoryUsage = memInfo.CurrentUsage;
            }
        }
    }

    if (!device) {
        return report;
    }

    // 1. Probe Feature Levels
    static constexpr D3D_FEATURE_LEVEL featureLevels[] = {
        D3D_FEATURE_LEVEL_12_1,
        D3D_FEATURE_LEVEL_12_0,
        D3D_FEATURE_LEVEL_11_1,
        D3D_FEATURE_LEVEL_11_0
    };
    D3D12_FEATURE_DATA_FEATURE_LEVELS levelsData{};
    levelsData.NumFeatureLevels = static_cast<UINT>(std::size(featureLevels));
    levelsData.pFeatureLevelsRequested = featureLevels;
    if (SUCCEEDED(device->CheckFeatureSupport(D3D12_FEATURE_FEATURE_LEVELS, &levelsData, sizeof(levelsData)))) {
        report.maxFeatureLevel = levelsData.MaxSupportedFeatureLevel;
    }

    // 2. Probe Shader Model
    static constexpr D3D_SHADER_MODEL shaderModels[] = {
        D3D_SHADER_MODEL_6_6,
        D3D_SHADER_MODEL_6_5,
        D3D_SHADER_MODEL_6_4,
        D3D_SHADER_MODEL_6_3,
        D3D_SHADER_MODEL_6_2,
        D3D_SHADER_MODEL_6_1,
        D3D_SHADER_MODEL_6_0,
        D3D_SHADER_MODEL_5_1
    };
    for (auto sm : shaderModels) {
        D3D12_FEATURE_DATA_SHADER_MODEL smData{};
        smData.HighestShaderModel = sm;
        if (SUCCEEDED(device->CheckFeatureSupport(D3D12_FEATURE_SHADER_MODEL, &smData, sizeof(smData)))) {
            report.highestShaderModel = smData.HighestShaderModel;
            break;
        }
    }

    // 3. Options (Core D3D12)
    D3D12_FEATURE_DATA_D3D12_OPTIONS options{};
    if (SUCCEEDED(device->CheckFeatureSupport(D3D12_FEATURE_D3D12_OPTIONS, &options, sizeof(options)))) {
        report.resourceBindingTier = options.ResourceBindingTier;
        report.conservativeRasterTier = options.ConservativeRasterizationTier;
        report.rovSupported = (options.ROVsSupported != FALSE);
        report.standardSwizzle64KBSupported = (options.StandardSwizzle64KBSupported != FALSE);
    }

    // 4. Options5 (Raytracing)
    D3D12_FEATURE_DATA_D3D12_OPTIONS5 options5{};
    if (SUCCEEDED(device->CheckFeatureSupport(D3D12_FEATURE_D3D12_OPTIONS5, &options5, sizeof(options5)))) {
        report.raytracingTier = options5.RaytracingTier;
    }

    // 5. Options6 (Variable Rate Shading)
    D3D12_FEATURE_DATA_D3D12_OPTIONS6 options6{};
    if (SUCCEEDED(device->CheckFeatureSupport(D3D12_FEATURE_D3D12_OPTIONS6, &options6, sizeof(options6)))) {
        report.vrsTier = options6.VariableShadingRateTier;
    }

    // 6. Options7 (Mesh Shaders)
    D3D12_FEATURE_DATA_D3D12_OPTIONS7 options7{};
    if (SUCCEEDED(device->CheckFeatureSupport(D3D12_FEATURE_D3D12_OPTIONS7, &options7, sizeof(options7)))) {
        report.meshShaderTier = options7.MeshShaderTier;
    }

    return report;
}

std::wstring GpuCapabilitiesReport::ToString() const {
    std::wstringstream ss;
    ss << L"GPU Adapter: " << adapterName << L"\n";
    ss << L"Vendor ID: 0x" << std::hex << std::uppercase << vendorId
       << L" | Device ID: 0x" << deviceId << std::dec << L"\n";
    ss << L"Dedicated VRAM: " << (dedicatedVideoMemory / (1024 * 1024)) << L" MB\n";
    ss << L"Shared Memory: " << (sharedSystemMemory / (1024 * 1024)) << L" MB\n";
    if (localMemoryBudget > 0) {
        ss << L"VRAM Budget: " << (localMemoryBudget / (1024 * 1024)) << L" MB | Usage: "
           << (localMemoryUsage / (1024 * 1024)) << L" MB\n";
    }

    ss << L"Max Feature Level: ";
    switch (maxFeatureLevel) {
    case D3D_FEATURE_LEVEL_12_1: ss << L"12_1"; break;
    case D3D_FEATURE_LEVEL_12_0: ss << L"12_0"; break;
    case D3D_FEATURE_LEVEL_11_1: ss << L"11_1"; break;
    case D3D_FEATURE_LEVEL_11_0: ss << L"11_0"; break;
    default: ss << L"Unknown"; break;
    }
    ss << L"\n";

    ss << L"Highest Shader Model: ";
    switch (highestShaderModel) {
    case D3D_SHADER_MODEL_6_6: ss << L"SM 6.6"; break;
    case D3D_SHADER_MODEL_6_5: ss << L"SM 6.5"; break;
    case D3D_SHADER_MODEL_6_4: ss << L"SM 6.4"; break;
    case D3D_SHADER_MODEL_6_3: ss << L"SM 6.3"; break;
    case D3D_SHADER_MODEL_6_2: ss << L"SM 6.2"; break;
    case D3D_SHADER_MODEL_6_1: ss << L"SM 6.1"; break;
    case D3D_SHADER_MODEL_6_0: ss << L"SM 6.0"; break;
    default: ss << L"SM 5.1"; break;
    }
    ss << L"\n";

    ss << L"Resource Binding Tier: Tier " << static_cast<u32>(resourceBindingTier) << L"\n";
    ss << L"Raytracing Tier: " << (raytracingTier != D3D12_RAYTRACING_TIER_NOT_SUPPORTED ? L"Supported" : L"None") << L"\n";
    ss << L"Variable Rate Shading: " << (vrsTier != D3D12_VARIABLE_SHADING_RATE_TIER_NOT_SUPPORTED ? L"Supported" : L"None") << L"\n";
    ss << L"Mesh Shader Tier: " << (meshShaderTier != D3D12_MESH_SHADER_TIER_NOT_SUPPORTED ? L"Supported" : L"None") << L"\n";

    return ss.str();
}

} // namespace LokYS
