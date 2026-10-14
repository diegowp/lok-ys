#pragma once

#include "LokYS/core/Types.hpp"
#include <string>

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <d3d12.h>
#include <dxgi1_6.h>

namespace LokYS {

struct GpuCapabilitiesReport {
    std::wstring adapterName{};
    u32 vendorId{0};
    u32 deviceId{0};
    u64 dedicatedVideoMemory{0};
    u64 sharedSystemMemory{0};

    // Direct3D 12 Feature Levels & Shader Model
    D3D_FEATURE_LEVEL maxFeatureLevel{D3D_FEATURE_LEVEL_11_0};
    D3D_SHADER_MODEL highestShaderModel{D3D_SHADER_MODEL_5_1};

    // Architectural Tiers
    D3D12_RESOURCE_BINDING_TIER resourceBindingTier{D3D12_RESOURCE_BINDING_TIER_1};
    D3D12_CONSERVATIVE_RASTERIZATION_TIER conservativeRasterTier{D3D12_CONSERVATIVE_RASTERIZATION_TIER_NOT_SUPPORTED};
    D3D12_RAYTRACING_TIER raytracingTier{D3D12_RAYTRACING_TIER_NOT_SUPPORTED};
    D3D12_VARIABLE_SHADING_RATE_TIER vrsTier{D3D12_VARIABLE_SHADING_RATE_TIER_NOT_SUPPORTED};
    D3D12_MESH_SHADER_TIER meshShaderTier{D3D12_MESH_SHADER_TIER_NOT_SUPPORTED};

    // Features
    bool rovSupported{false};
    bool standardSwizzle64KBSupported{false};
    bool enhancedBarriersSupported{false};

    // Current Memory Budget (bytes)
    u64 localMemoryBudget{0};
    u64 localMemoryUsage{0};

    [[nodiscard]] std::wstring ToString() const;
};

class GpuCapabilityProber {
public:
    static GpuCapabilitiesReport Probe(ID3D12Device* device, IDXGIAdapter1* adapter);
};

} // namespace LokYS
