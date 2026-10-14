#pragma once

#include "LokYS/core/Types.hpp"
#include "LokYS/graphics/Camera.hpp"
#include "LokYS/graphics/Mesh.hpp"
#include "LokYS/graphics/GltfModel.hpp"
#include "LokYS/math/Vector3.hpp"
#include "LokYS/math/Quaternion.hpp"
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

struct SceneLighting {
    Math::Vec3f sunDirection{ 0.4f, 0.8f, 0.4f }; // Sun vector pointing towards ground
    Math::Vec4f sunColor{ 1.0f, 0.98f, 0.90f, 1.0f }; // Warm directional sunlight
    Math::Vec4f ambientColor{ 0.40f, 0.45f, 0.55f, 1.0f }; // Sky ambient light
};

class Renderer3D {
public:
    Renderer3D() = default;
    ~Renderer3D() = default;

    [[nodiscard]] bool Initialize(ID3D12Device* device);
    void Shutdown();

    void SetLighting(const SceneLighting& lighting) noexcept { m_lighting = lighting; }
    [[nodiscard]] const SceneLighting& GetLighting() const noexcept { return m_lighting; }

    void Render(
        ID3D12GraphicsCommandList* cmdList,
        const Camera& camera,
        const Mesh& terrain,
        const Mesh& runway,
        const Mesh& aircraft,
        const Math::Vec3d& aircraftPosNed,
        const Math::Quat4d& aircraftAttitude) const noexcept;

    void Render(
        ID3D12GraphicsCommandList* cmdList,
        const Camera& camera,
        const Mesh& terrain,
        const Mesh& runway,
        const GltfModel& aircraftModel,
        const Math::Vec3d& aircraftPosNed,
        const Math::Quat4d& aircraftAttitude) const noexcept;

private:
    bool CreateRootSignature(ID3D12Device* device);
    bool CreatePipelineState(ID3D12Device* device);

    Microsoft::WRL::ComPtr<ID3D12RootSignature> m_rootSignature;
    Microsoft::WRL::ComPtr<ID3D12PipelineState> m_pipelineState;

    SceneLighting m_lighting{};
    bool m_initialized{ false };
};

} // namespace LokYS::Graphics
