#pragma once

#include "LokYS/core/Types.hpp"
#include "LokYS/graphics/Mesh.hpp"
#include "LokYS/math/Vector3.hpp"

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <d3d12.h>
#include <vector>

namespace LokYS::Graphics {

struct AlsStrobeStatus {
    int activeRabbitIndex{ 0 };     // [0, 20]: Currently firing centerline strobe
    bool reilFlashed{ false };      // Synchronized threshold REIL strobe flash
    f32 rabbitProgress{ 0.0f };     // Normalized progress [0.0, 1.0] along 720m approach path
};

class ApproachLightingSystem {
public:
    ApproachLightingSystem() = default;
    ~ApproachLightingSystem() { Shutdown(); }

    ApproachLightingSystem(const ApproachLightingSystem&) = delete;
    ApproachLightingSystem& operator=(const ApproachLightingSystem&) = delete;

    [[nodiscard]] bool Initialize(ID3D12Device* device);
    void Shutdown();

    /// @brief Advances sequenced flashing strobes ("The Rabbit") and REIL timers.
    void Update(f64 dt, f64 simTime, ID3D12Device* device) noexcept;

    void Update(f64 dt, ID3D12Device* device = nullptr) noexcept {
        m_simTime += dt;
        Update(dt, m_simTime, device);
    }

    /// @brief Draws static approach light bars, crossbars, green threshold bar, and red terminating bars.
    void DrawStaticBars(ID3D12GraphicsCommandList* cmdList) const noexcept;

    /// @brief Draws active dynamic sequenced strobes ("The Rabbit" traveling flash wave and REILs).
    void DrawDynamicStrobes(ID3D12GraphicsCommandList* cmdList) const noexcept;

    [[nodiscard]] const AlsStrobeStatus& GetStatus() const noexcept { return m_status; }
    [[nodiscard]] int GetActiveStrobeIndex() const noexcept { return m_status.activeRabbitIndex; }
    [[nodiscard]] bool IsReilFlashing() const noexcept { return m_status.reilFlashed; }
    [[nodiscard]] constexpr u32 GetStationCount() const noexcept { return 24; }
    [[nodiscard]] constexpr u32 GetStrobeCount() const noexcept { return 21; }

private:
    void BuildStaticBarsMesh(ID3D12Device* device);
    void BuildDynamicStrobesMeshes(ID3D12Device* device);

    Mesh m_staticBarsMesh;
    Mesh m_rabbitStrobes[21];
    Mesh m_reilStrobesMesh;

    AlsStrobeStatus m_status{};
    f64 m_simTime{ 0.0 };

    bool m_initialized{ false };
};

} // namespace LokYS::Graphics
