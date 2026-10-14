#pragma once

#include "LokYS/core/Types.hpp"
#include "LokYS/graphics/Mesh.hpp"
#include "LokYS/math/Vector3.hpp"
#include <array>

namespace LokYS::Graphics {

enum class PapiColor {
    White,
    Red
};

struct PapiStatus {
    std::array<PapiColor, 4> lights{ PapiColor::Red, PapiColor::Red, PapiColor::Red, PapiColor::Red };
    f32 approachAngleDeg{ 0.0f };
    bool inApproachSector{ false };
};

class ScenerySystem {
public:
    ScenerySystem() = default;
    ~ScenerySystem() = default;

    ScenerySystem(const ScenerySystem&) = delete;
    ScenerySystem& operator=(const ScenerySystem&) = delete;

    [[nodiscard]] bool Initialize(ID3D12Device* device);
    void Shutdown();

    /// @brief Updates PAPI glidepath lights and windsock orientation based on camera and wind.
    void Update(
        const Math::Vec3f& cameraPosGfx,
        f64 windSpeedMps,
        f64 windDirectionDeg,
        ID3D12Device* device) noexcept;

    /// @brief Draws static airport buildings (control tower, maintenance hangars, terminal).
    void DrawBuildings(ID3D12GraphicsCommandList* cmdList) const noexcept;

    /// @brief Draws runway edge lights and green/red threshold bars.
    void DrawRunwayLights(ID3D12GraphicsCommandList* cmdList) const noexcept;

    /// @brief Draws 4-unit PAPI light array.
    void DrawPapi(ID3D12GraphicsCommandList* cmdList) const noexcept;

    /// @brief Draws airport windsock.
    void DrawWindsock(ID3D12GraphicsCommandList* cmdList) const noexcept;

    [[nodiscard]] const PapiStatus& GetPapiStatus() const noexcept { return m_papiStatus; }
    [[nodiscard]] f32 GetWindsockYawRad() const noexcept { return m_windsockYawRad; }

    /// @brief Helper to compute PAPI light configuration from position.
    [[nodiscard]] static PapiStatus ComputePapiStatus(const Math::Vec3f& eyeGfx) noexcept;

private:
    void BuildBuildings(ID3D12Device* device);
    void BuildRunwayLights(ID3D12Device* device);
    void RebuildPapiMesh(ID3D12Device* device);
    void RebuildWindsockMesh(ID3D12Device* device);

    Mesh m_buildingsMesh;
    Mesh m_edgeLightsMesh;
    Mesh m_papiMesh;
    Mesh m_windsockMesh;

    PapiStatus m_papiStatus{};
    f32 m_windsockYawRad{ 0.0f };
    f32 m_windsockDroop{ 0.0f };

    f64 m_prevWindSpeed{ -1.0 };
    f64 m_prevWindDir{ -1.0 };
    std::array<PapiColor, 4> m_prevPapiColors{ PapiColor::White, PapiColor::White, PapiColor::White, PapiColor::White };

    bool m_initialized{ false };
};

} // namespace LokYS::Graphics
