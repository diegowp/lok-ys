#pragma once

#include "LokYS/core/Types.hpp"
#include "LokYS/avionics/AvionicsTypes.hpp"
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
#include <wrl/client.h>
#include <vector>

namespace LokYS::Graphics {

class AvionicsRenderer {
public:
    AvionicsRenderer() = default;
    ~AvionicsRenderer() { Shutdown(); }

    [[nodiscard]] bool Initialize(ID3D12Device* device);
    void Shutdown();

    /// Renders the complete Analog Six-Pack instrument cluster overlay
    /// @param cmdList Direct3D 12 graphics command list
    /// @param screenWidth Viewport width in pixels
    /// @param screenHeight Viewport height in pixels
    /// @param state Current SixPackState telemetry
    /// @param visible Whether the instrument panel is currently visible
    /// @param panelScale Zoom / scaling factor for the panel (default 1.0)
    void Render(
        ID3D12GraphicsCommandList* cmdList,
        u32 screenWidth,
        u32 screenHeight,
        const Avionics::SixPackState& state,
        bool visible,
        f32 panelScale = 1.0f) noexcept;

    void SetPanelOpacity(f32 opacity) noexcept { m_opacity = opacity; }
    [[nodiscard]] f32 GetPanelOpacity() const noexcept { return m_opacity; }

private:
    bool CreateRootSignature(ID3D12Device* device);
    bool CreatePipelineState(ID3D12Device* device);
    bool CreateBuffers(ID3D12Device* device);

    void BuildPanelGeometry(
        f32 centerX,
        f32 centerY,
        f32 dialRadius,
        const Avionics::SixPackState& state);

    // Bezel & background plate
    void DrawInstrumentHousing(f32 cx, f32 cy, f32 radius);

    // Individual instrument dial faces
    void DrawAirspeedIndicator(f32 cx, f32 cy, f32 r, const Avionics::AirspeedIndicatorState& asi);
    void DrawAttitudeIndicator(f32 cx, f32 cy, f32 r, const Avionics::AttitudeIndicatorState& ai);
    void DrawAltimeter(f32 cx, f32 cy, f32 r, const Avionics::AltimeterState& alt);
    void DrawTurnCoordinator(f32 cx, f32 cy, f32 r, const Avionics::TurnCoordinatorState& tc);
    void DrawHeadingIndicator(f32 cx, f32 cy, f32 r, const Avionics::HeadingIndicatorState& hdg);
    void DrawVerticalSpeedIndicator(f32 cx, f32 cy, f32 r, const Avionics::VerticalSpeedIndicatorState& vsi);

    // Vector drawing primitives
    void AddQuad(const Math::Vec3f& p0, const Math::Vec3f& p1, const Math::Vec3f& p2, const Math::Vec3f& p3, const Math::Vec4f& col);
    void AddTriangle(const Math::Vec3f& p0, const Math::Vec3f& p1, const Math::Vec3f& p2, const Math::Vec4f& col);
    void AddCircleFill(f32 cx, f32 cy, f32 radius, int segments, const Math::Vec4f& col);
    void AddRing(f32 cx, f32 cy, f32 innerR, f32 outerR, int segments, const Math::Vec4f& col);
    void AddArcRing(f32 cx, f32 cy, f32 innerR, f32 outerR, f32 startDeg, f32 endDeg, int segments, const Math::Vec4f& col);
    void AddLine(f32 x0, f32 y0, f32 x1, f32 y1, f32 thickness, const Math::Vec4f& col);
    void AddTick(f32 cx, f32 cy, f32 radius, f32 length, f32 angleDeg, f32 thickness, const Math::Vec4f& col);
    void AddNeedle(f32 cx, f32 cy, f32 length, f32 baseW, f32 angleDeg, const Math::Vec4f& col, f32 counterLen = 8.0f);

    Microsoft::WRL::ComPtr<ID3D12RootSignature> m_rootSignature;
    Microsoft::WRL::ComPtr<ID3D12PipelineState> m_pipelineState;

    Microsoft::WRL::ComPtr<ID3D12Resource> m_vertexBuffer;
    Microsoft::WRL::ComPtr<ID3D12Resource> m_indexBuffer;
    D3D12_VERTEX_BUFFER_VIEW m_vbView{};
    D3D12_INDEX_BUFFER_VIEW m_ibView{};

    Vertex3D* m_mappedVertices{ nullptr };
    u32* m_mappedIndices{ nullptr };

    std::vector<Vertex3D> m_vertices;
    std::vector<u32> m_indices;

    static constexpr size_t MaxVertices = 16384;
    static constexpr size_t MaxIndices = 32768;

    bool m_initialized{ false };
    f32 m_opacity{ 0.96f };
};

} // namespace LokYS::Graphics
