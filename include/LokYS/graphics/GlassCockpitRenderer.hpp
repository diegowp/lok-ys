#pragma once

#include "LokYS/core/Types.hpp"
#include "LokYS/avionics/AvionicsTypes.hpp"
#include "LokYS/navigation/NavTypes.hpp"
#include "LokYS/autopilot/AutopilotTypes.hpp"
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
#include <string>

namespace LokYS::Graphics {

enum class CockpitDisplayMode : u8 {
    AnalogSixPack,    // Classic 2x3 electro-mechanical instrument cluster (M14)
    GlassCockpitPFD,  // Modern Primary Flight Display with synthetic vector tapes & FMA (M21)
    MinimalHUD        // Minimal / Off
};

class GlassCockpitRenderer {
public:
    GlassCockpitRenderer() = default;
    ~GlassCockpitRenderer() { Shutdown(); }

    [[nodiscard]] bool Initialize(ID3D12Device* device);
    void Shutdown();

    /// @brief Renders the complete digital Primary Flight Display (PFD) with Autopilot FMA banner.
    void Render(
        ID3D12GraphicsCommandList* cmdList,
        u32 screenWidth,
        u32 screenHeight,
        const Avionics::SixPackState& avionics,
        const Navigation::NavReceiverState& nav1,
        const Autopilot::AutopilotStatus& apStatus,
        bool visible,
        f32 panelScale = 1.0f) noexcept;

    void SetPanelOpacity(f32 opacity) noexcept { m_opacity = opacity; }
    [[nodiscard]] f32 GetPanelOpacity() const noexcept { return m_opacity; }

private:
    bool CreateRootSignature(ID3D12Device* device);
    bool CreatePipelineState(ID3D12Device* device);
    bool CreateBuffers(ID3D12Device* device);

    void BuildPfdGeometry(
        f32 pfdX,
        f32 pfdY,
        f32 pfdW,
        f32 pfdH,
        const Avionics::SixPackState& avi,
        const Navigation::NavReceiverState& nav1,
        const Autopilot::AutopilotStatus& ap);

    // Sub-components of the PFD
    void DrawBezelAndHousing(f32 x, f32 y, f32 w, f32 h);
    void DrawFmaBanner(f32 x, f32 y, f32 w, f32 h, const Autopilot::AutopilotStatus& ap);
    void DrawAttitudeDirectorIndicator(
        f32 cx, f32 cy, f32 w, f32 h,
        f32 pitchDeg, f32 rollDeg, f32 slipBall,
        const Autopilot::FlightDirectorCues& fd);
    void DrawAirspeedTape(
        f32 x, f32 y, f32 w, f32 h,
        f32 iasKnots, f32 verticalSpeedFpm);
    void DrawAltimeterTape(
        f32 x, f32 y, f32 w, f32 h,
        f32 altFeet, f32 targetAltFeet, f32 kollsmanInHg);
    void DrawVerticalSpeedTape(
        f32 x, f32 y, f32 w, f32 h,
        f32 vsFpm, f32 targetVsFpm, bool vsModeActive);
    void DrawHsiCompass(
        f32 cx, f32 cy, f32 radius,
        f32 headingDeg, f32 targetHeadingDeg,
        const Navigation::NavReceiverState& nav1);
    void DrawTopFrequenciesHeader(
        f32 x, f32 y, f32 w,
        const Navigation::NavReceiverState& nav1);

    // Vector drawing primitives
    void AddQuad(const Math::Vec3f& p0, const Math::Vec3f& p1, const Math::Vec3f& p2, const Math::Vec3f& p3, const Math::Vec4f& col);
    void AddTriangle(const Math::Vec3f& p0, const Math::Vec3f& p1, const Math::Vec3f& p2, const Math::Vec4f& col);
    void AddCircleFill(f32 cx, f32 cy, f32 radius, int segments, const Math::Vec4f& col);
    void AddRing(f32 cx, f32 cy, f32 innerR, f32 outerR, int segments, const Math::Vec4f& col);
    void AddArcRing(f32 cx, f32 cy, f32 innerR, f32 outerR, f32 startDeg, f32 endDeg, int segments, const Math::Vec4f& col);
    void AddLine(f32 x0, f32 y0, f32 x1, f32 y1, f32 thickness, const Math::Vec4f& col);
    void AddTick(f32 cx, f32 cy, f32 radius, f32 length, f32 angleDeg, f32 thickness, const Math::Vec4f& col);

    // Vector procedural stroke font
    void DrawChar(char c, f32 x, f32 y, f32 size, const Math::Vec4f& col, f32 thickness = 1.5f);
    void DrawString(const std::string& str, f32 x, f32 y, f32 size, const Math::Vec4f& col, f32 thickness = 1.5f);

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

    static constexpr size_t MaxVertices = 24576;
    static constexpr size_t MaxIndices = 49152;

    bool m_initialized{ false };
    f32 m_opacity{ 0.96f };
};

} // namespace LokYS::Graphics
