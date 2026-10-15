#pragma once

#include "LokYS/core/Types.hpp"
#include "LokYS/core/GameState.hpp"
#include "LokYS/input/ActionMapping.hpp"
#include "LokYS/input/ConfigPersistence.hpp"
#include "LokYS/graphics/Mesh.hpp"

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
#include <optional>

namespace LokYS::Graphics {

enum class MenuScreen : u8 {
    PreFlight,
    Pause,
    Settings
};

class MenuRenderer {
public:
    static constexpr size_t MaxVertices = 16384;
    static constexpr size_t MaxIndices = 49152;

    MenuRenderer() = default;
    ~MenuRenderer() { Shutdown(); }

    [[nodiscard]] bool Initialize(ID3D12Device* device);
    void Shutdown();

    // Mouse & Keyboard Event Ingestion
    void UpdateMouse(f32 mouseX, f32 mouseY, bool leftDown, bool leftClicked) noexcept;
    void OnKeyDown(u32 vkey, bool shift, bool ctrl, bool alt) noexcept;

    // Rendering entry points
    void RenderPreFlight(
        ID3D12GraphicsCommandList* cmdList,
        u32 screenWidth,
        u32 screenHeight,
        const std::wstring& aircraftName,
        const std::string& airportName,
        const std::string& airportIcao,
        const std::string& runwayIdent,
        f64 runwayLengthM,
        f64 elevationM,
        const std::string& weatherPresetName,
        const std::string& spawnModeName,
        f64 fuelPercentage) noexcept;

    void RenderPause(
        ID3D12GraphicsCommandList* cmdList,
        u32 screenWidth,
        u32 screenHeight) noexcept;

    void RenderSettings(
        ID3D12GraphicsCommandList* cmdList,
        u32 screenWidth,
        u32 screenHeight,
        const Input::SimulatorConfig& config) noexcept;

    // Action Triggers queried by main loop
    [[nodiscard]] bool WasStartFlightTriggered() noexcept {
        bool val = m_startFlightTriggered;
        m_startFlightTriggered = false;
        return val;
    }
    [[nodiscard]] bool WasResumeFlightTriggered() noexcept {
        bool val = m_resumeFlightTriggered;
        m_resumeFlightTriggered = false;
        return val;
    }
    [[nodiscard]] bool WasRestartFlightTriggered() noexcept {
        bool val = m_restartFlightTriggered;
        m_restartFlightTriggered = false;
        return val;
    }
    [[nodiscard]] bool WasSettingsTriggered() noexcept {
        bool val = m_settingsTriggered;
        m_settingsTriggered = false;
        return val;
    }
    [[nodiscard]] bool WasExitToMenuTriggered() noexcept {
        bool val = m_exitToMenuTriggered;
        m_exitToMenuTriggered = false;
        return val;
    }
    [[nodiscard]] bool WasCycleAircraftTriggered() noexcept {
        bool val = m_cycleAircraftTriggered;
        m_cycleAircraftTriggered = false;
        return val;
    }
    [[nodiscard]] bool WasCycleAirportTriggered() noexcept {
        bool val = m_cycleAirportTriggered;
        m_cycleAirportTriggered = false;
        return val;
    }
    [[nodiscard]] bool WasCycleWeatherTriggered() noexcept {
        bool val = m_cycleWeatherTriggered;
        m_cycleWeatherTriggered = false;
        return val;
    }
    [[nodiscard]] bool WasToggleSpawnModeTriggered() noexcept {
        bool val = m_toggleSpawnModeTriggered;
        m_toggleSpawnModeTriggered = false;
        return val;
    }
    [[nodiscard]] bool WasResetBindingsTriggered() noexcept {
        bool val = m_resetBindingsTriggered;
        m_resetBindingsTriggered = false;
        return val;
    }
    [[nodiscard]] bool WasSaveConfigTriggered() noexcept {
        bool val = m_saveConfigTriggered;
        m_saveConfigTriggered = false;
        return val;
    }

    [[nodiscard]] std::optional<Input::InputAction> GetListeningAction() const noexcept {
        return m_listeningAction;
    }
    void ClearListeningAction() noexcept { m_listeningAction.reset(); }

    [[nodiscard]] f32 GetSelectedVolume() const noexcept { return m_volume; }
    void SetSelectedVolume(f32 vol) noexcept { m_volume = vol; }

    [[nodiscard]] u32 GetSelectedDisplayMode() const noexcept { return m_displayMode; }
    void SetSelectedDisplayMode(u32 mode) noexcept { m_displayMode = mode; }

private:
    bool CreateRootSignature(ID3D12Device* device);
    bool CreatePipelineState(ID3D12Device* device);
    bool CreateBuffers(ID3D12Device* device);

    // Vector drawing primitives
    void AddQuad(const Math::Vec3f& p0, const Math::Vec3f& p1, const Math::Vec3f& p2, const Math::Vec3f& p3, const Math::Vec4f& col);
    void AddLine(f32 x0, f32 y0, f32 x1, f32 y1, f32 thickness, const Math::Vec4f& col);
    void DrawChar(char c, f32 x, f32 y, f32 size, const Math::Vec4f& col, f32 thickness = 1.5f);
    void DrawString(const std::string& str, f32 x, f32 y, f32 size, const Math::Vec4f& col, f32 thickness = 1.5f);

    bool DrawButton(
        f32 x, f32 y, f32 w, f32 h,
        const std::string& label,
        const Math::Vec4f& baseColor,
        const Math::Vec4f& hoverColor,
        const Math::Vec4f& textColor,
        f32 fontSize = 13.0f);

    void FlushBuffer(ID3D12GraphicsCommandList* cmdList, u32 screenWidth, u32 screenHeight);

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

    bool m_initialized{ false };

    // Mouse cursor coordinates & state
    f32 m_mouseX{ 0.0f };
    f32 m_mouseY{ 0.0f };
    bool m_leftDown{ false };
    bool m_leftClicked{ false };

    // Interactive Action triggers
    bool m_startFlightTriggered{ false };
    bool m_resumeFlightTriggered{ false };
    bool m_restartFlightTriggered{ false };
    bool m_settingsTriggered{ false };
    bool m_exitToMenuTriggered{ false };
    bool m_cycleAircraftTriggered{ false };
    bool m_cycleAirportTriggered{ false };
    bool m_cycleWeatherTriggered{ false };
    bool m_toggleSpawnModeTriggered{ false };
    bool m_resetBindingsTriggered{ false };
    bool m_saveConfigTriggered{ false };

    std::optional<Input::InputAction> m_listeningAction;
    f32 m_volume{ 0.85f };
    u32 m_displayMode{ 0 };
    u32 m_settingsScrollOffset{ 0 };
};

} // namespace LokYS::Graphics
