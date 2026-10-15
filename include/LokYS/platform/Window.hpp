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

namespace LokYS {

struct WindowDesc {
    std::wstring title{L"LokYS Flight Simulator"};
    u32 width{1280};
    u32 height{720};
    bool resizable{true};
    bool startMaximized{false};
};

class Window {
public:
    Window();
    ~Window();

    Window(const Window&) = delete;
    Window& operator=(const Window&) = delete;
    Window(Window&& other) noexcept;
    Window& operator=(Window&& other) noexcept;

    [[nodiscard]] bool Create(const WindowDesc& desc = {});
    void Destroy();

    [[nodiscard]] bool ProcessMessages();

    [[nodiscard]] bool IsOpen() const noexcept { return m_hwnd != nullptr && !m_shouldClose; }
    [[nodiscard]] HWND GetHwnd() const noexcept { return m_hwnd; }
    [[nodiscard]] u32 GetWidth() const noexcept { return m_width; }
    [[nodiscard]] u32 GetHeight() const noexcept { return m_height; }
    [[nodiscard]] bool IsMinimized() const noexcept { return m_minimized; }
    [[nodiscard]] f32 GetDpiScale() const noexcept { return m_dpiScale; }
    [[nodiscard]] f32 GetAndResetMouseWheelDelta() noexcept {
        f32 d = m_mouseWheelDelta;
        m_mouseWheelDelta = 0.0f;
        return d;
    }

    void SetShouldClose(bool close) noexcept { m_shouldClose = close; }

private:
    static LRESULT CALLBACK StaticWndProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam);
    LRESULT HandleMessage(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam);

    HWND m_hwnd{nullptr};
    HINSTANCE m_hinstance{nullptr};
    std::wstring m_className{L"LokYS_FlightSim_WindowClass"};
    u32 m_width{1280};
    u32 m_height{720};
    f32 m_dpiScale{1.0f};
    f32 m_mouseWheelDelta{0.0f};
    bool m_minimized{false};
    bool m_shouldClose{false};
    bool m_classRegistered{false};
};

} // namespace LokYS
