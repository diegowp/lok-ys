#include "LokYS/platform/Window.hpp"
#include <utility>

namespace LokYS {

Window::Window()
    : m_hinstance(GetModuleHandleW(nullptr))
{
    // Enable Per-Monitor v2 DPI awareness if supported
    typedef BOOL(WINAPI * PFN_SetProcessDpiAwarenessContext)(DPI_AWARENESS_CONTEXT);
    HMODULE hUser32 = GetModuleHandleW(L"user32.dll");
    if (hUser32) {
        auto pfnSetContext = reinterpret_cast<PFN_SetProcessDpiAwarenessContext>(
            GetProcAddress(hUser32, "SetProcessDpiAwarenessContext"));
        if (pfnSetContext) {
            pfnSetContext(DPI_AWARENESS_CONTEXT_PER_MONITOR_AWARE_V2);
        }
    }
}

Window::~Window() {
    Destroy();
}

Window::Window(Window&& other) noexcept
    : m_hwnd(other.m_hwnd)
    , m_hinstance(other.m_hinstance)
    , m_className(std::move(other.m_className))
    , m_width(other.m_width)
    , m_height(other.m_height)
    , m_dpiScale(other.m_dpiScale)
    , m_minimized(other.m_minimized)
    , m_shouldClose(other.m_shouldClose)
    , m_classRegistered(other.m_classRegistered)
{
    other.m_hwnd = nullptr;
    other.m_classRegistered = false;
    if (m_hwnd) {
        SetWindowLongPtrW(m_hwnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(this));
    }
}

Window& Window::operator=(Window&& other) noexcept {
    if (this != &other) {
        Destroy();

        m_hwnd = other.m_hwnd;
        m_hinstance = other.m_hinstance;
        m_className = std::move(other.m_className);
        m_width = other.m_width;
        m_height = other.m_height;
        m_dpiScale = other.m_dpiScale;
        m_minimized = other.m_minimized;
        m_shouldClose = other.m_shouldClose;
        m_classRegistered = other.m_classRegistered;

        other.m_hwnd = nullptr;
        other.m_classRegistered = false;

        if (m_hwnd) {
            SetWindowLongPtrW(m_hwnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(this));
        }
    }
    return *this;
}

bool Window::Create(const WindowDesc& desc) {
    if (m_hwnd) {
        return true;
    }

    m_width = desc.width;
    m_height = desc.height;

    WNDCLASSEXW wc{};
    wc.cbSize = sizeof(WNDCLASSEXW);
    wc.style = CS_HREDRAW | CS_VREDRAW | CS_OWNDC;
    wc.lpfnWndProc = StaticWndProc;
    wc.hInstance = m_hinstance;
    wc.hIcon = LoadIconW(nullptr, MAKEINTRESOURCEW(32512));   // IDI_APPLICATION
    wc.hCursor = LoadCursorW(nullptr, MAKEINTRESOURCEW(32512)); // IDC_ARROW
    wc.hbrBackground = nullptr; // Direct3D 12 presents client area
    wc.lpszClassName = m_className.c_str();
    wc.hIconSm = LoadIconW(nullptr, MAKEINTRESOURCEW(32512));

    if (!RegisterClassExW(&wc)) {
        DWORD err = GetLastError();
        if (err != ERROR_CLASS_ALREADY_EXISTS) {
            return false;
        }
    }
    m_classRegistered = true;

    DWORD style = WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX;
    if (desc.resizable) {
        style |= WS_THICKFRAME | WS_MAXIMIZEBOX;
    }
    DWORD exStyle = WS_EX_APPWINDOW;

    RECT rect{ 0, 0, static_cast<LONG>(m_width), static_cast<LONG>(m_height) };
    AdjustWindowRectEx(&rect, style, FALSE, exStyle);

    LONG windowWidth = rect.right - rect.left;
    LONG windowHeight = rect.bottom - rect.top;

    int screenWidth = GetSystemMetrics(SM_CXSCREEN);
    int screenHeight = GetSystemMetrics(SM_CYSCREEN);
    int posX = std::max<int>(0, (screenWidth - static_cast<int>(windowWidth)) / 2);
    int posY = std::max<int>(0, (screenHeight - static_cast<int>(windowHeight)) / 2);

    m_hwnd = CreateWindowExW(
        exStyle,
        m_className.c_str(),
        desc.title.c_str(),
        style,
        posX,
        posY,
        windowWidth,
        windowHeight,
        nullptr,
        nullptr,
        m_hinstance,
        this);

    if (!m_hwnd) {
        return false;
    }

    ShowWindow(m_hwnd, desc.startMaximized ? SW_SHOWMAXIMIZED : SW_SHOWNORMAL);
    UpdateWindow(m_hwnd);

    return true;
}

void Window::Destroy() {
    if (m_hwnd) {
        SetWindowLongPtrW(m_hwnd, GWLP_USERDATA, 0);
        DestroyWindow(m_hwnd);
        m_hwnd = nullptr;
    }

    if (m_classRegistered) {
        UnregisterClassW(m_className.c_str(), m_hinstance);
        m_classRegistered = false;
    }

    m_shouldClose = false;
}

bool Window::ProcessMessages() {
    MSG msg{};
    while (PeekMessageW(&msg, nullptr, 0, 0, PM_REMOVE)) {
        if (msg.message == WM_QUIT) {
            m_shouldClose = true;
            return false;
        }
        TranslateMessage(&msg);
        DispatchMessageW(&msg);
    }
    return !m_shouldClose;
}

LRESULT CALLBACK Window::StaticWndProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam) {
    if (msg == WM_NCCREATE) {
        auto* pCreate = reinterpret_cast<CREATESTRUCTW*>(lParam);
        SetWindowLongPtrW(hwnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(pCreate->lpCreateParams));
        auto* pThis = reinterpret_cast<Window*>(pCreate->lpCreateParams);
        if (pThis) {
            pThis->m_hwnd = hwnd;
            return pThis->HandleMessage(hwnd, msg, wParam, lParam);
        }
    }

    auto* pThis = reinterpret_cast<Window*>(GetWindowLongPtrW(hwnd, GWLP_USERDATA));
    if (pThis) {
        return pThis->HandleMessage(hwnd, msg, wParam, lParam);
    }

    return DefWindowProcW(hwnd, msg, wParam, lParam);
}

LRESULT Window::HandleMessage(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam) {
    switch (msg) {
    case WM_SIZE: {
        if (wParam == SIZE_MINIMIZED) {
            m_minimized = true;
        } else {
            m_minimized = false;
            m_width = LOWORD(lParam);
            m_height = HIWORD(lParam);
        }
        return 0;
    }

    case WM_DPICHANGED: {
        m_dpiScale = static_cast<f32>(LOWORD(wParam)) / 96.0f;
        auto* prcNewWindow = reinterpret_cast<RECT*>(lParam);
        SetWindowPos(
            hwnd,
            nullptr,
            prcNewWindow->left,
            prcNewWindow->top,
            prcNewWindow->right - prcNewWindow->left,
            prcNewWindow->bottom - prcNewWindow->top,
            SWP_NOZORDER | SWP_NOACTIVATE);
        return 0;
    }

    case WM_CLOSE: {
        m_shouldClose = true;
        DestroyWindow(hwnd);
        return 0;
    }

    case WM_DESTROY: {
        m_hwnd = nullptr;
        PostQuitMessage(0);
        return 0;
    }

    case WM_MOUSEWHEEL: {
        short wheelDelta = GET_WHEEL_DELTA_WPARAM(wParam);
        m_mouseWheelDelta += static_cast<f32>(wheelDelta) / static_cast<f32>(WHEEL_DELTA);
        return 0;
    }

    case WM_ERASEBKGND:
        // Direct3D 12 handles background rendering; avoid flickering
        return 1;

    default:
        break;
    }

    return DefWindowProcW(hwnd, msg, wParam, lParam);
}

} // namespace LokYS
