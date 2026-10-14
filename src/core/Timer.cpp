#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif

#include "LokYS/core/Timer.hpp"
#include <windows.h>
#include <algorithm>

namespace LokYS {

Timer::Timer() {
    LARGE_INTEGER freq{};
    QueryPerformanceFrequency(&freq);
    m_frequency = freq.QuadPart;

    Reset();
}

void Timer::Reset() {
    LARGE_INTEGER count{};
    QueryPerformanceCounter(&count);

    m_startTime = count.QuadPart;
    m_previousTime = count.QuadPart;
    m_currentTime = count.QuadPart;
    m_deltaTime = 0.0;
    m_totalTime = 0.0;
    m_frameCount = 0;
    m_fpsTimer = 0.0;
    m_fpsCounter = 0;
    m_fps = 0.0;
}

void Timer::Tick() {
    LARGE_INTEGER count{};
    QueryPerformanceCounter(&count);
    m_currentTime = count.QuadPart;

    m_deltaTime = static_cast<f64>(m_currentTime - m_previousTime) / static_cast<f64>(m_frequency);
    m_previousTime = m_currentTime;

    // Guard against negative delta or extreme pause steps
    if (m_deltaTime < 0.0) {
        m_deltaTime = 0.0;
    }
    m_deltaTime = std::min(m_deltaTime, 0.1); // Clamp to 100ms max

    m_totalTime = static_cast<f64>(m_currentTime - m_startTime) / static_cast<f64>(m_frequency);
    ++m_frameCount;

    m_fpsTimer += m_deltaTime;
    ++m_fpsCounter;
    if (m_fpsTimer >= 1.0) {
        m_fps = static_cast<f64>(m_fpsCounter) / m_fpsTimer;
        m_fpsCounter = 0;
        m_fpsTimer = 0.0;
    }
}

} // namespace LokYS
