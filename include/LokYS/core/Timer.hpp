#pragma once

#include "LokYS/core/Types.hpp"

namespace LokYS {

class Timer {
public:
    Timer();

    void Reset();
    void Tick();

    [[nodiscard]] f64 GetDeltaTime() const noexcept { return m_deltaTime; }
    [[nodiscard]] f64 GetTotalTime() const noexcept { return m_totalTime; }
    [[nodiscard]] u64 GetFrameCount() const noexcept { return m_frameCount; }
    [[nodiscard]] f64 GetFPS() const noexcept { return m_fps; }

private:
    i64 m_frequency{0};
    i64 m_startTime{0};
    i64 m_previousTime{0};
    i64 m_currentTime{0};
    f64 m_deltaTime{0.0};
    f64 m_totalTime{0.0};
    u64 m_frameCount{0};

    f64 m_fpsTimer{0.0};
    u32 m_fpsCounter{0};
    f64 m_fps{0.0};
};

} // namespace LokYS
