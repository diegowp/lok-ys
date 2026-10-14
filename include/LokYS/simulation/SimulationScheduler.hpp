#pragma once

#include "LokYS/simulation/SimulationSnapshot.hpp"
#include <functional>
#include <algorithm>

namespace LokYS::Simulation {

class SimulationScheduler {
public:
    explicit SimulationScheduler(f64 fixedTimestepSec = 1.0 / 120.0, u32 maxSubSteps = 5)
        : m_fixedTimestep(fixedTimestepSec)
        , m_maxSubSteps(maxSubSteps)
    {}

    void SetFixedTimestep(f64 dt) noexcept {
        m_fixedTimestep = std::max(dt, 1e-4);
    }
    [[nodiscard]] f64 GetFixedTimestep() const noexcept { return m_fixedTimestep; }

    void SetMaxSubSteps(u32 maxSteps) noexcept { m_maxSubSteps = maxSteps; }
    [[nodiscard]] u32 GetMaxSubSteps() const noexcept { return m_maxSubSteps; }

    void SetTimeScale(f64 scale) noexcept { m_timeScale = std::max(0.0, scale); }
    [[nodiscard]] f64 GetTimeScale() const noexcept { return m_timeScale; }

    void Pause() noexcept { m_paused = true; }
    void Resume() noexcept { m_paused = false; }
    void TogglePause() noexcept { m_paused = !m_paused; }
    [[nodiscard]] bool IsPaused() const noexcept { return m_paused; }

    void SingleStep() noexcept {
        m_singleStepRequested = true;
    }

    void Reset() noexcept {
        m_accumulator = 0.0;
        m_simTime = 0.0;
        m_tickCount = 0;
        m_paused = false;
        m_singleStepRequested = false;
        m_previousSnapshot = {};
        m_currentSnapshot = {};
    }

    // StepCallback signature: void(f64 dt, u64 tick, f64 simTime)
    template <typename StepFn>
    u32 Update(f64 wallDeltaTime, StepFn&& onStep) {
        if (m_singleStepRequested) {
            m_singleStepRequested = false;
            m_simTime += m_fixedTimestep;
            m_tickCount++;
            onStep(m_fixedTimestep, m_tickCount, m_simTime);
            return 1;
        }

        if (m_paused) {
            return 0;
        }

        // Clamp large frame lags (e.g. window drag/drop) to 100ms
        f64 clampedWallDt = std::min(wallDeltaTime, 0.1);
        m_accumulator += clampedWallDt * m_timeScale;

        u32 stepsExecuted = 0;
        while (m_accumulator >= m_fixedTimestep && stepsExecuted < m_maxSubSteps) {
            m_simTime += m_fixedTimestep;
            m_tickCount++;

            onStep(m_fixedTimestep, m_tickCount, m_simTime);

            m_accumulator -= m_fixedTimestep;
            stepsExecuted++;
        }

        // If sub-steps capped out, discard excess accumulator to prevent spiral of death
        if (stepsExecuted >= m_maxSubSteps && m_accumulator > m_fixedTimestep) {
            m_accumulator = 0.0;
        }

        return stepsExecuted;
    }

    void PublishSnapshot(const SimulationSnapshot& snapshot) noexcept {
        m_previousSnapshot = m_currentSnapshot;
        m_currentSnapshot = snapshot;
    }

    [[nodiscard]] f64 GetInterpolationAlpha() const noexcept {
        if (m_fixedTimestep <= 0.0) return 0.0;
        return std::clamp(m_accumulator / m_fixedTimestep, 0.0, 1.0);
    }

    [[nodiscard]] SimulationSnapshot GetInterpolatedSnapshot() const noexcept {
        return SimulationSnapshot::Interpolate(m_previousSnapshot, m_currentSnapshot, GetInterpolationAlpha());
    }

    [[nodiscard]] const SimulationSnapshot& GetCurrentSnapshot() const noexcept {
        return m_currentSnapshot;
    }
    [[nodiscard]] const SimulationSnapshot& GetPreviousSnapshot() const noexcept {
        return m_previousSnapshot;
    }

    [[nodiscard]] f64 GetSimTime() const noexcept { return m_simTime; }
    [[nodiscard]] u64 GetTickCount() const noexcept { return m_tickCount; }

private:
    f64 m_fixedTimestep{ 1.0 / 120.0 }; // 120 Hz
    u32 m_maxSubSteps{ 5 };
    f64 m_timeScale{ 1.0 };
    f64 m_accumulator{ 0.0 };
    f64 m_simTime{ 0.0 };
    u64 m_tickCount{ 0 };

    bool m_paused{ false };
    bool m_singleStepRequested{ false };

    SimulationSnapshot m_previousSnapshot{};
    SimulationSnapshot m_currentSnapshot{};
};

} // namespace LokYS::Simulation
