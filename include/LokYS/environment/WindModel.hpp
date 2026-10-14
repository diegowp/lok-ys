#pragma once

#include "LokYS/environment/AtmosphereTypes.hpp"
#include <random>

namespace LokYS::Environment {

class WindModel {
public:
    WindModel();
    explicit WindModel(u32 randomSeed);

    void SetWindConfig(const WindConfig& config) noexcept { m_windConfig = config; }
    [[nodiscard]] const WindConfig& GetWindConfig() const noexcept { return m_windConfig; }

    void SetTurbulenceConfig(const TurbulenceConfig& config) noexcept { m_turbConfig = config; }
    [[nodiscard]] const TurbulenceConfig& GetTurbulenceConfig() const noexcept { return m_turbConfig; }

    /// @brief Resets turbulence filter states to zero.
    void ResetTurbulence() noexcept;

    /// @brief Advances turbulence simulation and evaluates 3D wind velocity vector in NED frame.
    /// @param aircraftPosNed Current aircraft position in local NED navigation coordinates.
    /// @param trueAirspeedMps Current aircraft true airspeed in m/s (for Dryden filter frequency scaling).
    /// @param dt Simulation timestep in seconds.
    [[nodiscard]] Math::Vec3d EvaluateTotalWindNed(
        const Math::Vec3d& aircraftPosNed,
        f64 trueAirspeedMps,
        f64 dt) noexcept;

    /// @brief Computes steady horizontal wind vector at a given altitude without turbulence.
    [[nodiscard]] Math::Vec3d CalculateSteadyWindNed(f64 altitudeM) const noexcept;

    /// @brief Computes thermal updraft/downdraft vertical velocity at a given NED position.
    [[nodiscard]] f64 CalculateThermalUpdraftMps(const Math::Vec3d& aircraftPosNed) const noexcept;

    /// @brief Decomposes horizontal wind into headwind and crosswind components relative to runway heading.
    /// @param windSpeedMps Total horizontal wind speed in m/s.
    /// @param windFromDeg Direction wind is blowing from in degrees.
    /// @param rwyHeadingDeg Runway heading in degrees.
    /// @param outHeadwindMps Output headwind (positive = headwind, negative = tailwind).
    /// @param outCrosswindMps Output crosswind (positive = from right, negative = from left).
    static void DecomposeRunwayWind(
        f64 windSpeedMps,
        f64 windFromDeg,
        f64 rwyHeadingDeg,
        f64& outHeadwindMps,
        f64& outCrosswindMps) noexcept;

    [[nodiscard]] Math::Vec3d GetLastSteadyWindNed() const noexcept { return m_lastSteadyWindNed; }
    [[nodiscard]] Math::Vec3d GetLastGustNed() const noexcept { return m_gustNed; }
    [[nodiscard]] f64 GetLastThermalUpdraftMps() const noexcept { return m_lastThermalUpdraft; }

private:
    WindConfig m_windConfig{};
    TurbulenceConfig m_turbConfig{};

    Math::Vec3d m_gustNed{ 0.0, 0.0, 0.0 };
    Math::Vec3d m_lastSteadyWindNed{ 0.0, 0.0, 0.0 };
    f64 m_lastThermalUpdraft{ 0.0 };

    // Standard Gaussian pseudo-random generator
    std::mt19937 m_rng;
    std::normal_distribution<f64> m_normalDist{ 0.0, 1.0 };
};

} // namespace LokYS::Environment
