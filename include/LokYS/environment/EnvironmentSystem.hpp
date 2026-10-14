#pragma once

#include "LokYS/environment/AtmosphereTypes.hpp"
#include "LokYS/environment/AtmosphereModel.hpp"
#include "LokYS/environment/WindModel.hpp"
#include "LokYS/math/Quaternion.hpp"

namespace LokYS::Environment {

class EnvironmentSystem {
public:
    EnvironmentSystem();
    ~EnvironmentSystem() = default;

    /// @brief Initializes default standard atmospheric and wind configuration.
    void Initialize();

    /// @brief Configures predefined standard weather scenarios.
    void ApplyPreset(WeatherPreset preset) noexcept;
    void CyclePreset() noexcept;

    /// @brief Main per-timestep update advancing atmosphere, wind, and turbulence.
    void Update(
        const Math::Vec3d& aircraftPosNed,
        const Math::Quat4d& aircraftAttitude,
        f64 trueAirspeedMps,
        f64 dt) noexcept;

    /// @brief Computes relative aerodynamic velocity in the aircraft Body (FRD) frame.
    /// V_rel = V_aircraft_body - V_wind_body
    [[nodiscard]] Math::Vec3d ComputeRelativeVelocityBody(
        const Math::Vec3d& aircraftVelocityBody) const noexcept;

    [[nodiscard]] const EnvironmentState& GetState() const noexcept { return m_state; }
    [[nodiscard]] const AtmosphericState& GetAtmosphere() const noexcept { return m_state.atmosphere; }
    [[nodiscard]] WeatherPreset GetActivePreset() const noexcept { return m_state.activePreset; }
    [[nodiscard]] std::wstring GetPresetName() const noexcept;

    void SetDeltaIsaK(f64 deltaK) noexcept { m_deltaIsaK = deltaK; }
    [[nodiscard]] f64 GetDeltaIsaK() const noexcept { return m_deltaIsaK; }

    void SetSeaLevelPressurePa(f64 qnhPa) noexcept { m_seaLevelPressurePa = qnhPa; }
    [[nodiscard]] f64 GetSeaLevelPressurePa() const noexcept { return m_seaLevelPressurePa; }

    WindModel& GetWindModel() noexcept { return m_windModel; }
    [[nodiscard]] const WindModel& GetWindModel() const noexcept { return m_windModel; }

private:
    WindModel m_windModel;
    EnvironmentState m_state{};

    f64 m_deltaIsaK{ 0.0 };
    f64 m_seaLevelPressurePa{ Constants::STANDARD_SEA_LEVEL_PRESS_PA };
    f64 m_runwayHeadingDeg{ 0.0 }; // Runway 36 True North
};

} // namespace LokYS::Environment
