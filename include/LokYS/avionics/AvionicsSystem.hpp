#pragma once

#include "LokYS/core/Types.hpp"
#include "LokYS/avionics/AvionicsTypes.hpp"
#include "LokYS/physics/RigidBodyState.hpp"
#include "LokYS/math/Vector3.hpp"
#include "LokYS/math/Units.hpp"

namespace LokYS::Avionics {

class AvionicsSystem {
public:
    AvionicsSystem();
    ~AvionicsSystem() = default;

    /// Advances avionics sensor simulation and instrument responses by dt seconds
    /// @param state Current rigid-body flight dynamics state (NED position, attitude, body velocities, angular rates)
    /// @param accelerationBody Linear acceleration in body coordinates [m/s^2]
    /// @param ambientPressurePa Ambient barometric static pressure [Pa]
    /// @param ambientDensityKgM3 Ambient atmospheric air density [kg/m^3]
    /// @param dt Simulation timestep [s]
    void Update(
        const Physics::RigidBodyState& state,
        const Math::Vec3d& accelerationBody,
        f64 ambientPressurePa,
        f64 ambientDensityKgM3,
        f64 dt) noexcept;

    /// Instantly snaps all instruments to current steady-state conditions (e.g. on spawn or reset)
    void ResetToSteadyState(
        const Physics::RigidBodyState& state,
        f64 ambientPressurePa,
        f64 ambientDensityKgM3) noexcept;

    // Inspection
    [[nodiscard]] const SixPackState& GetState() const noexcept { return m_state; }
    [[nodiscard]] const AirspeedIndicatorState& GetAirspeed() const noexcept { return m_state.airspeed; }
    [[nodiscard]] const AttitudeIndicatorState& GetAttitude() const noexcept { return m_state.attitude; }
    [[nodiscard]] const AltimeterState& GetAltimeter() const noexcept { return m_state.altimeter; }
    [[nodiscard]] const TurnCoordinatorState& GetTurnCoordinator() const noexcept { return m_state.turnCoordinator; }
    [[nodiscard]] const HeadingIndicatorState& GetHeading() const noexcept { return m_state.heading; }
    [[nodiscard]] const VerticalSpeedIndicatorState& GetVerticalSpeed() const noexcept { return m_state.verticalSpeed; }

    // Pilot Subscale & Heading Bug Controls
    void SetKollsman(f64 inHg) noexcept;
    void AdjustKollsman(f64 deltaInHg) noexcept;
    [[nodiscard]] f64 GetKollsman() const noexcept { return m_state.altimeter.kollsmanInHg; }

    void SetHeadingBug(f64 bugDeg) noexcept;
    void AdjustHeadingBug(f64 deltaDeg) noexcept;
    [[nodiscard]] f64 GetHeadingBug() const noexcept { return m_state.heading.headingBugDeg; }

    void AlignHeadingWithCompass() noexcept;
    void CageAttitude(bool caged) noexcept;
    void ToggleAttitudeCage() noexcept;

    // Sensor Lag Time Constants (seconds)
    void SetTimeConstants(
        f64 tauAsi,
        f64 tauGyro,
        f64 tauAlt,
        f64 tauTc,
        f64 tauBall,
        f64 tauVsi) noexcept;

private:
    void UpdateAirspeed(f64 speedMps, f64 ambientDensity, f64 dt) noexcept;
    void UpdateAttitude(const Math::Quat4d& attitude, const Math::Vec3d& angularVelBody, f64 dt) noexcept;
    void UpdateAltimeter(f64 staticPressurePa, f64 dt) noexcept;
    void UpdateTurnCoordinator(
        const Math::Vec3d& angularVelBody,
        const Math::Vec3d& accelerationBody,
        f64 rollRad,
        f64 pitchRad,
        f64 dt) noexcept;
    void UpdateHeading(f64 yawRad, f64 dt) noexcept;
    void UpdateVerticalSpeed(f64 verticalVelocityMps, f64 dt) noexcept;

    SixPackState m_state{};

    // Dynamic Sensor Lag Time Constants (in seconds)
    f64 m_tauAsi{ 0.10 };   ///< Pitot-static pneumatic line lag (100 ms)
    f64 m_tauGyro{ 0.05 };  ///< Attitude gyro gimbal response lag (50 ms)
    f64 m_tauAlt{ 0.15 };   ///< Static line aneroid pressure lag (150 ms)
    f64 m_tauTc{ 0.15 };    ///< Canted rate gyro spring-damper lag (150 ms)
    f64 m_tauBall{ 0.25 };  ///< Viscous kerosene fluid ball damping (250 ms)
    f64 m_tauVsi{ 2.50 };   ///< Calibrated metering capillary pneumatic lag (2.5 s standard FAA)

    // Directional Gyro Precession Drift Rate (degrees / second)
    // 3.0 degrees per 15 minutes = 3.0 / 900 = 0.003333 deg/s
    f64 m_gyroDriftRateDegPerSec{ 3.0 / 900.0 };
};

} // namespace LokYS::Avionics
