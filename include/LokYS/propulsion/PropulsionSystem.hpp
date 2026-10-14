#pragma once

#include "LokYS/core/Types.hpp"
#include "LokYS/math/Vector3.hpp"
#include "LokYS/math/Units.hpp"
#include "LokYS/aircraft/AircraftTypes.hpp"
#include <algorithm>
#include <cmath>

namespace LokYS::Propulsion {

enum class FuelSelector {
    Both = 0,
    Left = 1,
    Right = 2,
    Off = 3
};

struct EngineSpecs {
    f64 maxRatedPowerHp{ 160.0 };      // Rated 160 hp @ 2700 RPM (Lycoming O-320-D2J baseline)
    f64 maxRatedRpm{ 2700.0 };         // Redline RPM
    f64 idleRpm{ 680.0 };              // Ground idle RPM
    f64 displacementLiters{ 5.24 };    // 320 cubic inches
    f64 compressionRatio{ 8.5 };       // 8.5:1
    f64 propDiameterM{ 1.905 };        // 75 inches (1.905m) McCauley fixed-pitch prop
    f64 propInertiaKgM2{ 0.38 };       // Rotational inertia of prop + crankshaft assembly
    f64 bsfcKgPerKwh{ 0.28 };          // Brake Specific Fuel Consumption (~0.46 lb/hp-hr)
    f64 maxFuelCapacityKg{ 144.5 };    // 53 usable gallons 100LL (~144.5 kg)
};

struct PropulsionControls {
    f64 throttle{ 0.0 };    // [0.0, 1.0] (Idle to Full Throttle)
    f64 mixture{ 1.0 };     // [0.0, 1.0] (0 = Idle Cutoff, 1.0 = Full Rich)
    FuelSelector fuelSelector{ FuelSelector::Both };
    bool magnetosLeft{ true };
    bool magnetosRight{ true };
};

struct PropulsionOutput {
    f64 thrustNewtons{ 0.0 };          // Forward thrust along Body +X
    Math::Vec3d thrustForceBody{ 0.0, 0.0, 0.0 };
    Math::Vec3d momentsBody{ 0.0, 0.0, 0.0 }; // Torque reaction + P-Factor + Gyroscopic

    f64 engineRpm{ 0.0 };
    f64 manifoldPressureInHg{ 0.0 };   // MAP
    f64 brakeHorsepower{ 0.0 };        // Shaft power output (hp)
    f64 fuelFlowGph{ 0.0 };            // Fuel burn in Gallons Per Hour
    f64 fuelFlowKgPerSec{ 0.0 };       // Fuel burn in kg/s
    f64 remainingFuelKg{ 0.0 };
    f64 remainingFuelGallons{ 0.0 };
    f64 propAdvanceRatio{ 0.0 };       // J = V / (n * D)
    f64 propEfficiency{ 0.0 };         // eta = J * Ct / Cp

    bool isRunning{ true };
    bool isStarved{ false };
};

class PropulsionSystem {
public:
    explicit PropulsionSystem(const EngineSpecs& specs = {});

    void SetSpecs(const EngineSpecs& specs) noexcept { m_specs = specs; }
    [[nodiscard]] const EngineSpecs& GetSpecs() const noexcept { return m_specs; }

    void SetAircraftEngineConfig(const Aircraft::AircraftEngineConfig& config) noexcept {
        m_specs.maxRatedPowerHp = config.horsepowerPerEngine * static_cast<f64>(config.engineCount);
        m_specs.maxRatedRpm = config.maxRatedRpm;
        m_specs.idleRpm = config.idleRpm;
        m_specs.propDiameterM = config.propDiameterM;
        m_specs.propInertiaKgM2 = config.propInertiaKgM2;
        m_specs.maxFuelCapacityKg = config.maxFuelCapacityKg;
        m_fuelRemainingKg = std::min(m_fuelRemainingKg, m_specs.maxFuelCapacityKg);
    }

    void SetFuelQuantityKg(f64 fuelKg) noexcept {
        m_fuelRemainingKg = std::clamp(fuelKg, 0.0, m_specs.maxFuelCapacityKg);
    }
    [[nodiscard]] f64 GetFuelQuantityKg() const noexcept { return m_fuelRemainingKg; }
    [[nodiscard]] f64 GetFuelQuantityGallons() const noexcept {
        return m_fuelRemainingKg / (Math::Units::GallonsToLiters(1.0) * 0.72);
    }

    void SetRpm(f64 rpm) noexcept { m_engineRpm = std::max(0.0, rpm); }
    [[nodiscard]] f64 GetRpm() const noexcept { return m_engineRpm; }

    [[nodiscard]] PropulsionOutput Evaluate(
        const PropulsionControls& controls,
        const Math::Vec3d& velocityBody,
        const Math::Vec3d& angularVelocityBody,
        f64 airDensityKgM3 = Math::Constants::SEA_LEVEL_DENSITY,
        f64 ambientPressurePa = Math::Constants::STANDARD_SEA_LEVEL_PRESSURE,
        f64 dt = 1.0 / 120.0) noexcept;

private:
    [[nodiscard]] f64 ComputeEngineShaftPower(
        f64 throttle,
        f64 mixture,
        f64 rpm,
        f64 airDensityRatio,
        f64 ambientPressureInHg,
        f64& outMapInHg) const noexcept;

    [[nodiscard]] f64 ComputeThrustCoefficient(f64 advanceRatio) const noexcept;
    [[nodiscard]] f64 ComputePowerCoefficient(f64 advanceRatio) const noexcept;

    EngineSpecs m_specs{};

    f64 m_engineRpm{ 680.0 }; // Initial idling RPM
    f64 m_fuelRemainingKg{ 144.5 }; // Full tanks (~53 gal)
};

} // namespace LokYS::Propulsion
