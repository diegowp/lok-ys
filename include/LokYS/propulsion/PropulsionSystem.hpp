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

    // Phase F3 Architecture Additions
    bool isConstantSpeedProp{ false }; // Variable-pitch governor (Baron 58, Extra 330)
    bool isAerobaticInvertedFuel{ false }; // Christen 801 flop-tube negative-G system
    u32 engineCount{ 1 };              // 1 for C172/E330, 2 for Baron 58
    f64 lateralOffsetM{ 0.0 };         // +/- 1.83m for Baron 58 twin
};

struct PropulsionControls {
    f64 throttle{ 0.0 };    // [0.0, 1.0] (Idle to Full Throttle, or Engine 1)
    f64 mixture{ 1.0 };     // [0.0, 1.0] (0 = Idle Cutoff, 1.0 = Full Rich)
    f64 propLever{ 1.0 };   // [0.0, 1.0] (CSP Governor RPM target: 1 = Max RPM / Fine, 0 = Min RPM / Coarse)
    bool isFeathered{ false };// Feather command (Beta -> 88 deg)
    FuelSelector fuelSelector{ FuelSelector::Both };
    bool magnetosLeft{ true };
    bool magnetosRight{ true };

    // Discrete Engine 2 Controls (Baron 58 MEP)
    f64 throttle2{ 0.0 };
    f64 mixture2{ 1.0 };
    f64 propLever2{ 1.0 };
    bool isFeathered2{ false };
    bool magnetosLeft2{ true };
    bool magnetosRight2{ true };
    bool useDiscreteTwin{ false };
};

struct PropulsionOutput {
    f64 thrustNewtons{ 0.0 };          // Forward thrust along Body +X
    Math::Vec3d thrustForceBody{ 0.0, 0.0, 0.0 };
    Math::Vec3d momentsBody{ 0.0, 0.0, 0.0 }; // Torque reaction + P-Factor + Gyroscopic + Asymmetric twin yaw

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

    // Phase F3 Empirical Additions:
    f64 bladePitchDeg{ 16.0 };         // Instantaneous CSP blade angle (deg)
    f64 bladePitchDeg2{ 16.0 };
    f64 engineRpm2{ 0.0 };
    f64 thrustNewtons2{ 0.0 };
    f64 brakeHorsepower2{ 0.0 };
    f64 manifoldPressureInHg2{ 0.0 };
    bool isFeathered{ false };
    bool isFeathered2{ false };
    bool isInvertedFuelActive{ false };// Christen 801 valve
};

class PropulsionSystem {
public:
    explicit PropulsionSystem(const EngineSpecs& specs = {});

    void SetSpecs(const EngineSpecs& specs) noexcept { m_specs = specs; }
    [[nodiscard]] const EngineSpecs& GetSpecs() const noexcept { return m_specs; }

    void SetAircraftEngineConfig(const Aircraft::AircraftEngineConfig& config) noexcept {
        m_specs.maxRatedPowerHp = config.horsepowerPerEngine;
        m_specs.maxRatedRpm = config.maxRatedRpm;
        m_specs.idleRpm = config.idleRpm;
        m_specs.propDiameterM = config.propDiameterM;
        m_specs.propInertiaKgM2 = config.propInertiaKgM2;
        m_specs.maxFuelCapacityKg = config.maxFuelCapacityKg;
        m_specs.isAerobaticInvertedFuel = config.isAerobaticInvertedFuel;
        m_specs.engineCount = config.engineCount;
        m_specs.lateralOffsetM = config.lateralOffsetM;
        m_specs.isConstantSpeedProp = (config.engineCount > 1) || config.isAerobaticInvertedFuel;
        m_fuelRemainingKg = std::min(m_fuelRemainingKg, m_specs.maxFuelCapacityKg);
        m_bladePitchDeg = m_specs.isConstantSpeedProp ? 14.0 : 16.0;
        m_bladePitchDeg2 = m_specs.isConstantSpeedProp ? 14.0 : 16.0;
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

    void SetRpm2(f64 rpm) noexcept { m_engineRpm2 = std::max(0.0, rpm); }
    [[nodiscard]] f64 GetRpm2() const noexcept { return m_engineRpm2; }

    void SetBladePitchDeg(f64 deg) noexcept { m_bladePitchDeg = deg; }
    [[nodiscard]] f64 GetBladePitchDeg() const noexcept { return m_bladePitchDeg; }

    [[nodiscard]] PropulsionOutput Evaluate(
        const PropulsionControls& controls,
        const Math::Vec3d& velocityBody,
        const Math::Vec3d& angularVelocityBody,
        f64 airDensityKgM3 = Math::Constants::SEA_LEVEL_DENSITY,
        f64 ambientPressurePa = Math::Constants::STANDARD_SEA_LEVEL_PRESSURE,
        f64 dt = 1.0 / 120.0,
        f64 normalLoadFactorG = 1.0) noexcept;

private:
    [[nodiscard]] f64 ComputeEngineShaftPower(
        f64 throttle,
        f64 mixture,
        f64 rpm,
        f64 airDensityRatio,
        f64 ambientPressureInHg,
        f64& outMapInHg) const noexcept;

    [[nodiscard]] f64 ComputeThrustCoefficient(f64 advanceRatio, f64 bladePitchDeg) const noexcept;
    [[nodiscard]] f64 ComputePowerCoefficient(f64 advanceRatio, f64 bladePitchDeg) const noexcept;

    EngineSpecs m_specs{};

    f64 m_engineRpm{ 680.0 };        // Engine 1 RPM
    f64 m_engineRpm2{ 680.0 };       // Engine 2 RPM (Baron 58)
    f64 m_bladePitchDeg{ 16.0 };     // Engine 1 CSP pitch
    f64 m_bladePitchDeg2{ 16.0 };    // Engine 2 CSP pitch
    f64 m_fuelRemainingKg{ 144.5 };  // Usable fuel remaining
    f64 m_negativeGDuration{ 0.0 };  // Negative G duration counter for non-aerobatic starvation
};

} // namespace LokYS::Propulsion
