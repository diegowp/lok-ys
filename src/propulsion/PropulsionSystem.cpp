#include "LokYS/propulsion/PropulsionSystem.hpp"
#include <cmath>
#include <algorithm>

namespace LokYS::Propulsion {

PropulsionSystem::PropulsionSystem(const EngineSpecs& specs)
    : m_specs(specs), m_fuelRemainingKg(specs.maxFuelCapacityKg) {}

f64 PropulsionSystem::ComputeThrustCoefficient(f64 advanceRatio) const noexcept {
    // McCauley 75-inch fixed-pitch propeller polar (P/D = 0.76)
    f64 J = std::max(0.0, advanceRatio);
    f64 Ct = 0.092 + 0.045 * J - 0.165 * J * J;
    return std::max(0.0, Ct);
}

f64 PropulsionSystem::ComputePowerCoefficient(f64 advanceRatio) const noexcept {
    // McCauley 75-inch fixed-pitch propeller polar
    f64 J = std::max(0.0, advanceRatio);
    f64 Cp = 0.055 + 0.015 * J - 0.048 * J * J;
    return std::max(0.010, Cp);
}

f64 PropulsionSystem::ComputeEngineShaftPower(
    f64 throttle,
    f64 mixture,
    f64 rpm,
    f64 airDensityRatio,
    f64 ambientPressureInHg,
    f64& outMapInHg) const noexcept
{
    // Throttle controls manifold pressure (MAP)
    // Idle MAP: ~11 inHg, WOT MAP: ambient barometric pressure
    f64 idleMap = 11.0;
    f64 th = std::clamp(throttle, 0.0, 1.0);
    outMapInHg = idleMap + th * (ambientPressureInHg - idleMap);

    // Gagg-Farrar naturally aspirated altitude lapse factor
    f64 altitudeFactor = (airDensityRatio - 0.117) / 0.883;
    altitudeFactor = std::clamp(altitudeFactor, 0.0, 1.0);

    // Mixture efficiency curve:
    // 0.0 = Idle Cutoff (engine dies)
    // 0.85 = Best Power (100% rated output)
    // 1.0 = Full Rich (96% power due to rich cooling)
    f64 mix = std::clamp(mixture, 0.0, 1.0);
    f64 mixtureEff = 0.0;
    if (mix > 0.15) {
        if (mix <= 0.85) {
            mixtureEff = (mix - 0.15) / 0.70; // 0.0 to 1.0
        } else {
            mixtureEff = 1.0 - 0.04 * ((mix - 0.85) / 0.15); // 1.0 to 0.96
        }
    }

    // Engine gross indicated power (Gagg-Farrar naturally aspirated piston equation)
    f64 rpmRatio = rpm / m_specs.maxRatedRpm;
    f64 mapRatio = outMapInHg / 29.92;
    f64 grossHp = (m_specs.maxRatedPowerHp + 18.0) * rpmRatio * mapRatio * altitudeFactor;
    f64 frictionHp = 18.0 * (rpm / m_specs.maxRatedRpm); // Engine internal mechanical friction

    f64 brakeHp = std::max(0.0, grossHp - frictionHp) * mixtureEff;
    return brakeHp;
}

PropulsionOutput PropulsionSystem::Evaluate(
    const PropulsionControls& controls,
    const Math::Vec3d& velocityBody,
    const Math::Vec3d& angularVelocityBody,
    f64 airDensityKgM3,
    f64 ambientPressurePa,
    f64 dt) noexcept
{
    PropulsionOutput out{};

    // 1. Check Fuel Availability & Fuel Selector
    bool fuelAvailable = (m_fuelRemainingKg > 0.01) && (controls.fuelSelector != FuelSelector::Off);
    bool ignitionOn = controls.magnetosLeft || controls.magnetosRight;

    out.isRunning = fuelAvailable && ignitionOn && (controls.mixture > 0.10);
    out.isStarved = !fuelAvailable;

    // Ambient barometric pressure in inHg
    f64 ambientInHg = ambientPressurePa / 3386.388;
    f64 densityRatio = airDensityKgM3 / Math::Constants::SEA_LEVEL_DENSITY;

    // 2. Engine Shaft Power Output
    f64 mapInHg = 0.0;
    f64 shaftHp = 0.0;
    if (out.isRunning) {
        shaftHp = ComputeEngineShaftPower(controls.throttle, controls.mixture, m_engineRpm, densityRatio, ambientInHg, mapInHg);
        // Single magneto penalty (~75 RPM / 4% power drop)
        if (controls.magnetosLeft ^ controls.magnetosRight) {
            shaftHp *= 0.96;
        }
    } else {
        mapInHg = ambientInHg; // Engine off: MAP equals ambient
    }

    out.brakeHorsepower = shaftHp;
    out.manifoldPressureInHg = mapInHg;

    // Convert HP to Watts (1 hp = 745.7 W)
    f64 shaftWatts = shaftHp * 745.7;

    // 3. Propeller Aerodynamics & Torque Balance
    f64 omega = m_engineRpm * (2.0 * Math::Constants::PI / 60.0);
    f64 n = omega / (2.0 * Math::Constants::PI); // revs per sec

    f64 forwardSpeed = std::max(0.0, velocityBody.x);
    f64 J = 0.0;
    if (n > 1.0) {
        J = forwardSpeed / (n * m_specs.propDiameterM);
    }
    out.propAdvanceRatio = J;

    f64 Ct = ComputeThrustCoefficient(J);
    f64 Cp = ComputePowerCoefficient(J);

    // Propeller thrust: T = Ct * rho * n^2 * D^4
    f64 D = m_specs.propDiameterM;
    f64 D4 = D * D * D * D;
    f64 D5 = D4 * D;

    f64 thrustN = 0.0;
    f64 qProp = 0.0; // Torque demanded by propeller (N*m)

    if (n > 2.0) {
        // Low Reynolds blade stall at ground idle
        f64 reynoldsFactor = std::clamp(0.60 + 0.40 * (n / 35.0), 0.60, 1.0);
        thrustN = Ct * reynoldsFactor * airDensityKgM3 * (n * n) * D4;
        qProp = (Cp * airDensityKgM3 * (n * n) * D5) / (2.0 * Math::Constants::PI);
        if (Cp > 1e-4) {
            out.propEfficiency = std::clamp((J * Ct) / Cp, 0.0, 0.88);
        }
    }

    // Engine driving torque: Q = Power / omega
    f64 idleOmega = m_specs.idleRpm * (2.0 * Math::Constants::PI / 60.0);
    f64 qEngine = 0.0;
    if (out.isRunning) {
        if (controls.throttle < 0.02) {
            // Closed throttle: idle circuit supplies just enough torque to sustain idle RPM against propeller drag
            if (omega <= idleOmega) {
                qEngine = qProp;
            } else {
                qEngine = 0.0; // Excess engine braking above idle
            }
            out.brakeHorsepower = (qProp * omega) / 745.7;
        } else if (omega > 5.0) {
            qEngine = shaftWatts / omega;
        } else {
            // Idle starter torque assist when cranking
            qEngine = 90.0;
        }
    }

    // Rotational acceleration: d(omega)/dt = (Q_engine - Q_resistive) / I_prop
    f64 qResistive = qProp;
    if (!out.isRunning) {
        qResistive += 15.0 * (m_engineRpm / m_specs.idleRpm);
    }

    f64 dOmegaDt = (qEngine - qResistive) / m_specs.propInertiaKgM2;
    omega += dOmegaDt * dt;

    if (out.isRunning && controls.throttle < 0.02 && omega < idleOmega) {
        omega = idleOmega;
    }

    // Ensure non-negative rotational speed
    omega = std::max(0.0, omega);
    m_engineRpm = omega * (60.0 / (2.0 * Math::Constants::PI));
    out.engineRpm = m_engineRpm;

    // Final thrust (prevent large negative thrust in level flight)
    out.thrustNewtons = std::max(0.0, thrustN);
    out.thrustForceBody = Math::Vec3d{ out.thrustNewtons, 0.0, 0.0 };

    // 4. Engine & Propeller Moments
    // A. Torque Reaction: Clockwise prop rotation creates left roll moment (-X body)
    f64 rollTorqueReaction = -qProp;

    // B. P-Factor (Asymmetric blade loading at positive angle of attack)
    f64 alpha = 0.0;
    f64 trueAirspeed = velocityBody.Length();
    if (trueAirspeed > 1.0) {
        alpha = std::atan2(velocityBody.z, velocityBody.x);
    }
    // Downward moving blade produces rightward thrust shift, creating left yaw moment (-Z body)
    f64 pFactorArm = D * 0.12 * std::sin(alpha);
    f64 yawMomentPFactor = -out.thrustNewtons * pFactorArm;

    // C. Gyroscopic Precession: M_gyro = -omega_aircraft x H_prop
    // H_prop = [I_prop * omega, 0, 0]
    Math::Vec3d hProp{ m_specs.propInertiaKgM2 * omega, 0.0, 0.0 };
    Math::Vec3d gyroMoment = -(angularVelocityBody.Cross(hProp));

    out.momentsBody = Math::Vec3d{ rollTorqueReaction, 0.0, yawMomentPFactor } + gyroMoment;

    // 5. Fuel Consumption
    if (out.isRunning && shaftHp > 0.1) {
        // BSFC: kg per kW*hour
        f64 shaftKw = shaftWatts / 1000.0;
        out.fuelFlowKgPerSec = (shaftKw * m_specs.bsfcKgPerKwh) / 3600.0;
        // 1 gallon 100LL = 3.78541 L * 0.72 kg/L = 2.725 kg
        f64 kgPerGallon = 2.7255;
        out.fuelFlowGph = (out.fuelFlowKgPerSec * 3600.0) / kgPerGallon;

        m_fuelRemainingKg = std::max(0.0, m_fuelRemainingKg - out.fuelFlowKgPerSec * dt);
    } else {
        out.fuelFlowKgPerSec = 0.0;
        out.fuelFlowGph = 0.0;
    }

    out.remainingFuelKg = m_fuelRemainingKg;
    out.remainingFuelGallons = m_fuelRemainingKg / 2.7255;

    return out;
}

} // namespace LokYS::Propulsion
