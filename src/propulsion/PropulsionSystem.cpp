#include "LokYS/propulsion/PropulsionSystem.hpp"
#include <cmath>
#include <algorithm>

namespace LokYS::Propulsion {

PropulsionSystem::PropulsionSystem(const EngineSpecs& specs)
    : m_specs(specs), m_fuelRemainingKg(specs.maxFuelCapacityKg) {}

f64 PropulsionSystem::ComputeThrustCoefficient(f64 advanceRatio, f64 bladePitchDeg) const noexcept {
    f64 J = std::max(0.0, advanceRatio);

    // Feathered propeller polar (beta >= 80 deg) -> windmill drag minimal
    if (bladePitchDeg >= 80.0) {
        return -0.003 * std::min(1.0, J);
    }

    f64 pitchRatio = std::clamp(bladePitchDeg, 10.0, 45.0) / 16.0;
    f64 Ct = 0.092 * pitchRatio + 0.045 * J - (0.165 * J * J) / pitchRatio;
    return std::max(-0.02, Ct);
}

f64 PropulsionSystem::ComputePowerCoefficient(f64 advanceRatio, f64 bladePitchDeg) const noexcept {
    f64 J = std::max(0.0, advanceRatio);

    // Feathered propeller polar: 92% reduction in windmill resistance
    if (bladePitchDeg >= 80.0) {
        return 0.004 + 0.002 * std::min(1.0, J);
    }

    f64 pitchRatio = std::clamp(bladePitchDeg, 10.0, 45.0) / 16.0;
    f64 Cp = 0.055 * std::pow(pitchRatio, 1.4) + 0.015 * J - 0.048 * (J * J);
    return std::max(0.008, Cp);
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
    f64 dt,
    f64 normalLoadFactorG) noexcept
{
    PropulsionOutput out{};

    // 1. Check Fuel Availability & Christen 801 Inverted Negative-G Feed
    bool fuelPresent = (m_fuelRemainingKg > 0.01) && (controls.fuelSelector != FuelSelector::Off);

    if (normalLoadFactorG < 0.0) {
        if (m_specs.isAerobaticInvertedFuel) {
            // Weighted flop-tube & Christen 801 inverted oil valve active: continuous operation
            m_negativeGDuration = 0.0;
            out.isInvertedFuelActive = true;
        } else {
            // Non-aerobatic float carb / unvalved tank: fuel starves after 2.5 seconds
            m_negativeGDuration += dt;
            if (m_negativeGDuration > 2.5) {
                fuelPresent = false;
                out.isStarved = true;
            }
        }
    } else {
        m_negativeGDuration = std::max(0.0, m_negativeGDuration - dt * 2.0);
    }

    // Ambient barometric pressure in inHg
    f64 ambientInHg = ambientPressurePa / 3386.388;
    f64 densityRatio = airDensityKgM3 / Math::Constants::SEA_LEVEL_DENSITY;
    f64 D = m_specs.propDiameterM;
    f64 D4 = D * D * D * D;
    f64 D5 = D4 * D;
    f64 forwardSpeed = std::max(0.0, velocityBody.x);

    // Lambda to evaluate a single discrete engine & propeller
    auto EvaluateEngine = [&](
        f64 throttle,
        f64 mixture,
        f64 propLever,
        bool isFeathered,
        bool magL,
        bool magR,
        f64& inOutRpm,
        f64& inOutPitchDeg)
    {
        struct SubResult {
            f64 thrustN{ 0.0 };
            f64 qProp{ 0.0 };
            f64 brakeHp{ 0.0 };
            f64 mapInHg{ 0.0 };
            f64 fuelFlowKgS{ 0.0 };
            f64 advanceRatio{ 0.0 };
            f64 propEfficiency{ 0.0 };
            bool isRunning{ false };
        } res{};

        bool ignitionOn = magL || magR;
        res.isRunning = fuelPresent && ignitionOn && (mixture > 0.10);

        f64 shaftHp = 0.0;
        if (res.isRunning) {
            shaftHp = ComputeEngineShaftPower(throttle, mixture, inOutRpm, densityRatio, ambientInHg, res.mapInHg);
            if (magL ^ magR) {
                shaftHp *= 0.96;
            }
        } else {
            res.mapInHg = ambientInHg;
        }
        res.brakeHp = shaftHp;
        f64 shaftWatts = shaftHp * 745.7;

        // Governor / Blade Pitch Dynamics (Phase F3.1 & F3.3)
        if (m_specs.isConstantSpeedProp) {
            if (isFeathered) {
                // Feather slew: +15 deg/s towards 88 deg
                inOutPitchDeg = std::min(88.0, inOutPitchDeg + 15.0 * dt);
            } else {
                // Centrifugal flyweight governor differential equation:
                // dBeta/dt = K_gov * (RPM - RPM_command)
                f64 targetRpm = m_specs.idleRpm + std::clamp(propLever, 0.0, 1.0) * (m_specs.maxRatedRpm - m_specs.idleRpm);
                f64 rpmErr = inOutRpm - targetRpm;
                f64 dPitchDt = std::clamp(0.08 * rpmErr, -15.0, 15.0);
                inOutPitchDeg = std::clamp(inOutPitchDeg + dPitchDt * dt, 12.0, 35.0);
            }
        } else {
            inOutPitchDeg = 16.0; // Fixed pitch
        }

        // Propeller Aerodynamics & Advance Ratio
        f64 omega = inOutRpm * (2.0 * Math::Constants::PI / 60.0);
        f64 n = omega / (2.0 * Math::Constants::PI);

        if (n > 1.0) {
            res.advanceRatio = forwardSpeed / (n * D);
        }

        f64 Ct = ComputeThrustCoefficient(res.advanceRatio, inOutPitchDeg);
        f64 Cp = ComputePowerCoefficient(res.advanceRatio, inOutPitchDeg);

        if (n > 2.0) {
            f64 reynoldsFactor = std::clamp(0.60 + 0.40 * (n / 35.0), 0.60, 1.0);
            res.thrustN = Ct * reynoldsFactor * airDensityKgM3 * (n * n) * D4;
            res.qProp = (Cp * airDensityKgM3 * (n * n) * D5) / (2.0 * Math::Constants::PI);
            if (Cp > 1e-4) {
                res.propEfficiency = std::clamp((res.advanceRatio * Ct) / Cp, 0.0, 0.88);
            }
        }

        // Torque balance & Rotational acceleration
        f64 idleOmega = m_specs.idleRpm * (2.0 * Math::Constants::PI / 60.0);
        f64 qEngine = 0.0;
        if (res.isRunning) {
            if (throttle < 0.02) {
                qEngine = (omega <= idleOmega) ? res.qProp : 0.0;
                res.brakeHp = (res.qProp * omega) / 745.7;
            } else if (omega > 5.0) {
                qEngine = shaftWatts / omega;
            } else {
                qEngine = 90.0; // Starter assist
            }
        }

        f64 qResistive = res.qProp;
        if (!res.isRunning) {
            if (isFeathered || inOutPitchDeg >= 80.0) {
                qResistive *= 0.08; // 92% reduction in windmill drag when feathered
            } else {
                qResistive += 15.0 * (inOutRpm / m_specs.idleRpm);
            }
        }

        f64 dOmegaDt = (qEngine - qResistive) / m_specs.propInertiaKgM2;
        omega += dOmegaDt * dt;

        if (res.isRunning && throttle < 0.02 && omega < idleOmega) {
            omega = idleOmega;
        }
        omega = std::max(0.0, omega);
        inOutRpm = omega * (60.0 / (2.0 * Math::Constants::PI));

        if (res.isRunning && shaftHp > 0.1) {
            f64 shaftKw = (shaftHp * 745.7) / 1000.0;
            res.fuelFlowKgS = (shaftKw * m_specs.bsfcKgPerKwh) / 3600.0;
        }

        return res;
    };

    // Evaluate Engine 1 (Left / Single)
    auto e1 = EvaluateEngine(
        controls.throttle,
        controls.mixture,
        controls.propLever,
        controls.isFeathered,
        controls.magnetosLeft,
        controls.magnetosRight,
        m_engineRpm,
        m_bladePitchDeg);

    out.thrustNewtons = std::max(0.0, e1.thrustN);
    out.engineRpm = m_engineRpm;
    out.bladePitchDeg = m_bladePitchDeg;
    out.brakeHorsepower = e1.brakeHp;
    out.manifoldPressureInHg = e1.mapInHg;
    out.propAdvanceRatio = e1.advanceRatio;
    out.propEfficiency = e1.propEfficiency;
    out.isRunning = e1.isRunning;
    out.isFeathered = controls.isFeathered || (m_bladePitchDeg >= 80.0);

    f64 totalThrustN = out.thrustNewtons;
    f64 totalFuelFlowKgS = e1.fuelFlowKgS;

    // Moments for Engine 1
    f64 omega1 = m_engineRpm * (2.0 * Math::Constants::PI / 60.0);
    f64 rollTorque1 = -e1.qProp; // Counter-clockwise reaction to CW spin

    f64 alpha = (velocityBody.Length() > 1.0) ? std::atan2(velocityBody.z, velocityBody.x) : 0.0;
    f64 pFactorArm1 = D * 0.12 * std::sin(alpha);
    f64 yawPFactor1 = -out.thrustNewtons * pFactorArm1;

    Math::Vec3d hProp1{ m_specs.propInertiaKgM2 * omega1, 0.0, 0.0 };
    Math::Vec3d gyro1 = -(angularVelocityBody.Cross(hProp1));
    Math::Vec3d totalMoments = Math::Vec3d{ rollTorque1, 0.0, yawPFactor1 } + gyro1;

    // Evaluate Engine 2 (Right, for Baron 58 twin)
    if (m_specs.engineCount > 1) {
        f64 th2 = controls.useDiscreteTwin ? controls.throttle2 : controls.throttle;
        f64 mix2 = controls.useDiscreteTwin ? controls.mixture2 : controls.mixture;
        f64 prp2 = controls.useDiscreteTwin ? controls.propLever2 : controls.propLever;
        bool fth2 = controls.useDiscreteTwin ? controls.isFeathered2 : controls.isFeathered;
        bool magL2 = controls.useDiscreteTwin ? controls.magnetosLeft2 : controls.magnetosLeft;
        bool magR2 = controls.useDiscreteTwin ? controls.magnetosRight2 : controls.magnetosRight;

        auto e2 = EvaluateEngine(th2, mix2, prp2, fth2, magL2, magR2, m_engineRpm2, m_bladePitchDeg2);

        out.thrustNewtons2 = std::max(0.0, e2.thrustN);
        out.engineRpm2 = m_engineRpm2;
        out.bladePitchDeg2 = m_bladePitchDeg2;
        out.brakeHorsepower2 = e2.brakeHp;
        out.manifoldPressureInHg2 = e2.mapInHg;
        out.isFeathered2 = fth2 || (m_bladePitchDeg2 >= 80.0);

        totalThrustN += out.thrustNewtons2;
        totalFuelFlowKgS += e2.fuelFlowKgS;

        f64 omega2 = m_engineRpm2 * (2.0 * Math::Constants::PI / 60.0);
        f64 rollTorque2 = -e2.qProp;
        f64 yawPFactor2 = -out.thrustNewtons2 * (D * 0.12 * std::sin(alpha));

        Math::Vec3d hProp2{ m_specs.propInertiaKgM2 * omega2, 0.0, 0.0 };
        Math::Vec3d gyro2 = -(angularVelocityBody.Cross(hProp2));
        totalMoments += Math::Vec3d{ rollTorque2, 0.0, yawPFactor2 } + gyro2;

        // --- Phase F3.2: Critical Engine Asymmetric Thrust & Yaw Moment ---
        // Both engines rotate Clockwise. Down-turning blade shifts thrust line by +0.12*D to the right.
        // Left Engine nacelle:  Y1 = -m_specs.lateralOffsetM. Effective arm = -lateralOffsetM + 0.12*D
        // Right Engine nacelle: Y2 = +m_specs.lateralOffsetM. Effective arm = +lateralOffsetM + 0.12*D
        // Yaw moment: N = - (T1 * arm1 + T2 * arm2)
        f64 armLeft = -m_specs.lateralOffsetM + (0.12 * D);
        f64 armRight = m_specs.lateralOffsetM + (0.12 * D);
        f64 asymYaw = -(out.thrustNewtons * armLeft + out.thrustNewtons2 * armRight);
        totalMoments.z += asymYaw;
    }

    out.thrustForceBody = Math::Vec3d{ totalThrustN, 0.0, 0.0 };
    out.momentsBody = totalMoments;

    // Fuel consumption and accounting
    out.fuelFlowKgPerSec = totalFuelFlowKgS;
    out.fuelFlowGph = (totalFuelFlowKgS * 3600.0) / 2.7255;
    m_fuelRemainingKg = std::max(0.0, m_fuelRemainingKg - totalFuelFlowKgS * dt);
    out.remainingFuelKg = m_fuelRemainingKg;
    out.remainingFuelGallons = m_fuelRemainingKg / 2.7255;

    return out;
}

} // namespace LokYS::Propulsion
