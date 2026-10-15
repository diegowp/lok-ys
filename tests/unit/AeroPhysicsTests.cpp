#include "LokYS/aerodynamics/AerodynamicsSystem.hpp"
#include "LokYS/propulsion/PropulsionSystem.hpp"
#include "LokYS/physics/GroundReactions.hpp"
#include "LokYS/physics/RigidBody.hpp"
#include "LokYS/aircraft/AircraftRegistry.hpp"
#include "LokYS/math/Units.hpp"
#include <iostream>
#include <cmath>
#include <numbers>

using namespace LokYS;
using namespace LokYS::Math;
using namespace LokYS::Aerodynamics;
using namespace LokYS::Propulsion;
using namespace LokYS::Physics;
using namespace LokYS::Aircraft;

static int g_testsPassed = 0;
static int g_testsFailed = 0;

#define TEST_ASSERT(cond, msg) \
    do { \
        if (!(cond)) { \
            std::cerr << "[FAIL] " << msg << " (line " << __LINE__ << ")\n"; \
            g_testsFailed++; \
        } else { \
            g_testsPassed++; \
        } \
    } while (0)

#define TEST_NEAR(a, b, eps, msg) \
    do { \
        double diff = std::abs(static_cast<double>(a) - static_cast<double>(b)); \
        if (diff > static_cast<double>(eps)) { \
            std::cerr << "[FAIL] " << msg << " | Expected " << b << ", got " << a << " (diff=" << diff << " > " << eps << ")\n"; \
            g_testsFailed++; \
        } else { \
            g_testsPassed++; \
        } \
    } while (0)

// 1. Ground effect lift increase >= 8% and induced drag reduction >= 35% at h/b = 0.15
void TestGroundEffectEmpiricalLiftAndDrag() {
    AerodynamicsSystem aero;
    ControlDeflections controls{};

    // Cessna 172 wingspan = 11.0m, mean chord = 1.493m
    f64 alpha5Rad = Units::DegToRad(5.0);
    Vec3d wind{ 35.0 * std::cos(alpha5Rad), 0.0, 35.0 * std::sin(alpha5Rad) }; // 35 m/s (~68 kt approach speed)
    Vec3d zeroOmega{ 0.0, 0.0, 0.0 };

    // Free flight OGE (h = 100m >> b = 11m)
    auto outOge = aero.Evaluate(wind, zeroOmega, controls, Constants::SEA_LEVEL_DENSITY, 340.294, 100.0);

    // Ground effect IGE at h/b = 0.15 (h = 0.15 * 11.0 = 1.65m)
    auto outIge = aero.Evaluate(wind, zeroOmega, controls, Constants::SEA_LEVEL_DENSITY, 340.294, 1.65);

    // Analytical McCormick factor at h/b = 0.15: phi = 16*(0.15)^2 / (1 + 16*(0.15)^2) = 0.36 / 1.36 = 0.264705
    f64 expectedPhi = (16.0 * 0.15 * 0.15) / (1.0 + 16.0 * 0.15 * 0.15);
    TEST_NEAR(outIge.groundEffectFactor, expectedPhi, 1e-4, "McCormick phi(h/b) matches analytical formula");

    // Assertion 1: Lift increase >= 8% at h/b = 0.15
    f64 liftIncreasePct = (outIge.liftNewtons - outOge.liftNewtons) / outOge.liftNewtons * 100.0;
    TEST_ASSERT(liftIncreasePct >= 8.0, "Ground effect lift increase >= 8% at h/b = 0.15");

    // Assertion 2: Induced drag reduction >= 35% at h/b = 0.15
    f64 inducedDragReductionPct = (1.0 - outIge.groundEffectFactor) * 100.0;
    TEST_ASSERT(inducedDragReductionPct >= 35.0, "Induced drag reduction >= 35% at h/b = 0.15");

    // Total aerodynamic drag drops substantially in ground effect
    f64 totalDragReductionPct = (outOge.dragNewtons - outIge.dragNewtons) / outOge.dragNewtons * 100.0;
    TEST_ASSERT(totalDragReductionPct >= 20.0, "Total aerodynamic drag drops substantially in ground effect");

    // Ground effect flare suppresses downwash, inducing nose-down moment (stabilizing against tailstrike)
    TEST_ASSERT(outIge.momentsBody.y < outOge.momentsBody.y, "Ground effect downwash suppression creates nose-down flare moment");
}

// 2. Propwash elevator authority at zero airspeed (V_tas = 0, RPM = 2700 => M_elevator > 0)
void TestPropwashElevatorAuthorityAtZeroAirspeed() {
    AerodynamicsSystem aero;
    Vec3d zeroWind{ 0.0, 0.0, 0.0 };
    Vec3d zeroOmega{ 0.0, 0.0, 0.0 };

    ControlDeflections ctrlElevatorUp{};
    ctrlElevatorUp.elevator = 1.0; // Full nose-up stick
    ControlDeflections ctrlElevatorDown{};
    ctrlElevatorDown.elevator = -1.0; // Full nose-down stick

    constexpr f64 staticThrustN = 3200.0; // Full takeoff thrust ~2700 RPM on Lycoming O-320
    constexpr f64 propDiameterM = 1.905;

    // Full power static ground run-up (V = 0, RPM = 2700, Thrust = 3200N)
    auto outWashUp = aero.Evaluate(zeroWind, zeroOmega, ctrlElevatorUp, Constants::SEA_LEVEL_DENSITY, 340.294, 10.0, staticThrustN, propDiameterM, 0.60);
    auto outWashDown = aero.Evaluate(zeroWind, zeroOmega, ctrlElevatorDown, Constants::SEA_LEVEL_DENSITY, 340.294, 10.0, staticThrustN, propDiameterM, 0.60);

    TEST_ASSERT(outWashUp.slipstreamVelocityMps > 25.0, "Slipstream velocity > 25 m/s behind 1.9m prop at 3.2 kN thrust");
    TEST_ASSERT(outWashUp.tailDynamicPressurePa > 200.0, "Tail dynamic pressure > 200 Pa under active propwash at zero airspeed");

    // Up elevator produces negative pitching moment (nose-up in Body FRD)
    TEST_ASSERT(outWashUp.momentsBody.y < -300.0, "Up-elevator in propwash produces strong negative pitching moment at V=0");
    // Down elevator produces positive pitching moment (nose-down in Body FRD)
    TEST_ASSERT(outWashDown.momentsBody.y > 300.0, "Down-elevator in propwash produces strong positive pitching moment at V=0");

    // Zero thrust baseline produces zero moments at V=0
    auto outZeroPower = aero.Evaluate(zeroWind, zeroOmega, ctrlElevatorUp, Constants::SEA_LEVEL_DENSITY, 340.294, 10.0, 0.0, propDiameterM, 0.60);
    TEST_NEAR(outZeroPower.momentsBody.y, 0.0, 1e-4, "Without propwash, elevator authority at zero airspeed is strictly zero");
}

// 3. Baron 58 asymmetric yaw moment with critical engine out at 84 kt (Vmc)
void TestBaron58CriticalEngineAsymmetry() {
    EngineSpecs specs{};
    specs.maxRatedPowerHp = 300.0;
    specs.maxRatedRpm = 2700.0;
    specs.propDiameterM = 1.98;
    specs.propInertiaKgM2 = 0.45;
    specs.engineCount = 2;
    specs.lateralOffsetM = 1.83; // Nacelles at +/- 1.83m
    specs.isConstantSpeedProp = true;

    PropulsionSystem prop(specs);

    f64 vmcSpeedMps = Units::KnotsToMps(84.0); // 84 kt Vmc
    Vec3d velVmc{ vmcSpeedMps, 0.0, 0.0 };
    Vec3d zeroOmega{ 0.0, 0.0, 0.0 };
    constexpr f64 dt = 1.0 / 120.0;

    // Condition 1: Left Engine Inoperative / Feathered (Critical Engine Out)
    // Operating Engine: Right Engine (Engine 2) at full takeoff throttle
    PropulsionControls ctrlLeftOut{};
    ctrlLeftOut.useDiscreteTwin = true;
    ctrlLeftOut.throttle = 0.0; // Left engine dead
    ctrlLeftOut.mixture = 0.0;
    ctrlLeftOut.isFeathered = true;
    ctrlLeftOut.throttle2 = 1.0; // Right engine max power
    ctrlLeftOut.mixture2 = 1.0;
    ctrlLeftOut.propLever2 = 1.0;
    ctrlLeftOut.isFeathered2 = false;

    PropulsionOutput outLeftDead{};
    for (int i = 0; i < 120; ++i) {
        outLeftDead = prop.Evaluate(ctrlLeftOut, velVmc, zeroOmega, Constants::SEA_LEVEL_DENSITY, 101325.0, dt, 1.0);
    }

    // Condition 2: Right Engine Inoperative / Feathered (Non-Critical Engine Out)
    // Operating Engine: Left Engine (Engine 1) at full takeoff throttle
    PropulsionSystem prop2(specs);
    PropulsionControls ctrlRightOut{};
    ctrlRightOut.useDiscreteTwin = true;
    ctrlRightOut.throttle = 1.0; // Left engine max power
    ctrlRightOut.mixture = 1.0;
    ctrlRightOut.propLever = 1.0;
    ctrlRightOut.isFeathered = false;
    ctrlRightOut.throttle2 = 0.0; // Right engine dead
    ctrlRightOut.mixture2 = 0.0;
    ctrlRightOut.isFeathered2 = true;

    PropulsionOutput outRightDead{};
    for (int i = 0; i < 120; ++i) {
        outRightDead = prop2.Evaluate(ctrlRightOut, velVmc, zeroOmega, Constants::SEA_LEVEL_DENSITY, 101325.0, dt, 1.0);
    }

    f64 leftDeadYawMag = std::abs(outLeftDead.momentsBody.z);
    f64 rightDeadYawMag = std::abs(outRightDead.momentsBody.z);

    TEST_ASSERT(leftDeadYawMag > rightDeadYawMag,
        "Left engine failure generates substantially greater yaw moment than right engine failure");
    
    f64 ratio = leftDeadYawMag / rightDeadYawMag;
    TEST_ASSERT(ratio >= 1.25 && ratio <= 1.35, "Critical engine yaw moment ratio matches ~1.30 analytical P-factor arm");
}

// 4. Extra 330SC roll rate >= 400 deg/s at 140 kt with full aileron deflection
void TestExtra330SCRollRateEnvelope() {
    AircraftRegistry registry;
    const auto& config = registry.GetConfig(AircraftType::Extra330SC);

    AerodynamicsSystem aero;
    aero.SetConfig(config.geometry, config.aero, false);

    f64 tasMps = Units::KnotsToMps(140.0);
    Vec3d wind{ tasMps, 0.0, 0.0 };
    Vec3d zeroOmega{ 0.0, 0.0, 0.0 };

    ControlDeflections ctrlAileron{};
    ctrlAileron.aileron = 1.0; // Full right aileron deflection

    f64 b = config.geometry.wingspanM;
    f64 pSteadyRadPerSec = (config.aero.Clda * 1.0 / -config.aero.Clp) * (2.0 * tasMps / b);
    f64 pSteadyDegPerSec = Units::RadToDeg(pSteadyRadPerSec);

    TEST_ASSERT(pSteadyDegPerSec >= 400.0, "Extra 330SC steady-state roll rate >= 400 deg/s at 140 kt");

    // Dynamic verification using 6-DOF RigidBody integration
    RigidBody airframe;
    airframe.SetMass(config.mass.emptyMassKg + 50.0);
    airframe.SetInertiaTensor(config.mass.inertiaTensor);

    RigidBodyState state{};
    state.velocityBody = { tasMps, 0.0, 0.0 };
    airframe.SetState(state);

    constexpr f64 dt = 1.0 / 120.0;
    for (int step = 0; step < 60; ++step) {
        airframe.IntegrateRK4(dt, [&](const RigidBodyState& s, f64 /*tOffset*/, Math::Vec3d& fBody, Math::Vec3d& mBody) {
            auto aOut = aero.Evaluate(s.velocityBody, s.angularVelocityBody, ctrlAileron, Constants::SEA_LEVEL_DENSITY, 340.294, 500.0);
            fBody = aOut.forcesBody;
            mBody = aOut.momentsBody;
        });
    }

    f64 dynamicRollRateDegPerSec = Units::RadToDeg(airframe.GetState().angularVelocityBody.x);
    TEST_ASSERT(dynamicRollRateDegPerSec >= 400.0, "Dynamic simulation reaches > 400 deg/s within 0.5s of full aileron input");
}

// 5. Taildragger ground loop yaw divergence when rolling with 5 deg initial crab angle without rudder correction
void TestTaildraggerGroundLoopDivergence() {
    GroundReactionsSystem taildraggerSys(GroundReactionsSystem::CreateForArchetype(AircraftType::Extra330SC));
    GroundReactionsSystem tricycleSys(GroundReactionsSystem::CreateForArchetype(AircraftType::Cessna172Skyhawk));

    constexpr f64 speed = 15.0; // 15 m/s rollout (~30 knots)
    constexpr f64 crabAngleRad = Units::DegToRad(5.0); // 5 degrees initial crab/sideslip perturbation

    Vec3d crabVel{ speed * std::cos(crabAngleRad), speed * std::sin(crabAngleRad), 0.0 };

    RigidBodyState stateTail{};
    stateTail.positionNed = { 0.0, 0.0, -0.98 };
    stateTail.velocityBody = crabVel;
    stateTail.attitude = Quat4d::Identity();

    RigidBodyState stateTri{};
    stateTri.positionNed = { 0.0, 0.0, -0.935 };
    stateTri.velocityBody = crabVel;
    stateTri.attitude = Quat4d::Identity();

    auto outTail = taildraggerSys.Evaluate(stateTail, 0.0, 0.0, 1.0 / 120.0, 0.0);
    auto outTri = tricycleSys.Evaluate(stateTri, 0.0, 0.0, 1.0 / 120.0, 0.0);

    TEST_ASSERT(outTail.leftMainInContact && outTail.rightMainInContact, "Taildragger mains in contact");
    TEST_ASSERT(outTri.leftMainInContact && outTri.rightMainInContact, "Tricycle mains in contact");

    f64 tailMainYawMoment = outTail.gearStates[1].momentBody.z + outTail.gearStates[2].momentBody.z;
    f64 triMainYawMoment = outTri.gearStates[1].momentBody.z + outTri.gearStates[2].momentBody.z;

    TEST_ASSERT(triMainYawMoment > 0.0, "Tricycle gear produces positive stabilizing restoring yaw moment");
    TEST_ASSERT(tailMainYawMoment < 0.0, "Taildragger gear produces negative divergent ground-loop yaw moment");
    TEST_ASSERT(tailMainYawMoment * triMainYawMoment < 0.0, "Taildragger and tricycle gear moments have opposite signs under crab");
}

int main() {
    std::cout << "========================================\n";
    std::cout << " LokYS Aero Physics Empirical Tests     \n";
    std::cout << "========================================\n";

    std::cout << "[RUNNING] TestGroundEffectEmpiricalLiftAndDrag...\n" << std::flush;
    TestGroundEffectEmpiricalLiftAndDrag();
    std::cout << "[DONE] TestGroundEffectEmpiricalLiftAndDrag\n" << std::flush;

    std::cout << "[RUNNING] TestPropwashElevatorAuthorityAtZeroAirspeed...\n" << std::flush;
    TestPropwashElevatorAuthorityAtZeroAirspeed();
    std::cout << "[DONE] TestPropwashElevatorAuthorityAtZeroAirspeed\n" << std::flush;

    std::cout << "[RUNNING] TestBaron58CriticalEngineAsymmetry...\n" << std::flush;
    TestBaron58CriticalEngineAsymmetry();
    std::cout << "[DONE] TestBaron58CriticalEngineAsymmetry\n" << std::flush;

    std::cout << "[RUNNING] TestExtra330SCRollRateEnvelope...\n" << std::flush;
    TestExtra330SCRollRateEnvelope();
    std::cout << "[DONE] TestExtra330SCRollRateEnvelope\n" << std::flush;

    std::cout << "[RUNNING] TestTaildraggerGroundLoopDivergence...\n" << std::flush;
    TestTaildraggerGroundLoopDivergence();
    std::cout << "[DONE] TestTaildraggerGroundLoopDivergence\n" << std::flush;

    std::cout << "\n----------------------------------------\n";
    std::cout << "Tests Passed: " << g_testsPassed << "\n";
    std::cout << "Tests Failed: " << g_testsFailed << "\n";
    std::cout << "----------------------------------------\n";

    return (g_testsFailed == 0) ? 0 : 1;
}
