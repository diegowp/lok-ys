#include "LokYS/propulsion/PropulsionSystem.hpp"
#include "LokYS/math/Units.hpp"
#include <iostream>
#include <cmath>

using namespace LokYS;
using namespace LokYS::Math;
using namespace LokYS::Propulsion;

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

void TestStaticFullThrottleTakeoff() {
    PropulsionSystem engine;
    PropulsionControls ctrl{};
    ctrl.throttle = 1.0; // Full power
    ctrl.mixture = 1.0;  // Full rich

    Vec3d zeroVel{ 0.0, 0.0, 0.0 };
    Vec3d zeroOmega{ 0.0, 0.0, 0.0 };

    constexpr f64 dt = 1.0 / 120.0;
    // Spool up engine for 2.0 seconds
    PropulsionOutput out{};
    for (int i = 0; i < 240; ++i) {
        out = engine.Evaluate(ctrl, zeroVel, zeroOmega, Constants::SEA_LEVEL_DENSITY, Constants::STANDARD_SEA_LEVEL_PRESSURE, dt);
    }

    TEST_ASSERT(out.isRunning, "Engine running at full throttle");
    // Static ground RPM with fixed-pitch prop is typically 2250 - 2400 RPM
    TEST_ASSERT(out.engineRpm >= 2200.0 && out.engineRpm <= 2450.0, "Static full-throttle RPM in 2200-2450 range");
    // Static thrust should be ~2100 - 2350 N
    TEST_ASSERT(out.thrustNewtons >= 2000.0 && out.thrustNewtons <= 2400.0, "Static thrust in 2000-2400 N range");
    // Manifold pressure near ambient at wide open throttle
    TEST_NEAR(out.manifoldPressureInHg, 29.92, 0.2, "WOT Manifold Pressure equals ambient");
    // Brake horsepower output should be near rated static power (125-145 hp)
    TEST_ASSERT(out.brakeHorsepower >= 120.0 && out.brakeHorsepower <= 150.0, "Static shaft horsepower credible");
}

void TestGroundIdleStability() {
    PropulsionSystem engine;
    PropulsionControls ctrl{};
    ctrl.throttle = 0.0; // Idle
    ctrl.mixture = 1.0;

    Vec3d zeroVel{ 0.0, 0.0, 0.0 };
    Vec3d zeroOmega{ 0.0, 0.0, 0.0 };

    constexpr f64 dt = 1.0 / 120.0;
    PropulsionOutput out{};
    for (int i = 0; i < 120; ++i) {
        out = engine.Evaluate(ctrl, zeroVel, zeroOmega, Constants::SEA_LEVEL_DENSITY, Constants::STANDARD_SEA_LEVEL_PRESSURE, dt);
    }

    TEST_ASSERT(out.isRunning, "Engine running at idle");
    // Idle RPM should settle to 650 - 720 RPM
    TEST_ASSERT(out.engineRpm >= 650.0 && out.engineRpm <= 720.0, "Idle RPM maintained near 680");
    // Residual thrust at idle should be very low (< 150 N)
    TEST_ASSERT(out.thrustNewtons < 150.0, "Low residual thrust at idle");
    // Idle MAP should show intake manifold vacuum (~11 inHg)
    TEST_NEAR(out.manifoldPressureInHg, 11.0, 0.5, "Intake manifold vacuum at idle");
}

void TestInFlightPropellerUnloading() {
    PropulsionSystem engine;
    PropulsionControls ctrl{};
    ctrl.throttle = 1.0;
    ctrl.mixture = 0.85; // Best power

    // Level cruise flight at 100 knots (~51.44 m/s)
    Vec3d cruiseVel{ 51.4444, 0.0, 0.0 };
    Vec3d zeroOmega{ 0.0, 0.0, 0.0 };

    constexpr f64 dt = 1.0 / 120.0;
    PropulsionOutput out{};
    for (int i = 0; i < 240; ++i) {
        out = engine.Evaluate(ctrl, cruiseVel, zeroOmega, Constants::SEA_LEVEL_DENSITY, Constants::STANDARD_SEA_LEVEL_PRESSURE, dt);
    }

    // Advance ratio J in cruise: V / (n * D) ~ 0.60 - 0.75
    TEST_ASSERT(out.propAdvanceRatio >= 0.55 && out.propAdvanceRatio <= 0.75, "Cruise advance ratio in 0.55-0.75 range");
    // Propeller efficiency peaks in cruise: eta > 75%
    TEST_ASSERT(out.propEfficiency >= 0.75, "Propeller efficiency high in cruise");
    // Fixed-pitch prop unloads in flight, allowing RPM to climb to 2500 - 2700 RPM
    TEST_ASSERT(out.engineRpm >= 2500.0 && out.engineRpm <= 2700.0, "Propeller unloads to 2500-2700 RPM in flight");
}

void TestFuelConsumptionAndMixtureCutoff() {
    PropulsionSystem engine;
    PropulsionControls ctrl{};
    ctrl.throttle = 0.75; // 75% cruise power
    ctrl.mixture = 0.85;

    Vec3d cruiseVel{ 50.0, 0.0, 0.0 };
    Vec3d zeroOmega{ 0.0, 0.0, 0.0 };

    constexpr f64 dt = 1.0 / 120.0;
    PropulsionOutput out{};
    for (int i = 0; i < 240; ++i) {
        out = engine.Evaluate(ctrl, cruiseVel, zeroOmega, Constants::SEA_LEVEL_DENSITY, Constants::STANDARD_SEA_LEVEL_PRESSURE, dt);
    }

    // 75% power fuel flow: 8.0 - 9.5 GPH
    TEST_ASSERT(out.fuelFlowGph >= 7.5 && out.fuelFlowGph <= 10.0, "Fuel flow at 75% power matches 7.5-10 GPH");
    TEST_ASSERT(out.remainingFuelKg < 144.5, "Fuel quantity consumed dynamically");

    // Test Mixture Idle Cutoff
    ctrl.mixture = 0.0;
    for (int i = 0; i < 120; ++i) {
        out = engine.Evaluate(ctrl, cruiseVel, zeroOmega, Constants::SEA_LEVEL_DENSITY, Constants::STANDARD_SEA_LEVEL_PRESSURE, dt);
    }

    TEST_ASSERT(!out.isRunning, "Engine stopped after mixture cutoff");
    TEST_NEAR(out.brakeHorsepower, 0.0, 1e-4, "Zero horsepower output at cutoff");
    TEST_NEAR(out.fuelFlowGph, 0.0, 1e-4, "Zero fuel flow when stopped");
}

void TestEngineMomentsAndPFactor() {
    PropulsionSystem engine;
    PropulsionControls ctrl{};
    ctrl.throttle = 1.0;
    ctrl.mixture = 1.0;

    // 1. High angle of attack climb: V = [35.0, 0.0, 6.0] (alpha ~ 9.7 deg)
    Vec3d climbVel{ 35.0, 0.0, 6.0 };
    Vec3d zeroOmega{ 0.0, 0.0, 0.0 };

    constexpr f64 dt = 1.0 / 120.0;
    PropulsionOutput outClimb{};
    for (int i = 0; i < 120; ++i) {
        outClimb = engine.Evaluate(ctrl, climbVel, zeroOmega, Constants::SEA_LEVEL_DENSITY, Constants::STANDARD_SEA_LEVEL_PRESSURE, dt);
    }

    // Torque reaction: clockwise prop -> counter-clockwise roll moment (Mx < 0)
    TEST_ASSERT(outClimb.momentsBody.x < -100.0, "Propeller torque reaction creates counter-clockwise roll");

    // P-Factor: positive alpha climb creates left-turning yaw moment (Mz < 0)
    TEST_ASSERT(outClimb.momentsBody.z < -20.0, "P-Factor creates left-turning yaw moment during climb");

    // 2. Gyroscopic Precession: Pitch rate q = +0.5 rad/s (pulling up)
    Vec3d pitchUpOmega{ 0.0, 0.5, 0.0 };
    auto outGyro = engine.Evaluate(ctrl, climbVel, pitchUpOmega, Constants::SEA_LEVEL_DENSITY, Constants::STANDARD_SEA_LEVEL_PRESSURE, dt);

    // M_gyro = -omega x H_prop where H_prop = [Hx, 0, 0]
    // omega = [0, q, 0] -> omega x H_prop = [0, 0, -q * Hx]
    // M_gyro = -[0, 0, -q * Hx] = [0, 0, +q * Hx] (positive Z = right yaw moment)
    TEST_ASSERT(outGyro.momentsBody.z > outClimb.momentsBody.z, "Pitch up gyroscopic precession induces right yaw torque");
}

// Phase F3 Unit Tests: Constant-Speed Governor, Twin Engine, Feathering, and Inverted Fuel

void TestConstantSpeedGovernorFlyweightDynamics() {
    PropulsionSystem engine;
    EngineSpecs specs{};
    specs.maxRatedPowerHp = 300.0;
    specs.maxRatedRpm = 2700.0;
    specs.idleRpm = 700.0;
    specs.propDiameterM = 1.98;
    specs.isConstantSpeedProp = true;
    engine.SetSpecs(specs);

    PropulsionControls ctrl{};
    ctrl.throttle = 0.85; // Cruise power
    ctrl.mixture = 0.85;  // Best power
    // Pilot selects 2400 RPM via prop lever: (2400 - 700) / (2700 - 700) = 1700 / 2000 = 0.85
    ctrl.propLever = 0.85;

    engine.SetRpm(2200.0);
    engine.SetBladePitchDeg(14.0);

    constexpr f64 dt = 1.0 / 120.0;
    Vec3d velCruise{ 65.0, 0.0, 0.0 }; // 65 m/s (~126 kt)
    Vec3d zeroOmega{ 0.0, 0.0, 0.0 };

    PropulsionOutput out{};
    for (int i = 0; i < 240; ++i) { // 2.0 seconds
        out = engine.Evaluate(ctrl, velCruise, zeroOmega, Constants::SEA_LEVEL_DENSITY, Constants::STANDARD_SEA_LEVEL_PRESSURE, dt);
    }

    // Governor should stabilize RPM near 2400
    TEST_NEAR(out.engineRpm, 2400.0, 60.0, "CSP governor stabilizes engine near target 2400 RPM");
    f64 cruisePitch = out.bladePitchDeg;
    TEST_ASSERT(cruisePitch > 15.0, "Governor coarsened blade pitch above fine limit during cruise");

    // Aircraft enters high-speed dive: speed jumps to 110 m/s (~214 kt)
    Vec3d velDive{ 110.0, 0.0, 0.0 };
    for (int i = 0; i < 240; ++i) {
        out = engine.Evaluate(ctrl, velDive, zeroOmega, Constants::SEA_LEVEL_DENSITY, Constants::STANDARD_SEA_LEVEL_PRESSURE, dt);
    }

    // In a dive, CSP governor coarsens blade pitch to prevent engine overspeed
    TEST_ASSERT(out.bladePitchDeg > cruisePitch, "Governor further coarsens pitch in high speed dive");
    TEST_ASSERT(out.engineRpm < specs.maxRatedRpm + 50.0, "Engine RPM prevented from destructive overspeed in dive");
}

void TestDiscreteTwinEngineCriticalEngineAsymmetry() {
    PropulsionSystem twin;
    EngineSpecs specs{};
    specs.engineCount = 2;
    specs.maxRatedPowerHp = 300.0;
    specs.maxRatedRpm = 2700.0;
    specs.idleRpm = 700.0;
    specs.propDiameterM = 1.98;
    specs.lateralOffsetM = 1.83; // Nacelles at +/- 1.83m
    specs.isConstantSpeedProp = true;
    twin.SetSpecs(specs);

    constexpr f64 dt = 1.0 / 120.0;
    Vec3d velTakeoff{ 45.0, 0.0, 0.0 }; // ~87 kt (above Vmc = 84 kt)
    Vec3d zeroOmega{ 0.0, 0.0, 0.0 };

    // Case A: Both engines full throttle (Symmetric)
    PropulsionControls ctrlSymmetric{};
    ctrlSymmetric.throttle = 1.0;
    ctrlSymmetric.mixture = 1.0;
    ctrlSymmetric.propLever = 1.0;
    ctrlSymmetric.useDiscreteTwin = false;

    PropulsionOutput outSym{};
    for (int i = 0; i < 60; ++i) {
        outSym = twin.Evaluate(ctrlSymmetric, velTakeoff, zeroOmega, Constants::SEA_LEVEL_DENSITY, Constants::STANDARD_SEA_LEVEL_PRESSURE, dt);
    }
    TEST_ASSERT(outSym.thrustNewtons > 1500.0, "Engine 1 produces takeoff thrust");
    TEST_ASSERT(outSym.thrustNewtons2 > 1500.0, "Engine 2 produces takeoff thrust");

    // Case B: Left Engine Failure (Left = Idle/Failed, Right = Full Power)
    PropulsionControls ctrlLeftFailed{};
    ctrlLeftFailed.useDiscreteTwin = true;
    ctrlLeftFailed.throttle = 0.0; // Left engine out
    ctrlLeftFailed.mixture = 0.0;
    ctrlLeftFailed.propLever = 1.0;
    ctrlLeftFailed.throttle2 = 1.0; // Right engine full
    ctrlLeftFailed.mixture2 = 1.0;
    ctrlLeftFailed.propLever2 = 1.0;

    PropulsionOutput outLeftFailed{};
    for (int i = 0; i < 60; ++i) {
        outLeftFailed = twin.Evaluate(ctrlLeftFailed, velTakeoff, zeroOmega, Constants::SEA_LEVEL_DENSITY, Constants::STANDARD_SEA_LEVEL_PRESSURE, dt);
    }
    // Operating Right engine produces adverse yaw to the LEFT (negative Body Mz)
    TEST_ASSERT(outLeftFailed.momentsBody.z < -2000.0, "Right engine full thrust causes massive left yaw moment");

    // Case C: Right Engine Failure (Left = Full Power, Right = Idle/Failed)
    PropulsionControls ctrlRightFailed{};
    ctrlRightFailed.useDiscreteTwin = true;
    ctrlRightFailed.throttle = 1.0; // Left engine full
    ctrlRightFailed.mixture = 1.0;
    ctrlRightFailed.propLever = 1.0;
    ctrlRightFailed.throttle2 = 0.0; // Right engine out
    ctrlRightFailed.mixture2 = 0.0;
    ctrlRightFailed.propLever2 = 1.0;

    PropulsionOutput outRightFailed{};
    for (int i = 0; i < 60; ++i) {
        outRightFailed = twin.Evaluate(ctrlRightFailed, velTakeoff, zeroOmega, Constants::SEA_LEVEL_DENSITY, Constants::STANDARD_SEA_LEVEL_PRESSURE, dt);
    }
    // Operating Left engine produces adverse yaw to the RIGHT (positive Body Mz)
    TEST_ASSERT(outRightFailed.momentsBody.z > 1500.0, "Left engine full thrust causes right yaw moment");

    // Critical Engine Validation:
    // Both engines rotate clockwise. For the right engine, the down-turning blade is farther from centerline (+1.83 + 0.12*D = +2.068m).
    // For the left engine, the down-turning blade is closer to centerline (-1.83 + 0.12*D = -1.592m).
    // Therefore, Left Engine Failure generates a significantly larger adverse yaw than Right Engine Failure!
    f64 leftFailYawMagnitude = std::abs(outLeftFailed.momentsBody.z);
    f64 rightFailYawMagnitude = std::abs(outRightFailed.momentsBody.z);
    TEST_ASSERT(leftFailYawMagnitude > rightFailYawMagnitude * 1.15,
        "Left Engine is mathematically confirmed as the Critical Engine due to P-factor asymmetric arm");
}

void TestPropellerFeatheringDragReduction() {
    PropulsionSystem engine;
    EngineSpecs specs{};
    specs.isConstantSpeedProp = true;
    engine.SetSpecs(specs);

    PropulsionControls ctrlFeather{};
    ctrlFeather.throttle = 0.0;
    ctrlFeather.mixture = 0.0; // Engine shut down
    ctrlFeather.isFeathered = true; // Pilot pulls prop lever to FEATHER detent

    engine.SetRpm(1200.0);
    engine.SetBladePitchDeg(15.0);

    constexpr f64 dt = 1.0 / 120.0;
    Vec3d velGlide{ 50.0, 0.0, 0.0 };
    Vec3d zeroOmega{ 0.0, 0.0, 0.0 };

    PropulsionOutput out{};
    // Run for 5 seconds to allow full feather slew (15 deg/s * 5 = 75 deg increment)
    for (int i = 0; i < 600; ++i) {
        out = engine.Evaluate(ctrlFeather, velGlide, zeroOmega, Constants::SEA_LEVEL_DENSITY, Constants::STANDARD_SEA_LEVEL_PRESSURE, dt);
    }

    TEST_ASSERT(out.isFeathered, "Propeller successfully locked into feathered state");
    TEST_NEAR(out.bladePitchDeg, 88.0, 0.5, "Feathered blade pitch reached 88 degrees edge-on into relative wind");
}

void TestChristen801InvertedFuelSystem() {
    constexpr f64 dt = 1.0 / 120.0;
    Vec3d vel{ 50.0, 0.0, 0.0 };
    Vec3d zeroOmega{ 0.0, 0.0, 0.0 };

    PropulsionControls ctrl{};
    ctrl.throttle = 1.0;
    ctrl.mixture = 1.0;

    // 1. Standard Non-Aerobatic Engine (Cessna 172) under sustained negative G
    PropulsionSystem standardEngine;
    EngineSpecs stdSpecs{};
    stdSpecs.isAerobaticInvertedFuel = false;
    standardEngine.SetSpecs(stdSpecs);
    standardEngine.SetRpm(2500.0);

    // Negative G flight: G_z = -1.5G (outside loop or inverted flight)
    PropulsionOutput outStd{};
    // First 1.5 seconds: float bowl fuel buffer sustains engine
    for (int i = 0; i < 180; ++i) {
        outStd = standardEngine.Evaluate(ctrl, vel, zeroOmega, Constants::SEA_LEVEL_DENSITY, Constants::STANDARD_SEA_LEVEL_PRESSURE, dt, -1.5);
    }
    TEST_ASSERT(!outStd.isStarved, "Engine runs normally during brief negative G transition");

    // After 3.0 cumulative seconds: fuel starves and engine cuts out
    for (int i = 0; i < 180; ++i) {
        outStd = standardEngine.Evaluate(ctrl, vel, zeroOmega, Constants::SEA_LEVEL_DENSITY, Constants::STANDARD_SEA_LEVEL_PRESSURE, dt, -1.5);
    }
    TEST_ASSERT(outStd.isStarved, "Standard fuel system starves under sustained negative G (>2.5s)");
    TEST_ASSERT(!outStd.isRunning, "Standard engine quits when fuel starves");

    // 2. Competition Aerobatic Engine with Christen 801 System (Extra 330SC)
    PropulsionSystem acroEngine;
    EngineSpecs acroSpecs{};
    acroSpecs.isAerobaticInvertedFuel = true; // Flop-tube and inverted oil separator
    acroEngine.SetSpecs(acroSpecs);
    acroEngine.SetRpm(2700.0);

    PropulsionOutput outAcro{};
    // Sustained extreme negative G: -6.0G for 5 full seconds (600 ticks)
    for (int i = 0; i < 600; ++i) {
        outAcro = acroEngine.Evaluate(ctrl, vel, zeroOmega, Constants::SEA_LEVEL_DENSITY, Constants::STANDARD_SEA_LEVEL_PRESSURE, dt, -6.0);
    }
    TEST_ASSERT(outAcro.isInvertedFuelActive, "Christen 801 inverted system active");
    TEST_ASSERT(!outAcro.isStarved, "Aerobatic engine never starves under sustained negative G");
    TEST_ASSERT(outAcro.isRunning, "Aerobatic engine maintains continuous full power inverted");
    TEST_ASSERT(outAcro.brakeHorsepower > 100.0, "Full horsepower delivered during sustained inverted aerobatics");
}

int main() {
    std::cout << "========================================\n";
    std::cout << " LokYS Propulsion & Fuel Subsystem Tests\n";
    std::cout << "========================================\n";

    TestStaticFullThrottleTakeoff();
    TestGroundIdleStability();
    TestInFlightPropellerUnloading();
    TestFuelConsumptionAndMixtureCutoff();
    TestEngineMomentsAndPFactor();
    TestConstantSpeedGovernorFlyweightDynamics();
    TestDiscreteTwinEngineCriticalEngineAsymmetry();
    TestPropellerFeatheringDragReduction();
    TestChristen801InvertedFuelSystem();

    std::cout << "\n----------------------------------------\n";
    std::cout << "Tests Passed: " << g_testsPassed << "\n";
    std::cout << "Tests Failed: " << g_testsFailed << "\n";
    std::cout << "----------------------------------------\n";

    return (g_testsFailed == 0) ? 0 : 1;
}
