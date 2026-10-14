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

int main() {
    std::cout << "========================================\n";
    std::cout << " LokYS Propulsion & Fuel Subsystem Tests\n";
    std::cout << "========================================\n";

    TestStaticFullThrottleTakeoff();
    TestGroundIdleStability();
    TestInFlightPropellerUnloading();
    TestFuelConsumptionAndMixtureCutoff();
    TestEngineMomentsAndPFactor();

    std::cout << "\n----------------------------------------\n";
    std::cout << "Tests Passed: " << g_testsPassed << "\n";
    std::cout << "Tests Failed: " << g_testsFailed << "\n";
    std::cout << "----------------------------------------\n";

    return (g_testsFailed == 0) ? 0 : 1;
}
