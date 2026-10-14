#include "LokYS/physics/RigidBody.hpp"
#include "LokYS/simulation/SimulationScheduler.hpp"
#include "LokYS/math/Units.hpp"
#include <iostream>
#include <cmath>

using namespace LokYS;
using namespace LokYS::Math;
using namespace LokYS::Physics;
using namespace LokYS::Simulation;

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

// Test 1: Analytical Gravity Free-Fall & Forward Projectile
void TestBallisticGravityTrajectory() {
    RigidBody body(1000.0, Mat3d::Identity());
    body.SetGravityEnabled(true);

    // Initial state: Level attitude, forward speed 50 m/s
    RigidBodyState initial{};
    initial.positionNed = { 0.0, 0.0, 0.0 };
    initial.velocityBody = { 50.0, 0.0, 0.0 }; // 50 m/s forward
    initial.attitude = Quat4d::Identity();
    initial.angularVelocityBody = { 0.0, 0.0, 0.0 };
    body.SetState(initial);

    constexpr f64 dt = 1.0 / 120.0; // 120 Hz
    constexpr f64 targetTime = 5.0;  // 5 seconds
    constexpr u32 totalSteps = static_cast<u32>(targetTime / dt);

    for (u32 i = 0; i < totalSteps; ++i) {
        body.IntegrateRK4(dt);
    }

    const auto& finalState = body.GetState();

    // Analytical predictions:
    // x(5) = v_x * t = 50 * 5 = 250 m
    // z(5) = 0.5 * g * t^2 = 0.5 * 9.80665 * 25 = 122.583125 m
    // v_z(5) = g * t = 9.80665 * 5 = 49.03325 m/s
    f64 expectedX = 250.0;
    f64 expectedZ = 0.5 * Constants::STANDARD_GRAVITY * 25.0;
    f64 expectedVz = Constants::STANDARD_GRAVITY * 5.0;

    TEST_NEAR(finalState.positionNed.x, expectedX, 1e-3, "Ballistic X position match");
    TEST_NEAR(finalState.positionNed.z, expectedZ, 1e-3, "Ballistic Z position match");
    TEST_NEAR(finalState.velocityBody.x, 50.0, 1e-3, "Ballistic body forward speed preserved");
    TEST_NEAR(finalState.velocityBody.z, expectedVz, 1e-3, "Ballistic body downward velocity match");
}

// Test 2: Free Rotational Spin & Angular Momentum
void TestRotationalDynamics() {
    // Principal moments of inertia: Ixx=1000, Iyy=2000, Izz=3000
    Mat3d inertia(
        1000.0, 0.0, 0.0,
        0.0, 2000.0, 0.0,
        0.0, 0.0, 3000.0
    );
    RigidBody body(1000.0, inertia);
    body.SetGravityEnabled(false);

    // Initial spin: Roll rate p = 1.0 rad/s (~57.3 deg/s)
    RigidBodyState initial{};
    initial.angularVelocityBody = { 1.0, 0.0, 0.0 };
    body.SetState(initial);

    constexpr f64 dt = 1.0 / 120.0;
    for (int i = 0; i < 120; ++i) { // 1 second
        body.IntegrateRK4(dt);
    }

    const auto& s = body.GetState();
    // In torque-free spin about a principal axis, angular velocity must remain constant
    TEST_NEAR(s.angularVelocityBody.x, 1.0, 1e-5, "Principal axis spin rate conservation");
    TEST_NEAR(s.angularVelocityBody.y, 0.0, 1e-5, "Zero cross-axis roll-pitch coupling");
    TEST_NEAR(s.angularVelocityBody.z, 0.0, 1e-5, "Zero cross-axis roll-yaw coupling");

    // Quaternion norm must remain strictly 1.0
    TEST_NEAR(s.attitude.Norm(), 1.0, 1e-6, "Attitude quaternion norm invariant");
}

// Test 3: Long-Horizon Quaternion Drift (10,000 Steps)
void TestLongHorizonQuaternionNorm() {
    RigidBody body(1000.0, Mat3d::Identity());
    body.SetGravityEnabled(false);

    // Dynamic 3-axis rotation
    RigidBodyState state{};
    state.angularVelocityBody = { 0.5, 0.8, -0.3 };
    body.SetState(state);

    constexpr f64 dt = 1.0 / 120.0;
    for (int i = 0; i < 10000; ++i) { // 83.3 simulated seconds
        body.IntegrateRK4(dt);
    }

    f64 norm = body.GetState().attitude.Norm();
    TEST_NEAR(norm, 1.0, 1e-6, "10,000 steps quaternion normalization invariant");
}

// Test 4: Simulation Scheduler Accumulator & Cap
void TestSimulationScheduler() {
    SimulationScheduler scheduler(1.0 / 120.0, 5);
    u32 totalTicks = 0;

    auto stepFn = [&](f64 /*dt*/, u64 /*tick*/, f64 /*simTime*/) {
        totalTicks++;
    };

    // Frame 1: 16.666 ms (60 FPS frame) -> exactly 2 physics sub-steps at 120 Hz
    u32 steps1 = scheduler.Update(1.0 / 60.0, stepFn);
    TEST_ASSERT(steps1 == 2, "60 FPS frame produces 2 physics sub-steps");

    // Frame 2: 8.333 ms (120 FPS frame) -> exactly 1 physics sub-step
    u32 steps2 = scheduler.Update(1.0 / 120.0, stepFn);
    TEST_ASSERT(steps2 == 1, "120 FPS frame produces 1 physics sub-step");

    // Frame 3: Massive lag spike (200 ms) -> capped to maxSubSteps (5)
    u32 steps3 = scheduler.Update(0.200, stepFn);
    TEST_ASSERT(steps3 <= 5, "Lag spike clamped to maxSubSteps to prevent spiral of death");

    // Test pause/resume
    scheduler.Pause();
    u32 stepsPaused = scheduler.Update(0.100, stepFn);
    TEST_ASSERT(stepsPaused == 0, "Zero steps executed while paused");

    scheduler.Resume();
    u32 stepsResumed = scheduler.Update(0.0166666, stepFn);
    TEST_ASSERT(stepsResumed > 0, "Execution resumes after unpausing");
}

// Test 5: Snapshot Telemetry & Interpolation
void TestSnapshotTelemetry() {
    RigidBodyState s1{};
    s1.positionNed = { 0.0, 0.0, -1000.0 }; // 1000 m altitude
    s1.velocityBody = { 51.4444, 0.0, -5.0 }; // 100 knots, 5 m/s climb
    s1.attitude = Quat4d::Identity();

    auto snap1 = SimulationSnapshot::CreateFromRigidBody(1, 0.0, s1);
    TEST_NEAR(snap1.altitudeMeters, 1000.0, 1e-3, "Snapshot Altitude Meters");
    TEST_NEAR(snap1.airspeedKnots, 100.0, 0.5, "Snapshot Airspeed Knots");
    TEST_NEAR(snap1.verticalSpeedFpm, 5.0 * 196.85039, 1e-1, "Snapshot VSI fpm");

    RigidBodyState s2{};
    s2.positionNed = { 100.0, 0.0, -1010.0 }; // 1010 m altitude
    s2.velocityBody = { 51.4444, 0.0, -5.0 };
    s2.attitude = Quat4d::Identity();
    auto snap2 = SimulationSnapshot::CreateFromRigidBody(2, 0.008333, s2);

    auto interp = SimulationSnapshot::Interpolate(snap1, snap2, 0.5);
    TEST_NEAR(interp.altitudeMeters, 1005.0, 1e-3, "Interpolated Altitude");
    TEST_NEAR(interp.bodyState.positionNed.x, 50.0, 1e-3, "Interpolated X position");
}

int main() {
    std::cout << "========================================\n";
    std::cout << " LokYS 6-DOF Physics & Scheduler Tests  \n";
    std::cout << "========================================\n";

    TestBallisticGravityTrajectory();
    TestRotationalDynamics();
    TestLongHorizonQuaternionNorm();
    TestSimulationScheduler();
    TestSnapshotTelemetry();

    std::cout << "\n----------------------------------------\n";
    std::cout << "Tests Passed: " << g_testsPassed << "\n";
    std::cout << "Tests Failed: " << g_testsFailed << "\n";
    std::cout << "----------------------------------------\n";

    return (g_testsFailed == 0) ? 0 : 1;
}
