#include "LokYS/physics/GroundReactions.hpp"
#include "LokYS/physics/RigidBody.hpp"
#include "LokYS/math/Units.hpp"
#include <iostream>
#include <cmath>

using namespace LokYS;
using namespace LokYS::Math;
using namespace LokYS::Physics;

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

void TestStaticEquilibriumOnRunway() {
    GroundReactionsSystem gearSys;
    f64 weight = 1111.0 * Constants::STANDARD_GRAVITY; // 10,895 N

    // Aircraft resting on runway in pitch-trim equilibrium: Altitude ~0.947m, pitch ~0.6 deg (0.0105 rad)
    RigidBodyState state{};
    state.positionNed = { 0.0, 0.0, -0.947 };
    state.velocityBody = { 0.0, 0.0, 0.0 };
    state.angularVelocityBody = { 0.0, 0.0, 0.0 };
    f64 pitchEq = Units::DegToRad(0.6);
    state.attitude = Quat4d::FromEuler(0.0, pitchEq, 0.0);

    auto out = gearSys.Evaluate(state, 0.0, 0.0, 1.0 / 120.0, 0.0);

    TEST_ASSERT(out.anyGearInContact, "All gear in contact in static rest");
    TEST_ASSERT(out.noseInContact, "Nose gear in contact");
    TEST_ASSERT(out.leftMainInContact, "Left main gear in contact");
    TEST_ASSERT(out.rightMainInContact, "Right main gear in contact");

    // Total normal force should closely match aircraft weight (~10,895 N)
    TEST_NEAR(out.totalNormalForce, weight, 600.0, "Total normal force balances gross weight");


    // Load distribution check: Nose ~15-22%, Left Main ~38-43%, Right Main ~38-43%
    f64 noseRatio = out.gearStates[0].normalForceNewtons / out.totalNormalForce;
    f64 leftRatio = out.gearStates[1].normalForceNewtons / out.totalNormalForce;
    f64 rightRatio = out.gearStates[2].normalForceNewtons / out.totalNormalForce;

    TEST_ASSERT(noseRatio >= 0.12 && noseRatio <= 0.25, "Nose gear carries 12-25% of weight");
    TEST_ASSERT(leftRatio >= 0.35 && leftRatio <= 0.45, "Left main carries 35-45% of weight");
    TEST_ASSERT(rightRatio >= 0.35 && rightRatio <= 0.45, "Right main carries 35-45% of weight");
    TEST_NEAR(leftRatio, rightRatio, 0.01, "Symmetric main gear loading");

    // Static moments balance: zero rolling moment on level runway
    TEST_NEAR(out.totalMomentBody.x, 0.0, 1e-4, "Zero rolling moment on level runway");
}

void TestDynamicDropAndEnergyDissipation() {
    GroundReactionsSystem gearSys;
    RigidBody aircraft(1111.0, Mat3d(
        1285.0, 0.0, 0.0,
        0.0, 1824.0, 0.0,
        0.0, 0.0, 2667.0
    ));
    aircraft.SetGravityEnabled(true);

    // Initial state: dropped from 1.5m height with downward sink rate 2.5 m/s (~500 fpm touchdown)
    RigidBodyState state{};
    state.positionNed = { 0.0, 0.0, -1.5 };
    state.velocityBody = { 0.0, 0.0, 2.5 };
    state.attitude = Quat4d::Identity();
    aircraft.SetState(state);

    constexpr f64 dt = 1.0 / 120.0;
    constexpr int totalTicks = 240; // 2.0 seconds of simulation

    f64 maxCompressionSeen = 0.0;

    for (int tick = 0; tick < totalTicks; ++tick) {
        auto out = gearSys.Evaluate(aircraft.GetState(), 0.0, 0.0, dt, 0.0);
        gearSys.ApplyToRigidBody(aircraft, out);

        for (const auto& g : out.gearStates) {
            if (g.inContact && g.compression > maxCompressionSeen) {
                maxCompressionSeen = g.compression;
            }
        }

        aircraft.IntegrateRK4(dt);
    }

    const auto& finalState = aircraft.GetState();

    // Strut should absorb impact without bottoming out beyond maximum limit
    TEST_ASSERT(maxCompressionSeen > 0.05, "Oleo strut actively compressed during touchdown");
    TEST_ASSERT(maxCompressionSeen < 0.25, "Touchdown shock absorbed within allowable stroke");

    // Settled height after 2 seconds: Aircraft rests steadily near ~0.93m altitude
    f64 finalAltitude = -finalState.positionNed.z;
    TEST_NEAR(finalAltitude, 0.935, 0.06, "Aircraft settled to static runway height");

    // Vertical oscillation dissipated by oleo damping: |V_z| < 0.1 m/s
    TEST_ASSERT(std::abs(finalState.velocityBody.z) < 0.10, "Oleo damping dissipated vertical velocity");
}

void TestRunwayBrakingFriction() {
    GroundReactionsSystem gearSys;
    RigidBodyState state{};
    state.positionNed = { 0.0, 0.0, -0.935 };
    state.velocityBody = { 25.0, 0.0, 0.0 }; // Rolling at 25 m/s (~50 knots)
    state.attitude = Quat4d::Identity();

    // 1. Free rolling (no brakes)
    auto outFree = gearSys.Evaluate(state, 0.0, 0.0, 1.0 / 120.0, 0.0);
    f64 freeRollingForceX = outFree.totalForceBody.x;
    TEST_ASSERT(freeRollingForceX < 0.0, "Rolling resistance opposes forward motion");

    // 2. Maximum wheel braking
    auto outBraking = gearSys.Evaluate(state, 0.0, 1.0, 1.0 / 120.0, 0.0);
    f64 brakingForceX = outBraking.totalForceBody.x;
    TEST_ASSERT(brakingForceX < freeRollingForceX, "Brakes substantially increase retarding force");
    TEST_ASSERT(brakingForceX < -3000.0, "Braking force exceeds 3 kN for 1111kg airframe");

    // Expected deceleration: a = F / m in range [-3.0, -5.0] m/s^2
    f64 decel = brakingForceX / 1111.0;
    TEST_ASSERT(decel <= -2.5 && decel >= -6.0, "Braking deceleration is physically credible");
}

void TestNosewheelSteeringAuthority() {
    GroundReactionsSystem gearSys;
    RigidBodyState state{};
    state.positionNed = { 0.0, 0.0, -0.935 };
    state.velocityBody = { 5.0, 0.0, 0.0 }; // Taxi speed 5 m/s (~10 knots)
    state.attitude = Quat4d::Identity();

    // 1. Right rudder (+1.0) steering right
    auto outRight = gearSys.Evaluate(state, 1.0, 0.0, 1.0 / 120.0, 0.0);
    TEST_ASSERT(outRight.noseInContact, "Nose wheel in contact during taxi steering");
    // Right steer produces positive yawing moment (+Z moment in Body FRD = clockwise/right)
    TEST_ASSERT(outRight.totalMomentBody.z > 200.0, "Right rudder produces positive clockwise yaw moment");

    // 2. Left rudder (-1.0) steering left
    auto outLeft = gearSys.Evaluate(state, -1.0, 0.0, 1.0 / 120.0, 0.0);
    TEST_ASSERT(outLeft.totalMomentBody.z < -200.0, "Left rudder produces negative counter-clockwise yaw moment");

    // 3. Neutral rudder (0.0)
    auto outNeutral = gearSys.Evaluate(state, 0.0, 0.0, 1.0 / 120.0, 0.0);
    TEST_NEAR(outNeutral.totalMomentBody.z, 0.0, 1e-4, "Neutral rudder produces zero yaw moment");
}

void TestTailstrikeDetection() {
    GroundReactionsSystem gearSys;
    RigidBodyState state{};
    state.positionNed = { 0.0, 0.0, -0.85 }; // Low altitude with high pitch

    // Rotate nose up by 16 degrees (0.279 rad pitch)
    f64 pitch16 = Units::DegToRad(16.0);
    state.attitude = Quat4d::FromEuler(0.0, pitch16, 0.0);
    state.velocityBody = { 20.0, 0.0, 0.0 };

    auto out = gearSys.Evaluate(state, 0.0, 0.0, 1.0 / 120.0, 0.0);
    TEST_ASSERT(out.tailstrikeDetected, "Tailstrike detected at 16 degrees over-rotation");
    TEST_ASSERT(out.tailBumperState.normalForceNewtons > 0.0, "Tail bumper generates normal reaction force");
    // Moment should push nose down (negative pitching moment)
    TEST_ASSERT(out.tailBumperState.momentBody.y < 0.0, "Tailstrike produces nose-down restoring moment");
}

int main() {
    std::cout << "========================================\n";
    std::cout << " LokYS Ground Reactions & Strut Tests   \n";
    std::cout << "========================================\n";

    TestStaticEquilibriumOnRunway();
    TestDynamicDropAndEnergyDissipation();
    TestRunwayBrakingFriction();
    TestNosewheelSteeringAuthority();
    TestTailstrikeDetection();

    std::cout << "\n----------------------------------------\n";
    std::cout << "Tests Passed: " << g_testsPassed << "\n";
    std::cout << "Tests Failed: " << g_testsFailed << "\n";
    std::cout << "----------------------------------------\n";

    return (g_testsFailed == 0) ? 0 : 1;
}
