#include "LokYS/avionics/AvionicsSystem.hpp"
#include <iostream>
#include <cassert>
#include <cmath>

#define TEST_ASSERT(cond, msg) \
    do { \
        if (!(cond)) { \
            std::cerr << "Assertion failed: " << (msg) << " (" << __FILE__ << ":" << __LINE__ << ")\n"; \
            return false; \
        } \
    } while (0)

namespace {

using namespace LokYS;

bool TestAirspeedIndicator() {
    std::cout << "[RUNNING] TestAirspeedIndicator...\n";

    LokYS::Avionics::AvionicsSystem avionics;

    // 1. Static ground state (0 m/s)
    LokYS::Physics::RigidBodyState state{};
    state.velocityBody = { 0.0, 0.0, 0.0 };
    avionics.Update(state, { 0.0, 0.0, 0.0 }, 101325.0, 1.225, 0.1);

    const auto& asi0 = avionics.GetAirspeed();
    TEST_ASSERT(asi0.indicatedAirspeedKnots < 0.1, "IAS should be 0 at rest");
    TEST_ASSERT(asi0.dynamicPressurePa < 0.1, "Dynamic pressure should be 0 at rest");

    // 2. Step input to 50 m/s (~97.19 knots) at sea level (rho = 1.225)
    // Run for 3 seconds to reach steady state (tau = 0.10s -> 30 time constants)
    state.velocityBody = { 50.0, 0.0, 0.0 };
    for (int i = 0; i < 300; ++i) {
        avionics.Update(state, { 0.0, 0.0, 0.0 }, 101325.0, 1.225, 0.01);
    }

    const auto& asi1 = avionics.GetAirspeed();
    f64 expectedKnots = LokYS::Math::Units::MpsToKnots(50.0);
    TEST_ASSERT(std::abs(asi1.indicatedAirspeedKnots - expectedKnots) < 0.5, "IAS should reach ~97.2 knots");
    TEST_ASSERT(std::abs(asi1.trueAirspeedKnots - expectedKnots) < 0.5, "TAS should match IAS at sea level");
    TEST_ASSERT(asi1.dynamicPressurePa > 1500.0, "Dynamic pressure should be ~1531 Pa");

    // Needle angle check: 97.2 kts is in 40-200 range
    // angle = 20 + ((97.2 - 40) / 160) * 320 = 20 + 0.3575 * 320 = ~134 deg
    TEST_ASSERT(asi1.needleAngleDeg > 120.0 && asi1.needleAngleDeg < 150.0, "Needle angle should be ~134 degrees");

    // 3. Sensor lag verification: 1 time constant (tau = 0.10s)
    avionics.ResetToSteadyState(state, 101325.0, 1.225);
    // Sudden acceleration to 100 m/s
    state.velocityBody = { 100.0, 0.0, 0.0 };
    f64 initialIas = avionics.GetAirspeed().indicatedAirspeedKnots;
    f64 targetIas = LokYS::Math::Units::MpsToKnots(100.0);

    // Advance exactly 1 tau (0.10s)
    avionics.Update(state, { 0.0, 0.0, 0.0 }, 101325.0, 1.225, 0.10);
    f64 iasAt1Tau = avionics.GetAirspeed().indicatedAirspeedKnots;
    f64 expectedAt1Tau = initialIas + (targetIas - initialIas) * (1.0 - 1.0 / std::numbers::e);
    TEST_ASSERT(std::abs(iasAt1Tau - expectedAt1Tau) < 2.0, "Pneumatic lag should match first-order exponential response");

    std::cout << "[PASSED] TestAirspeedIndicator\n";
    return true;
}

bool TestAttitudeIndicator() {
    std::cout << "[RUNNING] TestAttitudeIndicator...\n";

    LokYS::Avionics::AvionicsSystem avionics;

    // 1. Level flight
    LokYS::Physics::RigidBodyState state{};
    state.attitude = LokYS::Math::Quat4d::Identity();
    avionics.ResetToSteadyState(state, 101325.0, 1.225);

    const auto& ai0 = avionics.GetAttitude();
    TEST_ASSERT(std::abs(ai0.pitchDeg) < 0.01, "Pitch should be 0 in level flight");
    TEST_ASSERT(std::abs(ai0.rollDeg) < 0.01, "Roll should be 0 in level flight");
    TEST_ASSERT(std::abs(ai0.pitchDisplacementNorm) < 0.01, "Horizon bar should be centered");
    TEST_ASSERT(!ai0.isCaged, "Gyro should start uncaged");
    TEST_ASSERT(!ai0.isTumbled, "Gyro should start untumbled");

    // 2. Climb & Bank: Pitch = +12 deg, Roll = +25 deg
    f64 pitchRad = LokYS::Math::Units::DegToRad(12.0);
    f64 rollRad = LokYS::Math::Units::DegToRad(25.0);
    state.attitude = LokYS::Math::Quat4d::FromEuler(0.0, pitchRad, rollRad);

    // Advance 1 second (tau = 0.05s -> 20 time constants)
    for (int i = 0; i < 100; ++i) {
        avionics.Update(state, { 0.0, 0.0, 0.0 }, 101325.0, 1.225, 0.01);
    }

    const auto& ai1 = avionics.GetAttitude();
    TEST_ASSERT(std::abs(ai1.pitchDeg - 12.0) < 0.2, "Indicated pitch should converge to +12 deg");
    TEST_ASSERT(std::abs(ai1.rollDeg - 25.0) < 0.2, "Indicated roll should converge to +25 deg");
    TEST_ASSERT(ai1.pitchDisplacementNorm > 0.5 && ai1.pitchDisplacementNorm < 0.7, "Horizon should shift proportionally to pitch");

    // 3. Caging
    avionics.CageAttitude(true);
    const auto& ai2 = avionics.GetAttitude();
    TEST_ASSERT(ai2.isCaged, "Attitude indicator should be caged");
    TEST_ASSERT(std::abs(ai2.pitchDeg) < 0.001, "Caged pitch should be 0");
    TEST_ASSERT(std::abs(ai2.rollDeg) < 0.001, "Caged roll should be 0");

    // Update while caged: should remain locked at 0
    avionics.Update(state, { 0.0, 0.0, 0.0 }, 101325.0, 1.225, 0.1);
    TEST_ASSERT(std::abs(avionics.GetAttitude().pitchDeg) < 0.001, "Caged gyro must not deflect");

    // Uncage
    avionics.CageAttitude(false);
    TEST_ASSERT(!avionics.GetAttitude().isCaged, "Should be uncaged");

    // 4. Gimbal tumble detection: Pitch > 85 deg
    state.attitude = LokYS::Math::Quat4d::FromEuler(0.0, LokYS::Math::Units::DegToRad(88.0), 0.0);
    for (int i = 0; i < 50; ++i) {
        avionics.Update(state, { 0.0, 0.0, 0.0 }, 101325.0, 1.225, 0.01);
    }
    TEST_ASSERT(avionics.GetAttitude().isTumbled, "Tumble flag should be set when pitch exceeds 85 deg");

    std::cout << "[PASSED] TestAttitudeIndicator\n";
    return true;
}

bool TestAltimeter() {
    std::cout << "[RUNNING] TestAltimeter...\n";

    LokYS::Avionics::AvionicsSystem avionics;

    // 1. Sea Level standard pressure: 101,325 Pa, Kollsman 29.92 inHg
    LokYS::Physics::RigidBodyState state{};
    state.positionNed = { 0.0, 0.0, 0.0 };
    avionics.ResetToSteadyState(state, 101325.0, 1.225);

    const auto& alt0 = avionics.GetAltimeter();
    TEST_ASSERT(std::abs(alt0.indicatedAltitudeFeet) < 2.0, "Altimeter should read ~0 ft at sea level");
    TEST_ASSERT(std::abs(alt0.pressureAltitudeFeet) < 2.0, "Pressure altitude should read ~0 ft");
    TEST_ASSERT(std::abs(alt0.kollsmanInHg - 29.92) < 0.001, "Default Kollsman should be 29.92");

    // 2. Climb to 3,000 ft (914.4m)
    // Standard pressure at 3,000 ft: P = 101325 * (1 - 0.0065 * 914.4 / 288.15)^5.25588 ~ 90,812 Pa
    f64 altM = 914.4;
    f64 pAt3000Ft = 101325.0 * std::pow(1.0 - 0.0065 * altM / 288.15, 5.25588);
    state.positionNed = { 0.0, 0.0, -altM };

    // Advance 3 seconds (tau = 0.15s -> 20 time constants)
    for (int i = 0; i < 300; ++i) {
        avionics.Update(state, { 0.0, 0.0, 0.0 }, pAt3000Ft, 1.12, 0.01);
    }

    const auto& alt1 = avionics.GetAltimeter();
    TEST_ASSERT(std::abs(alt1.indicatedAltitudeFeet - 3000.0) < 15.0, "Indicated altitude should reach 3,000 ft (+/- 15ft)");

    // 3. Multi-needle angle check at 3,000 ft:
    // 100-ft needle (1,000 ft/rev): 3000 ft is exactly 3 revs -> angle = 0 deg (12 o'clock)
    // 1,000-ft needle (10,000 ft/rev): 3000 / 10000 * 360 = 108 deg (3 o'clock)
    // 10,000-ft needle (100,000 ft/rev): 3000 / 100000 * 360 = 10.8 deg
    TEST_ASSERT(std::abs(alt1.needle100FtAngleDeg) < 5.0 || std::abs(alt1.needle100FtAngleDeg - 360.0) < 5.0,
        "100-ft needle should be at ~0 deg for 3,000 ft");
    TEST_ASSERT(std::abs(alt1.needle1000FtAngleDeg - 108.0) < 5.0,
        "1,000-ft needle should be at ~108 deg for 3,000 ft");

    // 4. Kollsman subscale adjustment:
    // Changing Kollsman from 29.92 to 30.02 (+0.10 inHg) should increase indicated altitude by ~100 ft
    f64 altBeforeKollsman = alt1.indicatedAltitudeFeet;
    avionics.AdjustKollsman(0.10);
    for (int i = 0; i < 200; ++i) {
        avionics.Update(state, { 0.0, 0.0, 0.0 }, pAt3000Ft, 1.12, 0.01);
    }
    f64 altAfterKollsman = avionics.GetAltimeter().indicatedAltitudeFeet;
    f64 deltaAlt = altAfterKollsman - altBeforeKollsman;
    TEST_ASSERT(deltaAlt > 80.0 && deltaAlt < 115.0, "Kollsman +0.10 inHg should shift indicated alt by ~95-100 ft");

    std::cout << "[PASSED] TestAltimeter\n";
    return true;
}

bool TestTurnCoordinator() {
    std::cout << "[RUNNING] TestTurnCoordinator...\n";

    LokYS::Avionics::AvionicsSystem avionics;

    // 1. Straight and level: 0 turn rate, 0 ball deflection
    LokYS::Physics::RigidBodyState state{};
    state.angularVelocityBody = { 0.0, 0.0, 0.0 };
    avionics.Update(state, { 0.0, 0.0, 0.0 }, 101325.0, 1.225, 0.1);

    const auto& tc0 = avionics.GetTurnCoordinator();
    TEST_ASSERT(std::abs(tc0.turnRateDps) < 0.01, "Turn rate should be 0");
    TEST_ASSERT(std::abs(tc0.symbolRollDeg) < 0.01, "Symbol bank should be 0");
    TEST_ASSERT(std::abs(tc0.slipSkidBallPosition) < 0.01, "Ball should be centered");

    // 2. Standard rate turn (3.0 deg/sec = 0.05236 rad/s pure yaw rate)
    f64 stdRateRad = LokYS::Math::Units::DegToRad(3.0);
    state.angularVelocityBody = { 0.0, 0.0, stdRateRad };

    // In a steady coordinated turn, centripetal acceleration equals horizontal gravity
    // a_y = g * sin(bank) * cos(pitch) -> latSpecificForce = 0 -> ball stays centered
    f64 bankAngleRad = LokYS::Math::Units::DegToRad(14.0); // typical coordinated bank for 100 kts at std rate
    state.attitude = LokYS::Math::Quat4d::FromEuler(0.0, 0.0, bankAngleRad);
    Math::Vec3d coordinatedAccel{ 0.0, LokYS::Math::Constants::STANDARD_GRAVITY * std::sin(bankAngleRad), 0.0 };

    // Advance 3 seconds (tau = 0.15s -> 20 time constants)
    for (int i = 0; i < 300; ++i) {
        avionics.Update(state, coordinatedAccel, 101325.0, 1.225, 0.01);
    }

    const auto& tc1 = avionics.GetTurnCoordinator();
    // Because gyro is canted 30 deg, sensed yaw rate is r * cos(30 deg) = 3.0 * 0.866 ~ 2.6 deg/s
    TEST_ASSERT(tc1.turnRateDps > 2.0 && tc1.turnRateDps < 3.2, "Turn rate should register standard rate turn");
    TEST_ASSERT(tc1.symbolRollDeg > 12.0 && tc1.symbolRollDeg < 22.0, "Symbol should deflect to standard rate bank");
    TEST_ASSERT(std::abs(tc1.slipSkidBallPosition) < 0.05, "Ball must remain centered in coordinated turn");

    // 3. Slipping turn: insufficient centripetal acceleration (ball falls to inside/low wing)
    Math::Vec3d slippingAccel{ 0.0, 0.5, 0.0 };
    for (int i = 0; i < 200; ++i) {
        avionics.Update(state, slippingAccel, 101325.0, 1.225, 0.01);
    }
    const auto& tc2 = avionics.GetTurnCoordinator();
    TEST_ASSERT(tc2.slipSkidBallPosition > 0.15, "Ball should fall toward inside/positive side in slip");

    // 4. Skidding turn: excessive centripetal acceleration (ball swings to outside/high wing)
    Math::Vec3d skiddingAccel{ 0.0, 4.0, 0.0 };
    for (int i = 0; i < 200; ++i) {
        avionics.Update(state, skiddingAccel, 101325.0, 1.225, 0.01);
    }
    const auto& tc3 = avionics.GetTurnCoordinator();
    TEST_ASSERT(tc3.slipSkidBallPosition < -0.15, "Ball should swing toward outside/negative side in skid");

    std::cout << "[PASSED] TestTurnCoordinator\n";
    return true;
}

bool TestHeadingIndicator() {
    std::cout << "[RUNNING] TestHeadingIndicator...\n";

    LokYS::Avionics::AvionicsSystem avionics;

    // 1. Heading North (0 deg)
    LokYS::Physics::RigidBodyState state{};
    state.attitude = LokYS::Math::Quat4d::FromEuler(0.0, 0.0, 0.0);
    avionics.ResetToSteadyState(state, 101325.0, 1.225);

    const auto& hdg0 = avionics.GetHeading();
    TEST_ASSERT(std::abs(hdg0.indicatedHeadingDeg) < 0.01, "Heading should be 0 North");
    TEST_ASSERT(std::abs(hdg0.compassCardAngleDeg) < 0.01, "Card angle should be 0");

    // 2. Turn to East (90 deg)
    state.attitude = LokYS::Math::Quat4d::FromEuler(LokYS::Math::Units::DegToRad(90.0), 0.0, 0.0);
    for (int i = 0; i < 100; ++i) {
        avionics.Update(state, { 0.0, 0.0, 0.0 }, 101325.0, 1.225, 0.01);
    }
    const auto& hdg1 = avionics.GetHeading();
    TEST_ASSERT(std::abs(hdg1.indicatedHeadingDeg - 90.0) < 0.2, "Heading should reach 90 East");
    TEST_ASSERT(std::abs(hdg1.compassCardAngleDeg + 90.0) < 0.2, "Compass card rotates counter-clockwise by -90 deg");

    // 3. 360-degree wrap-around (turn from 355 deg to 5 deg)
    state.attitude = LokYS::Math::Quat4d::FromEuler(LokYS::Math::Units::DegToRad(355.0), 0.0, 0.0);
    avionics.ResetToSteadyState(state, 101325.0, 1.225);
    state.attitude = LokYS::Math::Quat4d::FromEuler(LokYS::Math::Units::DegToRad(5.0), 0.0, 0.0);
    for (int i = 0; i < 100; ++i) {
        avionics.Update(state, { 0.0, 0.0, 0.0 }, 101325.0, 1.225, 0.01);
    }
    TEST_ASSERT(std::abs(avionics.GetHeading().indicatedHeadingDeg - 5.0) < 0.2,
        "Heading should cross 360/0 cleanly without wrapping jumps");

    // 4. Heading Bug Adjustment
    avionics.SetHeadingBug(270.0);
    TEST_ASSERT(std::abs(avionics.GetHeadingBug() - 270.0) < 0.01, "Heading bug should be set to 270 West");
    avionics.AdjustHeadingBug(15.0);
    TEST_ASSERT(std::abs(avionics.GetHeadingBug() - 285.0) < 0.01, "Heading bug should adjust to 285 deg");

    // 5. Gyro Precession Drift & Compass Alignment
    // Advance 900 seconds (15 minutes). At 3 deg / 15 min, drift should be ~3.0 degrees
    for (int i = 0; i < 900; ++i) {
        avionics.Update(state, { 0.0, 0.0, 0.0 }, 101325.0, 1.225, 1.0);
    }
    TEST_ASSERT(std::abs(avionics.GetHeading().gyroDriftDeg - 3.0) < 0.1, "Gyro should drift ~3 deg after 15 min");

    // Align with compass
    avionics.AlignHeadingWithCompass();
    TEST_ASSERT(std::abs(avionics.GetHeading().gyroDriftDeg) < 0.01, "Drift should be reset on compass alignment");

    std::cout << "[PASSED] TestHeadingIndicator\n";
    return true;
}

bool TestVerticalSpeedIndicator() {
    std::cout << "[RUNNING] TestVerticalSpeedIndicator...\n";

    LokYS::Avionics::AvionicsSystem avionics;

    // 1. Level flight: 0 vertical speed
    LokYS::Physics::RigidBodyState state{};
    state.velocityBody = { 50.0, 0.0, 0.0 };
    state.attitude = LokYS::Math::Quat4d::Identity();
    avionics.ResetToSteadyState(state, 101325.0, 1.225);

    const auto& vsi0 = avionics.GetVerticalSpeed();
    TEST_ASSERT(std::abs(vsi0.verticalSpeedFpm) < 0.1, "VSI should be 0 in level flight");
    TEST_ASSERT(std::abs(vsi0.needleAngleDeg) < 0.1, "VSI needle should be horizontal at 0");

    // 2. Climb Step Input: 5.08 m/s vertical speed = 1,000 fpm
    // Pitch up slightly so body velocity produces upward NED motion (v_D = -5.08 m/s)
    f64 targetFpm = 1000.0;
    f64 vUpMps = LokYS::Math::Units::FeetPerMinuteToMps(targetFpm);
    // Attitude pitched up so velocity produces vUpMps
    f64 pitchRad = std::asin(vUpMps / 50.0);
    state.attitude = LokYS::Math::Quat4d::FromEuler(0.0, pitchRad, 0.0);

    // 3. Test Capillary Pneumatic Metering Lag:
    // With tau_vsi = 2.50s, after exactly 2.50 seconds of climb,
    // indicated VSI should reach 1000 * (1 - 1/e) ~ 632.1 fpm
    for (int i = 0; i < 250; ++i) {
        avionics.Update(state, { 0.0, 0.0, 0.0 }, 101325.0, 1.225, 0.01);
    }
    const auto& vsi1 = avionics.GetVerticalSpeed();
    f64 expectedAt1Tau = targetFpm * (1.0 - 1.0 / std::numbers::e);
    TEST_ASSERT(std::abs(vsi1.verticalSpeedFpm - expectedAt1Tau) < 15.0,
        "VSI should exhibit authentic 2.5s capillary metering lag (~632 fpm after 2.5s)");

    // 4. Steady-state convergence: After 15 seconds (6 tau), should reach 1,000 fpm
    for (int i = 0; i < 1250; ++i) {
        avionics.Update(state, { 0.0, 0.0, 0.0 }, 101325.0, 1.225, 0.01);
    }
    const auto& vsi2 = avionics.GetVerticalSpeed();
    TEST_ASSERT(std::abs(vsi2.verticalSpeedFpm - 1000.0) < 5.0, "VSI should converge to steady 1,000 fpm");
    // Needle angle: 1000 fpm / 2000 fpm * 170 deg = +85 deg
    TEST_ASSERT(std::abs(vsi2.needleAngleDeg - 85.0) < 2.0, "VSI needle should be at +85 deg for +1000 fpm climb");

    // 5. Descent Step Input: -1,500 fpm
    f64 descendFpm = -1500.0;
    f64 vDnMps = LokYS::Math::Units::FeetPerMinuteToMps(descendFpm);
    f64 pitchDnRad = std::asin(vDnMps / 50.0);
    state.attitude = LokYS::Math::Quat4d::FromEuler(0.0, pitchDnRad, 0.0);

    for (int i = 0; i < 2500; ++i) {
        avionics.Update(state, { 0.0, 0.0, 0.0 }, 101325.0, 1.225, 0.01);
    }
    const auto& vsi3 = avionics.GetVerticalSpeed();
    TEST_ASSERT(std::abs(vsi3.verticalSpeedFpm - (-1500.0)) < 5.0, "VSI should converge to steady -1,500 fpm descent");
    // Needle angle: -1500 / 2000 * 170 deg = -127.5 deg
    TEST_ASSERT(std::abs(vsi3.needleAngleDeg - (-127.5)) < 2.0, "VSI needle should point downward to -127.5 deg");

    std::cout << "[PASSED] TestVerticalSpeedIndicator\n";
    return true;
}

} // anonymous namespace

int main() {
    std::cout << "====================================================\n";
    std::cout << "LokYS Analog Six-Pack Avionics Test Suite (Milestone M14)\n";
    std::cout << "====================================================\n";

    if (!TestAirspeedIndicator()) return 1;
    if (!TestAttitudeIndicator()) return 1;
    if (!TestAltimeter()) return 1;
    if (!TestTurnCoordinator()) return 1;
    if (!TestHeadingIndicator()) return 1;
    if (!TestVerticalSpeedIndicator()) return 1;

    std::cout << "====================================================\n";
    std::cout << "ALL AVIONICS TEST SUITES PASSED (6/6 Instruments Verified)\n";
    std::cout << "====================================================\n";
    return 0;
}
