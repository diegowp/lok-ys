#include "LokYS/autopilot/AutopilotSystem.hpp"
#include "LokYS/autopilot/PidController.hpp"
#include "LokYS/navigation/NavTypes.hpp"
#include "LokYS/physics/RigidBodyState.hpp"
#include "LokYS/input/InputSystem.hpp"
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

#define TEST_NEAR(a, b, eps, msg) \
    do { \
        double diff = std::abs(static_cast<double>(a) - static_cast<double>(b)); \
        if (diff > static_cast<double>(eps)) { \
            std::cerr << "Assertion failed: " << (msg) << " | Expected " << (b) << ", got " << (a) \
                      << " (diff=" << diff << " > " << (eps) << ") (" << __FILE__ << ":" << __LINE__ << ")\n"; \
            return false; \
        } \
    } while (0)

namespace {

using namespace LokYS;
using namespace LokYS::Autopilot;

bool TestPidControllerLimitsAndAntiWindup() {
    std::cout << "[RUNNING] TestPidControllerLimitsAndAntiWindup...\n";

    PidGains gains{};
    gains.kp = 2.0;
    gains.ki = 0.5;
    gains.kd = 0.1;
    gains.minIntegral = -1.5;
    gains.maxIntegral = +1.5;
    gains.minOutput = -2.0;
    gains.maxOutput = +2.0;
    gains.maxRate = 200.0;
    gains.derivativeFilterAlpha = 1.0;

    PidController pid(gains);

    // 1. Proportional response on initial step
    f64 out1 = pid.Update(0.5, 0.01);
    // P = 2.0 * 0.5 = 1.0, I = 0.5 * 0.005 = 0.0025
    TEST_ASSERT(out1 > 0.9 && out1 < 1.1, "Proportional output should be ~1.0");

    // 2. Anti-windup clamping under persistent error
    for (int i = 0; i < 500; ++i) {
        pid.Update(2.0, 0.01); // 5 seconds of large positive error
    }
    TEST_NEAR(pid.GetIntegral(), 1.5, 1e-4, "Integrator must be clamped to maxIntegral");
    TEST_NEAR(pid.GetPreviousOutput(), 2.0, 1e-4, "Output must be clamped to maxOutput");

    // 3. Reset test
    pid.Reset();
    TEST_NEAR(pid.GetIntegral(), 0.0, 1e-6, "Integral must be 0 after reset");
    TEST_NEAR(pid.GetPreviousOutput(), 0.0, 1e-6, "Output must be 0 after reset");

    std::cout << "[PASSED] TestPidControllerLimitsAndAntiWindup\n";
    return true;
}

bool TestAutopilotWingLevelerAndRollHold() {
    std::cout << "[RUNNING] TestAutopilotWingLevelerAndRollHold...\n";

    AutopilotSystem ap;
    ap.Initialize();
    ap.SetLateralMode(LateralMode::ROL);
    ap.SetMasterEngaged(true);

    Physics::RigidBodyState state{};
    Input::FlightControls pilotCtrl{};
    Navigation::NavReceiverState nav1{};

    // Case A: Small bank angle (4 deg) -> Wing Leveler drives to 0 deg bank
    ap.SetTargetPitch(2.0);
    ap.Update(
        0.00833, // 120 Hz dt
        state,
        100.0,   // IAS knots
        3500.0,  // Alt ft
        0.0,     // VS fpm
        2.0,     // pitch deg
        4.0,     // roll deg (4 deg right bank)
        090.0,   // heading deg
        nav1,
        pilotCtrl);

    const auto& statusA = ap.GetStatus();
    TEST_NEAR(statusA.fdCues.commandRollDeg, 0.0, 1e-3, "ROL mode with <6 deg bank commands wings level (0 deg)");
    // Aileron command should command left bank to counter right bank (roll error = 0 - 4 = -4 deg -> negative aileron)
    TEST_ASSERT(ap.GetOutput().aileronCommand < 0.0, "Aileron command should roll left to correct right bank");

    // Case B: Established bank angle (18 deg) -> Hold bank angle
    AutopilotTargets targets = ap.GetTargets();
    targets.targetRollDeg = 18.0;
    ap.SetTargets(targets);

    ap.Update(0.00833, state, 100.0, 3500.0, 0.0, 2.0, 18.0, 090.0, nav1, pilotCtrl);
    const auto& statusB = ap.GetStatus();
    TEST_NEAR(statusB.fdCues.commandRollDeg, 18.0, 1e-3, "ROL mode holds selected established bank angle");
    TEST_NEAR(ap.GetOutput().aileronCommand, 0.0, 0.05, "Aileron trim near zero when on target roll");

    std::cout << "[PASSED] TestAutopilotWingLevelerAndRollHold\n";
    return true;
}

bool TestAutopilotHeadingSelect() {
    std::cout << "[RUNNING] TestAutopilotHeadingSelect...\n";

    AutopilotSystem ap;
    ap.Initialize();
    ap.SetLateralMode(LateralMode::HDG);
    ap.SetMasterEngaged(true);

    Physics::RigidBodyState state{};
    Input::FlightControls pilotCtrl{};
    Navigation::NavReceiverState nav1{};

    // Case A: Target 120 deg, aircraft at 090 deg -> Turn Right (+30 deg error)
    ap.SetTargetHeading(120.0);
    for (int i = 0; i < 40; ++i) {
        ap.Update(0.00833, state, 100.0, 3500.0, 0.0, 2.0, 0.0, 090.0, nav1, pilotCtrl);
    }

    // Commanded bank angle should be positive (turn right), clamped to max 25 deg
    TEST_ASSERT(ap.GetStatus().fdCues.commandRollDeg > 20.0, "Turn right command should produce >20 deg bank");
    TEST_ASSERT(ap.GetStatus().fdCues.commandRollDeg <= 25.0, "Command bank must be clamped at 25 deg");
    TEST_ASSERT(ap.GetOutput().aileronCommand > 0.0, "Positive aileron commanded for right roll");

    // Case B: Shortest turn arc across North boundary
    // Target 350 deg, aircraft at 010 deg -> Shortest turn is -20 deg (Turn Left), NOT +340 deg!
    ap.SetTargetHeading(350.0);
    for (int i = 0; i < 40; ++i) {
        ap.Update(0.00833, state, 100.0, 3500.0, 0.0, 2.0, 0.0, 010.0, nav1, pilotCtrl);
    }

    TEST_ASSERT(ap.GetStatus().fdCues.commandRollDeg < -20.0, "Shortest turn across North must command left bank (< -20 deg)");
    TEST_ASSERT(ap.GetOutput().aileronCommand < 0.0, "Negative aileron commanded for left roll");

    std::cout << "[PASSED] TestAutopilotHeadingSelect\n";
    return true;
}

bool TestAutopilotAltitudeCaptureAndHold() {
    std::cout << "[RUNNING] TestAutopilotAltitudeCaptureAndHold...\n";

    AutopilotSystem ap;
    ap.Initialize();
    ap.SetVerticalMode(VerticalMode::ALT);
    ap.SetTargetAltitude(4000.0);
    ap.SetMasterEngaged(true);

    Physics::RigidBodyState state{};
    Input::FlightControls pilotCtrl{};
    Navigation::NavReceiverState nav1{};

    // Case A: Low altitude (3000 ft, target 4000 ft, climbing at +200 fpm)
    // 1000 ft below target -> strong climb command (run 60 steps = 0.5s for slew rate ramp)
    for (int i = 0; i < 60; ++i) {
        ap.Update(0.00833, state, 100.0, 3000.0, 200.0, 2.0, 0.0, 360.0, nav1, pilotCtrl);
    }
    TEST_ASSERT(!ap.GetStatus().altitudeCaptured, "Altitude not yet captured when 1000 ft away");
    TEST_ASSERT(ap.GetStatus().fdCues.commandPitchDeg > 4.0, "Pitch up command should exceed 4.0 deg during climb");
    TEST_ASSERT(ap.GetOutput().elevatorCommand > 0.0, "Positive elevator command to pitch up");

    // Case B: Near target (3995 ft, vertical speed +100 fpm)
    // Within 20 ft capture window
    for (int i = 0; i < 60; ++i) {
        ap.Update(0.00833, state, 100.0, 3995.0, 100.0, 2.5, 0.0, 360.0, nav1, pilotCtrl);
    }
    TEST_ASSERT(ap.GetStatus().altitudeCaptured, "Altitude capture flag must be set within 20 ft of target");
    TEST_ASSERT(std::abs(ap.GetStatus().fdCues.commandPitchDeg - 2.5) < 2.0, "Command pitch smoothly settles near nominal trim");

    std::cout << "[PASSED] TestAutopilotAltitudeCaptureAndHold\n";
    return true;
}

bool TestAutopilotVerticalSpeedHold() {
    std::cout << "[RUNNING] TestAutopilotVerticalSpeedHold...\n";

    AutopilotSystem ap;
    ap.Initialize();
    ap.SetVerticalMode(VerticalMode::VS);
    ap.SetTargetVerticalSpeed(700.0); // Climb at +700 fpm
    ap.SetMasterEngaged(true);

    Physics::RigidBodyState state{};
    Input::FlightControls pilotCtrl{};
    Navigation::NavReceiverState nav1{};

    // Currently climbing at only +100 fpm (error = +600 fpm)
    for (int i = 0; i < 60; ++i) {
        ap.Update(0.00833, state, 100.0, 2500.0, 100.0, 2.0, 0.0, 360.0, nav1, pilotCtrl);
    }
    TEST_ASSERT(ap.GetStatus().fdCues.commandPitchDeg > 3.0, "Command pitch must increase to meet +700 fpm climb");
    TEST_ASSERT(ap.GetOutput().elevatorCommand > 0.0, "Positive elevator command for pitch up");

    // Target descent at -500 fpm while currently level (0 fpm)
    ap.SetTargetVerticalSpeed(-500.0);
    for (int i = 0; i < 100; ++i) {
        ap.Update(0.00833, state, 100.0, 2500.0, 0.0, 2.0, 0.0, 360.0, nav1, pilotCtrl);
    }
    TEST_ASSERT(ap.GetStatus().fdCues.commandPitchDeg < 2.0, "Command pitch must decrease to enter descent");
    TEST_ASSERT(ap.GetOutput().elevatorCommand < 0.0, "Negative elevator command for pitch down");

    std::cout << "[PASSED] TestAutopilotVerticalSpeedHold\n";
    return true;
}

bool TestAutopilotNavCourseAndIlsGlideslope() {
    std::cout << "[RUNNING] TestAutopilotNavCourseAndIlsGlideslope...\n";

    AutopilotSystem ap;
    ap.Initialize();
    ap.SetLateralMode(LateralMode::NAV);
    ap.SetVerticalMode(VerticalMode::GS);
    ap.SetMasterEngaged(true);

    Physics::RigidBodyState state{};
    Input::FlightControls pilotCtrl{};
    Navigation::NavReceiverState nav1{};
    nav1.cdi.navFlag = false; // Valid signal
    nav1.cdi.isIls = true;
    nav1.cdi.obsDeg = 360.0;
    nav1.cdi.toFlag = true;

    // 1. NAV Lateral Tracking: CDI needle deflected right (+2.5 dots) -> Fly Right to intercept
    nav1.cdi.courseDeviationDots = 2.5;
    ap.Update(0.00833, state, 90.0, 1500.0, -500.0, -1.0, 0.0, 360.0, nav1, pilotCtrl);
    // When CDI deflection is positive (needle right of course), turn right to intercept
    TEST_ASSERT(ap.GetStatus().fdCues.commandRollDeg != 0.0, "NAV mode commands lateral bank when off radial");

    // 2. Glideslope Vertical Tracking: GS needle deflected UP (-3.0 dots -> fly-up) -> Beam is above aircraft -> Fly UP
    nav1.cdi.gsFlag = false; // Valid GS signal
    nav1.cdi.glideslopeDeviationDots = -3.0; // Negative is fly-up in CdiState
    ap.Update(0.00833, state, 90.0, 1200.0, -600.0, -2.5, 0.0, 360.0, nav1, pilotCtrl);
    TEST_ASSERT(ap.GetStatus().gsCaptured, "GS capture flag set when signal present");
    // Nominal descent is -2.5 deg; when beam is above, pitch should command up
    TEST_ASSERT(ap.GetStatus().fdCues.commandPitchDeg > -2.0, "Pitch command rises towards glideslope beam when below path");

    std::cout << "[PASSED] TestAutopilotNavCourseAndIlsGlideslope\n";
    return true;
}

bool TestFlightDirectorAndPilotOverrideSafety() {
    std::cout << "[RUNNING] TestFlightDirectorAndPilotOverrideSafety...\n";

    AutopilotSystem ap;
    ap.Initialize();
    ap.SetMasterEngaged(false);
    ap.SetFlightDirectorActive(true); // Manual flight with FD cues active

    Physics::RigidBodyState state{};
    Input::FlightControls pilotCtrl{};
    Navigation::NavReceiverState nav1{};

    // Flight Director active while AP servos are off
    ap.SetTargetHeading(045.0);
    ap.Update(0.00833, state, 100.0, 3500.0, 0.0, 2.0, 0.0, 090.0, nav1, pilotCtrl);
    TEST_ASSERT(ap.GetStatus().fdCues.active, "Flight Director cues must be active");
    TEST_ASSERT(!ap.GetOutput().engaged, "Autopilot actuator commands must be zero when master is OFF");
    TEST_NEAR(ap.GetOutput().elevatorCommand, 0.0, 1e-6, "Elevator command zero when disengaged");
    TEST_NEAR(ap.GetOutput().aileronCommand, 0.0, 1e-6, "Aileron command zero when disengaged");

    // Engage AP
    ap.SetMasterEngaged(true);
    TEST_ASSERT(ap.IsEngaged(), "AP is now engaged");

    // Pilot applies manual yoke override (pitch pull > 0.25)
    pilotCtrl.pitch = 0.35;
    ap.Update(0.00833, state, 100.0, 3500.0, 0.0, 2.0, 0.0, 090.0, nav1, pilotCtrl);

    // AP must immediately trip safety disconnect!
    TEST_ASSERT(!ap.IsEngaged(), "Autopilot must disconnect immediately upon pilot manual override");
    TEST_ASSERT(ap.GetStatus().disconnectWarning, "Disconnect warning flag must be active");
    TEST_ASSERT(ap.GetStatus().lastDisconnectReason == DisconnectReason::PilotManualOverride,
                "Disconnect reason must be PilotManualOverride");

    std::cout << "[PASSED] TestFlightDirectorAndPilotOverrideSafety\n";
    return true;
}

} // anonymous namespace

int main() {
    std::cout << "============================================================\n";
    std::cout << "LokYS Digital Glass Cockpit & Autopilot Test Suite (M21)\n";
    std::cout << "============================================================\n";

    bool success = true;
    success &= TestPidControllerLimitsAndAntiWindup();
    success &= TestAutopilotWingLevelerAndRollHold();
    success &= TestAutopilotHeadingSelect();
    success &= TestAutopilotAltitudeCaptureAndHold();
    success &= TestAutopilotVerticalSpeedHold();
    success &= TestAutopilotNavCourseAndIlsGlideslope();
    success &= TestFlightDirectorAndPilotOverrideSafety();

    std::cout << "============================================================\n";
    if (success) {
        std::cout << "[ALL 7 AUTOPILOT & GLASS COCKPIT TESTS PASSED]\n";
        return 0;
    } else {
        std::cerr << "[TEST SUITE FAILED]\n";
        return 1;
    }
}
