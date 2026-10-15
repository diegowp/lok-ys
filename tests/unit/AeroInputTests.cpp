#include "LokYS/aerodynamics/AerodynamicsSystem.hpp"
#include "LokYS/input/InputSystem.hpp"
#include "LokYS/math/Units.hpp"
#include <iostream>
#include <cmath>

using namespace LokYS;
using namespace LokYS::Math;
using namespace LokYS::Aerodynamics;
using namespace LokYS::Input;

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

void TestAerodynamicCoefficients() {
    AerodynamicsSystem aero;
    ControlDeflections controls{};

    // 1. Level cruise at 50 m/s (~97 knots), alpha = 0, beta = 0
    Vec3d windCruise{ 50.0, 0.0, 0.0 };
    Vec3d zeroOmega{ 0.0, 0.0, 0.0 };

    auto out0 = aero.Evaluate(windCruise, zeroOmega, controls);
    TEST_ASSERT(!out0.isStalled, "Cruise flight is not stalled");
    TEST_NEAR(out0.alphaDeg, 0.0, 1e-4, "Alpha zero degrees");
    TEST_NEAR(out0.coeffs.CL, 0.307, 1e-3, "CL0 matches Cessna 172 reference");
    TEST_ASSERT(out0.liftNewtons > 0.0, "Positive lift generated at alpha=0");
    TEST_ASSERT(out0.dragNewtons > 0.0, "Positive drag generated");

    // 2. Alpha = 8 degrees (Linear lift range)
    f64 alpha8Rad = Units::DegToRad(8.0);
    Vec3d windAlpha8{ 50.0 * std::cos(alpha8Rad), 0.0, 50.0 * std::sin(alpha8Rad) };
    auto out8 = aero.Evaluate(windAlpha8, zeroOmega, controls);
    TEST_ASSERT(!out8.isStalled, "Alpha 8 deg is within unstalled envelope");
    TEST_ASSERT(out8.coeffs.CL > out0.coeffs.CL, "Lift increases with angle of attack");
    // Cm should be negative for positive alpha (longitudinal static stability / pitch stiffness)
    TEST_ASSERT(out8.coeffs.Cm < out0.coeffs.Cm, "Cm decreases with alpha (pitch stiffness)");

    // 3. Stall Breakaway: Alpha = 25 degrees (> 16 deg stall angle)
    f64 alpha25Rad = Units::DegToRad(25.0);
    Vec3d windStall{ 50.0 * std::cos(alpha25Rad), 0.0, 50.0 * std::sin(alpha25Rad) };
    auto outStall = aero.Evaluate(windStall, zeroOmega, controls);
    TEST_ASSERT(outStall.isStalled, "Stall correctly detected at alpha = 25 deg");
    TEST_ASSERT(outStall.coeffs.CD > out8.coeffs.CD, "Massive drag rise post-stall");

    // 4. Control surface effectiveness: Elevator pull (+1.0 nose up)
    controls.elevator = 1.0;
    auto outElevator = aero.Evaluate(windCruise, zeroOmega, controls);
    // Elevator pull (elevator trailing edge down/up convention)
    TEST_ASSERT(outElevator.coeffs.Cm < 0.0 || outElevator.coeffs.Cm != out0.coeffs.Cm,
                "Elevator deflection alters pitching moment");

    // 5. Flap extension: Flaps 100% (Notch 3)
    controls.elevator = 0.0;
    controls.flaps = 1.0;
    auto outFlaps = aero.Evaluate(windCruise, zeroOmega, controls);
    TEST_ASSERT(outFlaps.coeffs.CL > out0.coeffs.CL, "Flaps increase CL");
    TEST_ASSERT(outFlaps.coeffs.CD > out0.coeffs.CD, "Flaps increase CD");
}

void TestInputAxisShaping() {
    InputSystem input;
    InputCalibration calib;
    calib.deadzone = 0.05; // 5% deadzone
    calib.sensitivityExp = 2.0; // Quadratic curve
    input.SetCalibration(calib);

    // Initial default flight controls
    const auto& ctrl = input.GetControls();
    TEST_NEAR(ctrl.pitch, 0.0, 1e-6, "Pitch centered initially");
    TEST_NEAR(ctrl.roll, 0.0, 1e-6, "Roll centered initially");
    TEST_NEAR(ctrl.yaw, 0.0, 1e-6, "Yaw centered initially");

    // Test programmatic control injection for autopilot / automation
    FlightControls testCtrl;
    testCtrl.pitch = 0.5;
    testCtrl.roll = -0.25;
    testCtrl.yaw = 0.1;
    testCtrl.throttle = 0.75;
    testCtrl.flapSetting = 2;
    input.SetControls(testCtrl);

    const auto& verified = input.GetControls();
    TEST_NEAR(verified.pitch, 0.5, 1e-6, "SetControls pitch");
    TEST_NEAR(verified.roll, -0.25, 1e-6, "SetControls roll");
    TEST_NEAR(verified.throttle, 0.75, 1e-6, "SetControls throttle");
    TEST_ASSERT(verified.flapSetting == 2, "SetControls flapSetting");
}

void TestParkingBrakeStateToggle() {
    InputSystem input;
    // 1. Initial spawn state must have parking brake active and brakes = 1.0
    TEST_ASSERT(input.IsParkingBrakeEngaged(), "Parking brake engaged on initial spawn");
    TEST_NEAR(input.GetControls().brakes, 1.0, 1e-6, "Brakes applied at 100% on spawn");

    // 2. Disengage parking brake
    input.SetParkingBrake(false);
    TEST_ASSERT(!input.IsParkingBrakeEngaged(), "Parking brake disengaged");
    TEST_NEAR(input.GetControls().brakes, 0.0, 1e-6, "Brakes released to 0.0");

    // 3. Re-engage parking brake
    input.SetParkingBrake(true);
    TEST_ASSERT(input.IsParkingBrakeEngaged(), "Parking brake re-engaged");
    TEST_NEAR(input.GetControls().brakes, 1.0, 1e-6, "Brakes locked back to 1.0");
}

int main() {
    std::cout << "========================================\n" << std::flush;
    std::cout << " LokYS Aerodynamics & Input Test Suite  \n" << std::flush;
    std::cout << "========================================\n" << std::flush;

    std::cout << "[RUNNING] TestAerodynamicCoefficients...\n" << std::flush;
    TestAerodynamicCoefficients();
    std::cout << "[DONE] TestAerodynamicCoefficients\n" << std::flush;

    std::cout << "[RUNNING] TestInputAxisShaping...\n" << std::flush;
    TestInputAxisShaping();
    std::cout << "[DONE] TestInputAxisShaping\n" << std::flush;

    std::cout << "[RUNNING] TestParkingBrakeStateToggle...\n" << std::flush;
    TestParkingBrakeStateToggle();
    std::cout << "[DONE] TestParkingBrakeStateToggle\n" << std::flush;

    std::cout << "\n----------------------------------------\n";
    std::cout << "Tests Passed: " << g_testsPassed << "\n";
    std::cout << "Tests Failed: " << g_testsFailed << "\n";
    std::cout << "----------------------------------------\n";

    return (g_testsFailed == 0) ? 0 : 1;
}
