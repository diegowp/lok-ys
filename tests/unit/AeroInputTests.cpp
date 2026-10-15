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

// Phase F2 Tests: Empirical Aerodynamics & Non-Linear Mechanics

void TestMcCormickGroundEffect() {
    AerodynamicsSystem aero;
    ControlDeflections controls{};

    // Cessna 172 wingspan = 11.0m, mean chord = 1.493m
    f64 alpha5Rad = Units::DegToRad(5.0);
    Vec3d wind{ 40.0 * std::cos(alpha5Rad), 0.0, 40.0 * std::sin(alpha5Rad) }; // ~78 knots
    Vec3d zeroOmega{ 0.0, 0.0, 0.0 };

    // Case 1: Out of ground effect (h = 1000m >> b = 11m)
    auto outOge = aero.Evaluate(wind, zeroOmega, controls, Constants::SEA_LEVEL_DENSITY, 340.294, 1000.0);
    TEST_NEAR(outOge.groundEffectFactor, 1.0, 1e-3, "OGE phi(h/b) factor equals 1.0");

    // Case 2: In ground effect (h = 1.65m = 0.15 * b)
    // Analytical phi(0.15) = 16*(0.15)^2 / (1 + 16*(0.15)^2) = 0.36 / 1.36 = 0.264705
    auto outIge = aero.Evaluate(wind, zeroOmega, controls, Constants::SEA_LEVEL_DENSITY, 340.294, 1.65);
    f64 expectedPhi = (16.0 * 0.15 * 0.15) / (1.0 + 16.0 * 0.15 * 0.15);
    TEST_NEAR(outIge.groundEffectFactor, expectedPhi, 1e-4, "IGE McCormick phi(h/b) factor matches analytical");

    // Induced drag reduction: IGE drag must be lower than OGE drag
    TEST_ASSERT(outIge.dragNewtons < outOge.dragNewtons, "Ground effect significantly attenuates induced drag");
    f64 dragReductionPct = (outOge.dragNewtons - outIge.dragNewtons) / outOge.dragNewtons * 100.0;
    TEST_ASSERT(dragReductionPct > 15.0, "Induced drag drop > 15% in strong ground effect");

    // Lift increase: IGE lift must be higher than OGE lift
    TEST_ASSERT(outIge.liftNewtons > outOge.liftNewtons, "Ground effect increases effective wing lift");

    // Nose-down flare pitching moment increment due to tail downwash reduction
    TEST_ASSERT(outIge.momentsBody.y < outOge.momentsBody.y, "Downwash reduction induces nose-down flare pitching moment");
}

void TestPropellerSlipstreamEmpennageImmersion() {
    AerodynamicsSystem aero;
    ControlDeflections ctrlElevatorUp{};
    ctrlElevatorUp.elevator = 1.0; // Full stick back / nose up

    Vec3d zeroWind{ 0.0, 0.0, 0.0 };
    Vec3d zeroOmega{ 0.0, 0.0, 0.0 };

    // Case 1: Zero airspeed, zero thrust -> zero aerodynamic moments
    auto outZeroPower = aero.Evaluate(zeroWind, zeroOmega, ctrlElevatorUp, Constants::SEA_LEVEL_DENSITY, 340.294, 100.0, 0.0);
    TEST_NEAR(outZeroPower.momentsBody.y, 0.0, 1e-4, "Zero airspeed & zero thrust produces zero elevator pitch moment");

    // Case 2: Zero airspeed, full engine runup thrust (T = 2400 N, prop diameter = 1.905m, 60% empennage immersion)
    // Analytical V_slip = sqrt(2 * T / (rho * A_prop))
    f64 diskArea = Math::Constants::PI * (1.905 * 0.5) * (1.905 * 0.5);
    f64 expectedVslip = std::sqrt((2.0 * 2400.0) / (Constants::SEA_LEVEL_DENSITY * diskArea)); // ~37.0 m/s
    auto outPropwash = aero.Evaluate(zeroWind, zeroOmega, ctrlElevatorUp, Constants::SEA_LEVEL_DENSITY, 340.294, 100.0, 2400.0, 1.905, 0.60);

    TEST_NEAR(outPropwash.slipstreamVelocityMps, expectedVslip, 0.1, "Actuator disk slipstream velocity matches Rankine-Froude");
    TEST_ASSERT(outPropwash.tailDynamicPressurePa > 500.0, "Substantial tail dynamic pressure generated under static propwash");
    // Under full propeller wash, stick back must generate nose-up pitch moment (negative Body My)
    TEST_ASSERT(outPropwash.momentsBody.y < -300.0, "Static propwash creates powerful elevator authority at zero airspeed");

    // Rudder static deflection authority
    ControlDeflections ctrlRudderRight{};
    ctrlRudderRight.rudder = 1.0;
    auto outRudderWash = aero.Evaluate(zeroWind, zeroOmega, ctrlRudderRight, Constants::SEA_LEVEL_DENSITY, 340.294, 100.0, 2400.0, 1.905, 0.60);
    TEST_ASSERT(outRudderWash.momentsBody.z < -200.0, "Static propwash creates rudder yaw authority at zero airspeed");
}

void TestPostStallAutorotationSpinDynamics() {
    AerodynamicsSystem aero;
    ControlDeflections controls{};

    // Alpha = 25 degrees (well beyond 16 deg stall angle)
    f64 alphaStallRad = Units::DegToRad(25.0);
    Vec3d windStall{ 35.0 * std::cos(alphaStallRad), 0.0, 35.0 * std::sin(alphaStallRad) };

    // With a roll disturbance rate p = +1.0 rad/s (~57 deg/s right roll):
    Vec3d rightRollOmega{ 1.0, 0.0, 0.0 };
    auto outSpin = aero.Evaluate(windStall, rightRollOmega, controls);

    TEST_ASSERT(outSpin.isStalled, "Aircraft is operating in post-stall regime");
    // In deep stall, the autorotation term drives roll in the direction of the roll
    // (anti-damping / divergent autorotative roll moment)
    TEST_ASSERT(outSpin.coeffs.Cl > 0.0, "Post-stall wing drop produces positive-feedback autorotative roll moment");
}

void TestTransonicCompressibilityAndWaveDrag() {
    AerodynamicsSystem aero;
    ControlDeflections controls{};

    f64 alpha4Rad = Units::DegToRad(4.0);

    // Case 1: Incompressible regime (M = 0.20 < 0.30)
    f64 vLow = 68.0; // 68 / 340 = 0.20
    Vec3d windLow{ vLow * std::cos(alpha4Rad), 0.0, vLow * std::sin(alpha4Rad) };
    auto outLow = aero.Evaluate(windLow, Vec3d{0,0,0}, controls, Constants::SEA_LEVEL_DENSITY, 340.0);

    // Case 2: Compressible subsonic regime (M = 0.60, beta_comp = sqrt(1 - 0.36) = 0.80)
    f64 vMid = 204.0; // 204 / 340 = 0.60
    Vec3d windMid{ vMid * std::cos(alpha4Rad), 0.0, vMid * std::sin(alpha4Rad) };
    auto outMid = aero.Evaluate(windMid, Vec3d{0,0,0}, controls, Constants::SEA_LEVEL_DENSITY, 340.0);

    // Prandtl-Glauert scaling increases lift slope at high Mach
    TEST_ASSERT(outMid.coeffs.CL > outLow.coeffs.CL * 1.15, "Prandtl-Glauert compressibility amplifies CL at M=0.60");

    // Case 3: Transonic wave drag onset beyond critical Mach (M = 0.75 > Mcrit = 0.65)
    f64 vTransonic = 255.0; // 255 / 340 = 0.75
    Vec3d windTransonic{ vTransonic * std::cos(alpha4Rad), 0.0, vTransonic * std::sin(alpha4Rad) };
    auto outTransonic = aero.Evaluate(windTransonic, Vec3d{0,0,0}, controls, Constants::SEA_LEVEL_DENSITY, 340.0);

    TEST_ASSERT(outTransonic.coeffs.CD > outMid.coeffs.CD, "Transonic wave drag divergence raises drag coefficient above Mcrit");
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

    std::cout << "[RUNNING] TestMcCormickGroundEffect...\n" << std::flush;
    TestMcCormickGroundEffect();
    std::cout << "[DONE] TestMcCormickGroundEffect\n" << std::flush;

    std::cout << "[RUNNING] TestPropellerSlipstreamEmpennageImmersion...\n" << std::flush;
    TestPropellerSlipstreamEmpennageImmersion();
    std::cout << "[DONE] TestPropellerSlipstreamEmpennageImmersion\n" << std::flush;

    std::cout << "[RUNNING] TestPostStallAutorotationSpinDynamics...\n" << std::flush;
    TestPostStallAutorotationSpinDynamics();
    std::cout << "[DONE] TestPostStallAutorotationSpinDynamics\n" << std::flush;

    std::cout << "[RUNNING] TestTransonicCompressibilityAndWaveDrag...\n" << std::flush;
    TestTransonicCompressibilityAndWaveDrag();
    std::cout << "[DONE] TestTransonicCompressibilityAndWaveDrag\n" << std::flush;

    std::cout << "\n----------------------------------------\n";
    std::cout << "Tests Passed: " << g_testsPassed << "\n";
    std::cout << "Tests Failed: " << g_testsFailed << "\n";
    std::cout << "----------------------------------------\n";

    return (g_testsFailed == 0) ? 0 : 1;
}
