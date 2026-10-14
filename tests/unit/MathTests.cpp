#include "LokYS/math/Units.hpp"
#include "LokYS/math/Vector3.hpp"
#include "LokYS/math/Matrix3x3.hpp"
#include "LokYS/math/Matrix4x4.hpp"
#include "LokYS/math/Quaternion.hpp"
#include "LokYS/math/CoordinateFrames.hpp"
#include <iostream>
#include <cassert>
#include <cmath>

using namespace LokYS;
using namespace LokYS::Math;

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

void TestUnitConversions() {
    // 100 knots = 51.4444 m/s
    f64 mps = Units::KnotsToMps(100.0);
    TEST_NEAR(mps, 51.444444, 1e-4, "Knots to m/s");
    TEST_NEAR(Units::MpsToKnots(mps), 100.0, 1e-4, "m/s to Knots round-trip");

    // 10,000 feet = 3048 meters
    f64 meters = Units::FeetToMeters(10000.0);
    TEST_NEAR(meters, 3048.0, 1e-4, "Feet to Meters");
    TEST_NEAR(Units::MetersToFeet(meters), 10000.0, 1e-4, "Meters to Feet round-trip");

    // Standard atmospheric pressure: 29.9212 inHg = 101325 Pa
    f64 pa = Units::InHgToPascals(29.92126);
    TEST_NEAR(pa, 101325.0, 1.0, "InHg to Pascals");
    TEST_NEAR(Units::PascalsToInHg(101325.0), 29.92126, 1e-4, "Pascals to InHg round-trip");

    // Mass: 2204.62 lbs = 1000 kg
    f64 kg = Units::PoundsToKg(2204.6226);
    TEST_NEAR(kg, 1000.0, 1e-3, "Pounds to Kg");
    TEST_NEAR(Units::KgToPounds(1000.0), 2204.6226, 1e-3, "Kg to Pounds round-trip");

    // Temperature: 15 C = 59 F = 288.15 K
    TEST_NEAR(Units::CelsiusToFahrenheit(15.0), 59.0, 1e-4, "Celsius to Fahrenheit");
    TEST_NEAR(Units::FahrenheitToCelsius(59.0), 15.0, 1e-4, "Fahrenheit to Celsius");
    TEST_NEAR(Units::CelsiusToKelvin(15.0), 288.15, 1e-4, "Celsius to Kelvin");
}

void TestVectorMath() {
    Vec3d v1{ 1.0, 2.0, 3.0 };
    Vec3d v2{ 4.0, 5.0, 6.0 };

    // Dot product: 1*4 + 2*5 + 3*6 = 32
    TEST_NEAR(v1.Dot(v2), 32.0, 1e-6, "Vector3 Dot Product");

    // Cross product: (2*6 - 3*5, 3*4 - 1*6, 1*5 - 2*4) = (-3, 6, -3)
    Vec3d cross = v1.Cross(v2);
    TEST_NEAR(cross.x, -3.0, 1e-6, "Vector3 Cross.x");
    TEST_NEAR(cross.y,  6.0, 1e-6, "Vector3 Cross.y");
    TEST_NEAR(cross.z, -3.0, 1e-6, "Vector3 Cross.z");

    // Length: sqrt(1 + 4 + 9) = sqrt(14)
    TEST_NEAR(v1.Length(), std::sqrt(14.0), 1e-6, "Vector3 Length");

    // Normalized vector length = 1.0
    Vec3d norm = v1.Normalized();
    TEST_NEAR(norm.Length(), 1.0, 1e-6, "Vector3 Normalized Length");
}

void TestMatrixMath() {
    Mat3d identity = Mat3d::Identity();
    TEST_NEAR(identity.Determinant(), 1.0, 1e-6, "Identity Matrix Determinant");

    // Rotation around Z by 90 degrees
    f64 angle = Units::DegToRad(90.0);
    Mat3d rotZ(
        std::cos(angle), -std::sin(angle), 0.0,
        std::sin(angle),  std::cos(angle), 0.0,
        0.0,             0.0,             1.0
    );

    Vec3d unitX = Vec3d::UnitX();
    Vec3d rotated = rotZ * unitX;
    TEST_NEAR(rotated.x, 0.0, 1e-6, "Matrix3x3 Rotated UnitX.x");
    TEST_NEAR(rotated.y, 1.0, 1e-6, "Matrix3x3 Rotated UnitX.y");
    TEST_NEAR(rotated.z, 0.0, 1e-6, "Matrix3x3 Rotated UnitX.z");

    // Invert rotation matrix
    Mat3d invRotZ = rotZ.Inverse();
    Mat3d prod = rotZ * invRotZ;
    TEST_NEAR(prod.m[0][0], 1.0, 1e-6, "Matrix3x3 Inverse Product [0][0]");
    TEST_NEAR(prod.m[1][1], 1.0, 1e-6, "Matrix3x3 Inverse Product [1][1]");
    TEST_NEAR(prod.m[2][2], 1.0, 1e-6, "Matrix3x3 Inverse Product [2][2]");
}

void TestQuaternionMath() {
    // Identity quaternion
    Quat4d qId = Quat4d::Identity();
    TEST_NEAR(qId.Norm(), 1.0, 1e-6, "Quaternion Identity Norm");

    // 90 deg Yaw rotation (around Z axis)
    Quat4d qYaw90 = Quat4d::FromEuler(Units::DegToRad(90.0), 0.0, 0.0);
    TEST_NEAR(qYaw90.Norm(), 1.0, 1e-6, "Quaternion Yaw90 Norm Invariant");

    // Forward vector (X) rotated by 90 deg yaw should point Right (Y) in Body/NED
    Vec3d fwd = Vec3d::UnitX();
    Vec3d yawed = qYaw90.Rotate(fwd);
    TEST_NEAR(yawed.x, 0.0, 1e-6, "Quaternion Rotated Forward.x");
    TEST_NEAR(yawed.y, 1.0, 1e-6, "Quaternion Rotated Forward.y");
    TEST_NEAR(yawed.z, 0.0, 1e-6, "Quaternion Rotated Forward.z");

    // Unrotate should restore original forward vector
    Vec3d restored = qYaw90.Unrotate(yawed);
    TEST_NEAR(restored.x, 1.0, 1e-6, "Quaternion Unrotate Restored.x");
    TEST_NEAR(restored.y, 0.0, 1e-6, "Quaternion Unrotate Restored.y");
    TEST_NEAR(restored.z, 0.0, 1e-6, "Quaternion Unrotate Restored.z");

    // Euler angle round-trip test: Yaw 45 deg, Pitch 20 deg, Roll -15 deg
    f64 inYaw = Units::DegToRad(45.0);
    f64 inPitch = Units::DegToRad(20.0);
    f64 inRoll = Units::DegToRad(-15.0);
    Quat4d qAttitude = Quat4d::FromEuler(inYaw, inPitch, inRoll);

    f64 outYaw = 0.0, outPitch = 0.0, outRoll = 0.0;
    qAttitude.ToEuler(outYaw, outPitch, outRoll);
    TEST_NEAR(outYaw, inYaw, 1e-5, "Euler round-trip Yaw");
    TEST_NEAR(outPitch, inPitch, 1e-5, "Euler round-trip Pitch");
    TEST_NEAR(outRoll, inRoll, 1e-5, "Euler round-trip Roll");
}

void TestCoordinateFrames() {
    // Relative wind: pure forward 50 m/s, zero alpha and beta
    Vec3d windForward{ 50.0, 0.0, 0.0 };
    auto aeroState1 = CoordinateFrames::ComputeAerodynamicState(windForward, 1.225, 340.0);
    TEST_NEAR(aeroState1.trueAirspeed, 50.0, 1e-4, "Aero True Airspeed");
    TEST_NEAR(aeroState1.alphaRad, 0.0, 1e-4, "Aero Alpha zero");
    TEST_NEAR(aeroState1.betaRad, 0.0, 1e-4, "Aero Beta zero");
    TEST_NEAR(aeroState1.dynamicPressure, 0.5 * 1.225 * 2500.0, 1e-2, "Aero Dynamic Pressure");

    // Relative wind with 10 deg alpha (w > 0 in body frame)
    f64 alphaExpected = Units::DegToRad(10.0);
    Vec3d windAlpha{ 50.0 * std::cos(alphaExpected), 0.0, 50.0 * std::sin(alphaExpected) };
    auto aeroState2 = CoordinateFrames::ComputeAerodynamicState(windAlpha, 1.225, 340.0);
    TEST_NEAR(aeroState2.trueAirspeed, 50.0, 1e-4, "Aero Alpha 10 True Airspeed");
    TEST_NEAR(aeroState2.alphaRad, alphaExpected, 1e-4, "Aero Alpha 10 deg");

    // Wind forces to body forces: Pure Lift 10,000 N, Zero Drag, Zero Alpha -> Body Fz = -10,000 N (upwards)
    Vec3d bodyForces = CoordinateFrames::WindForcesToBody(10000.0, 0.0, 0.0, 0.0, 0.0);
    TEST_NEAR(bodyForces.x, 0.0, 1e-4, "Body Fx from pure Lift");
    TEST_NEAR(bodyForces.y, 0.0, 1e-4, "Body Fy from pure Lift");
    TEST_NEAR(bodyForces.z, -10000.0, 1e-4, "Body Fz (upward) from pure Lift");

    // Camera relative precision test: 100 km from world origin
    Vec3d worldAircraft{ 100000.0, 50000.0, -1000.0 };
    Vec3d worldCamera{ 100000.0 + 10.0, 50000.0 - 5.0, -1000.0 + 2.0 };
    Vec3f camRel = CoordinateFrames::WorldToCameraRelative(worldAircraft, worldCamera);
    TEST_NEAR(camRel.x, -10.0f, 1e-4f, "Camera-relative X");
    TEST_NEAR(camRel.y,   5.0f, 1e-4f, "Camera-relative Y");
    TEST_NEAR(camRel.z,  -2.0f, 1e-4f, "Camera-relative Z");

    // NedToGraphics conversion
    Vec3d nedPos{ 100.0, 50.0, -300.0 }; // North 100, East 50, Alt 300
    Vec3f gfxPos = CoordinateFrames::NedToGraphics(nedPos);
    TEST_NEAR(gfxPos.x, 50.0f, 1e-4f, "NedToGraphics X is East");
    TEST_NEAR(gfxPos.y, 300.0f, 1e-4f, "NedToGraphics Y is Altitude (Up)");
    TEST_NEAR(gfxPos.z, 100.0f, 1e-4f, "NedToGraphics Z is North");
}

void TestMatrix4x4Math() {
    // 1. Identity
    Mat4f id = Mat4f::Identity();
    TEST_NEAR(id.m[0][0], 1.0f, 1e-6f, "Mat4 Identity [0][0]");
    TEST_NEAR(id.m[1][1], 1.0f, 1e-6f, "Mat4 Identity [1][1]");
    TEST_NEAR(id.m[2][2], 1.0f, 1e-6f, "Mat4 Identity [2][2]");
    TEST_NEAR(id.m[3][3], 1.0f, 1e-6f, "Mat4 Identity [3][3]");
    TEST_NEAR(id.m[0][1], 0.0f, 1e-6f, "Mat4 Identity [0][1]");

    // 2. Translation & Multiplication
    Mat4f t1 = Mat4f::Translation({ 10.0f, 20.0f, 30.0f });
    Mat4f t2 = Mat4f::Translation({ 5.0f, -5.0f, 2.0f });
    Mat4f tCombined = t1 * t2;
    TEST_NEAR(tCombined.m[3][0], 15.0f, 1e-5f, "Mat4 Translation X combined");
    TEST_NEAR(tCombined.m[3][1], 15.0f, 1e-5f, "Mat4 Translation Y combined");
    TEST_NEAR(tCombined.m[3][2], 32.0f, 1e-5f, "Mat4 Translation Z combined");

    // 3. Perspective Projection
    f32 fov = 1.04719755f; // 60 degrees
    f32 aspect = 16.0f / 9.0f;
    Mat4f proj = Mat4f::PerspectiveFovLH(fov, aspect, 0.5f, 1000.0f);
    TEST_ASSERT(proj.m[0][0] > 0.0f, "Projection M00 positive");
    TEST_ASSERT(proj.m[1][1] > 0.0f, "Projection M11 positive");
    TEST_NEAR(proj.m[2][3], 1.0f, 1e-5f, "Projection M23 is 1 for LH");

    // 4. LookAt
    Mat4f view = Mat4f::LookAtLH({ 0.0f, 0.0f, -10.0f }, { 0.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f });
    TEST_NEAR(view.m[3][2], 10.0f, 1e-5f, "LookAtLH Z translation");
}

int main() {
    std::cout << "========================================\n";
    std::cout << " LokYS Aerospace Math & Units Test Suite\n";
    std::cout << "========================================\n";

    TestUnitConversions();
    TestVectorMath();
    TestMatrixMath();
    TestQuaternionMath();
    TestCoordinateFrames();
    TestMatrix4x4Math();

    std::cout << "\n----------------------------------------\n";
    std::cout << "Tests Passed: " << g_testsPassed << "\n";
    std::cout << "Tests Failed: " << g_testsFailed << "\n";
    std::cout << "----------------------------------------\n";

    return (g_testsFailed == 0) ? 0 : 1;
}

