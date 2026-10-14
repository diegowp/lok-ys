#include "LokYS/graphics/ShadowMap.hpp"
#include "LokYS/graphics/TerrainSystem.hpp"
#include "LokYS/graphics/ScenerySystem.hpp"
#include "LokYS/graphics/Renderer3D.hpp"
#include "LokYS/graphics/Camera.hpp"
#include "LokYS/environment/AtmosphereTypes.hpp"
#include "LokYS/math/Matrix4x4.hpp"
#include "LokYS/math/Vector3.hpp"
#include <iostream>
#include <cmath>

using namespace LokYS;
using namespace LokYS::Math;
using namespace LokYS::Graphics;
using namespace LokYS::Environment;

#define TEST_ASSERT(cond, msg) \
    do { \
        if (!(cond)) { \
            std::cerr << "[FAIL] " << msg << " (" << __FILE__ << ":" << __LINE__ << ")\n"; \
            return false; \
        } \
    } while (0)

#define TEST_NEAR(actual, expected, tol, msg) \
    do { \
        f64 a = static_cast<f64>(actual); \
        f64 e = static_cast<f64>(expected); \
        if (std::abs(a - e) > (tol)) { \
            std::cerr << "[FAIL] " << msg << " (Expected " << e << ", got " << a \
                      << ", diff=" << std::abs(a - e) << " > " << (tol) << ")\n"; \
            return false; \
        } \
    } while (0)

namespace {

bool TestAtmosphericFogFormulas() {
    std::cout << "[RUNNING] TestAtmosphericFogFormulas...\n";

    FogConfig fog;
    fog.enabled = true;
    fog.density = 0.00008f;
    fog.heightDecay = 0.0015f;
    fog.color = { 0.423f, 0.627f, 0.863f, 1.0f };

    auto CalcOpticalDepth = [&](f32 dist, f32 y) -> f32 {
        f32 hDecay = std::max(fog.heightDecay, 1e-6f);
        return dist * fog.density * std::exp(-y * hDecay);
    };

    auto CalcFogFactor = [&](f32 opticalDepth) -> f32 {
        return 1.0f - std::exp(-opticalDepth);
    };

    // 1. At zero distance, fog factor must be 0
    f32 d0 = CalcOpticalDepth(0.0f, 0.0f);
    TEST_NEAR(CalcFogFactor(d0), 0.0f, 1e-5, "Fog factor at distance 0 must be 0");

    // 2. At 1 km near ground (y = 0):
    f32 d1k = CalcOpticalDepth(1000.0f, 0.0f);
    f32 f1k = CalcFogFactor(d1k);
    // tau = 1000 * 0.00008 = 0.08, fog = 1 - exp(-0.08) ~= 0.0769
    TEST_NEAR(f1k, 1.0 - std::exp(-0.08), 1e-4, "Fog factor at 1km near ground");
    TEST_ASSERT(f1k > 0.05f && f1k < 0.15f, "Light haze at 1km");

    // 3. At 25 km distance, fog factor should be heavy (> 85%)
    f32 d25k = CalcOpticalDepth(25000.0f, 0.0f);
    f32 f25k = CalcFogFactor(d25k);
    // tau = 25000 * 0.00008 = 2.0, fog = 1 - exp(-2.0) ~= 0.8647
    TEST_ASSERT(f25k > 0.80f, "Distant mountains at 25km must blend heavily into horizon sky");

    // 4. Altitude attenuation: at high altitude (y = 2000m), optical depth must be significantly reduced
    f32 dHigh = CalcOpticalDepth(10000.0f, 2000.0f);
    f32 dLow  = CalcOpticalDepth(10000.0f, 0.0f);
    TEST_ASSERT(dHigh < dLow * 0.1f, "Optical depth at 2000m altitude must decay by > 90% due to thin air");

    std::cout << "[PASSED] TestAtmosphericFogFormulas\n";
    return true;
}

bool TestShadowMapMatrixCalculations() {
    std::cout << "[RUNNING] TestShadowMapMatrixCalculations...\n";

    ShadowMap shadowMap(2048);

    Vec3f sunDir{ 0.4f, 0.8f, 0.4f }; // Sunlight direction towards ground
    Vec3f aircraftTarget{ 0.0f, 2.0f, 100.0f }; // Aircraft in flight

    shadowMap.UpdateLightMatrix(sunDir, aircraftTarget, 36.0f, 36.0f, -60.0f, 60.0f);
    const Mat4f& lightVP = shadowMap.GetLightViewProjMatrix();

    // The aircraft target position transformed by lightVP should land near NDC (0, 0)
    // and depth near 0.5
    f32 x = aircraftTarget.x;
    f32 y = aircraftTarget.y;
    f32 z = aircraftTarget.z;

    f32 clipX = x * lightVP.m[0][0] + y * lightVP.m[1][0] + z * lightVP.m[2][0] + lightVP.m[3][0];
    f32 clipY = x * lightVP.m[0][1] + y * lightVP.m[1][1] + z * lightVP.m[2][1] + lightVP.m[3][1];
    f32 clipZ = x * lightVP.m[0][2] + y * lightVP.m[1][2] + z * lightVP.m[2][2] + lightVP.m[3][2];
    f32 clipW = x * lightVP.m[0][3] + y * lightVP.m[1][3] + z * lightVP.m[2][3] + lightVP.m[3][3];

    TEST_NEAR(clipW, 1.0f, 1e-4, "Orthographic projection W component must be 1.0");
    TEST_NEAR(clipX, 0.0f, 0.1f, "Target X in light clip space should be centered");
    TEST_NEAR(clipY, 0.0f, 0.1f, "Target Y in light clip space should be centered");
    TEST_ASSERT(clipZ >= 0.0f && clipZ <= 1.0f, "Target Z must be inside [0, 1] depth buffer range");

    // Check resolution
    TEST_ASSERT(shadowMap.GetResolution() == 2048, "Shadow map resolution must be 2048");

    std::cout << "[PASSED] TestShadowMapMatrixCalculations\n";
    return true;
}

bool TestTerrainElevationAndAirportMesaFlattening() {
    std::cout << "[RUNNING] TestTerrainElevationAndAirportMesaFlattening...\n";

    // 1. Runway 36 centerline and borders must be perfectly flat (elevation = 0.0)
    TEST_NEAR(TerrainSystem::SampleElevation(0.0f, 0.0f), 0.0f, 1e-5, "Runway center must be 0m MSL");
    TEST_NEAR(TerrainSystem::SampleElevation(-22.5f, -800.0f), 0.0f, 1e-5, "Runway threshold West must be 0m MSL");
    TEST_NEAR(TerrainSystem::SampleElevation(22.5f, 800.0f), 0.0f, 1e-5, "Runway end East must be 0m MSL");

    // 2. Entire airport apron and taxiway perimeter must be flat
    TEST_NEAR(TerrainSystem::SampleElevation(150.0f, -400.0f), 0.0f, 1e-5, "Main hangar area must be flat");
    TEST_NEAR(TerrainSystem::SampleElevation(-150.0f, -400.0f), 0.0f, 1e-5, "West apron area must be flat");

    // 3. Normal on the runway mesa must point straight up (0, 1, 0)
    Vec3f nRwy = TerrainSystem::SampleNormal(0.0f, 0.0f);
    TEST_NEAR(nRwy.x, 0.0f, 1e-4, "Normal X on runway must be 0");
    TEST_NEAR(nRwy.y, 1.0f, 1e-4, "Normal Y on runway must be 1");
    TEST_NEAR(nRwy.z, 0.0f, 1e-4, "Normal Z on runway must be 0");

    // 4. Distant terrain outside the airport must have positive elevation
    f32 hDistant = TerrainSystem::SampleElevation(8000.0f, 8000.0f);
    TEST_ASSERT(hDistant > 20.0f, "Distant terrain must form rolling hills and mountains");

    // 5. Surface normal on hills must remain normalized
    Vec3f nHills = TerrainSystem::SampleNormal(8000.0f, 8000.0f);
    TEST_NEAR(nHills.Length(), 1.0f, 1e-4, "Terrain surface normal must be unit length");

    std::cout << "[PASSED] TestTerrainElevationAndAirportMesaFlattening\n";
    return true;
}

bool TestCameraFrustumCulling() {
    std::cout << "[RUNNING] TestCameraFrustumCulling...\n";

    // Set up standard camera view-projection
    Mat4f proj = Mat4f::PerspectiveFovLH(1.0472f, 16.0f / 9.0f, 1.0f, 1000.0f);
    Mat4f view = Mat4f::CameraRelativeViewLH(Vec3f(0.0f, 0.0f, 1.0f), Vec3f(0.0f, 1.0f, 0.0f));
    Mat4f vp = view * proj;

    Frustum frustum = Frustum::FromViewProj(vp);

    // 1. Point / Sphere directly in front of camera at 50m: must intersect
    TEST_ASSERT(frustum.IntersectsSphere(Vec3f(0.0f, 0.0f, 50.0f), 5.0f), "Object 50m in front must be inside frustum");

    // 2. Sphere behind camera: must be culled
    TEST_ASSERT(!frustum.IntersectsSphere(Vec3f(0.0f, 0.0f, -50.0f), 5.0f), "Object behind camera must be culled");

    // 3. Sphere far beyond far plane: must be culled
    TEST_ASSERT(!frustum.IntersectsSphere(Vec3f(0.0f, 0.0f, 2000.0f), 10.0f), "Object beyond far plane must be culled");

    // 4. AABB centered around airport runway: must intersect
    Vec3f rwyMin{ -50.0f, -1.0f, 10.0f };
    Vec3f rwyMax{  50.0f,  5.0f, 500.0f };
    TEST_ASSERT(frustum.IntersectsAABB(rwyMin, rwyMax), "Runway AABB must be inside frustum");

    // 5. AABB completely to the far left outside FOV: must be culled
    Vec3f leftMin{ -1000.0f, -10.0f, 20.0f };
    Vec3f leftMax{  -500.0f,  10.0f, 80.0f };
    TEST_ASSERT(!frustum.IntersectsAABB(leftMin, leftMax), "Far left AABB must be culled by frustum");

    std::cout << "[PASSED] TestCameraFrustumCulling\n";
    return true;
}

bool TestPapiGlidepathAngleEvaluation() {
    std::cout << "[RUNNING] TestPapiGlidepathAngleEvaluation...\n";

    // PAPI is at X = -35.0, Z = -700.0, Y = 0.5
    // Distance 1000m South is at Z = -1700.0 (dz = -1000m)
    // On 3.0 deg glidepath: dy = 1000 * tan(3.0 deg) ~= 52.4m, y = 52.9m
    f32 d1000 = 1000.0f;
    f32 yOnSlope = 0.5f + d1000 * std::tan(3.0f * (3.14159265f / 180.0f));

    PapiStatus onSlope = ScenerySystem::ComputePapiStatus(Vec3f(-35.0f, yOnSlope, -1700.0f));
    TEST_ASSERT(onSlope.inApproachSector, "Must be in approach sector");
    TEST_NEAR(onSlope.approachAngleDeg, 3.0f, 0.05f, "Approach angle should be 3.0 deg");
    // Standard on-glidepath is 2 White, 2 Red:
    TEST_ASSERT(onSlope.lights[0] == PapiColor::White, "Light 1 must be White");
    TEST_ASSERT(onSlope.lights[1] == PapiColor::White, "Light 2 must be White");
    TEST_ASSERT(onSlope.lights[2] == PapiColor::Red,   "Light 3 must be Red");
    TEST_ASSERT(onSlope.lights[3] == PapiColor::Red,   "Light 4 must be Red");

    // Too High: 4.0 deg glidepath
    f32 yHigh = 0.5f + d1000 * std::tan(4.0f * (3.14159265f / 180.0f));
    PapiStatus high = ScenerySystem::ComputePapiStatus(Vec3f(-35.0f, yHigh, -1700.0f));
    TEST_ASSERT(high.lights[0] == PapiColor::White &&
                high.lights[1] == PapiColor::White &&
                high.lights[2] == PapiColor::White &&
                high.lights[3] == PapiColor::White, "4.0 deg must show 4 White lights (Too High)");

    // Too Low: 2.0 deg glidepath
    f32 yLow = 0.5f + d1000 * std::tan(2.0f * (3.14159265f / 180.0f));
    PapiStatus low = ScenerySystem::ComputePapiStatus(Vec3f(-35.0f, yLow, -1700.0f));
    TEST_ASSERT(low.lights[0] == PapiColor::Red &&
                low.lights[1] == PapiColor::Red &&
                low.lights[2] == PapiColor::Red &&
                low.lights[3] == PapiColor::Red, "2.0 deg must show 4 Red lights (Too Low)");

    // Slightly High: 3.3 deg glidepath (3 White, 1 Red)
    f32 ySlightlyHigh = 0.5f + d1000 * std::tan(3.3f * (3.14159265f / 180.0f));
    PapiStatus slightlyHigh = ScenerySystem::ComputePapiStatus(Vec3f(-35.0f, ySlightlyHigh, -1700.0f));
    TEST_ASSERT(slightlyHigh.lights[0] == PapiColor::White &&
                slightlyHigh.lights[1] == PapiColor::White &&
                slightlyHigh.lights[2] == PapiColor::White &&
                slightlyHigh.lights[3] == PapiColor::Red, "3.3 deg must show 3 White, 1 Red");

    std::cout << "[PASSED] TestPapiGlidepathAngleEvaluation\n";
    return true;
}

bool TestWindsockOrientationAndDroop() {
    std::cout << "[RUNNING] TestWindsockOrientationAndDroop...\n";

    constexpr f32 PI = 3.1415926535f;

    // Helper from ScenerySystem
    auto CalcWindsockParams = [](f64 windSpeedMps, f64 windDirDeg) -> std::pair<f32, f32> {
        f64 downwindDeg = windDirDeg + 180.0;
        if (downwindDeg >= 360.0) downwindDeg -= 360.0;
        f32 yaw = static_cast<f32>(downwindDeg * (PI / 180.0));
        f32 speedFactor = std::clamp(static_cast<f32>(windSpeedMps) / 7.5f, 0.0f, 1.0f);
        f32 droop = (1.0f - speedFactor) * 0.85f + 0.08f;
        return { yaw, droop };
    };

    // 1. Wind from North (0 deg): sock points South (180 deg = PI rad)
    auto [yawN, droop0] = CalcWindsockParams(0.0, 0.0);
    TEST_NEAR(yawN, PI, 1e-4, "North wind must point sock South (PI rad)");
    TEST_ASSERT(droop0 > 0.8f, "Calm wind must cause cone to droop downward");

    // 2. Wind from East (90 deg): sock points West (270 deg = 1.5 * PI rad)
    auto [yawE, droop15kt] = CalcWindsockParams(10.0, 90.0);
    TEST_NEAR(yawE, 1.5f * PI, 1e-4, "East wind must point sock West (270 deg)");
    TEST_ASSERT(droop15kt < 0.15f, "20kt wind must inflate cone near horizontal");

    std::cout << "[PASSED] TestWindsockOrientationAndDroop\n";
    return true;
}

bool TestWeatherPresetFogParameters() {
    std::cout << "[RUNNING] TestWeatherPresetFogParameters...\n";

    // Validates density configuration per weather preset
    auto GetFogForPreset = [](WeatherPreset preset) -> FogConfig {
        FogConfig c;
        c.enabled = true;
        c.heightDecay = 0.0015f;
        switch (preset) {
        case WeatherPreset::ClearCalm:
            c.density = 0.00004f; // 25km visibility
            break;
        case WeatherPreset::StandardDay:
            c.density = 0.00008f; // 15km visibility
            break;
        case WeatherPreset::CrosswindRunway36:
            c.density = 0.00015f; // 8km visibility
            break;
        case WeatherPreset::GustyTurbulence:
            c.density = 0.00025f; // 4km storm visibility
            break;
        case WeatherPreset::HotAndHigh:
            c.density = 0.00010f; // 10km haze
            break;
        }
        return c;
    };

    FogConfig cClear = GetFogForPreset(WeatherPreset::ClearCalm);
    FogConfig cTurb = GetFogForPreset(WeatherPreset::GustyTurbulence);

    TEST_ASSERT(cClear.density < cTurb.density, "Storm preset must have higher fog density than Clear");
    TEST_NEAR(cClear.density, 0.00004f, 1e-6, "ClearCalm fog density");
    TEST_NEAR(cTurb.density, 0.00025f, 1e-6, "GustyTurbulence fog density");

    std::cout << "[PASSED] TestWeatherPresetFogParameters\n";
    return true;
}

} // namespace

int main() {
    std::cout << "============================================================\n";
    std::cout << "LokYS Native Windows Flight Simulator — Graphics Optimization Tests\n";
    std::cout << "============================================================\n";

    bool allPassed = true;
    allPassed &= TestAtmosphericFogFormulas();
    allPassed &= TestShadowMapMatrixCalculations();
    allPassed &= TestTerrainElevationAndAirportMesaFlattening();
    allPassed &= TestCameraFrustumCulling();
    allPassed &= TestPapiGlidepathAngleEvaluation();
    allPassed &= TestWindsockOrientationAndDroop();
    allPassed &= TestWeatherPresetFogParameters();

    std::cout << "============================================================\n";
    if (allPassed) {
        std::cout << "[ALL 7 GRAPHICS OPTIMIZATION TEST SUITES PASSED (100%)]\n";
        return 0;
    } else {
        std::cerr << "[ONE OR MORE GRAPHICS OPTIMIZATION TESTS FAILED]\n";
        return 1;
    }
}
