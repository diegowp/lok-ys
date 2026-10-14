#include "LokYS/environment/AtmosphereTypes.hpp"
#include "LokYS/environment/AtmosphereModel.hpp"
#include "LokYS/environment/WindModel.hpp"
#include "LokYS/environment/EnvironmentSystem.hpp"
#include <iostream>
#include <cmath>
#include <vector>

#define TEST_ASSERT(cond, msg) \
    do { \
        if (!(cond)) { \
            std::cerr << "Assertion failed: " << (msg) << " (" << __FILE__ << ":" << __LINE__ << ")\n"; \
            return false; \
        } \
    } while (0)

namespace {

using namespace LokYS;
using namespace LokYS::Environment;

constexpr f64 EPSILON = 1e-2;

bool FloatNear(f64 a, f64 b, f64 eps = EPSILON) {
    return std::fabs(a - b) <= eps;
}

bool TestStandardSeaLevelIsa() {
    std::cout << "[RUNNING] TestStandardSeaLevelIsa...\n";

    AtmosphericState s = AtmosphereModel::Evaluate(0.0, 0.0, Constants::STANDARD_SEA_LEVEL_PRESS_PA);

    TEST_ASSERT(FloatNear(s.temperatureKelvin, 288.15), "Sea level temperature must be 288.15 K");
    TEST_ASSERT(FloatNear(s.temperatureCelsius, 15.0), "Sea level temperature must be 15.0 °C");
    TEST_ASSERT(FloatNear(s.pressurePa, 101325.0, 1.0), "Sea level pressure must be 101325 Pa");
    TEST_ASSERT(FloatNear(s.pressureInHg, 29.92, 0.01), "Sea level pressure must be 29.92 inHg");
    TEST_ASSERT(FloatNear(s.densityKgM3, 1.225, 0.005), "Sea level density must be 1.225 kg/m^3");
    TEST_ASSERT(FloatNear(s.speedOfSoundMps, 340.29, 0.5), "Sea level speed of sound must be ~340.3 m/s");
    TEST_ASSERT(FloatNear(s.pressureAltitudeFt, 0.0, 1.0), "Sea level pressure altitude must be 0 ft");
    TEST_ASSERT(FloatNear(s.densityAltitudeFt, 0.0, 5.0), "Sea level density altitude must be 0 ft");

    std::cout << "[PASSED] TestStandardSeaLevelIsa\n";
    return true;
}

bool TestTroposphereLapseAndTropopause() {
    std::cout << "[RUNNING] TestTroposphereLapseAndTropopause...\n";

    // 1. Troposphere at 3000m (~9843 ft)
    AtmosphericState s3000 = AtmosphereModel::Evaluate(3000.0);
    // T = 288.15 - 0.0065 * 3000 = 268.65 K (-4.5 °C)
    TEST_ASSERT(FloatNear(s3000.temperatureKelvin, 268.65, 0.1), "3000m temperature must be 268.65 K");
    TEST_ASSERT(FloatNear(s3000.temperatureCelsius, -4.5, 0.1), "3000m temperature must be -4.5 °C");
    // Standard pressure at 3000m is ~70108 Pa (~20.70 inHg)
    TEST_ASSERT(FloatNear(s3000.pressurePa, 70108.0, 100.0), "3000m pressure must be ~70108 Pa");
    TEST_ASSERT(FloatNear(s3000.densityKgM3, 0.909, 0.01), "3000m density must be ~0.909 kg/m^3");
    TEST_ASSERT(FloatNear(s3000.pressureAltitudeFt, 9842.5, 20.0), "3000m pressure altitude must be ~9843 ft");

    // 2. Tropopause boundary at 11000m (~36089 ft)
    AtmosphericState s11000 = AtmosphereModel::Evaluate(11000.0);
    TEST_ASSERT(FloatNear(s11000.temperatureKelvin, 216.65, 0.1), "Tropopause temperature must be 216.65 K (-56.5 °C)");
    TEST_ASSERT(FloatNear(s11000.pressurePa, 22632.0, 100.0), "11000m pressure must be ~22632 Pa");
    TEST_ASSERT(FloatNear(s11000.pressureAltitudeFt, 36089.0, 50.0), "11000m pressure altitude must be ~36089 ft");

    // 3. Lower stratosphere at 15000m (isothermal layer)
    AtmosphericState s15000 = AtmosphereModel::Evaluate(15000.0);
    TEST_ASSERT(FloatNear(s15000.temperatureKelvin, 216.65, 0.1), "Stratosphere must remain isothermal at 216.65 K");
    TEST_ASSERT(s15000.pressurePa < s11000.pressurePa, "Stratospheric pressure must continue decaying exponentially");
    TEST_ASSERT(FloatNear(s15000.pressurePa, 12111.0, 150.0), "15000m pressure must be ~12111 Pa");

    std::cout << "[PASSED] TestTroposphereLapseAndTropopause\n";
    return true;
}

bool TestNonStandardDayDensityAltitude() {
    std::cout << "[RUNNING] TestNonStandardDayDensityAltitude...\n";

    // Hot day: ISA + 20 K (35.0 °C sea level)
    AtmosphericState hotState = AtmosphereModel::Evaluate(0.0, 20.0);
    TEST_ASSERT(FloatNear(hotState.temperatureCelsius, 35.0, 0.1), "Hot day sea level temp must be 35.0 °C");
    TEST_ASSERT(hotState.densityKgM3 < 1.225, "Hot air must be less dense than standard air");
    // At ISA+20, density altitude at sea level rises to ~2300-2500 ft
    TEST_ASSERT(hotState.densityAltitudeFt > 2000.0, "Hot day density altitude must be significantly above 0 ft");
    TEST_ASSERT(FloatNear(hotState.pressureAltitudeFt, 0.0, 5.0), "Pressure altitude must remain 0 ft at standard QNH");

    // Low pressure day: QNH = 99000 Pa (29.23 inHg)
    AtmosphericState lowQnh = AtmosphereModel::Evaluate(0.0, 0.0, 99000.0);
    TEST_ASSERT(lowQnh.pressureAltitudeFt > 600.0, "Low QNH must produce positive pressure altitude at sea level");

    std::cout << "[PASSED] TestNonStandardDayDensityAltitude\n";
    return true;
}

bool TestSpeedOfSoundAndViscosity() {
    std::cout << "[RUNNING] TestSpeedOfSoundAndViscosity...\n";

    f64 aSeaLevel = AtmosphereModel::CalculateSpeedOfSound(288.15);
    f64 aTrop = AtmosphereModel::CalculateSpeedOfSound(216.65);
    TEST_ASSERT(FloatNear(aSeaLevel, 340.29, 0.5), "Sea level speed of sound must be ~340.3 m/s");
    TEST_ASSERT(FloatNear(aTrop, 295.07, 0.5), "Tropopause speed of sound must be ~295.1 m/s");
    TEST_ASSERT(aTrop < aSeaLevel, "Speed of sound must decrease with falling temperature");

    f64 muWarm = AtmosphereModel::CalculateDynamicViscosity(300.0);
    f64 muCold = AtmosphereModel::CalculateDynamicViscosity(220.0);
    TEST_ASSERT(muWarm > muCold, "Air dynamic viscosity must drop with temperature according to Sutherland's law");

    std::cout << "[PASSED] TestSpeedOfSoundAndViscosity\n";
    return true;
}

bool TestLogarithmicBoundaryLayerWindShear() {
    std::cout << "[RUNNING] TestLogarithmicBoundaryLayerWindShear...\n";

    WindModel wind;
    WindConfig cfg{};
    cfg.surfaceSpeedMps = 10.0; // 10 m/s (~19.4 kts)
    cfg.surfaceDirectionDeg = 360.0; // From North (flows South: +X)
    cfg.referenceHeightM = 10.0;
    cfg.roughnessLengthM = 0.05; // Airport grass
    wind.SetWindConfig(cfg);

    // At reference height 10m: speed must equal 10.0 m/s
    Math::Vec3d w10 = wind.CalculateSteadyWindNed(10.0);
    f64 speed10 = std::sqrt(w10.x * w10.x + w10.y * w10.y);
    TEST_ASSERT(FloatNear(speed10, 10.0, 0.05), "Wind speed at reference 10m must equal 10.0 m/s");

    // Below 10m (e.g. 1m): wind speed must decrease due to ground boundary friction
    Math::Vec3d w1 = wind.CalculateSteadyWindNed(1.0);
    f64 speed1 = std::sqrt(w1.x * w1.x + w1.y * w1.y);
    TEST_ASSERT(speed1 < speed10, "Wind speed at 1m must be lower than at 10m");
    TEST_ASSERT(FloatNear(speed1, 5.65, 0.2), "Logarithmic wind at 1m must be ~5.65 m/s");

    // At or below ground roughness height: wind speed must drop to 0
    Math::Vec3d w0 = wind.CalculateSteadyWindNed(0.04);
    f64 speed0 = std::sqrt(w0.x * w0.x + w0.y * w0.y);
    TEST_ASSERT(FloatNear(speed0, 0.0, 0.01), "Wind speed below roughness height must be 0");

    // Aloft at 300m: wind speed must be higher than surface
    Math::Vec3d w300 = wind.CalculateSteadyWindNed(300.0);
    f64 speed300 = std::sqrt(w300.x * w300.x + w300.y * w300.y);
    TEST_ASSERT(speed300 > speed10, "Wind speed at 300m must be higher than surface");
    TEST_ASSERT(FloatNear(speed300, 16.4, 0.5), "Wind speed at 300m must be ~16.4 m/s");

    std::cout << "[PASSED] TestLogarithmicBoundaryLayerWindShear\n";
    return true;
}

bool TestRunwayWindDecomposition() {
    std::cout << "[RUNNING] TestRunwayWindDecomposition...\n";

    f64 headwind = 0.0;
    f64 crosswind = 0.0;
    f64 rwyHdg = 000.0; // Runway 36 heading North

    // 1. Direct headwind from 360° at 20 kts
    WindModel::DecomposeRunwayWind(20.0, 360.0, rwyHdg, headwind, crosswind);
    TEST_ASSERT(FloatNear(headwind, 20.0, 0.01), "Direct North wind must produce 20 kt headwind");
    TEST_ASSERT(FloatNear(crosswind, 0.0, 0.01), "Direct North wind must produce 0 kt crosswind");

    // 2. Direct tailwind from 180° at 15 kts
    WindModel::DecomposeRunwayWind(15.0, 180.0, rwyHdg, headwind, crosswind);
    TEST_ASSERT(FloatNear(headwind, -15.0, 0.01), "South wind must produce -15 kt headwind (tailwind)");
    TEST_ASSERT(FloatNear(crosswind, 0.0, 0.01), "South wind must produce 0 kt crosswind");

    // 3. Direct crosswind from 090° (East / right) at 12 kts
    WindModel::DecomposeRunwayWind(12.0, 90.0, rwyHdg, headwind, crosswind);
    TEST_ASSERT(FloatNear(headwind, 0.0, 0.01), "East wind must produce 0 kt headwind");
    TEST_ASSERT(FloatNear(crosswind, 12.0, 0.01), "East wind must produce +12 kt right crosswind");

    // 4. Direct crosswind from 270° (West / left) at 12 kts
    WindModel::DecomposeRunwayWind(12.0, 270.0, rwyHdg, headwind, crosswind);
    TEST_ASSERT(FloatNear(headwind, 0.0, 0.01), "West wind must produce 0 kt headwind");
    TEST_ASSERT(FloatNear(crosswind, -12.0, 0.01), "West wind must produce -12 kt left crosswind");

    // 5. Quartering headwind from 315° at 14.14 kts (10 kt headwind, 10 kt left crosswind)
    WindModel::DecomposeRunwayWind(14.142, 315.0, rwyHdg, headwind, crosswind);
    TEST_ASSERT(FloatNear(headwind, 10.0, 0.1), "315° quartering wind must produce 10 kt headwind");
    TEST_ASSERT(FloatNear(crosswind, -10.0, 0.1), "315° quartering wind must produce -10 kt left crosswind");

    std::cout << "[PASSED] TestRunwayWindDecomposition\n";
    return true;
}

bool TestDrydenTurbulenceAndRelativeVelocity() {
    std::cout << "[RUNNING] TestDrydenTurbulenceAndRelativeVelocity...\n";

    EnvironmentSystem env;
    env.ApplyPreset(WeatherPreset::GustyTurbulence);

    // Initial state at 500m altitude
    Math::Vec3d pos{ 0.0, 0.0, -500.0 };
    Math::Quat4d attitude = Math::Quat4d::Identity(); // Heading North
    f64 tas = 50.0; // 50 m/s (~97 kts)
    f64 dt = 1.0 / 120.0;

    // Advance 120 steps (1.0 second of simulation)
    f64 maxGustMag = 0.0;
    for (int i = 0; i < 120; ++i) {
        env.Update(pos, attitude, tas, dt);
        const auto& state = env.GetState();
        f64 gustMag = state.gustVelocityNed.Length();
        if (gustMag > maxGustMag) {
            maxGustMag = gustMag;
        }
    }

    TEST_ASSERT(maxGustMag > 0.1, "Dryden turbulence filter must generate active gust perturbations");
    TEST_ASSERT(maxGustMag < 20.0, "Gust magnitudes must remain within physical bounds");

    // Relative velocity check
    // Aircraft moving at 50 m/s along body X:
    Math::Vec3d vAircraftBody{ 50.0, 0.0, 0.0 };
    Math::Vec3d vRelBody = env.ComputeRelativeVelocityBody(vAircraftBody);
    // Relative velocity must differ from aircraft ground velocity when wind is present
    TEST_ASSERT((vRelBody - vAircraftBody).Length() > 0.01, "Relative velocity must differ from inertial velocity when wind is active");

    // Preset cycling
    env.ApplyPreset(WeatherPreset::ClearCalm);
    TEST_ASSERT(env.GetActivePreset() == WeatherPreset::ClearCalm, "Preset must reset to ClearCalm");
    env.CyclePreset();
    TEST_ASSERT(env.GetActivePreset() == WeatherPreset::StandardDay, "CyclePreset must advance to StandardDay");

    std::cout << "[PASSED] TestDrydenTurbulenceAndRelativeVelocity\n";
    return true;
}

} // namespace

int main() {
    std::cout << "============================================================\n";
    std::cout << "LokYS Native Windows Flight Simulator — Atmosphere & Weather Tests\n";
    std::cout << "============================================================\n";

    bool allPassed = true;
    allPassed &= TestStandardSeaLevelIsa();
    allPassed &= TestTroposphereLapseAndTropopause();
    allPassed &= TestNonStandardDayDensityAltitude();
    allPassed &= TestSpeedOfSoundAndViscosity();
    allPassed &= TestLogarithmicBoundaryLayerWindShear();
    allPassed &= TestRunwayWindDecomposition();
    allPassed &= TestDrydenTurbulenceAndRelativeVelocity();

    std::cout << "============================================================\n";
    if (allPassed) {
        std::cout << "[ALL 7 ATMOSPHERE & WEATHER TEST SUITES PASSED (100%)]\n";
        return 0;
    } else {
        std::cout << "[ONE OR MORE ATMOSPHERE & WEATHER TESTS FAILED]\n";
        return 1;
    }
}
