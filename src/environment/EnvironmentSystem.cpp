#include "LokYS/environment/EnvironmentSystem.hpp"

namespace LokYS::Environment {

EnvironmentSystem::EnvironmentSystem() {
    Initialize();
}

void EnvironmentSystem::Initialize() {
    ApplyPreset(WeatherPreset::ClearCalm);
}

void EnvironmentSystem::ApplyPreset(WeatherPreset preset) noexcept {
    m_state.activePreset = preset;
    m_windModel.ResetTurbulence();

    WindConfig windCfg{};
    TurbulenceConfig turbCfg{};

    switch (preset) {
    case WeatherPreset::ClearCalm:
        m_deltaIsaK = 0.0;
        m_seaLevelPressurePa = Constants::STANDARD_SEA_LEVEL_PRESS_PA;
        windCfg.surfaceSpeedMps = 0.0;
        windCfg.surfaceDirectionDeg = 0.0;
        windCfg.aloftSpeedMps = 0.0;
        windCfg.aloftDirectionDeg = 0.0;
        turbCfg.severity = TurbulenceSeverity::None;
        turbCfg.thermalUpdraftStrengthMps = 0.0;
        break;

    case WeatherPreset::StandardDay:
        m_deltaIsaK = 0.0;
        m_seaLevelPressurePa = Constants::STANDARD_SEA_LEVEL_PRESS_PA;
        windCfg.surfaceSpeedMps = 5.0 * Constants::KNOTS_TO_MPS; // 5 kts
        windCfg.surfaceDirectionDeg = 360.0;
        windCfg.aloftSpeedMps = 12.0 * Constants::KNOTS_TO_MPS;  // 12 kts
        windCfg.aloftDirectionDeg = 340.0;
        windCfg.aloftAltitudeM = 1000.0;
        turbCfg.severity = TurbulenceSeverity::Light;
        turbCfg.thermalUpdraftStrengthMps = 0.0;
        break;

    case WeatherPreset::CrosswindRunway36:
        m_deltaIsaK = 0.0;
        m_seaLevelPressurePa = Constants::STANDARD_SEA_LEVEL_PRESS_PA;
        windCfg.surfaceSpeedMps = 15.0 * Constants::KNOTS_TO_MPS; // 15 kts
        windCfg.surfaceDirectionDeg = 290.0; // 70 deg crosswind from West
        windCfg.aloftSpeedMps = 22.0 * Constants::KNOTS_TO_MPS;  // 22 kts
        windCfg.aloftDirectionDeg = 280.0;
        windCfg.aloftAltitudeM = 1000.0;
        turbCfg.severity = TurbulenceSeverity::Light;
        turbCfg.thermalUpdraftStrengthMps = 0.0;
        break;

    case WeatherPreset::GustyTurbulence:
        m_deltaIsaK = 5.0; // ISA + 5 K
        m_seaLevelPressurePa = 100800.0; // 29.77 inHg
        windCfg.surfaceSpeedMps = 18.0 * Constants::KNOTS_TO_MPS; // 18 kts
        windCfg.surfaceDirectionDeg = 320.0;
        windCfg.aloftSpeedMps = 28.0 * Constants::KNOTS_TO_MPS;  // 28 kts
        windCfg.aloftDirectionDeg = 310.0;
        windCfg.aloftAltitudeM = 1200.0;
        turbCfg.severity = TurbulenceSeverity::Moderate;
        turbCfg.gustFactor = 1.35;
        turbCfg.thermalUpdraftStrengthMps = 3.5; // +700 fpm thermal core
        turbCfg.thermalRadiusM = 300.0;
        turbCfg.thermalCenterNed = { -2000.0, 400.0, 0.0 };
        break;

    case WeatherPreset::HotAndHigh:
        m_deltaIsaK = 20.0; // 35 °C at sea level (ISA + 20 K)
        m_seaLevelPressurePa = 100200.0; // 29.59 inHg
        windCfg.surfaceSpeedMps = 6.0 * Constants::KNOTS_TO_MPS; // 6 kts
        windCfg.surfaceDirectionDeg = 180.0;
        windCfg.aloftSpeedMps = 10.0 * Constants::KNOTS_TO_MPS;
        windCfg.aloftDirectionDeg = 180.0;
        windCfg.aloftAltitudeM = 1000.0;
        turbCfg.severity = TurbulenceSeverity::Light;
        turbCfg.thermalUpdraftStrengthMps = 2.0;
        break;
    }

    m_windModel.SetWindConfig(windCfg);
    m_windModel.SetTurbulenceConfig(turbCfg);
}

void EnvironmentSystem::CyclePreset() noexcept {
    switch (m_state.activePreset) {
    case WeatherPreset::ClearCalm:
        ApplyPreset(WeatherPreset::StandardDay);
        break;
    case WeatherPreset::StandardDay:
        ApplyPreset(WeatherPreset::CrosswindRunway36);
        break;
    case WeatherPreset::CrosswindRunway36:
        ApplyPreset(WeatherPreset::GustyTurbulence);
        break;
    case WeatherPreset::GustyTurbulence:
        ApplyPreset(WeatherPreset::HotAndHigh);
        break;
    case WeatherPreset::HotAndHigh:
        ApplyPreset(WeatherPreset::ClearCalm);
        break;
    }
}

std::wstring EnvironmentSystem::GetPresetName() const noexcept {
    switch (m_state.activePreset) {
    case WeatherPreset::ClearCalm:         return L"Calm";
    case WeatherPreset::StandardDay:       return L"Standard";
    case WeatherPreset::CrosswindRunway36: return L"Crosswind 15kt";
    case WeatherPreset::GustyTurbulence:   return L"Turbulent";
    case WeatherPreset::HotAndHigh:        return L"Hot & High (+20C)";
    }
    return L"Unknown";
}

void EnvironmentSystem::Update(
    const Math::Vec3d& aircraftPosNed,
    const Math::Quat4d& aircraftAttitude,
    f64 trueAirspeedMps,
    f64 dt) noexcept
{
    f64 altM = -aircraftPosNed.z;

    // 1. Evaluate barometric and density atmospheric state
    m_state.atmosphere = AtmosphereModel::Evaluate(altM, m_deltaIsaK, m_seaLevelPressurePa);

    // 2. Evaluate 3D wind velocity in NED
    m_state.totalWindNed = m_windModel.EvaluateTotalWindNed(aircraftPosNed, trueAirspeedMps, dt);
    m_state.steadyWindNed = m_windModel.GetLastSteadyWindNed();
    m_state.gustVelocityNed = m_windModel.GetLastGustNed();
    m_state.thermalUpdraftMps = m_windModel.GetLastThermalUpdraftMps();

    // 3. Transform NED wind into aircraft Body frame (FRD)
    m_state.totalWindBody = aircraftAttitude.Unrotate(m_state.totalWindNed);

    // 4. Decompose wind relative to runway heading (Runway 36 = 000 deg)
    f64 surfaceDir = m_windModel.GetWindConfig().surfaceDirectionDeg;
    f64 surfaceSpeed = m_windModel.GetWindConfig().surfaceSpeedMps;
    WindModel::DecomposeRunwayWind(
        surfaceSpeed,
        surfaceDir,
        m_runwayHeadingDeg,
        m_state.headwindMps,
        m_state.crosswindMps);
}

Math::Vec3d EnvironmentSystem::ComputeRelativeVelocityBody(
    const Math::Vec3d& aircraftVelocityBody) const noexcept
{
    return aircraftVelocityBody - m_state.totalWindBody;
}

} // namespace LokYS::Environment
