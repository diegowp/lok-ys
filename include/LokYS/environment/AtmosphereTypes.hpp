#pragma once

#include "LokYS/core/Types.hpp"
#include "LokYS/math/Vector3.hpp"
#include <string>

namespace LokYS::Environment {

namespace Constants {
inline constexpr f64 STANDARD_SEA_LEVEL_TEMP_K   = 288.15;     // 15.0 °C
inline constexpr f64 STANDARD_SEA_LEVEL_PRESS_PA = 101325.0;   // 29.9213 inHg
inline constexpr f64 STANDARD_SEA_LEVEL_DENSITY  = 1.2250;     // kg/m^3
inline constexpr f64 STANDARD_GRAVITY_ACCEL      = 9.80665;    // m/s^2
inline constexpr f64 SPECIFIC_GAS_CONSTANT_AIR   = 287.05287;  // J / (kg * K)
inline constexpr f64 SPECIFIC_HEAT_RATIO_GAMMA   = 1.4;        // Adiabatic index
inline constexpr f64 TROPOSPHERE_LAPSE_RATE_K_M  = -0.0065;    // -6.5 K/km
inline constexpr f64 TROPOPAUSE_ALTITUDE_M       = 11000.0;    // 36,089 ft
inline constexpr f64 TROPOPAUSE_TEMP_K           = 216.65;     // -56.5 °C
inline constexpr f64 SUTHERLAND_REFERENCE_TEMP_K = 288.15;     // T0
inline constexpr f64 SUTHERLAND_REFERENCE_VISC   = 1.7894e-5;  // mu0 [kg/(m*s)]
inline constexpr f64 SUTHERLAND_CONSTANT_K       = 110.4;      // S [K]
inline constexpr f64 PASCALS_PER_INHG            = 3386.389;
inline constexpr f64 METERS_TO_FEET             = 3.280839895;
inline constexpr f64 FEET_TO_METERS             = 0.3048;
inline constexpr f64 KNOTS_TO_MPS               = 0.514444444;
inline constexpr f64 MPS_TO_KNOTS               = 1.943844492;
} // namespace Constants

struct AtmosphericState {
    f64 temperatureKelvin{ Constants::STANDARD_SEA_LEVEL_TEMP_K };
    f64 temperatureCelsius{ 15.0 };
    f64 pressurePa{ Constants::STANDARD_SEA_LEVEL_PRESS_PA };
    f64 pressureInHg{ 29.92 };
    f64 densityKgM3{ Constants::STANDARD_SEA_LEVEL_DENSITY };
    f64 speedOfSoundMps{ 340.294 };
    f64 dynamicViscosity{ Constants::SUTHERLAND_REFERENCE_VISC };
    f64 pressureAltitudeFt{ 0.0 };
    f64 densityAltitudeFt{ 0.0 };
};

enum class TurbulenceSeverity {
    None,
    Light,
    Moderate,
    Severe
};

enum class WeatherPreset {
    ClearCalm,
    StandardDay,
    CrosswindRunway36,
    GustyTurbulence,
    HotAndHigh
};

struct WindConfig {
    f64 surfaceSpeedMps{ 0.0 };       ///< Wind speed at reference height (10m)
    f64 surfaceDirectionDeg{ 0.0 };   ///< Direction wind blows FROM (0..360 deg, 0 = North)
    f64 aloftSpeedMps{ 0.0 };         ///< Wind speed at aloft altitude (e.g., 1000m)
    f64 aloftDirectionDeg{ 0.0 };     ///< Direction aloft wind blows FROM
    f64 aloftAltitudeM{ 1000.0 };     ///< Transition altitude for aloft wind layer
    f64 roughnessLengthM{ 0.05 };     ///< Ground roughness z0 (0.05m = airport runway/mown grass)
    f64 referenceHeightM{ 10.0 };     ///< Anemometer standard reference height (10m)
};

struct TurbulenceConfig {
    TurbulenceSeverity severity{ TurbulenceSeverity::None };
    f64 gustFactor{ 0.0 };            ///< Peak gust multiplier above steady wind
    f64 thermalUpdraftStrengthMps{ 0.0 }; ///< Peak thermal core vertical velocity
    f64 thermalRadiusM{ 250.0 };      ///< Thermal column characteristic radius
    Math::Vec3d thermalCenterNed{ -1500.0, 500.0, 0.0 }; ///< Spatial position of thermal core
};

struct EnvironmentState {
    AtmosphericState atmosphere{};
    Math::Vec3d steadyWindNed{ 0.0, 0.0, 0.0 };
    Math::Vec3d gustVelocityNed{ 0.0, 0.0, 0.0 };
    Math::Vec3d totalWindNed{ 0.0, 0.0, 0.0 };
    Math::Vec3d totalWindBody{ 0.0, 0.0, 0.0 };
    f64 headwindMps{ 0.0 };
    f64 crosswindMps{ 0.0 };
    f64 thermalUpdraftMps{ 0.0 };
    WeatherPreset activePreset{ WeatherPreset::ClearCalm };
};

} // namespace LokYS::Environment
