#pragma once

#include "LokYS/core/Types.hpp"
#include <numbers>

namespace LokYS::Math {

namespace Constants {
    inline constexpr f64 PI = std::numbers::pi;
    inline constexpr f64 TWO_PI = std::numbers::pi * 2.0;
    inline constexpr f64 HALF_PI = std::numbers::pi_v<f64> * 0.5;
    inline constexpr f64 STANDARD_GRAVITY = 9.80665;        // m/s^2 (WGS-84 sea level nominal)
    inline constexpr f64 SEA_LEVEL_PRESSURE = 101325.0;    // Pa (ISA standard)
    inline constexpr f64 STANDARD_SEA_LEVEL_PRESSURE = 101325.0; // Pa alias
    inline constexpr f64 SEA_LEVEL_TEMP = 288.15;          // K (15 C ISA standard)
    inline constexpr f64 SEA_LEVEL_DENSITY = 1.225;        // kg/m^3 (ISA standard)
    inline constexpr f64 GAS_CONSTANT_AIR = 287.05287;     // J/(kg*K)
    inline constexpr f64 ADIABATIC_INDEX = 1.4;            // Gamma for diatomic gas
}

class Units {
public:
    // --- Volume ---
    [[nodiscard]] static constexpr f64 GallonsToLiters(f64 gallons) noexcept {
        return gallons * 3.785411784;
    }
    [[nodiscard]] static constexpr f64 LitersToGallons(f64 liters) noexcept {
        return liters / 3.785411784;
    }

    // --- Angle ---
    [[nodiscard]] static constexpr f64 DegToRad(f64 deg) noexcept {
        return deg * (Constants::PI / 180.0);
    }
    [[nodiscard]] static constexpr f64 RadToDeg(f64 rad) noexcept {
        return rad * (180.0 / Constants::PI);
    }

    [[nodiscard]] static constexpr f32 DegToRad(f32 deg) noexcept {
        return deg * (static_cast<f32>(Constants::PI) / 180.0f);
    }
    [[nodiscard]] static constexpr f32 RadToDeg(f32 rad) noexcept {
        return rad * (180.0f / static_cast<f32>(Constants::PI));
    }

    // --- Distance / Altitude ---
    [[nodiscard]] static constexpr f64 MetersToFeet(f64 meters) noexcept {
        return meters * 3.280839895013123;
    }
    [[nodiscard]] static constexpr f64 FeetToMeters(f64 feet) noexcept {
        return feet * 0.3048;
    }
    [[nodiscard]] static constexpr f64 MetersToNauticalMiles(f64 meters) noexcept {
        return meters / 1852.0;
    }
    [[nodiscard]] static constexpr f64 NauticalMilesToMeters(f64 nm) noexcept {
        return nm * 1852.0;
    }

    // --- Speed ---
    [[nodiscard]] static constexpr f64 MpsToKnots(f64 mps) noexcept {
        return mps * 1.9438444924406;
    }
    [[nodiscard]] static constexpr f64 KnotsToMps(f64 knots) noexcept {
        return knots * 0.51444444444444;
    }
    [[nodiscard]] static constexpr f64 MpsToFeetPerMinute(f64 mps) noexcept {
        return mps * 196.850393700787;
    }
    [[nodiscard]] static constexpr f64 FeetPerMinuteToMps(f64 fpm) noexcept {
        return fpm / 196.850393700787;
    }

    // --- Pressure ---
    [[nodiscard]] static constexpr f64 PascalsToInHg(f64 pa) noexcept {
        return pa / 3386.389;
    }
    [[nodiscard]] static constexpr f64 InHgToPascals(f64 inHg) noexcept {
        return inHg * 3386.389;
    }
    [[nodiscard]] static constexpr f64 PascalsToHPa(f64 pa) noexcept {
        return pa * 0.01;
    }
    [[nodiscard]] static constexpr f64 HPaToPascals(f64 hpa) noexcept {
        return hpa * 100.0;
    }

    // --- Temperature ---
    [[nodiscard]] static constexpr f64 CelsiusToKelvin(f64 c) noexcept {
        return c + 273.15;
    }
    [[nodiscard]] static constexpr f64 KelvinToCelsius(f64 k) noexcept {
        return k - 273.15;
    }
    [[nodiscard]] static constexpr f64 CelsiusToFahrenheit(f64 c) noexcept {
        return c * 1.8 + 32.0;
    }
    [[nodiscard]] static constexpr f64 FahrenheitToCelsius(f64 f) noexcept {
        return (f - 32.0) / 1.8;
    }

    // --- Mass ---
    [[nodiscard]] static constexpr f64 KgToPounds(f64 kg) noexcept {
        return kg * 2.2046226218487757;
    }
    [[nodiscard]] static constexpr f64 PoundsToKg(f64 lbs) noexcept {
        return lbs * 0.45359237;
    }

    // --- Force & Torque ---
    [[nodiscard]] static constexpr f64 NewtonsToPoundsForce(f64 n) noexcept {
        return n * 0.2248089430997105;
    }
    [[nodiscard]] static constexpr f64 PoundsForceToNewtons(f64 lbf) noexcept {
        return lbf * 4.4482216152605;
    }
    [[nodiscard]] static constexpr f64 NewtonMetersToFootPounds(f64 nm) noexcept {
        return nm * 0.737562149277;
    }
    [[nodiscard]] static constexpr f64 FootPoundsToNewtonMeters(f64 ftlb) noexcept {
        return ftlb * 1.3558179483314;
    }
};

} // namespace LokYS::Math
