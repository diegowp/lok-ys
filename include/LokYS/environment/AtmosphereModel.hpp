#pragma once

#include "LokYS/environment/AtmosphereTypes.hpp"

namespace LokYS::Environment {

class AtmosphereModel {
public:
    AtmosphereModel() = default;

    /// @brief Evaluates complete atmospheric state at given altitude.
    /// @param altitudeM Height above mean sea level in meters.
    /// @param deltaIsaK Deviation from standard temperature in Kelvin (e.g., +10.0 for ISA+10).
    /// @param seaLevelPressurePa Reference sea level barometric pressure (QNH in Pa).
    [[nodiscard]] static AtmosphericState Evaluate(
        f64 altitudeM,
        f64 deltaIsaK = 0.0,
        f64 seaLevelPressurePa = Constants::STANDARD_SEA_LEVEL_PRESS_PA) noexcept;

    [[nodiscard]] static f64 CalculateSpeedOfSound(f64 tempK) noexcept;
    [[nodiscard]] static f64 CalculateDynamicViscosity(f64 tempK) noexcept;
    [[nodiscard]] static f64 CalculatePressureAltitudeFt(f64 pressurePa) noexcept;
    [[nodiscard]] static f64 CalculateDensityAltitudeFt(f64 densityKgM3) noexcept;
};

} // namespace LokYS::Environment
