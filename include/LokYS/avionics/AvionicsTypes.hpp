#pragma once

#include "LokYS/core/Types.hpp"
#include "LokYS/math/Units.hpp"
#include <cmath>
#include <string>

namespace LokYS::Avionics {

/// State and readings for the Airspeed Indicator (ASI)
struct AirspeedIndicatorState {
    f64 indicatedAirspeedKnots{ 0.0 };  ///< Lag-filtered Indicated Airspeed (IAS)
    f64 trueAirspeedKnots{ 0.0 };       ///< True Airspeed (TAS)
    f64 dynamicPressurePa{ 0.0 };       ///< Pitot-static dynamic pressure q = 0.5 * rho * V^2
    f64 needleAngleDeg{ 0.0 };          ///< Dial needle angular deflection in degrees [0, 360)

    // Cessna 172SP Flight Operating Limits (Knots)
    static constexpr f64 Vso = 40.0;    ///< Stall speed with flaps full down (start of white arc)
    static constexpr f64 Vfe = 85.0;    ///< Max flap extended speed (top of white arc)
    static constexpr f64 Vs1 = 48.0;    ///< Stall speed clean (start of green arc)
    static constexpr f64 Vno = 129.0;   ///< Max structural cruising speed (top of green arc / bottom of yellow)
    static constexpr f64 Vne = 163.0;   ///< Never exceed speed (red radial line)
    static constexpr f64 MaxDialSpeed = 200.0; ///< Full scale on dial face
};

/// State and readings for the Attitude Indicator (AI) / Artificial Horizon
struct AttitudeIndicatorState {
    f64 pitchDeg{ 0.0 };                ///< Indicated pitch angle in degrees (positive nose up)
    f64 rollDeg{ 0.0 };                 ///< Indicated roll/bank angle in degrees (positive right bank)
    f64 pitchDisplacementNorm{ 0.0 };   ///< Normalized horizon bar vertical displacement [-1.0, 1.0]
    f64 rollAngleRad{ 0.0 };            ///< Bank rotation in radians
    bool isCaged{ false };              ///< True if gyro is mechanically caged / locked
    bool isTumbled{ false };            ///< True if pitch exceeded gimbal mechanical stops (> +/-85 deg)
};

/// State and readings for the Altimeter
struct AltimeterState {
    f64 indicatedAltitudeFeet{ 0.0 };   ///< Indicated altitude based on sensed static pressure & Kollsman setting
    f64 pressureAltitudeFeet{ 0.0 };    ///< Pressure altitude referenced to standard 29.92 inHg
    f64 kollsmanInHg{ 29.92 };          ///< Barometric subscale setting in inches of mercury (default 29.92)
    f64 staticPressurePa{ 101325.0 };   ///< Ambient static pressure at aircraft port (Pa)

    // Needle rotational angles (0 deg = 12 o'clock, clockwise)
    f64 needle100FtAngleDeg{ 0.0 };     ///< 100-ft pointer (1 rev = 1,000 ft, 0.36 deg/ft)
    f64 needle1000FtAngleDeg{ 0.0 };    ///< 1,000-ft pointer (1 rev = 10,000 ft, 0.036 deg/ft)
    f64 needle10000FtAngleDeg{ 0.0 };   ///< 10,000-ft needle with triangular tip (1 rev = 100,000 ft)
};

/// State and readings for the Turn Coordinator & Inclinometer
struct TurnCoordinatorState {
    f64 turnRateDps{ 0.0 };             ///< Indicated turn rate in degrees/sec (positive right turn)
    f64 symbolRollDeg{ 0.0 };           ///< Miniature aircraft symbol bank deflection angle
    f64 slipSkidBallPosition{ 0.0 };    ///< Inclinometer ball position [-1.0, 1.0]: 0 = coordinated, -1 = left, +1 = right
    f64 lateralSpecificForceMps2{ 0.0 };///< Net lateral body acceleration f_y = a_y - g*sin(phi)*cos(theta)

    static constexpr f64 StandardRateDps = 3.0; ///< Standard rate turn = 3.0 deg/sec (360 deg in 2 minutes)
    static constexpr f64 MaxDeflectionRateDps = 6.0; ///< Full scale deflection
};

/// State and readings for the Heading Indicator / Directional Gyro (DG)
struct HeadingIndicatorState {
    f64 indicatedHeadingDeg{ 0.0 };     ///< Indicated magnetic/gyro heading in degrees [0, 360)
    f64 headingBugDeg{ 0.0 };           ///< Autopilot / pilot reference heading bug [0, 360)
    f64 gyroDriftDeg{ 0.0 };            ///< Accumulated gyro precession drift error in degrees
    f64 compassCardAngleDeg{ 0.0 };     ///< Azimuth card rotation angle (opposite of heading)
};

/// State and readings for the Vertical Speed Indicator (VSI)
struct VerticalSpeedIndicatorState {
    f64 verticalSpeedFpm{ 0.0 };        ///< Indicated vertical speed in feet per minute (with capillary lag!)
    f64 rawVerticalSpeedFpm{ 0.0 };     ///< Instantaneous true vertical speed (ft/min)
    f64 needleAngleDeg{ 0.0 };          ///< Needle deflection angle: 0 deg at 9 o'clock horizontal, positive climb up

    static constexpr f64 MaxScaleFpm = 2000.0; ///< Standard GA dial range: +/- 2,000 fpm
};

/// Aggregate container for the complete Analog Six-Pack instrument cluster
struct SixPackState {
    AirspeedIndicatorState airspeed;
    AttitudeIndicatorState attitude;
    AltimeterState altimeter;
    TurnCoordinatorState turnCoordinator;
    HeadingIndicatorState heading;
    VerticalSpeedIndicatorState verticalSpeed;
};

} // namespace LokYS::Avionics
