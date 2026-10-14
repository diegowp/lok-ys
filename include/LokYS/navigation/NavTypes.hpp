#pragma once

#include "LokYS/core/Types.hpp"
#include "LokYS/math/Vector3.hpp"
#include <string>

namespace LokYS::Navigation {

/// @brief Classification of terrestrial radio navigation facilities.
enum class NavFacilityType {
    VOR,       ///< VHF Omnidirectional Range (VHF 108.00 - 117.95 MHz)
    VOR_DME,   ///< VOR co-located with Distance Measuring Equipment
    ILS,       ///< Instrument Landing System (Paired Localizer + Glideslope)
    DME        ///< Standalone Distance Measuring Equipment
};

/// @brief Approach marker beacon types.
enum class MarkerBeaconType {
    None,
    Outer,   ///< OM: Typically 4-7 NM out (Blue lamp, 400 Hz tone)
    Middle,  ///< MM: Typically 3500 ft out (Amber lamp, 1300 Hz tone)
    Inner    ///< IM: Runway threshold (White lamp, 3000 Hz tone)
};

/// @brief Representation of a ground-based navigational aid.
struct NavFacility {
    std::string id;              ///< 3-4 letter ICAO identifier (e.g. "LOK", "PAO", "ILOK")
    std::string name;            ///< Plain text description
    NavFacilityType type;        ///< Facility category
    f64 frequencyMhz{ 114.50 };  ///< Operating frequency in MHz (e.g. 109.50, 114.50)
    Math::Vec3d positionNed{ 0.0, 0.0, 0.0 }; ///< Ground location in world NED coordinates (meters)
    f64 elevationM{ 0.0 };       ///< Ground elevation in meters MSL
    f64 magneticVariationDeg{ 0.0 }; ///< Local magnetic declination (positive East)
    f64 rangeNm{ 40.0 };         ///< Service volume reception radius in nautical miles
    bool hasDme{ false };        ///< True if equipped with DME transponder

    // ILS Specific Geometry
    f64 inboundCourseDeg{ 0.0 }; ///< Runway magnetic heading (e.g. 000.0 for RWY 36)
    f64 glideslopeAngleDeg{ 3.0 }; ///< Glideslope descent angle (standard 3.0 deg)
    Math::Vec3d glideslopeAntennaNed{ 0.0, 0.0, 0.0 }; ///< Glideslope transmitter offset location
};

/// @brief State of the Course Deviation Indicator (CDI / OBS instrument).
struct CdiState {
    f64 obsDeg{ 0.0 };                  ///< Selected Omni Bearing Selector course [0, 360)
    f64 radialDeg{ 0.0 };               ///< Sensed magnetic radial FROM the station [0, 360)
    f64 courseDeviationDots{ 0.0 };     ///< CDI vertical needle deflection [-5.0, 5.0] dots (negative left, positive right)
    f64 glideslopeDeviationDots{ 0.0 }; ///< GS horizontal needle deflection [-5.0, 5.0] dots (negative fly-up, positive fly-down)
    bool toFlag{ false };               ///< TO indicator triangle visible
    bool fromFlag{ false };             ///< FROM indicator triangle visible
    bool navFlag{ true };               ///< NAV warning flag visible (no signal / cone of ambiguity)
    bool gsFlag{ true };                ///< GS warning flag visible (no glideslope signal)
    bool isIls{ false };                ///< True if tuned to an active ILS localizer/glideslope
};

/// @brief Distance Measuring Equipment interrogation state.
struct DmeState {
    bool hasSignal{ false };            ///< True if DME station is within line-of-sight interrogation range
    f64 slantRangeNm{ 0.0 };            ///< Direct radar slant range in nautical miles
    f64 groundSpeedKnots{ 0.0 };        ///< Closing ground speed in knots relative to station
    f64 timeToStationMinutes{ 0.0 };    ///< Estimated time to station in minutes
};

/// @brief Runway approach marker beacon receiver state.
struct MarkerBeaconState {
    MarkerBeaconType activeMarker{ MarkerBeaconType::None };
    f64 audioToneHz{ 0.0 };             ///< Morse audio tone frequency (Hz)
    bool lightBlue{ false };            ///< Outer Marker active
    bool lightAmber{ false };           ///< Middle Marker active
    bool lightWhite{ false };           ///< Inner Marker active
};

/// @brief Complete state of a VHF Navigation Receiver (NAV1 or NAV2).
struct NavReceiverState {
    f64 activeFrequencyMhz{ 114.50 };   ///< Currently active tuned frequency (108.00 - 117.95 MHz)
    f64 standbyFrequencyMhz{ 109.50 };  ///< Standby frequency ready for flip-flop transfer
    bool audioIdentEnabled{ false };    ///< Pilot audio monitor active for 1020 Hz Morse ident
    std::string stationIdent;           ///< Identifier string of acquired station
    f64 signalStrength{ 0.0 };          ///< Sensed carrier power [0.0, 1.0]

    CdiState cdi{};
    DmeState dme{};
    MarkerBeaconState marker{};
};

} // namespace LokYS::Navigation
