#pragma once

#include "LokYS/core/Types.hpp"
#include "LokYS/math/Vector3.hpp"

namespace LokYS {

/// @brief Operational audio configuration parameters.
struct AudioConfig {
    f32 masterVolume{1.0f};
    f32 engineVolume{0.85f};
    f32 windVolume{0.70f};
    f32 groundVolume{0.80f};
    f32 speedOfSound{343.5f}; // m/s in dry air at 20°C
    bool enableSpatialAudio{true};
    bool isCockpitPerspective{true};
};

/// @brief Spatial listener position, orientation, and velocity state.
struct AudioListenerState {
    Math::Vec3f position{0.0f, 0.0f, 0.0f};
    Math::Vec3f velocity{0.0f, 0.0f, 0.0f};
    Math::Vec3f forward{0.0f, 0.0f, 1.0f};
    Math::Vec3f up{0.0f, 1.0f, 0.0f};
    bool inCockpit{true};
};

/// @brief Spatial sound emitter state.
struct AudioEmitterState {
    Math::Vec3f position{0.0f, 0.0f, 0.0f};
    Math::Vec3f velocity{0.0f, 0.0f, 0.0f};
    Math::Vec3f forward{0.0f, 0.0f, 1.0f};
    Math::Vec3f up{0.0f, 1.0f, 0.0f};
    f32 minDistance{1.0f};
    f32 maxDistance{500.0f};
};

/// @brief Physical engine audio parameters for real-time synthesis.
struct EngineAudioParams {
    f32 rpm{1000.0f};         // Current crankshaft RPM
    f32 throttle{0.0f};       // Throttle lever [0, 1]
    f32 manifoldPressure{29.92f}; // inHg
    bool running{true};
};

/// @brief Aerodynamic wind audio parameters.
struct WindAudioParams {
    f32 dynamicPressure{0.0f}; // Pascals (q = 0.5 * rho * V^2)
    f32 trueAirspeed{0.0f};    // m/s
    f32 angleOfAttack{0.0f};   // radians
    f32 slipAngle{0.0f};       // radians
};

/// @brief Ground contact and roll audio parameters.
struct GroundAudioParams {
    f32 groundSpeed{0.0f};        // m/s
    f32 totalStrutCompression{0.0f}; // meters
    bool noseContact{false};
    bool leftMainContact{false};
    bool rightMainContact{false};
    bool touchdownEvent{false};   // True for one frame upon touchdown
    f32 touchdownVelocity{0.0f};  // Vertical/horizontal impact speed (m/s)
    f32 brakeFactor{0.0f};        // [0, 1]
};

/// @brief Real-time low-pass filter configuration for cockpit attenuation.
struct AcousticFilterParams {
    f32 frequencyCutoff{8000.0f}; // Hz
    f32 oneOverQ{1.0f};           // Filter resonance dampening
};

} // namespace LokYS
