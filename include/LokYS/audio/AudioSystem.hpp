#pragma once

#include "LokYS/core/Types.hpp"
#include "LokYS/audio/AudioTypes.hpp"
#include <memory>

namespace LokYS {

class AudioSystem {
public:
    AudioSystem();
    ~AudioSystem();

    // Non-copyable, movable
    AudioSystem(const AudioSystem&) = delete;
    AudioSystem& operator=(const AudioSystem&) = delete;
    AudioSystem(AudioSystem&&) noexcept;
    AudioSystem& operator=(AudioSystem&&) noexcept;

    /// @brief Initializes XAudio2 engine, mastering voice, 3D spatial subsystem, and procedural synthesizers.
    /// @return True if initialized successfully or fallback mode safely engaged.
    bool Initialize(const AudioConfig& config = AudioConfig{});

    /// @brief Gracefully releases all voice streams, buffers, and audio devices.
    void Shutdown();

    /// @brief Checks whether the native XAudio2 device and audio endpoints are active.
    [[nodiscard]] bool IsAvailable() const noexcept;

    /// @brief Main per-frame audio update running synchronized with the frame renderer.
    void Update(
        f32 dt,
        const AudioListenerState& listener,
        const AudioEmitterState& emitter,
        const EngineAudioParams& engine,
        const WindAudioParams& wind,
        const GroundAudioParams& ground);

    /// @brief Toggles between cockpit internal acoustic attenuation and chase open-air acoustics.
    void SetCockpitPerspective(bool inCockpit);

    /// @brief Sets the global mastering audio volume [0.0, 1.0].
    void SetMasterVolume(f32 volume);

    /// @brief Triggers a transient tire touchdown screech event.
    void TriggerTouchdown(f32 forwardSpeed);

    // =========================================================================
    // Mathematical & Physical Sound Modeling Pure Functions (Accessible for Testing)
    // =========================================================================

    /// @brief Computes XAudio2 frequency ratio for Lycoming O-320 engine.
    /// Reference baseline: 1200 RPM = 1.0 frequency ratio.
    [[nodiscard]] static f32 CalculateEnginePitchRatio(f32 rpm) noexcept;

    /// @brief Computes synthesized engine volume from RPM and throttle load.
    [[nodiscard]] static f32 CalculateEngineVolume(f32 rpm, f32 throttle) noexcept;

    /// @brief Computes aerodynamic wind noise volume from dynamic pressure q (Pa).
    [[nodiscard]] static f32 CalculateWindVolume(f32 dynamicPressure) noexcept;

    /// @brief Computes low-pass filter cutoff frequency (Hz) for aerodynamic wind.
    [[nodiscard]] static f32 CalculateWindCutoff(f32 airspeed) noexcept;

    /// @brief Computes ground roll vibration volume from ground speed and contact state.
    [[nodiscard]] static f32 CalculateGroundRumbleVolume(f32 groundSpeed, bool anyWheelContact) noexcept;

    /// @brief Returns the acoustic filter cutoff frequency based on cockpit perspective.
    [[nodiscard]] static f32 CalculateCockpitCutoffFrequency(bool inCockpit) noexcept;

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace LokYS
