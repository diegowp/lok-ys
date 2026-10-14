#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif

#include "LokYS/audio/AudioSystem.hpp"
#include <windows.h>
#include <xaudio2.h>
#include <x3daudio.h>
#include <vector>
#include <cmath>
#include <algorithm>
#include <numbers>

namespace LokYS {

namespace {

constexpr u32 AUDIO_SAMPLE_RATE = 44100;
constexpr f32 AUDIO_PI = 3.14159265358979323846f;
constexpr f32 BASE_ENGINE_RPM = 1200.0f;

// Clamp helper
template<typename T>
constexpr T ClampVal(T val, T minVal, T maxVal) noexcept {
    return (val < minVal) ? minVal : ((val > maxVal) ? maxVal : val);
}

// Convert cutoff frequency to normalized XAudio2 radian filter parameter
f32 CutoffToXAudioFilterFreq(f32 cutoffHz, u32 sampleRate) noexcept {
    f32 clampedCutoff = ClampVal(cutoffHz, 20.0f, static_cast<f32>(sampleRate) * 0.49f);
    f32 filterParam = 2.0f * std::sin(AUDIO_PI * clampedCutoff / static_cast<f32>(sampleRate));
    return ClampVal(filterParam, 0.01f, 1.0f);
}

// Procedural waveform synthesis for 1.0 second seamless Lycoming O-320 loop
std::vector<i16> SynthesizeLycomingEngineLoop(u32 sampleRate) {
    // Exactly 1 second = sampleRate samples.
    // At 1200 RPM, rotation freq is 20 Hz (2205 samples per cycle).
    // Firing fundamental is 40 Hz (1102.5 samples, 40 cycles in 44100 samples).
    std::vector<i16> buffer(sampleRate);

    for (u32 i = 0; i < sampleRate; ++i) {
        f32 t = static_cast<f32>(i) / static_cast<f32>(sampleRate);

        // Firing pulses (40 Hz fundamental + harmonics)
        f32 fundamental = 40.0f;
        f32 wave = 0.0f;

        // Asymmetric power stroke envelope (combining 8 Fourier harmonics)
        wave += 0.35f * std::sin(2.0f * AUDIO_PI * fundamental * 1.0f * t);
        wave += 0.25f * std::sin(2.0f * AUDIO_PI * fundamental * 2.0f * t + 0.3f);
        wave += 0.18f * std::sin(2.0f * AUDIO_PI * fundamental * 3.0f * t + 0.7f);
        wave += 0.12f * std::sin(2.0f * AUDIO_PI * fundamental * 4.0f * t + 1.1f);
        wave += 0.08f * std::sin(2.0f * AUDIO_PI * fundamental * 5.0f * t + 1.6f);
        wave += 0.05f * std::sin(2.0f * AUDIO_PI * fundamental * 6.0f * t + 2.0f);
        wave += 0.03f * std::sin(2.0f * AUDIO_PI * fundamental * 7.0f * t + 2.4f);
        wave += 0.02f * std::sin(2.0f * AUDIO_PI * fundamental * 8.0f * t + 2.8f);

        // Sub-harmonic rotation pulse (20 Hz) representing intake runner & piston unbalance
        wave += 0.15f * std::sin(2.0f * AUDIO_PI * 20.0f * t + 0.5f);

        // Propeller blade passage slap (40 Hz sharp wavefront)
        f32 propPhase = std::fmod(t * fundamental, 1.0f);
        f32 propSlap = (propPhase < 0.2f) ? (1.0f - propPhase / 0.2f) * 0.20f : 0.0f;
        wave += propSlap;

        // Subtle mechanical valve train rasp
        f32 valveRasp = 0.04f * std::sin(2.0f * AUDIO_PI * 320.0f * t);
        wave += valveRasp;

        // Clamp & quantize to 16-bit PCM
        f32 clamped = ClampVal(wave, -1.0f, 1.0f);
        buffer[i] = static_cast<i16>(clamped * 30000.0f);
    }

    return buffer;
}

// Procedural seamless looped wind noise buffer (1.0 second)
std::vector<i16> SynthesizeWindNoiseLoop(u32 sampleRate) {
    std::vector<i16> buffer(sampleRate);
    u32 lcg = 123456789;

    // Generate band-limited pinkish noise via multi-rate integration
    f32 b0 = 0.0f, b1 = 0.0f, b2 = 0.0f;
    for (u32 i = 0; i < sampleRate; ++i) {
        // Simple fast deterministic pseudo-random generator
        lcg = lcg * 1664525u + 1013904223u;
        f32 white = (static_cast<f32>(lcg & 0xFFFF) / 32768.0f) - 1.0f;

        // Pink noise approximation filter (Paul Kellet's method)
        b0 = 0.99886f * b0 + white * 0.0555179f;
        b1 = 0.99332f * b1 + white * 0.0750759f;
        b2 = 0.96900f * b2 + white * 0.1538520f;
        f32 pink = b0 + b1 + b2 + white * 0.5362f;

        // Window edge 50ms at start and end for zero-click loop transition
        f32 fade = 1.0f;
        u32 fadeSamples = sampleRate / 20; // 50ms
        if (i < fadeSamples) {
            fade = static_cast<f32>(i) / static_cast<f32>(fadeSamples);
        } else if (i > sampleRate - fadeSamples) {
            fade = static_cast<f32>(sampleRate - i) / static_cast<f32>(fadeSamples);
        }

        f32 sample = pink * 0.25f * fade;
        buffer[i] = static_cast<i16>(ClampVal(sample, -1.0f, 1.0f) * 28000.0f);
    }

    return buffer;
}

// Procedural ground roll runway rumble loop (1.0 second)
std::vector<i16> SynthesizeGroundRumbleLoop(u32 sampleRate) {
    std::vector<i16> buffer(sampleRate);
    u32 lcg = 987654321;
    f32 lowAcc = 0.0f;

    for (u32 i = 0; i < sampleRate; ++i) {
        f32 t = static_cast<f32>(i) / static_cast<f32>(sampleRate);

        lcg = lcg * 1664525u + 1013904223u;
        f32 white = (static_cast<f32>(lcg & 0xFFFF) / 32768.0f) - 1.0f;

        // Low-pass filter to isolate structural vibrations (60-120 Hz)
        lowAcc += 0.015f * (white - lowAcc);

        // Add asphalt aggregate texture modulation (55 Hz + 110 Hz)
        f32 texture = 0.4f * std::sin(2.0f * AUDIO_PI * 55.0f * t) +
                      0.2f * std::sin(2.0f * AUDIO_PI * 110.0f * t);

        f32 total = (lowAcc * 0.6f + texture * 0.4f);

        // 50ms boundary crossfade
        u32 fadeSamples = sampleRate / 20;
        f32 fade = 1.0f;
        if (i < fadeSamples) {
            fade = static_cast<f32>(i) / static_cast<f32>(fadeSamples);
        } else if (i > sampleRate - fadeSamples) {
            fade = static_cast<f32>(sampleRate - i) / static_cast<f32>(fadeSamples);
        }

        buffer[i] = static_cast<i16>(ClampVal(total * fade, -1.0f, 1.0f) * 25000.0f);
    }

    return buffer;
}

// Procedural transient tire touchdown screech buffer (0.35 seconds)
std::vector<i16> SynthesizeTouchdownScreech(u32 sampleRate) {
    u32 durationSamples = static_cast<u32>(sampleRate * 0.35f);
    std::vector<i16> buffer(durationSamples);
    u32 lcg = 554433221;

    for (u32 i = 0; i < durationSamples; ++i) {
        f32 t = static_cast<f32>(i) / static_cast<f32>(durationSamples);

        // Downward frequency chirp from 2400 Hz to 1800 Hz
        f32 freq = 2400.0f - 600.0f * t;
        f32 chirp = std::sin(2.0f * AUDIO_PI * freq * (static_cast<f32>(i) / static_cast<f32>(sampleRate)));

        // Friction noise
        lcg = lcg * 1664525u + 1013904223u;
        f32 noise = (static_cast<f32>(lcg & 0xFFFF) / 32768.0f) - 1.0f;

        // Envelope: 10ms attack, exponential decay
        f32 attack = ClampVal(t / 0.05f, 0.0f, 1.0f);
        f32 decay = std::exp(-4.5f * t);
        f32 envelope = attack * decay;

        f32 sample = (0.6f * chirp + 0.4f * noise) * envelope;
        buffer[i] = static_cast<i16>(ClampVal(sample, -1.0f, 1.0f) * 31000.0f);
    }

    return buffer;
}

} // namespace

// =============================================================================
// Implementation Structure
// =============================================================================

struct AudioSystem::Impl {
    AudioConfig config{};
    bool available{false};
    bool coInitialized{false};

    IXAudio2* pXAudio2{nullptr};
    IXAudio2MasteringVoice* pMasteringVoice{nullptr};

    IXAudio2SourceVoice* pEngineVoice{nullptr};
    IXAudio2SourceVoice* pWindVoice{nullptr};
    IXAudio2SourceVoice* pGroundRumbleVoice{nullptr};
    IXAudio2SourceVoice* pTireScreechVoice{nullptr};

    std::vector<i16> engineBufferData;
    std::vector<i16> windBufferData;
    std::vector<i16> groundBufferData;
    std::vector<i16> screechBufferData;

    X3DAUDIO_HANDLE x3dInstance{};
    u32 channelMask{0};
    u32 channelCount{2};
    std::vector<f32> matrixCoefficients;

    Impl() = default;
    ~Impl() {
        Shutdown();
    }

    void Shutdown() {
        if (pTireScreechVoice) {
            pTireScreechVoice->Stop(0);
            pTireScreechVoice->FlushSourceBuffers();
            pTireScreechVoice->DestroyVoice();
            pTireScreechVoice = nullptr;
        }
        if (pGroundRumbleVoice) {
            pGroundRumbleVoice->Stop(0);
            pGroundRumbleVoice->FlushSourceBuffers();
            pGroundRumbleVoice->DestroyVoice();
            pGroundRumbleVoice = nullptr;
        }
        if (pWindVoice) {
            pWindVoice->Stop(0);
            pWindVoice->FlushSourceBuffers();
            pWindVoice->DestroyVoice();
            pWindVoice = nullptr;
        }
        if (pEngineVoice) {
            pEngineVoice->Stop(0);
            pEngineVoice->FlushSourceBuffers();
            pEngineVoice->DestroyVoice();
            pEngineVoice = nullptr;
        }
        if (pMasteringVoice) {
            pMasteringVoice->DestroyVoice();
            pMasteringVoice = nullptr;
        }
        if (pXAudio2) {
            pXAudio2->Release();
            pXAudio2 = nullptr;
        }
        if (coInitialized) {
            CoUninitialize();
            coInitialized = false;
        }
        available = false;
    }
};

// =============================================================================
// Pure Mathematical & Physical Modeling Functions
// =============================================================================

f32 AudioSystem::CalculateEnginePitchRatio(f32 rpm) noexcept {
    f32 clampedRpm = ClampVal(rpm, 200.0f, 3200.0f);
    // Baseline: 1200 RPM = 1.0 frequency ratio
    return clampedRpm / BASE_ENGINE_RPM;
}

f32 AudioSystem::CalculateEngineVolume(f32 rpm, f32 throttle) noexcept {
    f32 clampedRpm = ClampVal(rpm, 0.0f, 3000.0f);
    f32 clampedThrottle = ClampVal(throttle, 0.0f, 1.0f);

    // Idle base volume (0.35) scaling up to full RPM (1.0)
    f32 rpmFactor = 0.35f + 0.65f * (clampedRpm / 2700.0f);

    // Throttle load adds punch and intake/exhaust volume boost (+25%)
    f32 loadBoost = 1.0f + 0.25f * clampedThrottle;

    return ClampVal(rpmFactor * loadBoost, 0.0f, 1.0f);
}

f32 AudioSystem::CalculateWindVolume(f32 dynamicPressure) noexcept {
    f32 clampedQ = ClampVal(dynamicPressure, 0.0f, 4000.0f);
    // Reference cruise q for Cessna 172 at 110 knots (~56 m/s): q ~= 1900 Pa
    constexpr f32 Q_REF = 1900.0f;
    f32 normQ = clampedQ / Q_REF;
    return ClampVal(std::pow(normQ, 1.15f), 0.0f, 1.0f);
}

f32 AudioSystem::CalculateWindCutoff(f32 airspeed) noexcept {
    f32 clampedV = ClampVal(airspeed, 0.0f, 100.0f); // m/s
    // At low speed, muffled low-frequency flutter (500 Hz).
    // At high speed, crisp tearing wind rush up to 6000 Hz.
    return 500.0f + 55.0f * clampedV;
}

f32 AudioSystem::CalculateGroundRumbleVolume(f32 groundSpeed, bool anyWheelContact) noexcept {
    if (!anyWheelContact) {
        return 0.0f;
    }
    f32 clampedSpeed = ClampVal(groundSpeed, 0.0f, 40.0f); // m/s
    return ClampVal(clampedSpeed / 25.0f, 0.0f, 0.85f);
}

f32 AudioSystem::CalculateCockpitCutoffFrequency(bool inCockpit) noexcept {
    // Interior cabin acoustic low-pass filtering:
    // Cockpit: 1100 Hz cutoff (structural attenuation of high-frequency exhaust & wind)
    // Exterior chase: 8000 Hz cutoff (open atmosphere, clear prop slap and exhaust crackle)
    return inCockpit ? 1100.0f : 8000.0f;
}

// =============================================================================
// Lifecycle and Management
// =============================================================================

AudioSystem::AudioSystem()
    : m_impl(std::make_unique<Impl>())
{
}

AudioSystem::~AudioSystem() = default;

AudioSystem::AudioSystem(AudioSystem&&) noexcept = default;
AudioSystem& AudioSystem::operator=(AudioSystem&&) noexcept = default;

bool AudioSystem::Initialize(const AudioConfig& config) {
    m_impl->config = config;

    // Step 1: Initialize COM for multi-threaded audio pipeline
    HRESULT hr = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
    if (SUCCEEDED(hr)) {
        m_impl->coInitialized = true;
    }

    // Step 2: Create native XAudio2 engine instance
    hr = XAudio2Create(&m_impl->pXAudio2, 0, XAUDIO2_DEFAULT_PROCESSOR);
    if (FAILED(hr)) {
        // Fallback: Audio hardware disabled or headless execution environment
        m_impl->available = false;
        return true;
    }

    // Step 3: Create mastering voice linked to default system audio endpoint
    hr = m_impl->pXAudio2->CreateMasteringVoice(&m_impl->pMasteringVoice);
    if (FAILED(hr)) {
        m_impl->pXAudio2->Release();
        m_impl->pXAudio2 = nullptr;
        m_impl->available = false;
        return true;
    }

    // Query speaker geometry and channel count
    DWORD dwChannelMask = 0;
    hr = m_impl->pMasteringVoice->GetChannelMask(&dwChannelMask);
    if (SUCCEEDED(hr)) {
        m_impl->channelMask = dwChannelMask;
    } else {
        m_impl->channelMask = SPEAKER_STEREO;
    }

    XAUDIO2_VOICE_DETAILS voiceDetails{};
    m_impl->pMasteringVoice->GetVoiceDetails(&voiceDetails);
    m_impl->channelCount = voiceDetails.InputChannels;
    if (m_impl->channelCount == 0) {
        m_impl->channelCount = 2;
    }

    // Allocate matrix coefficients for X3DAudio (Src 1 x Dst Channels)
    m_impl->matrixCoefficients.resize(m_impl->channelCount, 0.5f);

    // Step 4: Initialize X3DAudio engine
    hr = X3DAudioInitialize(m_impl->channelMask, m_impl->config.speedOfSound, m_impl->x3dInstance);
    if (FAILED(hr)) {
        // Continue without 3D acceleration if not supported
    }

    // Step 5: Synthesize procedural audio waveforms into memory
    m_impl->engineBufferData  = SynthesizeLycomingEngineLoop(AUDIO_SAMPLE_RATE);
    m_impl->windBufferData    = SynthesizeWindNoiseLoop(AUDIO_SAMPLE_RATE);
    m_impl->groundBufferData  = SynthesizeGroundRumbleLoop(AUDIO_SAMPLE_RATE);
    m_impl->screechBufferData = SynthesizeTouchdownScreech(AUDIO_SAMPLE_RATE);

    // Wave format descriptor: 16-bit Mono PCM, 44.1 kHz
    WAVEFORMATEX wfx{};
    wfx.wFormatTag = WAVE_FORMAT_PCM;
    wfx.nChannels = 1;
    wfx.nSamplesPerSec = AUDIO_SAMPLE_RATE;
    wfx.wBitsPerSample = 16;
    wfx.nBlockAlign = (wfx.nChannels * wfx.wBitsPerSample) / 8;
    wfx.nAvgBytesPerSec = wfx.nSamplesPerSec * wfx.nBlockAlign;
    wfx.cbSize = 0;

    // Step 6: Create source voices with filter support (XAUDIO2_VOICE_USEFILTER)
    auto CreateLoopedVoice = [&](IXAudio2SourceVoice*& pVoice, const std::vector<i16>& data) -> bool {
        HRESULT voiceHr = m_impl->pXAudio2->CreateSourceVoice(
            &pVoice,
            &wfx,
            XAUDIO2_VOICE_USEFILTER,
            XAUDIO2_DEFAULT_FREQ_RATIO,
            nullptr,
            nullptr,
            nullptr);
        if (FAILED(voiceHr)) {
            return false;
        }

        XAUDIO2_BUFFER buffer{};
        buffer.AudioBytes = static_cast<UINT32>(data.size() * sizeof(i16));
        buffer.pAudioData = reinterpret_cast<const BYTE*>(data.data());
        buffer.Flags = XAUDIO2_END_OF_STREAM;
        buffer.LoopCount = XAUDIO2_LOOP_INFINITE;

        voiceHr = pVoice->SubmitSourceBuffer(&buffer);
        if (FAILED(voiceHr)) {
            return false;
        }

        pVoice->Start(0);
        return true;
    };

    if (!CreateLoopedVoice(m_impl->pEngineVoice, m_impl->engineBufferData)) {
        m_impl->Shutdown();
        return true;
    }
    if (!CreateLoopedVoice(m_impl->pWindVoice, m_impl->windBufferData)) {
        m_impl->Shutdown();
        return true;
    }
    if (!CreateLoopedVoice(m_impl->pGroundRumbleVoice, m_impl->groundBufferData)) {
        m_impl->Shutdown();
        return true;
    }

    // Step 7: Create one-shot tire screech source voice
    hr = m_impl->pXAudio2->CreateSourceVoice(
        &m_impl->pTireScreechVoice,
        &wfx,
        XAUDIO2_VOICE_USEFILTER,
        XAUDIO2_DEFAULT_FREQ_RATIO,
        nullptr,
        nullptr,
        nullptr);
    if (FAILED(hr)) {
        m_impl->Shutdown();
        return true;
    }

    // Set initial volumes
    m_impl->pEngineVoice->SetVolume(0.0f);
    m_impl->pWindVoice->SetVolume(0.0f);
    m_impl->pGroundRumbleVoice->SetVolume(0.0f);
    m_impl->pTireScreechVoice->SetVolume(0.0f);

    m_impl->available = true;
    SetCockpitPerspective(m_impl->config.isCockpitPerspective);
    SetMasterVolume(m_impl->config.masterVolume);

    return true;
}

void AudioSystem::Shutdown() {
    m_impl->Shutdown();
}

bool AudioSystem::IsAvailable() const noexcept {
    return m_impl->available;
}

void AudioSystem::SetMasterVolume(f32 volume) {
    m_impl->config.masterVolume = ClampVal(volume, 0.0f, 1.0f);
    if (m_impl->pMasteringVoice) {
        m_impl->pMasteringVoice->SetVolume(m_impl->config.masterVolume);
    }
}

void AudioSystem::SetCockpitPerspective(bool inCockpit) {
    m_impl->config.isCockpitPerspective = inCockpit;
    if (!m_impl->available) {
        return;
    }

    f32 cutoff = CalculateCockpitCutoffFrequency(inCockpit);
    f32 filterParam = CutoffToXAudioFilterFreq(cutoff, AUDIO_SAMPLE_RATE);

    XAUDIO2_FILTER_PARAMETERS filter{};
    filter.Type = LowPassFilter;
    filter.Frequency = filterParam;
    filter.OneOverQ = 1.0f;

    if (m_impl->pEngineVoice) {
        m_impl->pEngineVoice->SetFilterParameters(&filter);
    }
    if (m_impl->pGroundRumbleVoice) {
        m_impl->pGroundRumbleVoice->SetFilterParameters(&filter);
    }
}

void AudioSystem::TriggerTouchdown(f32 forwardSpeed) {
    if (!m_impl->available || !m_impl->pTireScreechVoice) {
        return;
    }

    f32 intensity = ClampVal((forwardSpeed - 10.0f) / 30.0f, 0.0f, 1.0f);
    if (intensity <= 0.02f) {
        return;
    }

    m_impl->pTireScreechVoice->Stop(0);
    m_impl->pTireScreechVoice->FlushSourceBuffers();

    XAUDIO2_BUFFER buffer{};
    buffer.AudioBytes = static_cast<UINT32>(m_impl->screechBufferData.size() * sizeof(i16));
    buffer.pAudioData = reinterpret_cast<const BYTE*>(m_impl->screechBufferData.data());
    buffer.Flags = XAUDIO2_END_OF_STREAM;
    buffer.LoopCount = 0; // One-shot trigger

    m_impl->pTireScreechVoice->SubmitSourceBuffer(&buffer);
    m_impl->pTireScreechVoice->SetVolume(intensity * m_impl->config.groundVolume * m_impl->config.masterVolume);
    m_impl->pTireScreechVoice->Start(0);
}

void AudioSystem::Update(
    [[maybe_unused]] f32 dt,
    const AudioListenerState& listenerState,
    const AudioEmitterState& emitterState,
    const EngineAudioParams& engine,
    const WindAudioParams& wind,
    const GroundAudioParams& ground)
{
    if (!m_impl->available) {
        return;
    }

    // Update perspective if state changed
    if (listenerState.inCockpit != m_impl->config.isCockpitPerspective) {
        SetCockpitPerspective(listenerState.inCockpit);
    }

    // 1. Lycoming O-320 Engine Dynamics
    f32 pitchRatio = CalculateEnginePitchRatio(engine.rpm);
    f32 engineVol = CalculateEngineVolume(engine.rpm, engine.throttle) * m_impl->config.engineVolume;

    // Apply 3D spatialization when in exterior chase perspective
    f32 dopplerMultiplier = 1.0f;
    if (!listenerState.inCockpit && m_impl->config.enableSpatialAudio) {
        X3DAUDIO_LISTENER listener{};
        listener.OrientFront = { listenerState.forward.x, listenerState.forward.y, listenerState.forward.z };
        listener.OrientTop   = { listenerState.up.x, listenerState.up.y, listenerState.up.z };
        listener.Position    = { listenerState.position.x, listenerState.position.y, listenerState.position.z };
        listener.Velocity    = { listenerState.velocity.x, listenerState.velocity.y, listenerState.velocity.z };

        X3DAUDIO_EMITTER emitter{};
        emitter.OrientFront = { emitterState.forward.x, emitterState.forward.y, emitterState.forward.z };
        emitter.OrientTop   = { emitterState.up.x, emitterState.up.y, emitterState.up.z };
        emitter.Position    = { emitterState.position.x, emitterState.position.y, emitterState.position.z };
        emitter.Velocity    = { emitterState.velocity.x, emitterState.velocity.y, emitterState.velocity.z };
        emitter.InnerRadius = emitterState.minDistance;
        emitter.InnerRadiusAngle = 0.0f;
        emitter.ChannelCount = 1;
        emitter.ChannelRadius = 1.0f;
        emitter.CurveDistanceScaler = 1.0f;
        emitter.DopplerScaler = 1.0f;

        X3DAUDIO_DSP_SETTINGS dspSettings{};
        dspSettings.SrcChannelCount = 1;
        dspSettings.DstChannelCount = m_impl->channelCount;
        dspSettings.pMatrixCoefficients = m_impl->matrixCoefficients.data();

        X3DAudioCalculate(
            m_impl->x3dInstance,
            &listener,
            &emitter,
            X3DAUDIO_CALCULATE_MATRIX | X3DAUDIO_CALCULATE_DOPPLER,
            &dspSettings);

        dopplerMultiplier = ClampVal(dspSettings.DopplerFactor, 0.5f, 2.0f);
        m_impl->pEngineVoice->SetOutputMatrix(
            m_impl->pMasteringVoice,
            1,
            m_impl->channelCount,
            m_impl->matrixCoefficients.data());
    } else {
        // Cockpit perspective: balanced stereo centered on firewall
        for (u32 c = 0; c < m_impl->channelCount; ++c) {
            m_impl->matrixCoefficients[c] = 1.0f;
        }
        m_impl->pEngineVoice->SetOutputMatrix(
            m_impl->pMasteringVoice,
            1,
            m_impl->channelCount,
            m_impl->matrixCoefficients.data());
    }

    m_impl->pEngineVoice->SetFrequencyRatio(ClampVal(pitchRatio * dopplerMultiplier, 0.1f, 4.0f));
    m_impl->pEngineVoice->SetVolume(engineVol * m_impl->config.masterVolume);

    // 2. Aerodynamic Wind Rush Dynamics
    f32 windVol = CalculateWindVolume(wind.dynamicPressure) * m_impl->config.windVolume;
    f32 windCutoff = CalculateWindCutoff(wind.trueAirspeed);
    if (listenerState.inCockpit) {
        windCutoff = std::min(windCutoff, 1200.0f); // Cockpit windshield insulation
    }
    f32 windFilterParam = CutoffToXAudioFilterFreq(windCutoff, AUDIO_SAMPLE_RATE);

    XAUDIO2_FILTER_PARAMETERS windFilter{};
    windFilter.Type = LowPassFilter;
    windFilter.Frequency = windFilterParam;
    windFilter.OneOverQ = 1.0f;

    m_impl->pWindVoice->SetFilterParameters(&windFilter);
    m_impl->pWindVoice->SetVolume(windVol * m_impl->config.masterVolume);

    // 3. Ground Reactions (Runway Rumble & Touchdown Transient)
    bool anyContact = ground.noseContact || ground.leftMainContact || ground.rightMainContact;
    f32 rumbleVol = CalculateGroundRumbleVolume(ground.groundSpeed, anyContact) * m_impl->config.groundVolume;

    // Pitch shift rumble with wheel rotation speed
    f32 rumbleRatio = ClampVal(0.6f + (ground.groundSpeed / 30.0f) * 1.4f, 0.5f, 3.0f);
    m_impl->pGroundRumbleVoice->SetFrequencyRatio(rumbleRatio);
    m_impl->pGroundRumbleVoice->SetVolume(rumbleVol * m_impl->config.masterVolume);

    // Trigger touchdown screech if event flagged
    if (ground.touchdownEvent) {
        TriggerTouchdown(ground.touchdownVelocity > 2.0f ? ground.touchdownVelocity : ground.groundSpeed);
    }
}

} // namespace LokYS
