#include "LokYS/audio/AudioSystem.hpp"
#include <iostream>
#include <cmath>

#define TEST_ASSERT(cond, msg) \
    do { \
        if (!(cond)) { \
            std::cerr << "Assertion failed: " << (msg) << " (" << __FILE__ << ":" << __LINE__ << ")\n"; \
            return false; \
        } \
    } while (0)

namespace {

using namespace LokYS;

constexpr f32 EPSILON = 1e-3f;

bool FloatNear(f32 a, f32 b, f32 eps = EPSILON) {
    return std::fabs(a - b) <= eps;
}

bool TestEnginePitchRatio() {
    std::cout << "[RUNNING] TestEnginePitchRatio...\n";

    // Baseline: 1200 RPM = 1.0
    f32 ratio1200 = AudioSystem::CalculateEnginePitchRatio(1200.0f);
    TEST_ASSERT(FloatNear(ratio1200, 1.0f), "1200 RPM must produce exactly 1.0 frequency ratio");

    // Idle: 600 RPM = 0.50
    f32 ratio600 = AudioSystem::CalculateEnginePitchRatio(600.0f);
    TEST_ASSERT(FloatNear(ratio600, 0.5f), "600 RPM must produce 0.50 frequency ratio");

    // Normal cruise: 2400 RPM = 2.00
    f32 ratio2400 = AudioSystem::CalculateEnginePitchRatio(2400.0f);
    TEST_ASSERT(FloatNear(ratio2400, 2.0f), "2400 RPM must produce 2.00 frequency ratio");

    // Redline: 2700 RPM = 2.25
    f32 ratio2700 = AudioSystem::CalculateEnginePitchRatio(2700.0f);
    TEST_ASSERT(FloatNear(ratio2700, 2.25f), "2700 RPM redline must produce 2.25 frequency ratio");

    // Clamping limits
    f32 ratioExtremeLow = AudioSystem::CalculateEnginePitchRatio(50.0f);
    TEST_ASSERT(ratioExtremeLow >= (200.0f / 1200.0f) - EPSILON, "RPM clamp lower bound");

    f32 ratioExtremeHigh = AudioSystem::CalculateEnginePitchRatio(5000.0f);
    TEST_ASSERT(ratioExtremeHigh <= (3200.0f / 1200.0f) + EPSILON, "RPM clamp upper bound");

    std::cout << "[PASSED] TestEnginePitchRatio\n";
    return true;
}

bool TestEngineVolume() {
    std::cout << "[RUNNING] TestEngineVolume...\n";

    f32 idleVol = AudioSystem::CalculateEngineVolume(600.0f, 0.0f);
    f32 cruiseVolNoLoad = AudioSystem::CalculateEngineVolume(2400.0f, 0.0f);
    f32 cruiseVolFullLoad = AudioSystem::CalculateEngineVolume(2400.0f, 1.0f);

    // Volume must increase with RPM
    TEST_ASSERT(cruiseVolNoLoad > idleVol, "Cruise RPM must have higher volume than idle");

    // Throttle load boost: under load, volume must be louder than off-throttle
    TEST_ASSERT(cruiseVolFullLoad > cruiseVolNoLoad, "Full throttle load must be louder than zero throttle");

    // Bounds check
    TEST_ASSERT(idleVol >= 0.0f && idleVol <= 1.0f, "Idle volume bounds");
    TEST_ASSERT(cruiseVolFullLoad >= 0.0f && cruiseVolFullLoad <= 1.0f, "Full load volume bounds");

    std::cout << "[PASSED] TestEngineVolume\n";
    return true;
}

bool TestAerodynamicWindAcoustics() {
    std::cout << "[RUNNING] TestAerodynamicWindAcoustics...\n";

    // Stationary (q = 0)
    f32 volZero = AudioSystem::CalculateWindVolume(0.0f);
    TEST_ASSERT(FloatNear(volZero, 0.0f), "Zero dynamic pressure must produce zero wind volume");

    // Moderate dynamic pressure
    f32 volModerate = AudioSystem::CalculateWindVolume(950.0f);
    TEST_ASSERT(volModerate > 0.0f && volModerate < 1.0f, "Moderate dynamic pressure wind volume bounds");

    // Cruise dynamic pressure (~1900 Pa)
    f32 volCruise = AudioSystem::CalculateWindVolume(1900.0f);
    TEST_ASSERT(FloatNear(volCruise, 1.0f, 0.05f), "Cruise dynamic pressure should reach approximately 1.0 volume");

    // Monotonicity
    TEST_ASSERT(volModerate < volCruise, "Wind volume must increase monotonically with dynamic pressure");

    // Filter Cutoff frequency scales with true airspeed
    f32 cutoffZero = AudioSystem::CalculateWindCutoff(0.0f);
    f32 cutoffCruise = AudioSystem::CalculateWindCutoff(55.0f); // ~110 kts
    TEST_ASSERT(FloatNear(cutoffZero, 500.0f), "Stationary airspeed cutoff baseline");
    TEST_ASSERT(cutoffCruise > cutoffZero, "Airspeed cutoff must increase with speed");
    TEST_ASSERT(cutoffCruise > 3000.0f && cutoffCruise < 4000.0f, "Cruise airspeed cutoff range check");

    std::cout << "[PASSED] TestAerodynamicWindAcoustics\n";
    return true;
}

bool TestGroundReactionsAcoustics() {
    std::cout << "[RUNNING] TestGroundReactionsAcoustics...\n";

    // When airborne (no contact), rumble volume must be strictly zero
    f32 rumbleAirborne = AudioSystem::CalculateGroundRumbleVolume(60.0f, false);
    TEST_ASSERT(FloatNear(rumbleAirborne, 0.0f), "Airborne aircraft must generate zero ground rumble");

    // When on ground, rumble scales with speed
    f32 rumbleStationary = AudioSystem::CalculateGroundRumbleVolume(0.0f, true);
    f32 rumbleTaxi = AudioSystem::CalculateGroundRumbleVolume(10.0f, true);
    f32 rumbleHighSpeed = AudioSystem::CalculateGroundRumbleVolume(25.0f, true);

    TEST_ASSERT(FloatNear(rumbleStationary, 0.0f), "Stationary aircraft on ground has zero rumble");
    TEST_ASSERT(rumbleTaxi > 0.0f, "Taxi speed produces audible ground rumble");
    TEST_ASSERT(rumbleHighSpeed > rumbleTaxi, "High speed roll produces higher rumble volume");

    std::cout << "[PASSED] TestGroundReactionsAcoustics\n";
    return true;
}

bool TestCockpitPerspectiveFilter() {
    std::cout << "[RUNNING] TestCockpitPerspectiveFilter...\n";

    f32 cockpitCutoff = AudioSystem::CalculateCockpitCutoffFrequency(true);
    f32 chaseCutoff = AudioSystem::CalculateCockpitCutoffFrequency(false);

    // Interior cabin must act as a low-pass filter (muffling highs)
    TEST_ASSERT(cockpitCutoff < chaseCutoff, "Cockpit cutoff must be significantly lower than chase cutoff");
    TEST_ASSERT(FloatNear(cockpitCutoff, 1100.0f), "Cockpit low-pass cutoff frequency check");
    TEST_ASSERT(FloatNear(chaseCutoff, 8000.0f), "Chase camera open-air cutoff frequency check");

    std::cout << "[PASSED] TestCockpitPerspectiveFilter\n";
    return true;
}

bool TestAudioSystemLifecycle() {
    std::cout << "[RUNNING] TestAudioSystemLifecycle...\n";

    AudioConfig config;
    config.masterVolume = 0.9f;
    config.isCockpitPerspective = true;

    AudioSystem audioSystem;
    bool initResult = audioSystem.Initialize(config);
    TEST_ASSERT(initResult, "AudioSystem::Initialize must succeed (real hardware or safe fallback)");

    std::cout << "  [INFO] Audio device status: "
              << (audioSystem.IsAvailable() ? "OPERATIONAL" : "HEADLESS_FALLBACK") << "\n";

    AudioListenerState listener{};
    listener.inCockpit = true;
    listener.position = {0.0f, 1.0f, -0.5f};
    listener.forward = {0.0f, 0.0f, 1.0f};
    listener.up = {0.0f, 1.0f, 0.0f};

    AudioEmitterState emitter{};
    emitter.position = {0.0f, 0.0f, 1.5f}; // Engine firewall

    EngineAudioParams engine{};
    engine.rpm = 2300.0f;
    engine.throttle = 0.75f;

    WindAudioParams wind{};
    wind.dynamicPressure = 1500.0f;
    wind.trueAirspeed = 48.0f;

    GroundAudioParams ground{};
    ground.groundSpeed = 0.0f;
    ground.leftMainContact = true;
    ground.rightMainContact = true;

    // Simulate 10 frames of audio updates
    for (int frame = 0; frame < 10; ++frame) {
        audioSystem.Update(1.0f / 60.0f, listener, emitter, engine, wind, ground);
    }

    // Toggle camera perspective
    audioSystem.SetCockpitPerspective(false);
    listener.inCockpit = false;
    audioSystem.Update(1.0f / 60.0f, listener, emitter, engine, wind, ground);

    // Test touchdown screech trigger
    audioSystem.TriggerTouchdown(25.0f);

    audioSystem.SetMasterVolume(0.5f);

    audioSystem.Shutdown();
    TEST_ASSERT(!audioSystem.IsAvailable(), "AudioSystem must report unavailable after Shutdown");

    std::cout << "[PASSED] TestAudioSystemLifecycle\n";
    return true;
}

} // namespace

int main() {
    std::cout << "============================================================\n";
    std::cout << "LokYS Native Windows Flight Simulator — Spatial Audio Tests\n";
    std::cout << "============================================================\n";

    bool success = true;
    success &= TestEnginePitchRatio();
    success &= TestEngineVolume();
    success &= TestAerodynamicWindAcoustics();
    success &= TestGroundReactionsAcoustics();
    success &= TestCockpitPerspectiveFilter();
    success &= TestAudioSystemLifecycle();

    std::cout << "============================================================\n";
    if (success) {
        std::cout << "[ALL 6 SPATIAL AUDIO TEST SUITES PASSED (100%)]\n";
        return 0;
    } else {
        std::cerr << "[ONE OR MORE SPATIAL AUDIO TESTS FAILED]\n";
        return 1;
    }
}
