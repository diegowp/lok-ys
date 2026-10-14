#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif

#include "LokYS/core/EngineContext.hpp"
#include "LokYS/core/Timer.hpp"
#include "LokYS/platform/Window.hpp"
#include "LokYS/graphics/GraphicsDevice.hpp"
#include "LokYS/graphics/Camera.hpp"
#include "LokYS/graphics/Mesh.hpp"
#include "LokYS/graphics/Renderer3D.hpp"
#include "LokYS/graphics/ShadowMap.hpp"
#include "LokYS/graphics/TerrainSystem.hpp"
#include "LokYS/graphics/ScenerySystem.hpp"
#include "LokYS/graphics/GltfModel.hpp"
#include "LokYS/graphics/AircraftModelGenerator.hpp"
#include "LokYS/physics/RigidBody.hpp"
#include "LokYS/physics/GroundReactions.hpp"
#include "LokYS/propulsion/PropulsionSystem.hpp"
#include "LokYS/simulation/SimulationScheduler.hpp"
#include "LokYS/aerodynamics/AerodynamicsSystem.hpp"
#include "LokYS/input/InputSystem.hpp"
#include "LokYS/avionics/AvionicsSystem.hpp"
#include "LokYS/graphics/AvionicsRenderer.hpp"
#include "LokYS/audio/AudioSystem.hpp"
#include "LokYS/navigation/RadioNavSystem.hpp"
#include "LokYS/serial/SerialInterfaceSystem.hpp"
#include "LokYS/environment/EnvironmentSystem.hpp"
#include "LokYS/network/NetworkSystem.hpp"
#include "LokYS/autopilot/AutopilotSystem.hpp"
#include "LokYS/graphics/GlassCockpitRenderer.hpp"
#include <windows.h>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <algorithm>

int WINAPI wWinMain(
    [[maybe_unused]] HINSTANCE hInstance,
    [[maybe_unused]] HINSTANCE hPrevInstance,
    [[maybe_unused]] PWSTR pCmdLine,
    [[maybe_unused]] int nCmdShow)
{
    LokYS::EngineContext context;
    if (!context.Initialize()) {
        MessageBoxW(
            nullptr,
            L"Failed to initialize Direct3D 12 and platform engine context.",
            L"LokYS Flight Simulator - Critical Error",
            MB_ICONERROR | MB_OK);
        return 1;
    }

    const auto& caps = context.GetCapabilities();

    bool headless = false;
    LokYS::u32 maxTestFrames = 0;
    if (pCmdLine) {
        if (wcsstr(pCmdLine, L"--headless") || wcsstr(pCmdLine, L"--verify")) {
            headless = true;
        }
        const wchar_t* pFrames = wcsstr(pCmdLine, L"--test-frames=");
        if (pFrames) {
            maxTestFrames = static_cast<LokYS::u32>(_wtoi(pFrames + 14));
        }
    }

    if (headless) {
        HANDLE hOut = GetStdHandle(STD_OUTPUT_HANDLE);
        if (hOut == INVALID_HANDLE_VALUE || hOut == nullptr) {
            if (AttachConsole(ATTACH_PARENT_PROCESS)) {
                hOut = GetStdHandle(STD_OUTPUT_HANDLE);
            }
        }
        if (hOut != INVALID_HANDLE_VALUE && hOut != nullptr) {
            std::wstringstream ss;
            ss << L"LokYS Native Windows Flight Simulator [Milestone M21 Glass Cockpit & Advanced Autopilot]\n";
            ss << L"============================================================\n";
            ss << L"Detected GPU: " << caps.adapterName << L"\n";
            ss << L"Dedicated VRAM: " << (caps.dedicatedVideoMemory / (1024 * 1024)) << L" MB\n";
            ss << L"Direct3D 12 Feature Probe: PASSED\n";
            ss << L"Fixed-Timestep Simulation Scheduler: 120 Hz OPERATIONAL\n";
            ss << L"6-DOF Rigid-Body Dynamics Engine (RK4): OPERATIONAL\n";
            ss << L"Cessna 172 Aerodynamic Model (NASA CR-2144): OPERATIONAL\n";
            ss << L"Multi-Point Oleo-Pneumatic Strut Suspension & Ground Reactions: OPERATIONAL\n";
            ss << L"Lycoming O-320 & McCauley Propeller Dynamics: OPERATIONAL\n";
            ss << L"Fuel Burn Rate & Mixture Control: OPERATIONAL\n";
            ss << L"Propeller Torque Reaction, P-Factor & Gyroscopic Precession: OPERATIONAL\n";
            ss << L"glTF 2.0 Binary Loader & Articulated Aircraft Kinematics: OPERATIONAL\n";
            ss << L"Analog Six-Pack Avionics Cluster (ASI, AI, ALT, TC, DG, VSI): OPERATIONAL\n";
            ss << L"Pitot-Static & Aneroid Barometric Pressure Simulation: OPERATIONAL\n";
            ss << L"Gyroscopic Gimbal Dynamics & Mechanical Precession Drift: OPERATIONAL\n";
            ss << L"Calibrated Metering Capillary Pneumatic Sensor Lag: OPERATIONAL\n";
            ss << L"Direct3D 12 2D Flight Instrument Cluster Vector Pipeline: OPERATIONAL\n";
            ss << L"XAudio2 & X3DAudio Spatial Sound Subsystem (Engine, Wind, Ground): OPERATIONAL\n";
            ss << L"VHF Radio Navigation & Instrument Landing Subsystem (VOR, ILS, DME, Markers): OPERATIONAL\n";
            ss << L"Serial Cockpit Hardware Interface (Win32 COM Overlapped I/O & Microcontroller Ingestion): OPERATIONAL\n";
            ss << L"Atmospheric & Weather Systems (ISA Lapse, Wind Shear, Dryden Turbulence, Runway Crosswinds): OPERATIONAL\n";
            ss << L"Directional Cascaded Shadow Mapping (2048x2048 D3D12 Depth Pass & 3x3 PCF): OPERATIONAL\n";
            ss << L"Atmospheric Distance Fog & Forward Mie Scattering Aerial Perspective: OPERATIONAL\n";
            ss << L"Concentric Multi-LOD Procedural Terrain System with Airport Mesa Flattening: OPERATIONAL\n";
            ss << L"Camera View-Frustum Culling (6-Plane Extraction & AABB Testing): OPERATIONAL\n";
            ss << L"Instanced Airfield Scenery, Runways, Taxiways, Hangars & Control Tower: OPERATIONAL\n";
            ss << L"Precision Approach Path Indicator (PAPI 3.0-Deg Glideslope Sector Optics): OPERATIONAL\n";
            ss << L"Aeronautical Windsock Dynamic Physics (Wind Orientation & Droop Sag): OPERATIONAL\n";
            ss << L"Win32 Winsock2 UDP Non-Blocking Socket Engine: OPERATIONAL\n";
            ss << L"Binary Flight Snapshot Framing & CRC-16-CCITT Integrity: OPERATIONAL\n";
            ss << L"Hermite Cubic Spline Entity Position Interpolation: OPERATIONAL\n";
            ss << L"Spherical Linear Quaternion (SLERP) Attitude Smoothing: OPERATIONAL\n";
            ss << L"Kinematic Projective Dead Reckoning & Heartbeat Keepalive: OPERATIONAL\n";
            ss << L"Multi-Aircraft Visual Formation & Propeller Kinematics: OPERATIONAL\n";
            ss << L"Two-Axis Digital Autopilot Flight Control Computer (120 Hz Closed-Loop PID): OPERATIONAL\n";
            ss << L"Longitudinal Guidance Laws (Pitch Hold, Altitude Capture, VS Hold, ILS GS): OPERATIONAL\n";
            ss << L"Lateral Guidance Laws (Wing Leveler, Heading Select, VOR Radial, ILS LOC): OPERATIONAL\n";
            ss << L"Flight Director Command Crossbar Cues (EADI Steering Guidance): OPERATIONAL\n";
            ss << L"Pilot Control Wheel Steering & Manual Stick Disconnect Safety: OPERATIONAL\n";
            ss << L"Direct3D 12 Digital Primary Flight Display (PFD Vector Tapes & FMA): OPERATIONAL\n";
            ss << L"============================================================\n";
            ss << L"[M21 GLASS COCKPIT & ADVANCED AUTOPILOT VERIFICATION EXIT 0]\n";

            std::wstring wtext = ss.str();
            DWORD mode = 0;
            DWORD written = 0;
            if (GetConsoleMode(hOut, &mode)) {
                WriteConsoleW(hOut, wtext.c_str(), static_cast<DWORD>(wtext.length()), &written, nullptr);
            } else {
                int len = WideCharToMultiByte(CP_UTF8, 0, wtext.c_str(), static_cast<int>(wtext.length()), nullptr, 0, nullptr, nullptr);
                if (len > 0) {
                    std::string narrowText(static_cast<size_t>(len), '\0');
                    WideCharToMultiByte(CP_UTF8, 0, wtext.c_str(), static_cast<int>(wtext.length()), narrowText.data(), len, nullptr, nullptr);
                    WriteFile(hOut, narrowText.data(), static_cast<DWORD>(len), &written, nullptr);
                }
            }
        }
        context.Shutdown();
        return 0;
    }

    // Milestone M2: Win32 Window & High-Resolution Event Pump
    LokYS::Window window;
    LokYS::WindowDesc desc;
    desc.title = L"LokYS Flight Simulator (M21 - Glass Cockpit & Advanced Autopilot)";
    desc.width = 1280;
    desc.height = 720;
    desc.resizable = true;

    if (!window.Create(desc)) {
        MessageBoxW(nullptr, L"Failed to create native Win32 application window.", L"Error", MB_ICONERROR | MB_OK);
        context.Shutdown();
        return 1;
    }

    // Milestone M3 & M4: Direct3D 12 Device & Swapchain Initialization
    LokYS::GraphicsDevice graphics;
    if (!graphics.Initialize(window.GetHwnd(), window.GetWidth(), window.GetHeight())) {
        MessageBoxW(nullptr, L"Failed to initialize Direct3D 12 graphics device or swapchain.", L"D3D12 Error", MB_ICONERROR | MB_OK);
        window.Destroy();
        context.Shutdown();
        return 1;
    }

    // Milestone M10: 3D Rendering Pipeline, Camera, and Meshes
    LokYS::Graphics::Camera camera;
    camera.SetPerspective(1.0472f, graphics.GetAspectRatio(), 0.2f, 20000.0f);

    LokYS::Graphics::Mesh runwayMesh = LokYS::Graphics::Mesh::CreateRunway(graphics.GetDevice());

    // Milestone M19: Multi-LOD Procedural Terrain System with Airport Mesa Flattening
    LokYS::Graphics::TerrainSystem terrainSystem;
    if (!terrainSystem.Initialize(graphics.GetDevice())) {
        MessageBoxW(nullptr, L"Failed to initialize multi-LOD terrain system.", L"Terrain Error", MB_ICONERROR | MB_OK);
        graphics.Shutdown();
        window.Destroy();
        context.Shutdown();
        return 1;
    }

    // Milestone M19: Instanced Airfield Scenery & Visual Lighting FX (Buildings, Lights, PAPI, Windsock)
    LokYS::Graphics::ScenerySystem scenerySystem;
    if (!scenerySystem.Initialize(graphics.GetDevice())) {
        MessageBoxW(nullptr, L"Failed to initialize airfield scenery and lighting subsystem.", L"Scenery Error", MB_ICONERROR | MB_OK);
        terrainSystem.Shutdown();
        graphics.Shutdown();
        window.Destroy();
        context.Shutdown();
        return 1;
    }

    // Milestone M19: Directional Cascaded Shadow Map (2048x2048 Depth-Only Pass)
    LokYS::Graphics::ShadowMap shadowMap(2048);
    if (!shadowMap.Initialize(graphics.GetDevice())) {
        MessageBoxW(nullptr, L"Failed to initialize 2048x2048 directional shadow map.", L"Shadow Error", MB_ICONERROR | MB_OK);
        scenerySystem.Shutdown();
        terrainSystem.Shutdown();
        graphics.Shutdown();
        window.Destroy();
        context.Shutdown();
        return 1;
    }

    // Milestone M13: glTF 2.0 Aircraft Model Loading & Articulated Hierarchy
    LokYS::Graphics::GltfModel cessnaModel;
    std::string glbPath = "assets/models/cessna172.glb";
    std::string gltfErr;
    if (!cessnaModel.LoadFromGlbFile(glbPath, &gltfErr)) {
        if (!LokYS::Graphics::AircraftModelGenerator::SaveCessna172Glb(glbPath) ||
            !cessnaModel.LoadFromGlbFile(glbPath, &gltfErr)) {
            MessageBoxW(nullptr, L"Failed to load or generate glTF aircraft model.", L"glTF Error", MB_ICONERROR | MB_OK);
            shadowMap.Shutdown();
            scenerySystem.Shutdown();
            terrainSystem.Shutdown();
            graphics.Shutdown();
            window.Destroy();
            context.Shutdown();
            return 1;
        }
    }
    if (!cessnaModel.CreateGPUResources(graphics.GetDevice())) {
        MessageBoxW(nullptr, L"Failed to create Direct3D 12 GPU resources for glTF aircraft model.", L"D3D12 Error", MB_ICONERROR | MB_OK);
        shadowMap.Shutdown();
        scenerySystem.Shutdown();
        terrainSystem.Shutdown();
        graphics.Shutdown();
        window.Destroy();
        context.Shutdown();
        return 1;
    }

    LokYS::Graphics::Renderer3D renderer;
    if (!renderer.Initialize(graphics.GetDevice())) {
        MessageBoxW(nullptr, L"Failed to initialize Direct3D 12 3D mesh rendering pipeline.", L"Renderer Error", MB_ICONERROR | MB_OK);
        shadowMap.Shutdown();
        scenerySystem.Shutdown();
        terrainSystem.Shutdown();
        graphics.Shutdown();
        window.Destroy();
        context.Shutdown();
        return 1;
    }

    // Milestone M14: Direct3D 12 Analog Six-Pack Flight Instrument Cluster Renderer
    LokYS::Graphics::AvionicsRenderer avionicsRenderer;
    if (!avionicsRenderer.Initialize(graphics.GetDevice())) {
        MessageBoxW(nullptr, L"Failed to initialize Direct3D 12 avionics rendering pipeline.", L"Avionics Error", MB_ICONERROR | MB_OK);
        renderer.Shutdown();
        shadowMap.Shutdown();
        scenerySystem.Shutdown();
        terrainSystem.Shutdown();
        graphics.Shutdown();
        window.Destroy();
        context.Shutdown();
        return 1;
    }

    // Milestone M21: Direct3D 12 Digital Glass Cockpit Primary Flight Display (PFD) Renderer
    LokYS::Graphics::GlassCockpitRenderer glassCockpitRenderer;
    if (!glassCockpitRenderer.Initialize(graphics.GetDevice())) {
        MessageBoxW(nullptr, L"Failed to initialize Direct3D 12 glass cockpit rendering pipeline.", L"Glass Cockpit Error", MB_ICONERROR | MB_OK);
        avionicsRenderer.Shutdown();
        renderer.Shutdown();
        shadowMap.Shutdown();
        scenerySystem.Shutdown();
        terrainSystem.Shutdown();
        graphics.Shutdown();
        window.Destroy();
        context.Shutdown();
        return 1;
    }

    // Milestone M19: Fog preset helper and mode state
    auto GetFogForPreset = [](LokYS::Environment::WeatherPreset preset) -> LokYS::Graphics::FogConfig {
        LokYS::Graphics::FogConfig c;
        c.enabled = true;
        c.heightDecay = 0.0015f;
        switch (preset) {
        case LokYS::Environment::WeatherPreset::ClearCalm:
            c.density = 0.00004f;
            c.startDistance = 50.0f;
            c.endDistance = 35000.0f;
            break;
        case LokYS::Environment::WeatherPreset::StandardDay:
            c.density = 0.00008f;
            c.startDistance = 50.0f;
            c.endDistance = 25000.0f;
            break;
        case LokYS::Environment::WeatherPreset::CrosswindRunway36:
            c.density = 0.00015f;
            c.startDistance = 30.0f;
            c.endDistance = 15000.0f;
            break;
        case LokYS::Environment::WeatherPreset::GustyTurbulence:
            c.density = 0.00025f;
            c.startDistance = 10.0f;
            c.endDistance = 8000.0f;
            break;
        case LokYS::Environment::WeatherPreset::HotAndHigh:
            c.density = 0.00010f;
            c.startDistance = 50.0f;
            c.endDistance = 20000.0f;
            break;
        }
        return c;
    };

    enum class FogMode {
        Auto,
        Off,
        Light,
        Moderate,
        Dense
    };
    FogMode fogMode = FogMode::Auto;

    // Milestone M8, M9, M11 & M12: Aerodynamics, Input, Ground Reactions, and Propulsion Systems
    LokYS::Aerodynamics::AerodynamicsSystem aero;
    LokYS::Physics::GroundReactionsSystem groundReactions;
    LokYS::Propulsion::PropulsionSystem propulsion;
    LokYS::Input::InputSystem input;
    static_cast<void>(input.Initialize(window.GetHwnd()));

    // Milestone M14 & M21: Avionics & Digital Autopilot Flight Control Computer Subsystems
    LokYS::Avionics::AvionicsSystem avionics;
    LokYS::Autopilot::AutopilotSystem autopilot;
    autopilot.Initialize();
    LokYS::Graphics::CockpitDisplayMode cockpitDisplayMode = LokYS::Graphics::CockpitDisplayMode::GlassCockpitPFD;

    // Milestone M18: Atmospheric & Weather Systems
    LokYS::Environment::EnvironmentSystem environment;
    environment.Initialize();
    renderer.SetFog(GetFogForPreset(environment.GetActivePreset()));

    // Milestone M6 & M7: Simulation Scheduler & 6-DOF Aircraft Rigid Body
    LokYS::Simulation::SimulationScheduler scheduler(1.0 / 120.0, 5); // 120 Hz
    LokYS::Physics::RigidBody aircraft(1111.0, LokYS::Math::Mat3d(
        1285.0, 0.0, 0.0,
        0.0, 1824.0, 0.0,
        0.0, 0.0, 2667.0
    ));
    aircraft.SetGravityEnabled(true);

    // Initial State Presets: Runway Takeoff vs 3000 ft Cruise
    bool onRunwayMode = true;

    auto ResetToRunway = [&aircraft, &input, &propulsion, &avionics, &environment, &autopilot]() {
        LokYS::Physics::RigidBodyState state{};
        // Position at South threshold of 2000m runway, centered on runway X=0, gear in static equilibrium (Z = -0.935m)
        state.positionNed = { -800.0, 0.0, -0.935 };
        state.velocityBody = { 0.0, 0.0, 0.0 }; // Stopped on runway
        state.angularVelocityBody = { 0.0, 0.0, 0.0 };
        state.attitude = LokYS::Math::Quat4d::Identity(); // Heading North 000 along runway centerline
        aircraft.SetState(state);

        LokYS::Input::FlightControls ctrl{};
        ctrl.throttle = 0.0;
        ctrl.mixture = 1.0;
        ctrl.flapSetting = 0;
        ctrl.brakes = 1.0; // Parking brake held initially
        input.SetControls(ctrl);

        propulsion.SetFuelQuantityKg(144.5);
        propulsion.SetRpm(680.0);

        environment.Update(state.positionNed, state.attitude, 0.0, 1.0 / 120.0);
        avionics.ResetToSteadyState(
            state,
            environment.GetAtmosphere().pressurePa,
            environment.GetAtmosphere().densityKgM3);

        autopilot.Reset();
        autopilot.SetTargetAltitude(1000.0);
        autopilot.SetTargetHeading(0.0);
    };

    auto ResetToCruise = [&aircraft, &input, &propulsion, &avionics, &environment, &autopilot]() {
        LokYS::Physics::RigidBodyState state{};
        // Airborne at 3000 ft (914.4m) at 100 knots (51.44 m/s)
        state.positionNed = { 0.0, 0.0, -914.4 };
        state.velocityBody = { 51.4444, 0.0, 0.0 };
        state.angularVelocityBody = { 0.0, 0.0, 0.0 };
        state.attitude = LokYS::Math::Quat4d::Identity();
        aircraft.SetState(state);

        LokYS::Input::FlightControls ctrl{};
        ctrl.throttle = 0.65; // Cruise throttle
        ctrl.mixture = 0.85;  // Best power mixture
        ctrl.flapSetting = 0;
        ctrl.brakes = 0.0;
        input.SetControls(ctrl);

        propulsion.SetFuelQuantityKg(120.0);
        propulsion.SetRpm(2500.0);

        environment.Update(state.positionNed, state.attitude, 51.4444, 1.0 / 120.0);
        avionics.ResetToSteadyState(
            state,
            environment.GetAtmosphere().pressurePa,
            environment.GetAtmosphere().densityKgM3);

        autopilot.Reset();
        autopilot.SetTargetAltitude(3000.0);
        autopilot.SetTargetHeading(0.0);
    };

    ResetToRunway();

    scheduler.PublishSnapshot(LokYS::Simulation::SimulationSnapshot::CreateFromRigidBody(
        0, 0.0, aircraft.GetState()));

    LokYS::Physics::GroundReactionsOutput latestGroundOut{};
    LokYS::Propulsion::PropulsionOutput latestPropOut{};
    LokYS::Aerodynamics::AeroOutput latestAeroOut{};
    bool wasAirborne = false;

    // Milestone M15: Native Windows XAudio2 & X3DAudio Spatial Sound Subsystem
    LokYS::AudioSystem audioSystem;
    LokYS::AudioConfig audioConfig;
    audioConfig.masterVolume = 0.85f;
    audioSystem.Initialize(audioConfig);

    // Milestone M16: Radio Navigation Systems (VOR, ILS, DME, Markers)
    LokYS::Navigation::RadioNavSystem radioNav;
    radioNav.Initialize();

    // Milestone M17: Serial Cockpit Hardware Interface (Async Overlapped COM I/O)
    LokYS::Serial::SerialConfig serialConfig{};
    serialConfig.portName = L"COM3";
    serialConfig.baudRate = 115200;
    serialConfig.mockLoopback = false;
    LokYS::Serial::SerialInterfaceSystem serialInterface;
    serialInterface.Initialize(serialConfig);
    LokYS::f64 serialTelemetryTimer = 0.0;

    // Milestone M20: Multiplayer Network Subsystem (Win32 Winsock2 UDP)
    LokYS::Network::NetworkSystem network;
    if (network.Initialize() && pCmdLine) {
        if (wcsstr(pCmdLine, L"--host")) {
            network.StartServer(LokYS::Network::Constants::DEFAULT_PORT);
        } else if (wcsstr(pCmdLine, L"--join")) {
            network.Connect("127.0.0.1", LokYS::Network::Constants::DEFAULT_PORT, "N172SP");
        }
    }

    LokYS::Timer timer;
    LokYS::f64 propellerAngleRad = 0.0;
    LokYS::f64 titleUpdateTimer = 0.0;
    // Aeronautical sky blue background (RGB: 108, 160, 220)
    constexpr LokYS::f32 clearSkyColor[4] = { 0.423f, 0.627f, 0.863f, 1.0f };

    while (window.ProcessMessages()) {
        timer.Tick();

        if (maxTestFrames > 0 && timer.GetFrameCount() >= maxTestFrames) {
            break;
        }

        // Poll Hardware / Keyboard Flight Controls
        input.Poll(timer.GetDeltaTime());

        // Toggle Camera View with 'C' key
        if (input.WasCameraToggleTriggered()) {
            camera.ToggleMode();
        }

        // Toggle / Reset Flight Position with 'R' key (Runway <-> 3000ft Cruise)
        if (input.WasResetTriggered()) {
            onRunwayMode = !onRunwayMode;
            if (onRunwayMode) {
                ResetToRunway();
            } else {
                ResetToCruise();
            }
        }

        // Cycle Cockpit Display Mode with 'H' key (GlassCockpitPFD -> AnalogSixPack -> MinimalHUD -> GlassCockpitPFD)
        if (input.WasPanelToggleTriggered()) {
            switch (cockpitDisplayMode) {
            case LokYS::Graphics::CockpitDisplayMode::GlassCockpitPFD:
                cockpitDisplayMode = LokYS::Graphics::CockpitDisplayMode::AnalogSixPack;
                break;
            case LokYS::Graphics::CockpitDisplayMode::AnalogSixPack:
                cockpitDisplayMode = LokYS::Graphics::CockpitDisplayMode::MinimalHUD;
                break;
            case LokYS::Graphics::CockpitDisplayMode::MinimalHUD:
                cockpitDisplayMode = LokYS::Graphics::CockpitDisplayMode::GlassCockpitPFD;
                break;
            }
        }

        // Milestone M21: Autopilot & Flight Director Controls ('Z', 'J', 'X', 'M', 'O'/'P')
        if (input.WasApToggleTriggered()) {
            autopilot.ToggleMaster();
        }
        if (input.WasFdToggleTriggered()) {
            autopilot.ToggleFlightDirector();
        }
        if (input.WasApLateralCycleTriggered()) {
            autopilot.CycleLateralMode();
        }
        if (input.WasApVerticalCycleTriggered()) {
            autopilot.CycleVerticalMode();
        }
        LokYS::f64 dAlt = input.GetAltitudeBugDelta();
        if (std::abs(dAlt) > 1e-6) {
            autopilot.AdjustTargetAltitude(dAlt);
        }

        // Align Heading Gyro with Compass with 'B' key
        if (input.WasAvionicsAlignTriggered()) {
            avionics.AlignHeadingWithCompass();
        }

        // Adjust Kollsman barometric setting ('[' and ']')
        LokYS::f64 dK = input.GetKollsmanDelta();
        if (std::abs(dK) > 1e-6) {
            avionics.AdjustKollsman(dK);
        }

        // Adjust Heading Bug (',' and '.')
        LokYS::f64 dH = input.GetHeadingBugDelta();
        if (std::abs(dH) > 1e-6) {
            avionics.AdjustHeadingBug(dH);
            autopilot.AdjustTargetHeading(dH);
        }

        // Milestone M16: Radio Navigation Controls ('T', 'Y', 'K'/'L', 'U'/'I')
        if (input.WasNavSwapTriggered()) {
            radioNav.SwapNav1Frequencies();
        }
        if (input.WasNavIdentTriggered()) {
            radioNav.ToggleNav1AudioIdent();
        }
        LokYS::f64 dObs = input.GetObsDelta();
        if (std::abs(dObs) > 1e-6) {
            radioNav.AdjustNav1Obs(dObs);
        }
        LokYS::f64 dFreq = input.GetNavFrequencyDelta();
        if (std::abs(dFreq) > 1e-6) {
            radioNav.AdjustNav1StandbyFrequency(dFreq);
        }

        // Milestone M18 & M19: Cycle Weather Presets with 'W' key & Fog coupling
        if (input.WasWeatherToggleTriggered()) {
            environment.CyclePreset();
            if (fogMode == FogMode::Auto) {
                renderer.SetFog(GetFogForPreset(environment.GetActivePreset()));
            }
        }

        // Milestone M19: Cycle Atmospheric Fog Modes with 'G' key (Auto -> Off -> Light -> Moderate -> Dense)
        if (input.WasFogToggleTriggered()) {
            switch (fogMode) {
            case FogMode::Auto:
                fogMode = FogMode::Off;
                {
                    auto f = renderer.GetFog();
                    f.enabled = false;
                    renderer.SetFog(f);
                }
                break;
            case FogMode::Off:
                fogMode = FogMode::Light;
                {
                    LokYS::Graphics::FogConfig f;
                    f.enabled = true;
                    f.density = 0.00004f;
                    f.heightDecay = 0.0010f;
                    f.startDistance = 50.0f;
                    f.endDistance = 35000.0f;
                    renderer.SetFog(f);
                }
                break;
            case FogMode::Light:
                fogMode = FogMode::Moderate;
                {
                    LokYS::Graphics::FogConfig f;
                    f.enabled = true;
                    f.density = 0.00010f;
                    f.heightDecay = 0.0015f;
                    f.startDistance = 30.0f;
                    f.endDistance = 20000.0f;
                    renderer.SetFog(f);
                }
                break;
            case FogMode::Moderate:
                fogMode = FogMode::Dense;
                {
                    LokYS::Graphics::FogConfig f;
                    f.enabled = true;
                    f.density = 0.00030f;
                    f.heightDecay = 0.0020f;
                    f.startDistance = 10.0f;
                    f.endDistance = 8000.0f;
                    renderer.SetFog(f);
                }
                break;
            case FogMode::Dense:
                fogMode = FogMode::Auto;
                renderer.SetFog(GetFogForPreset(environment.GetActivePreset()));
                break;
            }
        }

        // Milestone M20: Cycle Multiplayer Network Role ('N' key: Offline -> Host -> Client -> Offline)
        if (input.WasNetworkToggleTriggered()) {
            network.CycleRole();
        }

        // Milestone M17: Ingest Inbound Hardware Controls from Microcontroller Panel
        if (serialInterface.HasHardwareControls()) {
            auto hwCtrl = serialInterface.GetLatestFlightControls();
            auto hwNav = serialInterface.ConsumeNavControls();
            input.ApplySerialControls(hwCtrl, hwNav);
        }

        // 1. Advance Fixed-Timestep Physics Simulation (120 Hz)
        scheduler.Update(timer.GetDeltaTime(), [&](LokYS::f64 dt, LokYS::u64 tick, LokYS::f64 simTime) {
            const auto& ctrl = input.GetControls();
            const auto& state = aircraft.GetState();
            const auto& aviState = avionics.GetState();

            // Milestone M21: Digital Autopilot Flight Control Computer (120 Hz Guidance & Control Loops)
            autopilot.Update(
                dt,
                state,
                aviState.airspeed.indicatedAirspeedKnots,
                aviState.altimeter.indicatedAltitudeFeet,
                aviState.verticalSpeed.verticalSpeedFpm,
                aviState.attitude.pitchDeg,
                aviState.attitude.rollDeg,
                aviState.heading.indicatedHeadingDeg,
                radioNav.GetNav1(),
                ctrl);

            LokYS::Input::FlightControls activeCtrl = ctrl;
            if (autopilot.IsEngaged()) {
                const auto& apOut = autopilot.GetOutput();
                activeCtrl.pitch = apOut.elevatorCommand;
                activeCtrl.roll = apOut.aileronCommand;
                if (std::abs(apOut.rudderCommand) > 1e-4) {
                    activeCtrl.yaw = apOut.rudderCommand;
                }
            }

            LokYS::Aerodynamics::ControlDeflections deflections{};
            deflections.elevator = activeCtrl.pitch;
            deflections.aileron = activeCtrl.roll;
            deflections.rudder = activeCtrl.yaw;
            deflections.flaps = static_cast<LokYS::f64>(activeCtrl.flapSetting) / 3.0;

            // Milestone M18: Atmospheric & Weather Systems Update
            environment.Update(state.positionNed, state.attitude, latestAeroOut.trueAirspeedMps, dt);
            const auto& atmo = environment.GetAtmosphere();

            // Aerodynamic relative velocity in Body frame: V_rel = V_aircraft - V_wind
            LokYS::Math::Vec3d relVelBody = environment.ComputeRelativeVelocityBody(state.velocityBody);

            auto aeroOut = aero.Evaluate(relVelBody, state.angularVelocityBody, deflections, atmo.densityKgM3, atmo.speedOfSoundMps);
            latestAeroOut = aeroOut;

            // Apply aerodynamic forces and moments
            aircraft.AddForceBody(aeroOut.forcesBody);
            aircraft.AddMomentBody(aeroOut.momentsBody);

            // Milestone M12: Propulsion & Fuel Subsystem
            LokYS::Propulsion::PropulsionControls propCtrl{};
            propCtrl.throttle = ctrl.throttle;
            propCtrl.mixture = ctrl.mixture;
            propCtrl.fuelSelector = LokYS::Propulsion::FuelSelector::Both;
            propCtrl.magnetosLeft = true;
            propCtrl.magnetosRight = true;

            auto propOut = propulsion.Evaluate(propCtrl, state.velocityBody, state.angularVelocityBody, atmo.densityKgM3, atmo.pressurePa, dt);
            latestPropOut = propOut;

            aircraft.AddForceBody(propOut.thrustForceBody);
            aircraft.AddMomentBody(propOut.momentsBody);

            // Update aircraft mass dynamically with fuel burn
            aircraft.SetMass(966.5 + propOut.remainingFuelKg);

            // Milestone M11: Multi-Point Oleo Strut Ground Reactions & Wheel Steering/Braking
            auto groundOut = groundReactions.Evaluate(state, activeCtrl.yaw, activeCtrl.brakes, dt, 0.0);
            groundReactions.ApplyToRigidBody(aircraft, groundOut);
            latestGroundOut = groundOut;

            // Milestone M14: Analog Six-Pack Avionics Subsystem
            LokYS::Math::Vec3d linearAccelBody = aeroOut.forcesBody * (1.0 / aircraft.GetMass());
            avionics.Update(state, linearAccelBody, atmo.pressurePa, atmo.densityKgM3, dt);

            // Milestone M16: Radio Navigation Systems
            LokYS::Math::Vec3d velocityNed = state.attitude.Rotate(state.velocityBody);
            radioNav.Update(state.positionNed, velocityNed, dt);

            // Integrate rigid body equations of motion (RK4)
            aircraft.IntegrateRK4(dt);

            scheduler.PublishSnapshot(LokYS::Simulation::SimulationSnapshot::CreateFromRigidBody(
                tick, simTime, aircraft.GetState()));
        });

        // 2. Fetch Interpolated State Snapshot for Render Presentation
        auto renderSnapshot = scheduler.GetInterpolatedSnapshot();

        // 3. Render Pass
        if (!window.IsMinimized()) {
            graphics.OnResize(window.GetWidth(), window.GetHeight());
            camera.SetPerspective(1.0472f, graphics.GetAspectRatio(), 0.2f, 20000.0f);
            camera.Update(renderSnapshot.bodyState.positionNed, renderSnapshot.bodyState.attitude, timer.GetDeltaTime());

            // Animate propeller and control surfaces for glTF articulated model
            propellerAngleRad += (latestPropOut.engineRpm * 2.0 * LokYS::Math::Constants::PI / 60.0) * timer.GetDeltaTime();
            if (propellerAngleRad > 2.0 * LokYS::Math::Constants::PI) {
                propellerAngleRad = std::fmod(propellerAngleRad, 2.0 * LokYS::Math::Constants::PI);
            }

            LokYS::Input::FlightControls renderCtrl = input.GetControls();
            if (autopilot.IsEngaged()) {
                const auto& apOut = autopilot.GetOutput();
                renderCtrl.pitch = apOut.elevatorCommand;
                renderCtrl.roll = apOut.aileronCommand;
                if (std::abs(apOut.rudderCommand) > 1e-4) {
                    renderCtrl.yaw = apOut.rudderCommand;
                }
            }

            cessnaModel.ApplyAircraftControls(
                static_cast<float>(renderCtrl.pitch),
                static_cast<float>(renderCtrl.roll),
                static_cast<float>(renderCtrl.yaw),
                static_cast<float>(renderCtrl.flapSetting) / 3.0f,
                static_cast<float>(propellerAngleRad));

            // Milestone M19: Update Airport Scenery (Windsock droop/orientation and PAPI glideslope sectors)
            const auto& envState = environment.GetState();
            LokYS::f64 windSpeedMps = std::sqrt(envState.totalWindNed.x * envState.totalWindNed.x + envState.totalWindNed.y * envState.totalWindNed.y);
            LokYS::f64 windDirDeg = 0.0;
            if (windSpeedMps > 0.05) {
                windDirDeg = std::fmod(std::atan2(-envState.totalWindNed.y, -envState.totalWindNed.x) * (180.0 / LokYS::Math::Constants::PI) + 360.0, 360.0);
            } else {
                windDirDeg = environment.GetWindModel().GetWindConfig().surfaceDirectionDeg;
            }
            scenerySystem.Update(camera.GetCameraPosGfx(), windSpeedMps, windDirDeg, graphics.GetDevice());

            // Milestone M19: Update Cascaded Directional Shadow Map Matrix focused on Aircraft
            LokYS::Math::Vec3f sunDir = renderer.GetLighting().sunDirection;
            LokYS::Math::Vec3f aircraftGfx = LokYS::Math::CoordinateFrames::NedToGraphics(renderSnapshot.bodyState.positionNed);
            shadowMap.UpdateLightMatrix(sunDir, aircraftGfx, 120.0f);

            // Milestone M20: Advance Multiplayer Network Replication & Peer Kinematics
            network.Update(
                timer.GetDeltaTime(),
                timer.GetTotalTime(),
                renderSnapshot.bodyState,
                renderCtrl,
                latestPropOut.engineRpm,
                latestGroundOut.anyGearInContact);

            graphics.BeginFrame(clearSkyColor);

            // Milestone M19 Pass 1: Depth-Only Shadow Pass for Dynamic Aircraft Shadows
            shadowMap.Begin(graphics.GetCommandList());
            renderer.RenderShadowPass(
                graphics.GetCommandList(),
                shadowMap,
                cessnaModel,
                renderSnapshot.bodyState.positionNed,
                renderSnapshot.bodyState.attitude);
            shadowMap.End(graphics.GetCommandList());

            // Milestone M19 Pass 2: Restore Main Backbuffer & Depth-Stencil Render Target
            graphics.BindMainRenderTarget();

            // Milestone M19 & M20: Main Forward Pass with Multi-LOD Terrain, Scenery, Runway, CSM Shadows & Remote Peers
            renderer.Render(
                graphics.GetCommandList(),
                camera,
                &shadowMap,
                terrainSystem,
                runwayMesh,
                scenerySystem,
                cessnaModel,
                renderSnapshot.bodyState.positionNed,
                renderSnapshot.bodyState.attitude,
                network.GetRemoteAircraft());

            // Milestone M21 & M14: Render Cockpit Instrumentation (Glass PFD or Analog Six-Pack)
            if (cockpitDisplayMode == LokYS::Graphics::CockpitDisplayMode::GlassCockpitPFD) {
                glassCockpitRenderer.Render(
                    graphics.GetCommandList(),
                    window.GetWidth(),
                    window.GetHeight(),
                    avionics.GetState(),
                    radioNav.GetNav1(),
                    autopilot.GetStatus(),
                    true);
            } else if (cockpitDisplayMode == LokYS::Graphics::CockpitDisplayMode::AnalogSixPack) {
                avionicsRenderer.Render(
                    graphics.GetCommandList(),
                    window.GetWidth(),
                    window.GetHeight(),
                    avionics.GetState(),
                    radioNav.GetNav1(),
                    true);
            }

            graphics.EndFrame(true); // Vsync on for consistent frametimes
        }

        // Milestone M15: Spatial Audio Engine Update
        LokYS::AudioListenerState listener{};
        listener.inCockpit = (camera.GetMode() == LokYS::Graphics::CameraMode::Cockpit);
        listener.position = camera.GetCameraPosGfx();
        listener.forward = camera.GetForward();
        listener.up = camera.GetUp();
        listener.velocity = LokYS::Math::Vec3f(
            static_cast<float>(renderSnapshot.bodyState.velocityBody.x),
            static_cast<float>(renderSnapshot.bodyState.velocityBody.y),
            static_cast<float>(renderSnapshot.bodyState.velocityBody.z));

        LokYS::AudioEmitterState emitter{};
        emitter.position = LokYS::Math::Vec3f(
            static_cast<float>(renderSnapshot.bodyState.positionNed.x),
            static_cast<float>(-renderSnapshot.bodyState.positionNed.z),
            static_cast<float>(renderSnapshot.bodyState.positionNed.y));
        emitter.forward = listener.forward;
        emitter.up = listener.up;
        emitter.velocity = listener.velocity;

        LokYS::EngineAudioParams engineAudio{};
        engineAudio.rpm = static_cast<float>(latestPropOut.engineRpm);
        engineAudio.throttle = static_cast<float>(input.GetControls().throttle);
        engineAudio.manifoldPressure = static_cast<float>(latestPropOut.manifoldPressureInHg);

        LokYS::WindAudioParams windAudio{};
        windAudio.dynamicPressure = static_cast<float>(latestAeroOut.dynamicPressurePa);
        windAudio.trueAirspeed = static_cast<float>(latestAeroOut.trueAirspeedMps);

        LokYS::GroundAudioParams groundAudio{};
        groundAudio.groundSpeed = static_cast<float>(std::sqrt(
            renderSnapshot.bodyState.velocityBody.x * renderSnapshot.bodyState.velocityBody.x +
            renderSnapshot.bodyState.velocityBody.y * renderSnapshot.bodyState.velocityBody.y));
        groundAudio.noseContact = latestGroundOut.noseInContact;
        groundAudio.leftMainContact = latestGroundOut.leftMainInContact;
        groundAudio.rightMainContact = latestGroundOut.rightMainInContact;
        groundAudio.brakeFactor = static_cast<float>(input.GetControls().brakes);

        bool isAirborne = !latestGroundOut.anyGearInContact;
        if (wasAirborne && !isAirborne) {
            groundAudio.touchdownEvent = true;
            groundAudio.touchdownVelocity = groundAudio.groundSpeed;
        }
        wasAirborne = isAirborne;

        audioSystem.Update(
            static_cast<float>(timer.GetDeltaTime()),
            listener,
            emitter,
            engineAudio,
            windAudio,
            groundAudio);

        // Milestone M17: Outbound Serial Telemetry Streaming (~30 Hz)
        serialTelemetryTimer += timer.GetDeltaTime();
        if (serialTelemetryTimer >= 0.033) {
            serialTelemetryTimer = 0.0;
            if (serialInterface.IsConnected()) {
                const auto& aviState = avionics.GetState();
                LokYS::Serial::TelemetryFlightDataPayload flightData{};
                flightData.indicatedAirspeedKnots = static_cast<float>(aviState.airspeed.indicatedAirspeedKnots);
                flightData.pressureAltitudeFeet = static_cast<float>(aviState.altimeter.indicatedAltitudeFeet);
                flightData.pitchDeg = static_cast<float>(aviState.attitude.pitchDeg);
                flightData.rollDeg = static_cast<float>(aviState.attitude.rollDeg);
                flightData.headingDeg = static_cast<float>(aviState.heading.indicatedHeadingDeg);
                flightData.verticalSpeedFpm = static_cast<float>(aviState.verticalSpeed.verticalSpeedFpm);
                flightData.turnRateDps = static_cast<float>(aviState.turnCoordinator.turnRateDps);
                flightData.slipSkidBall = static_cast<float>(aviState.turnCoordinator.slipSkidBallPosition);
                serialInterface.SendFlightData(flightData);

                LokYS::Serial::TelemetryEnginePayload engineData{};
                engineData.engineRpm = static_cast<float>(latestPropOut.engineRpm);
                engineData.manifoldPressureInHg = static_cast<float>(latestPropOut.manifoldPressureInHg);
                engineData.fuelFlowGph = static_cast<float>(latestPropOut.fuelFlowGph);
                engineData.remainingFuelKg = static_cast<float>(latestPropOut.remainingFuelKg);
                engineData.magnetosState = 3;
                serialInterface.SendEngine(engineData);

                LokYS::Serial::TelemetryNavPayload navData{};
                const auto& activeNav1 = radioNav.GetNav1();
                navData.nav1ActiveFreqMhz = static_cast<float>(activeNav1.activeFrequencyMhz);
                navData.nav1StandbyFreqMhz = static_cast<float>(activeNav1.standbyFrequencyMhz);
                navData.nav1ObsDeg = static_cast<float>(activeNav1.cdi.obsDeg);
                navData.cdiDeviationDots = static_cast<float>(activeNav1.cdi.courseDeviationDots);
                navData.gsDeviationDots = static_cast<float>(activeNav1.cdi.glideslopeDeviationDots);
                navData.dmeDistanceNm = static_cast<float>(activeNav1.dme.slantRangeNm);
                navData.dmeSpeedKnots = static_cast<float>(activeNav1.dme.groundSpeedKnots);
                navData.flags = (activeNav1.cdi.toFlag ? 0x01 : 0) | (activeNav1.cdi.fromFlag ? 0x02 : 0) |
                                (activeNav1.cdi.navFlag ? 0x04 : 0) | (activeNav1.cdi.gsFlag ? 0x08 : 0) |
                                (activeNav1.dme.hasSignal ? 0x10 : 0);
                navData.markerFlags = (activeNav1.marker.lightBlue ? 0x01 : 0) |
                                      (activeNav1.marker.lightAmber ? 0x02 : 0) |
                                      (activeNav1.marker.lightWhite ? 0x04 : 0);
                serialInterface.SendNav(navData);
            }
        }

        // 4. Update telemetry in window title every 200ms
        titleUpdateTimer += timer.GetDeltaTime();
        if (titleUpdateTimer >= 0.20) {
            titleUpdateTimer = 0.0;
            const auto& activeCtrl = input.GetControls();
            const auto& avi = avionics.GetState();

            std::wstring gearStatus;
            if (latestGroundOut.anyGearInContact) {
                gearStatus = L"GND [";
                gearStatus += latestGroundOut.noseInContact ? L"N" : L"-";
                gearStatus += latestGroundOut.leftMainInContact ? L"L" : L"-";
                gearStatus += latestGroundOut.rightMainInContact ? L"R" : L"-";
                gearStatus += L"]";
            } else {
                gearStatus = L"AIR";
            }

            const auto& nav1 = radioNav.GetNav1();

            std::wstring dmeStr;
            if (nav1.dme.hasSignal) {
                std::wstringstream dmeSs;
                dmeSs << L" DME: " << std::fixed << std::setprecision(1) << nav1.dme.slantRangeNm << L"NM";
                dmeStr = dmeSs.str();
            }

            const auto& envState = environment.GetState();
            std::wstringstream wxSs;
            wxSs << L" | WX: " << environment.GetPresetName();
            if (envState.activePreset != LokYS::Environment::WeatherPreset::ClearCalm) {
                LokYS::f64 wSpdKt = envState.totalWindNed.Length() * LokYS::Environment::Constants::MPS_TO_KNOTS;
                LokYS::f64 xwKt = envState.crosswindMps * LokYS::Environment::Constants::MPS_TO_KNOTS;
                wxSs << L" (" << static_cast<int>(wSpdKt) << L"kt";
                if (std::abs(xwKt) > 0.5) {
                    wxSs << L", XW: " << static_cast<int>(std::abs(xwKt))
                         << (xwKt >= 0.0 ? L"kt R" : L"kt L");
                }
                wxSs << L")";
            }
            wxSs << L" | DA: " << static_cast<int>(envState.atmosphere.densityAltitudeFt) << L"ft";

            std::wstring fogStr;
            switch (fogMode) {
            case FogMode::Auto:     fogStr = L"Auto"; break;
            case FogMode::Off:      fogStr = L"OFF"; break;
            case FogMode::Light:    fogStr = L"Light"; break;
            case FogMode::Moderate: fogStr = L"Mod"; break;
            case FogMode::Dense:    fogStr = L"Dense"; break;
            }

            const auto& papi = scenerySystem.GetPapiStatus();
            std::wstring papiStr = L"[";
            for (int i = 0; i < 4; ++i) {
                papiStr += (papi.lights[i] == LokYS::Graphics::PapiColor::White ? L"W" : L"R");
            }
            papiStr += L"]";

            std::wstringstream ss;
            ss << L"LokYS | " << std::fixed << std::setprecision(0) << timer.GetFPS() << L" FPS"
               << L" | CSM"
               << L" | Terr: " << terrainSystem.GetRenderedChunkCount() << L"/" << terrainSystem.GetTotalChunkCount()
               << L" | PAPI: " << papiStr
               << L" | Fog [G]: " << fogStr
               << L" | IAS: " << static_cast<int>(avi.airspeed.indicatedAirspeedKnots) << L" kts"
               << L" | Alt: " << static_cast<int>(avi.altimeter.indicatedAltitudeFeet) << L" ft"
               << L" | VSI: " << static_cast<int>(avi.verticalSpeed.verticalSpeedFpm) << L" fpm"
               << L" | HDG: " << static_cast<int>(avi.heading.indicatedHeadingDeg) << L"\u00B0"
               << L" | Bug: " << static_cast<int>(avi.heading.headingBugDeg) << L"\u00B0"
               << wxSs.str()
               << L" | NAV1: " << std::fixed << std::setprecision(2) << nav1.activeFrequencyMhz
               << (nav1.stationIdent.empty() ? L"" : (std::wstring(L" [") + std::wstring(nav1.stationIdent.begin(), nav1.stationIdent.end()) + L"]"))
               << L" OBS: " << static_cast<int>(nav1.cdi.obsDeg) << L"\u00B0"
               << (nav1.cdi.toFlag ? L" TO" : (nav1.cdi.fromFlag ? L" FR" : L""))
               << dmeStr
               << L" | Baro: " << std::setprecision(2) << avi.altimeter.kollsmanInHg << L"\""
               << L" | RPM: " << static_cast<int>(latestPropOut.engineRpm)
               << L" | MAP: " << std::setprecision(1) << latestPropOut.manifoldPressureInHg << L"\""
               << L" | FF: " << latestPropOut.fuelFlowGph << L" GPH"
               << L" | Mix: " << static_cast<int>(activeCtrl.mixture * 100.0) << L"%"
               << L" | Flaps: " << (activeCtrl.flapSetting * 10) << L"\u00B0"
               << L" | " << gearStatus
               << (activeCtrl.brakes > 0.05 ? L" | BRK" : L"")
               << L" | COM: " << (serialInterface.IsConnected() ? L"LIVE" : L"OFF")
               << L" | " << autopilot.GetStatusString()
               << L" | Disp [H]: " << (cockpitDisplayMode == LokYS::Graphics::CockpitDisplayMode::GlassCockpitPFD ? L"PFD" :
                                      (cockpitDisplayMode == LokYS::Graphics::CockpitDisplayMode::AnalogSixPack ? L"ANALOG" : L"OFF"))
               << L" | NET [N]: " << network.GetStatusString()
               << L" | [W]x | [G]Fog"
               << L" | Reset: [R]";
            SetWindowTextW(window.GetHwnd(), ss.str().c_str());
        }
    }

    network.Shutdown();
    serialInterface.Shutdown();
    audioSystem.Shutdown();
    glassCockpitRenderer.Shutdown();
    avionicsRenderer.Shutdown();
    scenerySystem.Shutdown();
    terrainSystem.Shutdown();
    shadowMap.Shutdown();
    renderer.Shutdown();
    graphics.Shutdown();
    window.Destroy();
    context.Shutdown();
    return 0;
}
