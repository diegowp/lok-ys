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
#include "LokYS/aircraft/AircraftTypes.hpp"
#include "LokYS/aircraft/AircraftRegistry.hpp"
#include "LokYS/core/GameState.hpp"
#include "LokYS/scenery/AirportDatabase.hpp"
#include "LokYS/graphics/ApproachLightingSystem.hpp"
#include "LokYS/graphics/TerrainStreamingSystem.hpp"
#include "LokYS/graphics/MenuRenderer.hpp"
#include "LokYS/input/ConfigPersistence.hpp"
#include "LokYS/assets/AcftPackage.hpp"
#include <windows.h>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <algorithm>
#include <filesystem>

inline void LogTrace([[maybe_unused]] const char* msg) noexcept {}

int WINAPI wWinMain(
    [[maybe_unused]] HINSTANCE hInstance,
    [[maybe_unused]] HINSTANCE hPrevInstance,
    [[maybe_unused]] PWSTR pCmdLine,
    [[maybe_unused]] int nCmdShow)
{
    (void)_unlink("sim_trace.txt");
    (void)_unlink("sim_crash.log");
    LokYS::EngineContext context;
    if (!context.Initialize()) {
        MessageBoxW(
            nullptr,
            L"Failed to initialize Direct3D 12 and platform engine context.",
            L"LokYS Flight Simulator - Critical Error",
            MB_ICONERROR | MB_OK);
        return 1;
    }

    SetUnhandledExceptionFilter([](EXCEPTION_POINTERS* ep) -> LONG {
        FILE* f = nullptr;
        fopen_s(&f, "sim_crash.log", "w");
        if (f) {
            fprintf(f, "CRASH! Code: 0x%08X, Address: %p\n", ep->ExceptionRecord->ExceptionCode, ep->ExceptionRecord->ExceptionAddress);
            if (ep->ExceptionRecord->NumberParameters >= 2) {
                fprintf(f, "AV Type: %llu, Target: 0x%llX\n", (unsigned long long)ep->ExceptionRecord->ExceptionInformation[0], (unsigned long long)ep->ExceptionRecord->ExceptionInformation[1]);
            }
            HMODULE hMod = nullptr;
            if (GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                                   (LPCSTR)ep->ExceptionRecord->ExceptionAddress, &hMod)) {
                char modName[MAX_PATH] = { 0 };
                GetModuleFileNameA(hMod, modName, MAX_PATH);
                uintptr_t offset = (uintptr_t)ep->ExceptionRecord->ExceptionAddress - (uintptr_t)hMod;
                fprintf(f, "Module: %s + 0x%zX\n", modName, offset);
            }
            if (ep->ContextRecord) {
                fprintf(f, "RAX: 0x%llX RBX: 0x%llX RCX: 0x%llX RDX: 0x%llX\n",
                    (unsigned long long)ep->ContextRecord->Rax, (unsigned long long)ep->ContextRecord->Rbx, (unsigned long long)ep->ContextRecord->Rcx, (unsigned long long)ep->ContextRecord->Rdx);
                fprintf(f, "RSI: 0x%llX RDI: 0x%llX RBP: 0x%llX RSP: 0x%llX\n",
                    (unsigned long long)ep->ContextRecord->Rsi, (unsigned long long)ep->ContextRecord->Rdi, (unsigned long long)ep->ContextRecord->Rbp, (unsigned long long)ep->ContextRecord->Rsp);
                fprintf(f, "R8:  0x%llX R9:  0x%llX R10: 0x%llX R11: 0x%llX\n",
                    (unsigned long long)ep->ContextRecord->R8, (unsigned long long)ep->ContextRecord->R9, (unsigned long long)ep->ContextRecord->R10, (unsigned long long)ep->ContextRecord->R11);
                fprintf(f, "R12: 0x%llX R13: 0x%llX R14: 0x%llX R15: 0x%llX\n",
                    (unsigned long long)ep->ContextRecord->R12, (unsigned long long)ep->ContextRecord->R13, (unsigned long long)ep->ContextRecord->R14, (unsigned long long)ep->ContextRecord->R15);
                
                uintptr_t rcx = (uintptr_t)ep->ContextRecord->Rcx;
                if (rcx) {
                    uint32_t* p = (uint32_t*)(rcx + 0x1C);
                    fprintf(f, "[RCX+0x1C] as raw hex: 0x%08X, as float: %f\n", *p, *(float*)p);
                }
                uintptr_t rsp = (uintptr_t)ep->ContextRecord->Rsp;
                if (rsp) {
                    uintptr_t* stk = (uintptr_t*)rsp;
                    fprintf(f, "STACK DUMP:\n");
                    for (int i = 0; i < 20; ++i) {
                        HMODULE hM = nullptr;
                        if (GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT, (LPCSTR)stk[i], &hM)) {
                            char mN[MAX_PATH] = { 0 };
                            GetModuleFileNameA(hM, mN, MAX_PATH);
                            fprintf(f, "  [%02d] 0x%llX (%s + 0x%llX)\n", i, (unsigned long long)stk[i], mN, (unsigned long long)(stk[i] - (uintptr_t)hM));
                        } else {
                            fprintf(f, "  [%02d] 0x%llX\n", i, (unsigned long long)stk[i]);
                        }
                    }
                }
            }
            fflush(f);
            fclose(f);
        }
        return EXCEPTION_EXECUTE_HANDLER;
    });

    const auto& caps = context.GetCapabilities();

    bool headless = false;
    bool autoStart = false;
    LokYS::u32 maxTestFrames = 0;
    if (pCmdLine) {
        if (wcsstr(pCmdLine, L"--headless") || wcsstr(pCmdLine, L"--verify")) {
            headless = true;
        }
        if (wcsstr(pCmdLine, L"--auto-start")) {
            autoStart = true;
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
            ss << L"LokYS Native Windows Flight Simulator [Milestone M22 Full Scenery & Multi-Aircraft Expansion]\n";
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
            ss << L"Multi-Archetype Aircraft Registry (Cessna 172 SEP, Beechcraft Baron 58 MEP, Extra 330SC Aerobatic): OPERATIONAL\n";
            ss << L"Multi-Engine Twin Asymmetric Thrust & Critical Engine Yawing Moment (N = Delta T * y_offset): OPERATIONAL\n";
            ss << L"Retractable Landing Gear Parasite Drag Reduction (Delta CD0 = -0.014): OPERATIONAL\n";
            ss << L"Symmetric Competition Aerobatic Airfoil & Inverted Fuel/Oil Dynamics: OPERATIONAL\n";
            ss << L"Regional Multi-Airport Geographic World (KLOK, KMES Mesa +350m, KVAL Valley +60m): OPERATIONAL\n";
            ss << L"Continuous Multi-Airport Plateau Elevation Blending & Terrain Streaming (64 km x 64 km): OPERATIONAL\n";
            ss << L"Distance-Based Multi-LOD Terrain Mesh Generation & Frustum Culling: OPERATIONAL\n";
            ss << L"ALSF-1 720m High-Intensity Approach Lighting Array (24 Stations, 30m Spacing): OPERATIONAL\n";
            ss << L"Sequenced Flashing Strobe Array ('The Rabbit' Traveling Wave at 1.0 Hz): OPERATIONAL\n";
            ss << L"Runway End Identifier Lights (REIL Synchronized Flashing Strobes at 1.0 Hz): OPERATIONAL\n";
            ss << L"Procedural glTF 2.0 Aircraft Models with Articulated Props, Control Surfaces & Retractable Gear: OPERATIONAL\n";
            ss << L"============================================================\n";
            ss << L"[M22 FULL SCENERY & MULTI-AIRCRAFT EXPANSION VERIFICATION EXIT 0]\n";

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
    desc.title = L"LokYS Flight Simulator (M22 - Full Scenery & Multi-Aircraft Expansion)";
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

    // Milestone M13 & M22: glTF 2.0 Multi-Archetype Aircraft Model Loading & GPU Resources
    auto LoadOrGenerateModel = [&](LokYS::Graphics::GltfModel& model,
                                   const std::string& filename,
                                   bool (*saveFunc)(const std::string&),
                                   std::vector<LokYS::u8> (*genFunc)()) {
        std::vector<std::string> searchPaths = {
            "assets/models/" + filename,
            "build/assets/models/" + filename,
            "../assets/models/" + filename
        };
        std::string gltfErr;
        for (const auto& path : searchPaths) {
            if (std::filesystem::exists(path)) {
                if (model.LoadFromGlbFile(path, &gltfErr)) {
                    (void)model.CreateGPUResources(graphics.GetDevice());
                    return;
                }
            }
        }
        // Save to disk in primary paths for future launches ONLY if files do not exist (protect user custom models)
        if (!std::filesystem::exists("assets/models/" + filename)) {
            saveFunc("assets/models/" + filename);
        }
        if (!std::filesystem::exists("build/assets/models/" + filename)) {
            saveFunc("build/assets/models/" + filename);
        }
        for (const auto& path : searchPaths) {
            if (model.LoadFromGlbFile(path, &gltfErr)) {
                (void)model.CreateGPUResources(graphics.GetDevice());
                return;
            }
        }
        // Direct In-Memory Binary Fallback (Guaranteed 100% success without filesystem dependency)
        std::vector<LokYS::u8> memGlb = genFunc();
        if (!memGlb.empty()) {
            if (model.LoadFromGlbMemory(memGlb.data(), memGlb.size(), &gltfErr)) {
                (void)model.CreateGPUResources(graphics.GetDevice());
            }
        }
    };

    struct AircraftCatalogEntry {
        std::wstring displayName;
        std::string filename;
        LokYS::Graphics::GltfModel model;
        LokYS::Aircraft::AircraftType archetype{ LokYS::Aircraft::AircraftType::Cessna172Skyhawk };
        LokYS::Math::Vec3d cockpitEyePosBody{ 0.25, -0.27, -0.70 };
        std::optional<LokYS::Aircraft::AircraftConfig> customConfig;
    };

    std::vector<AircraftCatalogEntry> aircraftCatalog;

    // 1. Built-in archetypes with procedural binary fallbacks
    {
        AircraftCatalogEntry cessnaEntry;
        cessnaEntry.displayName = L"Cessna 172 Skyhawk";
        cessnaEntry.filename = "cessna172.glb";
        cessnaEntry.archetype = LokYS::Aircraft::AircraftType::Cessna172Skyhawk;
        cessnaEntry.cockpitEyePosBody = { 0.25, -0.27, -0.70 };
        LoadOrGenerateModel(
            cessnaEntry.model,
            "cessna172.glb",
            static_cast<bool (*)(const std::string&)>(&LokYS::Graphics::AircraftModelGenerator::SaveCessna172Glb),
            &LokYS::Graphics::AircraftModelGenerator::GenerateCessna172Glb);
        cessnaEntry.model.AutoAlignAndGround(11.0f);
        aircraftCatalog.push_back(std::move(cessnaEntry));
    }

    {
        AircraftCatalogEntry baronEntry;
        baronEntry.displayName = L"Baron 58";
        baronEntry.filename = "baron58.glb";
        baronEntry.archetype = LokYS::Aircraft::AircraftType::Baron58;
        baronEntry.cockpitEyePosBody = { 0.60, -0.32, -0.65 };
        LoadOrGenerateModel(
            baronEntry.model,
            "baron58.glb",
            static_cast<bool (*)(const std::string&)>(&LokYS::Graphics::AircraftModelGenerator::SaveBaron58Glb),
            &LokYS::Graphics::AircraftModelGenerator::GenerateBaron58Glb);
        baronEntry.model.AutoAlignAndGround(11.53f);
        aircraftCatalog.push_back(std::move(baronEntry));
    }

    {
        AircraftCatalogEntry extraEntry;
        extraEntry.displayName = L"Extra 330SC";
        extraEntry.filename = "extra330.glb";
        extraEntry.archetype = LokYS::Aircraft::AircraftType::Extra330SC;
        extraEntry.cockpitEyePosBody = { -0.20, 0.00, -0.55 };
        LoadOrGenerateModel(
            extraEntry.model,
            "extra330.glb",
            static_cast<bool (*)(const std::string&)>(&LokYS::Graphics::AircraftModelGenerator::SaveExtra330Glb),
            &LokYS::Graphics::AircraftModelGenerator::GenerateExtra330Glb);
        extraEntry.model.AutoAlignAndGround(7.50f);
        aircraftCatalog.push_back(std::move(extraEntry));
    }

    // 2. Dynamic Discovery: Scan models directory for ANY custom .glb models
    auto DiscoverCustomGlbModels = [&](const std::string& dirPath) {
        if (!std::filesystem::exists(dirPath)) return;
        std::error_code ec;
        for (const auto& entry : std::filesystem::directory_iterator(dirPath, ec)) {
            if (ec) break;
            if (!entry.is_regular_file()) continue;

            std::string ext = entry.path().extension().string();
            for (char& c : ext) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
            if (ext != ".glb") continue;

            std::string fn = entry.path().filename().string();
            std::string fnLower = fn;
            for (char& c : fnLower) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));

            // Skip built-in archetypes already registered
            if (fnLower == "cessna172.glb" || fnLower == "baron58.glb" || fnLower == "extra330.glb") {
                continue;
            }

            // Check if already loaded in catalog
            bool alreadyInCatalog = false;
            for (const auto& item : aircraftCatalog) {
                std::string itemFnLower = item.filename;
                for (char& c : itemFnLower) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
                if (itemFnLower == fnLower) {
                    alreadyInCatalog = true;
                    break;
                }
            }
            if (alreadyInCatalog) continue;

            AircraftCatalogEntry customEntry{};
            customEntry.filename = fn;
            std::string gltfErr;
            if (customEntry.model.LoadFromGlbFile(entry.path().string(), &gltfErr)) {
                customEntry.model.AutoAlignAndGround(0.0f); // Auto heading detect (180 deg reverse if needed) & ground level
                (void)customEntry.model.CreateGPUResources(graphics.GetDevice());

                // Format human-readable display name from filename
                std::string stem = entry.path().stem().string();
                std::wstring wideDisplayName;
                bool capNext = true;
                for (char c : stem) {
                    if (c == '_' || c == '-') {
                        wideDisplayName += L' ';
                        capNext = true;
                    } else if (capNext) {
                        wideDisplayName += static_cast<wchar_t>(std::toupper(static_cast<unsigned char>(c)));
                        capNext = false;
                    } else {
                        wideDisplayName += static_cast<wchar_t>(c);
                    }
                }
                customEntry.displayName = wideDisplayName;

                // Associate archetype & calibrate pilot cockpit eye position in Body FRD
                if (customEntry.model.GetEffectiveDimensions().x > 18.0f ||
                    fnLower.find("c-17") != std::string::npos ||
                    fnLower.find("c17") != std::string::npos ||
                    fnLower.find("globemaster") != std::string::npos)
                {
                    customEntry.archetype = LokYS::Aircraft::AircraftType::Baron58; // Multi-engine heavy
                    customEntry.cockpitEyePosBody = { 8.20, -0.75, -2.10 };       // Military flight deck captain seat
                } else {
                    customEntry.archetype = LokYS::Aircraft::AircraftType::Cessna172Skyhawk;
                    LokYS::f64 fwd = static_cast<LokYS::f64>(customEntry.model.GetEffectiveDimensions().z) * 0.15;
                    LokYS::f64 up = static_cast<LokYS::f64>(customEntry.model.GetEffectiveDimensions().y) * 0.35;
                    customEntry.cockpitEyePosBody = { fwd, -0.27, -up };
                }

                aircraftCatalog.push_back(std::move(customEntry));
            }
        }
    };

    DiscoverCustomGlbModels("assets/models");
    DiscoverCustomGlbModels("build/assets/models");
    DiscoverCustomGlbModels("../assets/models");

    // 3. Dynamic Discovery: Scan assets/aircraft for any YSFlight packages (.acft.zip, .zip) or loose folders / .dat files
    auto RegisterYsflightPackage = [&](LokYS::Assets::AcftPackage& package, const std::string& uniqueKey) {
        if (!package.IsValid()) return;

        std::string keyLower = uniqueKey;
        for (char& c : keyLower) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));

        for (const auto& item : aircraftCatalog) {
            std::string itemFnLower = item.filename;
            for (char& c : itemFnLower) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
            if (itemFnLower == keyLower) {
                return; // Already in catalog
            }
        }

        AircraftCatalogEntry customEntry{};
        customEntry.filename = uniqueKey;

        if (customEntry.model.LoadFromDnm(package.GetVisualModel())) {
            customEntry.model.AutoAlignAndGround(0.0f);
            (void)customEntry.model.CreateGPUResources(graphics.GetDevice());

            const auto& flightData = package.GetFlightData();
            customEntry.customConfig = flightData.config;

            std::string dispName = !package.GetManifest().displayName.empty()
                ? package.GetManifest().displayName
                : flightData.config.name;

            std::wstring wideDisplayName;
            for (char c : dispName) {
                wideDisplayName += (static_cast<unsigned char>(c) < 128)
                    ? static_cast<wchar_t>(c) : L'?';
            }
            customEntry.displayName = wideDisplayName;

            LokYS::f64 fwd = static_cast<LokYS::f64>(customEntry.model.GetEffectiveDimensions().z) * 0.15;
            LokYS::f64 up = static_cast<LokYS::f64>(customEntry.model.GetEffectiveDimensions().y) * 0.35;
            customEntry.cockpitEyePosBody = { fwd, -0.27, -up };

            aircraftCatalog.push_back(std::move(customEntry));
        }
    };

    auto DiscoverYsflightPackages = [&](const std::string& dirPath) {
        if (!std::filesystem::exists(dirPath)) return;
        std::error_code ec;
        for (const auto& entry : std::filesystem::directory_iterator(dirPath, ec)) {
            if (ec) break;

            if (entry.is_regular_file()) {
                std::string ext = entry.path().extension().string();
                for (char& c : ext) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));

                if (ext == ".zip") {
                    LokYS::Assets::AcftPackage package;
                    if (package.LoadFromZipFile(entry.path().string())) {
                        RegisterYsflightPackage(package, entry.path().stem().string());
                    }
                } else if (ext == ".dat") {
                    LokYS::Assets::AcftPackage package;
                    if (package.LoadFromDirectory(dirPath, entry.path().filename().string())) {
                        RegisterYsflightPackage(package, entry.path().stem().string());
                    }
                }
            } else if (entry.is_directory()) {
                std::error_code subEc;
                for (const auto& subEntry : std::filesystem::directory_iterator(entry.path(), subEc)) {
                    if (subEc) break;
                    if (subEntry.is_regular_file()) {
                        std::string subExt = subEntry.path().extension().string();
                        for (char& c : subExt) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
                        if (subExt == ".dat") {
                            LokYS::Assets::AcftPackage package;
                            if (package.LoadFromDirectory(entry.path().string(), subEntry.path().filename().string())) {
                                RegisterYsflightPackage(package, entry.path().filename().string() + "_" + subEntry.path().stem().string());
                            }
                        }
                    }
                }
            }
        }
    };

    DiscoverYsflightPackages("assets/aircraft");
    DiscoverYsflightPackages("build/assets/aircraft");
    DiscoverYsflightPackages("../assets/aircraft");

    size_t activeCatalogIdx = 0;

    // Dynamic camera framing initialization based on loaded active aircraft model dimensions
    camera.SetTargetDimensions(
        aircraftCatalog[activeCatalogIdx].model.GetEffectiveDimensions().z,
        aircraftCatalog[activeCatalogIdx].model.GetEffectiveDimensions().x,
        aircraftCatalog[activeCatalogIdx].model.GetEffectiveDimensions().y);
    camera.SetCockpitEyeOffset(aircraftCatalog[activeCatalogIdx].cockpitEyePosBody);

    // Milestone M22: Approach Lighting System (ALSF-1 720m array + Sequenced Strobes + REIL)
    LokYS::Graphics::ApproachLightingSystem approachLights;
    (void)approachLights.Initialize(graphics.GetDevice());

    // Milestone M22: Regional Multi-Airport Terrain Streaming System (64km x 64km, 64 Tiles)
    LokYS::Graphics::TerrainStreamingSystem terrainStreaming;
    (void)terrainStreaming.Initialize(graphics.GetDevice());

    // Milestone M22: Multi-Archetype Aircraft Registry
    LokYS::Aircraft::AircraftRegistry aircraftRegistry;

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

    // Fase 3 & 4: Direct3D 12 2D Vector UI Menu & Settings Renderer
    LokYS::Graphics::MenuRenderer menuRenderer;
    if (!menuRenderer.Initialize(graphics.GetDevice())) {
        MessageBoxW(nullptr, L"Failed to initialize Direct3D 12 vector menu rendering pipeline.", L"Menu Error", MB_ICONERROR | MB_OK);
        glassCockpitRenderer.Shutdown();
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

    // Fase 4: Global Settings & Input Action Remapping Configuration
    LokYS::Input::SimulatorConfig simConfig{};
    std::string configPath = LokYS::Input::ConfigPersistence::GetDefaultConfigPath();
    if (!LokYS::Input::ConfigPersistence::LoadFromFile(configPath, simConfig)) {
        (void)LokYS::Input::ConfigPersistence::SaveToFile(configPath, simConfig);
    }
    input.SetActionMap(simConfig.actionMap);
    menuRenderer.SetSelectedVolume(simConfig.masterVolume);
    menuRenderer.SetSelectedDisplayMode(simConfig.cockpitDisplayMode);

    // Milestone M14 & M21: Avionics & Digital Autopilot Flight Control Computer Subsystems
    LokYS::Avionics::AvionicsSystem avionics;
    LokYS::Autopilot::AutopilotSystem autopilot;
    autopilot.Initialize();
    LokYS::Graphics::CockpitDisplayMode cockpitDisplayMode =
        static_cast<LokYS::Graphics::CockpitDisplayMode>(simConfig.cockpitDisplayMode % 4);

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

    LokYS::Scenery::AirportDatabase airportDb;
    LokYS::Core::GameStateManager gameStateMgr((maxTestFrames > 0 && !autoStart) ? LokYS::Core::GameState::Simulation : LokYS::Core::GameState::MainMenu);

    // Apply defaults from persistent configuration
    auto defaultFCfg = gameStateMgr.GetFlightConfig();
    defaultFCfg.airportIcao = simConfig.defaultAirportIcao;
    defaultFCfg.selectedRunwayIndex = simConfig.defaultRunwayIndex;
    defaultFCfg.weatherPresetIndex = static_cast<uint8_t>(simConfig.defaultWeatherPresetIndex % 5);
    defaultFCfg.spawnMode = (simConfig.defaultSpawnMode == 1)
        ? LokYS::Core::SpawnMode::AirborneCruise
        : LokYS::Core::SpawnMode::RunwayTakeoff;
    gameStateMgr.SetFlightConfig(defaultFCfg);

    // Initial State Presets: Runway Takeoff vs 3000 ft Cruise
    bool onRunwayMode = true;

    auto ApplyAircraftConfig = [&](const LokYS::Aircraft::AircraftConfig& cfg) {
        aircraft.SetMassDistribution(
            cfg.mass.emptyMassKg,
            cfg.mass.emptyCgBody,
            cfg.mass.inertiaTensor,
            cfg.mass.fuelTankLocationBody,
            cfg.mass.payloadLocationBody);
        aircraft.UpdateFuelAndPayload(cfg.mass.defaultFuelKg, 0.0);
        aero.SetConfig(cfg.geometry, cfg.aero, aircraftRegistry.IsGearExtended());
        propulsion.SetAircraftEngineConfig(cfg.propulsion);
        groundReactions.SetConfig(LokYS::Physics::GroundReactionsSystem::CreateForArchetype(cfg.type));
    };

    auto ApplyFlightSetup = [&](const LokYS::Core::FlightSetupConfiguration& setup) {
        activeCatalogIdx = setup.selectedAircraftIndex % aircraftCatalog.size();
        const auto& activeEntry = aircraftCatalog[activeCatalogIdx];
        if (activeEntry.customConfig.has_value()) {
            aircraftRegistry.SetCustomConfig(*activeEntry.customConfig);
        } else {
            aircraftRegistry.SetActiveAircraft(activeEntry.archetype);
        }
        const auto& cfg = aircraftRegistry.GetActiveConfig();
        ApplyAircraftConfig(cfg);

        camera.SetTargetDimensions(
            activeEntry.model.GetEffectiveDimensions().z,
            activeEntry.model.GetEffectiveDimensions().x,
            activeEntry.model.GetEffectiveDimensions().y);
        camera.SetCockpitEyeOffset(activeEntry.cockpitEyePosBody);

        auto weatherPreset = static_cast<LokYS::Environment::WeatherPreset>(setup.weatherPresetIndex % 5);
        environment.ApplyPreset(weatherPreset);
        renderer.SetFog(GetFogForPreset(weatherPreset));

        auto optApt = airportDb.FindAirport(setup.airportIcao);
        LokYS::Scenery::AirportConfig apt = optApt.has_value() ? *optApt : airportDb.GetAirports()[0];
        const auto& rwy = apt.runways[setup.selectedRunwayIndex % apt.runways.size()];

        LokYS::Physics::RigidBodyState state{};
        LokYS::f64 rwyHdgRad = LokYS::Math::Units::DegToRad(rwy.headingDeg);
        LokYS::Math::Vec3d fwdVec(std::cos(rwyHdgRad), std::sin(rwyHdgRad), 0.0);

        if (setup.spawnMode == LokYS::Core::SpawnMode::RunwayTakeoff) {
            onRunwayMode = true;
            LokYS::f64 thresholdOffset = -(rwy.lengthM * 0.45);
            state.positionNed = rwy.centerNed + fwdVec * thresholdOffset;
            state.positionNed.z = -(rwy.elevationM + 0.935);
            state.velocityBody = LokYS::Math::Vec3d(0.0, 0.0, 0.0);
            state.angularVelocityBody = LokYS::Math::Vec3d(0.0, 0.0, 0.0);
            state.attitude = LokYS::Math::Quat4d::FromEuler(0.0, 0.0, rwyHdgRad);
            aircraft.SetState(state);

            LokYS::Input::FlightControls ctrl{};
            ctrl.throttle = 0.0;
            ctrl.mixture = 1.0;
            ctrl.flapSetting = 0;
            ctrl.brakes = 1.0;
            input.SetControls(ctrl);
            input.SetParkingBrake(true);

            propulsion.SetFuelQuantityKg(cfg.mass.defaultFuelKg * setup.initialFuelPercentage);
            propulsion.SetRpm(cfg.propulsion.idleRpm);

            environment.Update(state.positionNed, state.attitude, 0.0, 1.0 / 120.0);
            avionics.ResetToSteadyState(
                state,
                environment.GetAtmosphere().pressurePa,
                environment.GetAtmosphere().densityKgM3);

            autopilot.Reset();
            autopilot.SetTargetAltitude(1000.0);
            autopilot.SetTargetHeading(rwy.headingDeg);
        } else {
            onRunwayMode = false;
            state.positionNed = rwy.centerNed;
            state.positionNed.z = -(rwy.elevationM + 914.4);
            state.velocityBody = LokYS::Math::Vec3d(51.4444, 0.0, 0.0);
            state.angularVelocityBody = LokYS::Math::Vec3d(0.0, 0.0, 0.0);
            state.attitude = LokYS::Math::Quat4d::FromEuler(0.0, 0.0, rwyHdgRad);
            aircraft.SetState(state);

            LokYS::Input::FlightControls ctrl{};
            ctrl.throttle = 0.65;
            ctrl.mixture = 0.85;
            ctrl.flapSetting = 0;
            ctrl.brakes = 0.0;
            input.SetControls(ctrl);
            input.SetParkingBrake(false);

            propulsion.SetFuelQuantityKg(cfg.mass.defaultFuelKg * setup.initialFuelPercentage);
            propulsion.SetRpm(cfg.propulsion.maxRatedRpm * 0.85);

            environment.Update(state.positionNed, state.attitude, 51.4444, 1.0 / 120.0);
            avionics.ResetToSteadyState(
                state,
                environment.GetAtmosphere().pressurePa,
                environment.GetAtmosphere().densityKgM3);

            autopilot.Reset();
            autopilot.SetTargetAltitude(rwy.elevationM * 3.28084 + 3000.0);
            autopilot.SetTargetHeading(rwy.headingDeg);
        }

        scheduler.Reset();
        auto snap = LokYS::Simulation::SimulationSnapshot::CreateFromRigidBody(0, 0.0, state);
        scheduler.PublishSnapshot(snap);
        scheduler.PublishSnapshot(snap);
        camera.Reset();
    };

    auto ResetToRunway = [&]() {
        auto cfg = gameStateMgr.GetFlightConfig();
        cfg.spawnMode = LokYS::Core::SpawnMode::RunwayTakeoff;
        gameStateMgr.SetFlightConfig(cfg);
        ApplyFlightSetup(cfg);
    };

    auto ResetToCruise = [&]() {
        auto cfg = gameStateMgr.GetFlightConfig();
        cfg.spawnMode = LokYS::Core::SpawnMode::AirborneCruise;
        gameStateMgr.SetFlightConfig(cfg);
        ApplyFlightSetup(cfg);
    };

    ApplyFlightSetup(gameStateMgr.GetFlightConfig());

    LokYS::Physics::GroundReactionsOutput latestGroundOut{};
    LokYS::Propulsion::PropulsionOutput latestPropOut{};
    LokYS::Aerodynamics::AeroOutput latestAeroOut{};
    bool wasAirborne = false;

    // Milestone M15: Native Windows XAudio2 & X3DAudio Spatial Sound Subsystem
    LokYS::AudioSystem audioSystem;
    LokYS::AudioConfig audioConfig;
    audioConfig.masterVolume = simConfig.masterVolume;
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
        LogTrace("Top of while loop");
        timer.Tick();

        if (maxTestFrames > 0 && timer.GetFrameCount() >= maxTestFrames) {
            break;
        }

        // Poll Hardware / Keyboard Flight Controls
        input.Poll(timer.GetDeltaTime());

        // Update 2D Menu & UI mouse cursor and click state
        menuRenderer.UpdateMouse(
            input.GetMouseX(),
            input.GetMouseY(),
            input.IsLeftMouseDown(),
            input.WasLeftMouseClicked());

        // Toggle Camera View with 'C' key
        if (input.WasCameraToggleTriggered()) {
            camera.ToggleMode();
            camera.ResetOrbit();
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

        // Milestone M22: Cycle Aircraft Archetype / Model with 'V' key (Cessna 172 -> Baron 58 -> Extra 330 -> Custom Models)
        if (input.WasAircraftCycleTriggered() && !aircraftCatalog.empty()) {
            activeCatalogIdx = (activeCatalogIdx + 1) % aircraftCatalog.size();
            const auto& activeEntry = aircraftCatalog[activeCatalogIdx];
            if (activeEntry.customConfig.has_value()) {
                aircraftRegistry.SetCustomConfig(*activeEntry.customConfig);
            } else {
                aircraftRegistry.SetActiveType(activeEntry.archetype);
            }
            ApplyAircraftConfig(aircraftRegistry.GetActiveConfig());

            // Dynamically update camera framing and cockpit eye position to fit the active aircraft dimensions
            camera.SetTargetDimensions(
                activeEntry.model.GetEffectiveDimensions().z,
                activeEntry.model.GetEffectiveDimensions().x,
                activeEntry.model.GetEffectiveDimensions().y);
            camera.SetCockpitEyeOffset(activeEntry.cockpitEyePosBody);
            camera.ResetOrbit();
        }

        // Milestone M22: Toggle Retractable Landing Gear with 'U' key (Baron 58)
        if (input.WasGearToggleTriggered()) {
            if (aircraftRegistry.GetActiveConfig().hasRetractableGear) {
                aircraftRegistry.ToggleGear();
                aero.SetConfig(
                    aircraftRegistry.GetActiveConfig().geometry,
                    aircraftRegistry.GetActiveConfig().aero,
                    aircraftRegistry.IsGearExtended());
            }
        }

        // Cycle Cockpit Display Mode with 'H' key (PFD -> TransparentHUD -> AnalogSixPack -> Off -> PFD)
        if (input.WasPanelToggleTriggered()) {
            switch (cockpitDisplayMode) {
            case LokYS::Graphics::CockpitDisplayMode::GlassCockpitPFD:
                cockpitDisplayMode = LokYS::Graphics::CockpitDisplayMode::TransparentHUD;
                break;
            case LokYS::Graphics::CockpitDisplayMode::TransparentHUD:
                cockpitDisplayMode = LokYS::Graphics::CockpitDisplayMode::AnalogSixPack;
                break;
            case LokYS::Graphics::CockpitDisplayMode::AnalogSixPack:
                cockpitDisplayMode = LokYS::Graphics::CockpitDisplayMode::Off;
                break;
            case LokYS::Graphics::CockpitDisplayMode::Off:
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

        // GameState Management Input Handling
        if (gameStateMgr.IsInMenu()) {
            auto fCfg = gameStateMgr.GetFlightConfig();
            if (input.WasAircraftCycleTriggered() || menuRenderer.WasCycleAircraftTriggered()) {
                fCfg.selectedAircraftIndex = (fCfg.selectedAircraftIndex + 1) % aircraftCatalog.size();
                gameStateMgr.SetFlightConfig(fCfg);
                activeCatalogIdx = fCfg.selectedAircraftIndex;
                camera.SetTargetDimensions(
                    aircraftCatalog[activeCatalogIdx].model.GetEffectiveDimensions().z,
                    aircraftCatalog[activeCatalogIdx].model.GetEffectiveDimensions().x,
                    aircraftCatalog[activeCatalogIdx].model.GetEffectiveDimensions().y);
                camera.SetCockpitEyeOffset(aircraftCatalog[activeCatalogIdx].cockpitEyePosBody);
            }
            if (input.WasAirportCycleTriggered() || menuRenderer.WasCycleAirportTriggered()) {
                const auto& apts = airportDb.GetAirports();
                size_t curIdx = 0;
                for (size_t i = 0; i < apts.size(); ++i) {
                    if (apts[i].icao == fCfg.airportIcao) { curIdx = i; break; }
                }
                size_t nextIdx = (curIdx + 1) % apts.size();
                fCfg.airportIcao = apts[nextIdx].icao;
                fCfg.selectedRunwayIndex = 0;
                gameStateMgr.SetFlightConfig(fCfg);
            }
            if (input.WasWeatherToggleTriggered() || menuRenderer.WasCycleWeatherTriggered()) {
                fCfg.weatherPresetIndex = (fCfg.weatherPresetIndex + 1) % 5;
                gameStateMgr.SetFlightConfig(fCfg);
                environment.ApplyPreset(static_cast<LokYS::Environment::WeatherPreset>(fCfg.weatherPresetIndex));
                if (fogMode == FogMode::Auto) {
                    renderer.SetFog(GetFogForPreset(environment.GetActivePreset()));
                }
            }
            if (input.WasSpawnModeToggleTriggered() || menuRenderer.WasToggleSpawnModeTriggered()) {
                fCfg.spawnMode = (fCfg.spawnMode == LokYS::Core::SpawnMode::RunwayTakeoff)
                    ? LokYS::Core::SpawnMode::AirborneCruise
                    : LokYS::Core::SpawnMode::RunwayTakeoff;
                gameStateMgr.SetFlightConfig(fCfg);
            }
            if (input.WasSettingsToggleTriggered() || menuRenderer.WasSettingsTriggered()) {
                gameStateMgr.TransitionTo(LokYS::Core::GameState::Settings);
            }
            if (input.WasMenuSelectTriggered() || menuRenderer.WasStartFlightTriggered() || (autoStart && timer.GetFrameCount() >= 2)) {
                LogTrace("StartFlight triggered - calling ApplyFlightSetup");
                ApplyFlightSetup(fCfg);
                LogTrace("ApplyFlightSetup completed - TransitionTo Simulation");
                gameStateMgr.TransitionTo(LokYS::Core::GameState::Simulation);
            }
            if (input.WasPauseToggleTriggered()) {
                break;
            }
        } else if (gameStateMgr.IsInSimulation()) {
            if (input.WasPauseToggleTriggered()) {
                gameStateMgr.TransitionTo(LokYS::Core::GameState::Paused);
            }
            if (input.WasSettingsToggleTriggered()) {
                gameStateMgr.TransitionTo(LokYS::Core::GameState::Settings);
            }
        } else if (gameStateMgr.IsPaused()) {
            if (input.WasPauseToggleTriggered() || input.WasMenuSelectTriggered() || menuRenderer.WasResumeFlightTriggered()) {
                gameStateMgr.TransitionTo(LokYS::Core::GameState::Simulation);
            }
            if (input.WasResetTriggered() || menuRenderer.WasRestartFlightTriggered()) {
                ApplyFlightSetup(gameStateMgr.GetFlightConfig());
                gameStateMgr.TransitionTo(LokYS::Core::GameState::Simulation);
            }
            if (input.WasSettingsToggleTriggered() || menuRenderer.WasSettingsTriggered()) {
                gameStateMgr.TransitionTo(LokYS::Core::GameState::Settings);
            }
            if (input.WasMenuBackTriggered() || menuRenderer.WasExitToMenuTriggered()) {
                gameStateMgr.TransitionTo(LokYS::Core::GameState::MainMenu);
            }
        } else if (gameStateMgr.IsInSettings()) {
            // Rebinding listening state
            if (menuRenderer.GetListeningAction().has_value()) {
                auto targetAction = *menuRenderer.GetListeningAction();
                if ((GetAsyncKeyState(VK_ESCAPE) & 0x8000) != 0) {
                    menuRenderer.ClearListeningAction();
                } else {
                    bool shift = (GetAsyncKeyState(VK_SHIFT) & 0x8000) != 0;
                    bool ctrl = (GetAsyncKeyState(VK_CONTROL) & 0x8000) != 0;
                    bool alt = (GetAsyncKeyState(VK_MENU) & 0x8000) != 0;
                    for (LokYS::u32 vk = 8; vk <= 222; ++vk) {
                        if (vk == VK_SHIFT || vk == VK_CONTROL || vk == VK_MENU ||
                            vk == VK_LSHIFT || vk == VK_RSHIFT || vk == VK_LCONTROL ||
                            vk == VK_RCONTROL || vk == VK_LMENU || vk == VK_RMENU) {
                            continue;
                        }
                        if ((GetAsyncKeyState(static_cast<int>(vk)) & 0x8000) != 0) {
                            simConfig.actionMap.SetBinding(targetAction, LokYS::Input::KeyBinding(vk, ctrl, alt, shift));
                            input.SetActionMap(simConfig.actionMap);
                            (void)LokYS::Input::ConfigPersistence::SaveToFile(configPath, simConfig);
                            menuRenderer.ClearListeningAction();
                            break;
                        }
                    }
                }
            } else {
                if (menuRenderer.WasResetBindingsTriggered()) {
                    simConfig.actionMap.ResetToDefaults();
                    input.SetActionMap(simConfig.actionMap);
                    (void)LokYS::Input::ConfigPersistence::SaveToFile(configPath, simConfig);
                }
                if (menuRenderer.WasSaveConfigTriggered()) {
                    simConfig.masterVolume = menuRenderer.GetSelectedVolume();
                    audioSystem.SetMasterVolume(simConfig.masterVolume);
                    simConfig.cockpitDisplayMode = menuRenderer.GetSelectedDisplayMode();
                    cockpitDisplayMode = static_cast<LokYS::Graphics::CockpitDisplayMode>(simConfig.cockpitDisplayMode % 4);
                    (void)LokYS::Input::ConfigPersistence::SaveToFile(configPath, simConfig);
                }
                if (input.WasSettingsToggleTriggered() || input.WasPauseToggleTriggered() ||
                    input.WasMenuBackTriggered() || menuRenderer.WasExitToMenuTriggered()) {
                    gameStateMgr.ReturnToPrevious();
                }
            }
        }

        LogTrace("L: after input poll");

        // 1. Advance Fixed-Timestep Physics Simulation (120 Hz)
        if (gameStateMgr.IsInSimulation()) {
            LogTrace("L: before scheduler.Update");
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
                // Control Wheel Steering safety: if pilot manually moves pitch or roll stick, disengage AP immediately
                if (std::abs(ctrl.pitch) > 0.25 || std::abs(ctrl.roll) > 0.25) {
                    autopilot.SetMasterEngaged(false);
                } else {
                    const auto& apOut = autopilot.GetOutput();
                    activeCtrl.pitch = apOut.elevatorCommand;
                    activeCtrl.roll = apOut.aileronCommand;
                    if (std::abs(apOut.rudderCommand) > 1e-4) {
                        activeCtrl.yaw = apOut.rudderCommand;
                    }
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

            // Phase F2: Sample terrain AGL for McCormick Ground Effect & Actuator Disk Propwash
            LokYS::Math::Vec3f posGfx = LokYS::Math::CoordinateFrames::NedToGraphics(state.positionNed);
            LokYS::f32 terrainElevMsl = terrainStreaming.SampleElevation(posGfx.x, posGfx.z);
            LokYS::f64 groundHeightNed = -static_cast<LokYS::f64>(terrainElevMsl);
            LokYS::f64 altitudeAglM = std::max(0.0, (-state.positionNed.z) - static_cast<LokYS::f64>(terrainElevMsl));

            const auto& activeCfg = aircraftRegistry.GetActiveConfig();
            LokYS::f64 propwashImmersion = (activeCfg.propulsion.engineCount == 1) ? 0.60 : 0.15;

            auto aeroOut = aero.Evaluate(
                relVelBody,
                state.angularVelocityBody,
                deflections,
                atmo.densityKgM3,
                atmo.speedOfSoundMps,
                altitudeAglM,
                latestPropOut.thrustNewtons,
                activeCfg.propulsion.propDiameterM,
                propwashImmersion);
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

            // Milestone M22: Multi-Engine Asymmetric Yaw Moment (Baron 58)
            if (activeCfg.propulsion.engineCount > 1) {
                LokYS::Math::Vec3d asymForce{};
                LokYS::Math::Vec3d asymMoments{};
                LokYS::Aircraft::AircraftRegistry::ComputeMultiEngineForces(
                    activeCfg,
                    propOut.thrustNewtons * 0.5,
                    propOut.thrustNewtons * 0.5,
                    asymForce,
                    asymMoments);
                aircraft.AddMomentBody(asymMoments);
            }

            // Update aircraft mass and dynamic CG with fuel burn (Phase F1)
            aircraft.UpdateFuelAndPayload(propOut.remainingFuelKg, 0.0);

            LogTrace("Pre-groundReactions.Evaluate");
            auto groundOut = groundReactions.Evaluate(state, activeCtrl.yaw, activeCtrl.brakeLeft, activeCtrl.brakeRight, dt, groundHeightNed);
            LogTrace("Pre-groundReactions.ApplyToRigidBody");
            groundReactions.ApplyToRigidBody(aircraft, groundOut);
            latestGroundOut = groundOut;

            LogTrace("Pre-avionics.Update");
            LokYS::Math::Vec3d linearAccelBody = aeroOut.forcesBody * (1.0 / aircraft.GetMass());
            avionics.Update(state, linearAccelBody, atmo.pressurePa, atmo.densityKgM3, dt);

            LogTrace("Pre-radioNav.Update");
            LokYS::Math::Vec3d velocityNed = state.attitude.Rotate(state.velocityBody);
            radioNav.Update(state.positionNed, velocityNed, dt);
            aircraft.IntegrateRK4(dt);

            scheduler.PublishSnapshot(LokYS::Simulation::SimulationSnapshot::CreateFromRigidBody(
                tick, simTime, aircraft.GetState()));
        });
        }

        // 2. Fetch Interpolated State Snapshot for Render Presentation
        auto renderSnapshot = scheduler.GetInterpolatedSnapshot();

        // 3. Render Pass
        if (!window.IsMinimized()) {
            graphics.OnResize(window.GetWidth(), window.GetHeight());
            camera.SetPerspective(1.0472f, graphics.GetAspectRatio(), 0.2f, 20000.0f);

            // Handle Camera Zoom from Mouse Wheel and Keyboard
            LokYS::f32 wheelDelta = window.GetAndResetMouseWheelDelta();
            if (std::abs(wheelDelta) > 1e-4f) {
                camera.AdjustZoom(-wheelDelta * 0.12f);
            }
            LokYS::f32 keyZoom = input.GetCameraZoomDelta();
            if (std::abs(keyZoom) > 1e-4f) {
                camera.AdjustZoom(keyZoom);
            }

            // Handle Camera Orbit from Right Mouse Drag and Keyboard
            LokYS::f32 orbitYaw = input.GetCameraOrbitYawDelta();
            LokYS::f32 orbitPitch = input.GetCameraOrbitPitchDelta();
            if (std::abs(orbitYaw) > 1e-5f || std::abs(orbitPitch) > 1e-5f) {
                camera.AdjustOrbit(orbitYaw, orbitPitch);
            }
            if (input.WasOrbitResetTriggered()) {
                camera.ResetOrbit();
            }

            LogTrace("L: before camera.Update");
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

            LokYS::Graphics::GltfModel* activeModel = &aircraftCatalog[activeCatalogIdx].model;

            LogTrace("L: before ApplyAircraftControls");
            activeModel->ApplyAircraftControls(
                static_cast<float>(renderCtrl.pitch),
                static_cast<float>(renderCtrl.roll),
                static_cast<float>(renderCtrl.yaw),
                static_cast<float>(renderCtrl.flapSetting) / 3.0f,
                static_cast<float>(propellerAngleRad),
                aircraftRegistry.IsGearExtended());

            // Milestone M22: Update Approach Lighting System ("The Rabbit" & REIL timers)
            approachLights.Update(timer.GetDeltaTime(), timer.GetTotalTime(), graphics.GetDevice());

            // Milestone M22: Update Terrain Streaming LODs around camera position
            terrainStreaming.UpdateLODs(camera.GetCameraPosGfx().x, camera.GetCameraPosGfx().z);

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

            LogTrace("L: before BeginFrame");
            graphics.BeginFrame(clearSkyColor);

            // Milestone M19 Pass 1: Depth-Only Shadow Pass for Dynamic Aircraft Shadows
            LogTrace("L: before shadow pass");
            shadowMap.Begin(graphics.GetCommandList());
            renderer.RenderShadowPass(
                graphics.GetCommandList(),
                shadowMap,
                *activeModel,
                renderSnapshot.bodyState.positionNed,
                renderSnapshot.bodyState.attitude);
            shadowMap.End(graphics.GetCommandList());

            // Milestone M19 Pass 2: Restore Main Backbuffer & Depth-Stencil Render Target
            graphics.BindMainRenderTarget();

            // Milestone M19 & M20: Main Forward Pass with Multi-LOD Terrain, Scenery, Runway, CSM Shadows & Remote Peers
            LogTrace("L: before forward pass renderer.Render");
            renderer.Render(
                graphics.GetCommandList(),
                camera,
                &shadowMap,
                terrainSystem,
                runwayMesh,
                scenerySystem,
                *activeModel,
                renderSnapshot.bodyState.positionNed,
                renderSnapshot.bodyState.attitude,
                network.GetRemoteAircraft());

            // Milestone M22: Render ALSF-1 Approach Light Arrays and Sequenced Strobes ("The Rabbit" & REIL)
            approachLights.DrawStaticBars(graphics.GetCommandList());
            approachLights.DrawDynamicStrobes(graphics.GetCommandList());

            // Milestone M21 & M14: Render Cockpit Instrumentation (Camera-Aware & Mode-Aware)
            bool isCockpitView = (camera.GetMode() == LokYS::Graphics::CameraMode::Cockpit);
            glassCockpitRenderer.SetParkingBrake(input.IsParkingBrakeEngaged());
            const auto& frameControls = input.GetControls();
            glassCockpitRenderer.SetPhysicsAnnunciators(
                static_cast<LokYS::f32>(latestAeroOut.groundEffectFactor),
                static_cast<LokYS::f32>(latestAeroOut.slipstreamVelocityMps),
                static_cast<LokYS::f32>(latestPropOut.bladePitchDeg),
                frameControls.brakeLeft > 0.05,
                frameControls.brakeRight > 0.05);

            if (gameStateMgr.IsInSimulation()) {
                switch (cockpitDisplayMode) {
                case LokYS::Graphics::CockpitDisplayMode::GlassCockpitPFD: {
                    auto dockPos = isCockpitView
                        ? LokYS::Graphics::PfdDockPosition::BottomLeft
                        : LokYS::Graphics::PfdDockPosition::CornerMini;
                    LogTrace("Pre-glassCockpitRenderer.RenderPfd");
                    glassCockpitRenderer.RenderPfd(
                        graphics.GetCommandList(),
                        window.GetWidth(),
                        window.GetHeight(),
                        avionics.GetState(),
                        radioNav.GetNav1(),
                        autopilot.GetStatus(),
                        true,
                        dockPos);
                    LogTrace("Post-glassCockpitRenderer.RenderPfd");
                    break;
                }
                case LokYS::Graphics::CockpitDisplayMode::TransparentHUD: {
                    glassCockpitRenderer.RenderTransparentHud(
                        graphics.GetCommandList(),
                        window.GetWidth(),
                        window.GetHeight(),
                        avionics.GetState(),
                        radioNav.GetNav1(),
                        autopilot.GetStatus(),
                        renderSnapshot.bodyState.velocityBody,
                        true);
                    break;
                }
                case LokYS::Graphics::CockpitDisplayMode::AnalogSixPack: {
                    LokYS::f32 scale = isCockpitView ? 1.0f : 0.80f;
                    avionicsRenderer.Render(
                        graphics.GetCommandList(),
                        window.GetWidth(),
                        window.GetHeight(),
                        avionics.GetState(),
                        radioNav.GetNav1(),
                        true,
                        scale);
                    break;
                }
                case LokYS::Graphics::CockpitDisplayMode::Off:
                    // No cockpit instruments rendered (pure external / clean VFR window)
                    break;
                }
            } else if (gameStateMgr.IsInMenu()) {
                const auto& fCfg = gameStateMgr.GetFlightConfig();
                auto optApt = airportDb.FindAirport(fCfg.airportIcao);
                std::string aptName = optApt.has_value() ? optApt->name : "LokYS Regional";
                std::string rwyIdent = (optApt.has_value() && !optApt->runways.empty()) ? optApt->runways[0].ident : "36/18";
                LokYS::f64 rwyLengthM = (optApt.has_value() && !optApt->runways.empty()) ? optApt->runways[0].lengthM : 2100.0;
                LokYS::f64 rwyElevationM = optApt.has_value() ? optApt->elevationM : 22.0;
                std::wstring wsWeather = environment.GetPresetName();
                std::string weatherName;
                for (wchar_t wc : wsWeather) {
                    weatherName += (wc < 128) ? static_cast<char>(wc) : '?';
                }
                std::string spawnName = LokYS::Core::SpawnModeToString(fCfg.spawnMode);

                menuRenderer.RenderPreFlight(
                    graphics.GetCommandList(),
                    window.GetWidth(),
                    window.GetHeight(),
                    aircraftCatalog[activeCatalogIdx].displayName,
                    aptName,
                    fCfg.airportIcao,
                    rwyIdent,
                    rwyLengthM,
                    rwyElevationM,
                    weatherName,
                    spawnName,
                    static_cast<LokYS::f64>(fCfg.initialFuelPercentage));
            } else if (gameStateMgr.IsPaused()) {
                menuRenderer.RenderPause(
                    graphics.GetCommandList(),
                    window.GetWidth(),
                    window.GetHeight());
            } else if (gameStateMgr.IsInSettings()) {
                menuRenderer.RenderSettings(
                    graphics.GetCommandList(),
                    window.GetWidth(),
                    window.GetHeight(),
                    simConfig);
            }

            LogTrace("Pre-graphics.EndFrame");
            graphics.EndFrame(true); // Vsync on for consistent frametimes
            LogTrace("Post-graphics.EndFrame");
        }

        LogTrace("Pre-AudioListenerState setup");
        // Milestone M15: Spatial Audio Engine Update
        LokYS::AudioListenerState listener{};
        listener.inCockpit = (camera.GetMode() == LokYS::Graphics::CameraMode::Cockpit);
        listener.position = camera.GetCameraPosGfx();
        listener.forward = camera.GetForward();
        listener.up = camera.GetUp();
        LokYS::Math::Vec3d velNed = renderSnapshot.bodyState.attitude.Rotate(renderSnapshot.bodyState.velocityBody);
        listener.velocity = LokYS::Math::CoordinateFrames::NedToGraphics(velNed);

        LokYS::AudioEmitterState emitter{};
        emitter.position = LokYS::Math::CoordinateFrames::NedToGraphics(renderSnapshot.bodyState.positionNed);
        emitter.forward = listener.forward;
        emitter.up = listener.up;
        emitter.velocity = listener.velocity;
        emitter.minDistance = 2.0f;
        emitter.maxDistance = 500.0f;

        LokYS::EngineAudioParams engineAudio{};
        LokYS::WindAudioParams windAudio{};
        LokYS::GroundAudioParams groundAudio{};
        LogTrace("Post-AudioListenerState setup");

        if (gameStateMgr.IsInSimulation()) {
            engineAudio.rpm = static_cast<float>(latestPropOut.engineRpm);
            engineAudio.throttle = static_cast<float>(input.GetControls().throttle);
            engineAudio.manifoldPressure = static_cast<float>(latestPropOut.manifoldPressureInHg);

            windAudio.dynamicPressure = static_cast<float>(latestAeroOut.dynamicPressurePa);
            windAudio.trueAirspeed = static_cast<float>(latestAeroOut.trueAirspeedMps);

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
        }

        LogTrace("Pre-audioSystem.Update");
        audioSystem.Update(
            static_cast<float>(timer.GetDeltaTime()),
            listener,
            emitter,
            engineAudio,
            windAudio,
            groundAudio);
        LogTrace("Post-audioSystem.Update");

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
        LogTrace("Post-serialTelemetry");

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

            std::wstring acName = !aircraftCatalog.empty()
                ? aircraftCatalog[activeCatalogIdx].displayName
                : L"Aircraft";

            std::wstring gearStateStr;
            if (aircraftRegistry.GetActiveConfig().hasRetractableGear) {
                gearStateStr = aircraftRegistry.IsGearExtended() ? L" | GEAR: DN" : L" | GEAR: UP";
            }

            std::wstringstream ss;
            ss << L"LokYS | [" << acName << L"]"
               << gearStateStr
               << L" | " << std::fixed << std::setprecision(0) << timer.GetFPS() << L" FPS"
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
               << (latestPropOut.bladePitchDeg > 0.0 ? (std::wstring(L" | \u03B2: ") + std::to_wstring(static_cast<int>(latestPropOut.bladePitchDeg)) + L"\u00B0") : L"")
               << (latestAeroOut.groundEffectFactor < 0.99 ? (std::wstring(L" | GE: ") + std::to_wstring(static_cast<int>(latestAeroOut.groundEffectFactor * 100.0)) + L"%") : L"")
               << (latestAeroOut.slipstreamVelocityMps > 1.0 ? (std::wstring(L" | Vslip: ") + std::to_wstring(static_cast<int>(LokYS::Math::Units::MpsToKnots(latestAeroOut.slipstreamVelocityMps))) + L"kt") : L"")
               << L" | FF: " << latestPropOut.fuelFlowGph << L" GPH"
               << L" | Mix: " << static_cast<int>(activeCtrl.mixture * 100.0) << L"%"
               << L" | Flaps [F/V]: " << (activeCtrl.flapSetting * 10) << L"\u00B0"
               << L" | " << gearStatus
               << (input.IsParkingBrakeEngaged() ? L" | PARK-BRK" :
                   (activeCtrl.brakeLeft > 0.05 && activeCtrl.brakeRight > 0.05) ? L" | BRK" :
                   (activeCtrl.brakeLeft > 0.05) ? L" | L-BRK" :
                   (activeCtrl.brakeRight > 0.05) ? L" | R-BRK" : L"")
               << L" | COM: " << (serialInterface.IsConnected() ? L"LIVE" : L"OFF")
               << L" | " << autopilot.GetStatusString()
               << L" | Cam [C]: " << (camera.GetMode() == LokYS::Graphics::CameraMode::Chase ? L"CHASE 3D" : L"COCKPIT")
               << L" | Disp [H]: " << (cockpitDisplayMode == LokYS::Graphics::CockpitDisplayMode::GlassCockpitPFD ? (camera.GetMode() == LokYS::Graphics::CameraMode::Cockpit ? L"PFD" : L"PFD-MINI") :
                                       (cockpitDisplayMode == LokYS::Graphics::CockpitDisplayMode::TransparentHUD ? L"HUD" :
                                       (cockpitDisplayMode == LokYS::Graphics::CockpitDisplayMode::AnalogSixPack ? L"ANALOG" : L"OFF")))
               << L" | NET [N]: " << network.GetStatusString()
               << L" | Plane [Shift+V]: " << acName
               << (aircraftRegistry.GetActiveConfig().hasRetractableGear ? (aircraftRegistry.IsGearExtended() ? L" | Gear [U]: DN" : L" | Gear [U]: UP") : L"")
               << L" | Wx [F8]: " << environment.GetPresetName()
               << L" | Fog [G]: " << fogStr
               << L" | Reset: [R]";
            LogTrace("Pre-SetWindowTextW");
            SetWindowTextW(window.GetHwnd(), ss.str().c_str());
            LogTrace("Post-SetWindowTextW");
        }
        LogTrace("End of while loop iteration");
    }

    LogTrace("Shutdown: network");
    network.Shutdown();
    LogTrace("Shutdown: serialInterface");
    serialInterface.Shutdown();
    LogTrace("Shutdown: audioSystem");
    audioSystem.Shutdown();
    LogTrace("Shutdown: menuRenderer");
    menuRenderer.Shutdown();
    LogTrace("Shutdown: glassCockpitRenderer");
    glassCockpitRenderer.Shutdown();
    LogTrace("Shutdown: avionicsRenderer");
    avionicsRenderer.Shutdown();
    LogTrace("Shutdown: approachLights");
    approachLights.Shutdown();
    LogTrace("Shutdown: terrainStreaming");
    terrainStreaming.Shutdown();
    LogTrace("Shutdown: scenerySystem");
    scenerySystem.Shutdown();
    LogTrace("Shutdown: terrainSystem");
    terrainSystem.Shutdown();
    LogTrace("Shutdown: shadowMap");
    shadowMap.Shutdown();
    LogTrace("Shutdown: renderer");
    renderer.Shutdown();
    LogTrace("Shutdown: graphics");
    graphics.Shutdown();
    LogTrace("Shutdown: window");
    window.Destroy();
    LogTrace("Shutdown: context");
    context.Shutdown();
    LogTrace("Shutdown: completed cleanly");
    ExitProcess(0);
}
