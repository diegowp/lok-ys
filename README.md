# LokYS Native Windows Flight Simulator — Session Context

> **Last Updated:** 2026-10-16  
> **Status:** All 23 Unit & Benchmark Test Suites Passing (100%), Zero Compiler Warnings (`/W4 /WX /permissive-`), YSFlight Interoperability Phases 1–5 Fully Completed & Hardened in Runtime.

---

## 1. Project Overview & Architecture

**LokYS** is an aerospace-grade, native Windows C++20 flight simulator built entirely from first principles without game engines (no Unreal, no Unity, no Godot).

* **Language & Toolchain:** Modern C++20, MSVC 2019/2022 x64, CMake 3.20+, Ninja build system.
* **Compiler Flags:** Strict `/W4 /WX /permissive-` (zero warnings treated as errors), unified `/MD` CRT runtime.
* **Graphics API:** Pure native Direct3D 12 (D3D12), DXGI 1.6, Agility SDK / D3D12 Core, custom HLSL SM 6.0 root signature pipelines, forward+ dynamic lighting, cascaded directional shadow maps (CSM).
* **Physics / Simulation:** Fixed-timestep 120 Hz Simulation Scheduler, 6-DOF Newtonian/Eulerian Rigid Body Dynamics with 4th-order Runge-Kutta (RK4) numerical integration, parallel axis theorem (Steiner) for dynamic fuel/payload center of gravity.
* **Aerodynamics:** Lift curve slope $C_{L_\alpha}$, induced drag polars, stall hysteresis, McCormick ground effect factor $\phi(h/b)$, Rankine-Froude actuator disk propwash slipstream immersion.
* **Audio:** Native Windows XAudio2 7.1/stereo spatial acoustics, speed-of-sound Doppler, low-pass cockpit attenuation.
* **Avionics:** Dual-mode instrumentation — Analog Six-Pack cluster (M14) and Digital Glass Cockpit PFD/HUD (M21) with digital autopilot (roll/pitch/heading/altitude/approach guidance loops).
* **YSFlight Interoperability:** Zero-disk extraction in-memory PKZIP VFS reader, robust tokenizer, `.srf` geometry, `.dnm` articulated hierarchies, `.dat` flight dynamics, `.ist` instrument panels, `.fld` terrain elevation heightfields, `.stp` spawn points, standalone editors (`AircraftEditor`, `SceneryEditor`).
* **Multiplayer:** Native Winsock2 non-blocking UDP protocol with Hermite cubic spline position interpolation and projective dead reckoning (M20).
* **Hardware Interface:** Win32 Overlapped Async COM3 serial telemetry protocol streaming flight and engine packets at 30 Hz (M17).

---

## 2. YSFlight Interoperability Ecosystem (Phases 1 to 5)

| Phase | Module | Component Description |
| :--- | :--- | :--- |
| **Phase 1** | Foundation & VFS | `ZipVfsReader` (in-memory streaming PKZIP/deflate without disk I/O), `AssetTokenizer` (zero-allocation parser), `SrfParser` (Newell polygon normal, triangulation), `DnmParser` (articulated scene graph with animation classes). |
| **Phase 2** | Aircraft Ecosystem | `AircraftDatParser` (.dat flight dynamics, SI unit conversions, inertia estimation), `CockpitIstParser` (.ist analogue gauge positions), `AcftPackage` (`.acft.zip` container loading/caching). |
| **Phase 3** | Scenery & World | `SceneryFldParser` (.fld elevation mesh blocks, runways, placed ground objects), `SpawnStpParser` (.stp spawn positions), `LfsPackage` (`.lfs` unified scenery container). |
| **Phase 4** | Standalone Tooling | `ZipWriter` (PKZIP compression engine), `AircraftEditor` (visual node hierarchy & aerodynamic stability verification), `SceneryEditor` (3D elevation sculpting & placement). |
| **Phase 5** | QA & Benchmarks | `YsflightInterOpTests` validating real-world assets (`cessna172r`, `f-16`, `b747`, `ys-11`, `small.fld`, `hawaii.fld`, `aomori.fld`, `heathrow.fld`) and a 240-frame 120 Hz headless simulation benchmark ($0.93\,\mu\text{s}$ per physics tick, budget $< 1000\,\mu\text{s}$). |

---

## 3. Recent Critical Bug Fixes & Numerical Hardening

### 3.1. Correção de Renderização Invertida (Ponta-Cabeça) em Modelos Importados
* **Causa Raiz:** Em `GltfModel::LoadFromDnm`, o nó era inicializado com `Math::Quat4f(0.0f, 0.0f, 0.0f, 1.0f)`. Como o construtor em `include/LokYS/math/Quaternion.hpp` possui a assinatura `(w, x, y, z)`, isso atribuía $w=0, x=0, y=0, z=1$, gerando uma rotação de $180^\circ$ em torno do eixo $Z$ ($\theta = 2 \arccos(0) = \pi$). A matriz resultante invertia $X \to -X$ e $Y \to -Y$, fazendo com que toda a fuselagem renderizasse de ponta-cabeça.
* **Solução:** Substituído por `Math::Quat4f::Identity()` ($w=1, x=0, y=0, z=0$) e sincronizados `restTranslation`, `restRotation`, `restScale` e `pivotPoint` a partir dos metadados do nó DNM.

### 3.2. Correção de Travamento na Troca de Aeronave em Voo (*In-Flight Hot-Swap*)
* **Causa Raiz:** A tecla `V` acionava a troca de aeronave incondicionalmente no loop principal (`main.cpp`), causando dupla troca no menu. Em voo ativo (120 Hz), a troca abrupta de massa e inércia sem zerar as forças acumuladas e o choque de momento angular desestabilizava o integrador RK4.
* **Solução:** 
  1. O acionamento de `V` no loop de voo foi estritamente delimitado a `gameStateMgr.IsInSimulation()`.
  2. Implementada harmonização de transientes: `aircraft.ClearForcesAndMoments()`, amortecimento de velocidade angular residual ($\vec{\omega} = \vec{0}$), renormalização do quatérnion de atitude, sincronização de RPM/combustível e reset do piloto automático/aviônicos.
  3. Publicação imediata de snapshot consistente no `SimulationScheduler` para evitar descontinuidades temporais.

### 3.3. Blindagem $O(1)$ Contra Laços Infinitos em Normalização Angular
* **Causa Raiz:** Em `AutopilotSystem.cpp`, `AvionicsSystem.cpp` e `GlassCockpitRenderer.cpp`, a normalização para $[-180^\circ, +180^\circ]$ utilizava `while (deg > 180.0) deg -= 360.0;`. Caso um valor infinito ($\pm\infty$) ou astronômico fosse gerado por um transiente, a condição de parada nunca era atingida, travando o simulador em loop com 100% de CPU.
* **Solução:** Substituição de todos os loops `while` por aritmética modular determinística (`std::fmod`) com checagem estrita de finitude (`std::isfinite`).

### 3.4. Unificação do Runtime CRT (/MD)
* **Causa Raiz:** Inconsistência de linkage entre bibliotecas estáticas compiladas com `/MD` e módulos auxiliares gerava potenciais corrupções de heap no descompactador Deflate (`ZipVfsReader`).
* **Solução:** Todas as bibliotecas e executáveis do CMake foram padronizados para compilação dinâmica consistente com o CRT (`/MD`).

---

## 4. Directory Layout & Key Source Files

```
LokYS/
├── assets/
│   ├── aircraft/                      # YSFlight packages (.zip, .acft.zip) and folders (e.g. t-25, cessna172r, f16)
│   ├── models/                        # Native glTF 2.0 binary models (.glb: cessna172, baron58, extra330, c-17)
│   └── scenery/                       # YSFlight terrain packages (.lfs, .fld, .stp)
├── include/LokYS/
│   ├── aerodynamics/                  # AerodynamicsSystem, Airfoils
│   ├── aircraft/                      # AircraftRegistry, AircraftTypes
│   ├── assets/                        # SrfParser, DnmParser, AircraftDatParser, CockpitIstParser, AcftPackage, etc.
│   ├── audio/                         # AudioSystem, AudioTypes
│   ├── autopilot/                     # AutopilotSystem, PidController
│   ├── avionics/                      # AvionicsSystem, AvionicsTypes
│   ├── core/                          # EngineContext, GameState, JsonParser, Timer, Types
│   ├── environment/                   # EnvironmentSystem, AtmosphereModel, WindModel
│   ├── graphics/                      # GltfModel, GltfTypes, Camera, Renderer3D, GlassCockpit, AvionicsRenderer
│   ├── input/                         # InputSystem, ActionMapping, ConfigPersistence
│   ├── math/                          # Vector3, Quaternion, Matrix4x4, Matrix3x3, Units, Constants
│   ├── navigation/                    # RadioNavSystem, NavDatabase, NavTypes
│   ├── network/                       # NetworkSystem, UdpSocket, RemoteAircraft
│   ├── physics/                       # RigidBody, GroundReactions, StrutProperties
│   ├── platform/                      # Window, PlatformSystem
│   ├── propulsion/                    # PropulsionSystem, PistonEngine
│   ├── scenery/                       # AirportDatabase, ScenerySystem
│   ├── serial/                        # SerialInterfaceSystem, SerialProtocol
│   ├── simulation/                    # SimulationScheduler, SimulationSnapshot
│   ├── tools/                         # AircraftEditor, SceneryEditor
│   └── vfs/                           # ZipVfsReader, ZipWriter
├── src/
│   ├── app/main.cpp                   # Main loop, state machine, catalog discovery, render pass
│   ├── aerodynamics/                  # AerodynamicsSystem.cpp
│   ├── aircraft/                      # AircraftRegistry.cpp
│   ├── assets/                        # SrfParser, DnmParser, AircraftDatParser, CockpitIstParser, AcftPackage, etc.
│   ├── audio/                         # AudioSystem.cpp
│   ├── autopilot/                     # AutopilotSystem.cpp
│   ├── avionics/                      # AvionicsSystem.cpp
│   ├── environment/                   # EnvironmentSystem.cpp, AtmosphereModel.cpp, WindModel.cpp
│   ├── graphics/                      # GltfModel.cpp, Camera.cpp, Renderer3D.cpp, GlassCockpitRenderer.cpp, etc.
│   ├── input/                         # InputSystem.cpp, ConfigPersistence.cpp
│   ├── physics/                       # RigidBody.cpp, GroundReactions.cpp
│   ├── propulsion/                    # PropulsionSystem.cpp
│   ├── tools/                         # AircraftEditor.cpp, SceneryEditor.cpp
│   └── vfs/                           # ZipVfsReader.cpp, ZipWriter.cpp
├── docs/                              # Architecture, Requirements, ADRs, Roadmaps, Physics specifications
└── tests/unit/                        # 23 Unit and Benchmark test suites (100% pass)
```

---

## 5. Test Suite Matrix (23/23 Passing)

| Test Executable | Target Domain | Status |
| :--- | :--- | :--- |
| `LokYSActionMappingTests.exe` | Action mapping persistence & re-binding | PASS |
| `LokYSAeroInputTests.exe` | Input filtering & flight control mappings | PASS |
| `LokYSAeroPhysicsTests.exe` | 6-DOF aerodynamics & lift/drag polars | PASS |
| `LokYSAtmosphereWeatherTests.exe` | ISA atmosphere, lapse rate, wind & gusts | PASS |
| `LokYSAudioTests.exe` | XAudio2 3D spatial acoustics & Doppler | PASS |
| `LokYSAvionicsTests.exe` | Analog Six-Pack gauges (ASI, AI, ALT, TC, HI, VSI) | PASS |
| `LokYSGameStateTests.exe` | State transitions (Menu, Simulation, Pause, Settings) | PASS |
| `LokYSGlassCockpitAutopilotTests.exe` | Digital PFD, HUD, Flight Director & AP loops | PASS |
| `LokYSGltfTests.exe` | glTF 2.0 & DNM kinematics, bounds, articulations | PASS |
| `LokYSGraphicsOptimizationTests.exe` | Mesh generation & LOD performance | PASS |
| `LokYSGroundReactionsTests.exe` | Tricycle & taildragger landing gear dynamics | PASS |
| `LokYSMathTests.exe` | Vector, Quaternion, Matrix algebra | PASS |
| `LokYSMultiAircraftSceneryTests.exe` | Multi-archetype registry & airport scenery | PASS |
| `LokYSNetworkMultiplayerTests.exe` | UDP network replication & dead reckoning | PASS |
| `LokYSPhysicsTests.exe` | RK4 integration, mass distribution & Steiner CG | PASS |
| `LokYSPropulsionTests.exe` | Piston engine, governor & propeller blade pitch | PASS |
| `LokYSRadioNavTests.exe` | VOR, DME, ILS localizer & glideslope receivers | PASS |
| `LokYSSerialInterfaceTests.exe` | Hardware serial telemetry streaming | PASS |
| `LokYSYsflightPhase1Tests.exe` | SRF geometry, DNM scene graph, ZipVfsReader | PASS |
| `LokYSYsflightPhase2Tests.exe` | DAT flight physics, IST panels, AcftPackage | PASS |
| `LokYSYsflightPhase3Tests.exe` | FLD elevation, STP spawn points, LfsPackage | PASS |
| `LokYSYsflightPhase4Tests.exe` | ZipWriter, AircraftEditor, SceneryEditor | PASS |
| `LokYSYsflightInterOpTests.exe` | Phase 5 real-asset ingestion & 120 Hz benchmark | PASS |

---

## 6. Keybindings & Interactive Controls

| Category | Key / Input | Action |
| :--- | :--- | :--- |
| **Flight Controls** | `Up` / `Down` Arrow | Pitch Down / Up (Elevator) |
| | `Left` / `Right` Arrow | Roll Left / Right (Ailerons) |
| | `Z` / `X` | Rudder Left / Right (Yaw) |
| | `F` / `Shift+F` | Extend / Retract Flaps ($0^\circ, 10^\circ, 20^\circ, 30^\circ$) |
| | `B` | Apply Wheel Brakes |
| | `Space` | Toggle Parking Brake (Active on spawn) |
| **Throttle & Engine** | `1` / `2` | Throttle Down / Up |
| | `3` / `4` | Mixture Lean / Rich |
| | `U` | Toggle Retractable Landing Gear |
| **Aircraft Selection**| **`V`** | **Cycle Active Aircraft (Built-ins $\to$ Custom glTF $\to$ Imported YSFlight)** |
| **Camera Controls** | **`C`** | **Toggle Camera Mode (Chase External $\leftrightarrow$ Cockpit Internal)** |
| | **Right Mouse Drag** | **360° Orbit Look-Around (Pitch and Yaw in Chase or Cockpit)** |
| | **Mouse Wheel** | **Zoom In / Zoom Out ($0.35\times$ to $4.0\times$)** |
| **Position Presets** | `R` | Toggle Position (Runway Static Takeoff $\leftrightarrow$ 3000 ft Cruise) |
| **Cockpit Displays** | `H` | Cycle Display Mode (Glass PFD $\to$ HUD $\to$ Six-Pack $\to$ Off) |
| **Autopilot (AP)** | `A` | Toggle Autopilot Master (AP ON/OFF) |
| | `D` | Toggle Heading Select Mode (HDG SEL) |
| | `S` | Toggle Altitude Hold Mode (ALT HOLD) |
| | `,` / `.` | Adjust Heading Bug Left / Right |
| **Multiplayer / Comms** | `N` | Cycle Network Mode (Offline $\to$ Host $\to$ Client) |

---

## 7. How to Add New YSFlight Aircraft to LokYS

1. **ZIP Container Format (`.acft.zip` or `.zip`):**
   - Place any `.acft.zip` or `.zip` containing `.dat` and `.dnm` files into `assets/aircraft/`.
   - The engine automatically scans and registers the package on startup.
2. **Directory-Based Format (Loose Mod Folder):**
   - Place the folder containing the `.dat` and `.dnm` files (e.g. `assets/aircraft/t-25/`) directly into `assets/aircraft/`.
   - The package is automatically indexed, its 3D geometry is loaded, and its aerodynamic polars are integrated into the runtime catalog.
3. **Selection:**
   - Choose the aircraft from the Main Menu pre-flight carousel, or press **`V`** during simulation to hot-swap in mid-flight.
