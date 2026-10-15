# LokYS Native Windows Flight Simulator — Session Context

> **Last Updated:** 2026-10-14  
> **Status:** All 15 Unit Test Suites Passing (100%), Zero Compiler Warnings (`/W4 /WX /permissive-`), Ready for Live Simulation & Next Development Phase.

---

## 1. Project Overview & Architecture

**LokYS** is an aerospace-grade, native Windows C++20 flight simulator built entirely from first principles without game engines (no Unreal, no Unity).

* **Language / Toolchain:** C++20, MSVC 2019/2022 x64, CMake 3.20+, Ninja build system.
* **Compiler Flags:** Strict `/W4 /WX /permissive-` (zero warnings treated as errors).
* **Graphics API:** Pure native Direct3D 12 (D3D12), DXGI 1.6, Agility SDK / D3D12 Core, custom HLSL SM 6.0 root signature pipelines.
* **Physics / Simulation:** Fixed-timestep 120 Hz Simulation Scheduler, 6-DOF Newtonian/Eulerian Rigid Body Dynamics with Runge-Kutta numerical integration.
* **Audio:** Native Windows XAudio2 7.1/stereo spatial acoustics, speed-of-sound Doppler, low-pass cockpit attenuation.
* **Avionics:** Dual-mode rendering — Analog Six-Pack cluster (M14) and Digital Glass Cockpit PFD (M21) with digital autopilot.
* **Multiplayer:** Native Winsock2 non-blocking UDP protocol with Hermite cubic spline position interpolation and projective dead reckoning (M20).
* **Hardware Interface:** Win32 Overlapped Async COM3 serial telemetry protocol streaming flight and engine packets at 30 Hz (M17).

---

## 2. Recent Implementations & Fixes

### 2.1. glTF 2.0 Node Matrix Support (Upside-Down Model Remediation)
* **Root Cause:** Sketchfab/Blender FBX exports use the glTF `"matrix"` property on root nodes (e.g. `Sketchfab_model`) containing $-1.0$ vertical flips to convert from FBX axis conventions. The original parser only evaluated `translation`/`rotation`/`scale`, ignoring node matrices and causing vertices to render inverted.
* **Solution:** Added `hasMatrix` and `Math::Mat4f matrix` support to `GltfNode` and `GltfModel::ParseJson`. Forward kinematics now correctly evaluates both TRS and matrix nodes, orienting all imported models upright.

### 2.2. Dynamic Aircraft Catalog & Auto-Discovery
* Implemented automatic directory scanner (`DiscoverCustomGlbModels`) scanning `assets/models/`.
* Any `.glb` placed in `assets/models/` is automatically discovered at startup, registered with GPU resources, added to the active aircraft catalog, and cycled via the **`V`** key.

### 2.3. Universal Model Grounding & Orientation Auto-Alignment
* **C-17 Ground Penetration Fix:** `c-17a_globemaster_iii.glb` was exported with origin at the wing box ($Y_{min} = -4.40\text{ m}$). Applied universal grounding formula:
  $$\Delta Y = -0.935\text{ m} - Y_{min}$$
  This guarantees the lowest point of the landing gear of **any** model rests at exactly $Y_{world} = 0.000\text{ m}$ on the runway surface.
* **C-17 Reverse Orientation Fix:** Detected semantic node tags (`rudder`, `tail` vs `cockpit`, `nose`). Models facing backwards ($-Z$) are automatically rotated $180^\circ$ around the Y axis ($\text{RotY}(\pi)$) to face forward ($+Z$).

### 2.4. Dedicated Cockpit Camera Eye-Level Positions
* Replaced heuristic dimension scaling in `Camera.cpp` with per-aircraft Body FRD (*Forward, Right, Down*) eye positions:
  * **Cessna 172 Skyhawk:** $X=+0.25\text{ m}, Y=-0.27\text{ m}, Z=-0.70\text{ m}$ (pilot seated in left seat, eye aligned with left yoke and looking through windshield above instrument panel).
  * **C-17 Globemaster III:** $X=+8.20\text{ m}, Y=-0.75\text{ m}, Z=-2.10\text{ m}$ (flight deck captain seat ~3m above runway with panoramic flight deck view).
  * **Baron 58:** $X=+0.60\text{ m}, Y=-0.32\text{ m}, Z=-0.65\text{ m}$ (executive twin).
  * **Extra 330SC:** $X=-0.20\text{ m}, Y=0.00\text{ m}, Z=-0.55\text{ m}$ (centerline aerobatic seat under bubble canopy).

---

## 3. Directory Layout & Key Source Files

```
LokYS/
├── assets/
│   └── models/
│       ├── cessna172.glb              # High-detail 26 MB Cessna 172 (51 meshes, 243k verts)
│       ├── c-17a_globemaster_iii.glb  # 45.5 MB Boeing C-17 Globemaster III (103 meshes, 219k verts)
│       ├── baron58.glb                # Beechcraft Baron 58 twin-engine model
│       └── extra330.glb               # Extra 330SC aerobatic model
├── include/LokYS/
│   ├── aircraft/                      # AircraftRegistry, AircraftTypes
│   ├── graphics/                      # GltfModel, GltfTypes, Camera, Renderer3D, GlassCockpit, Avionics
│   ├── physics/                       # RigidBody, GroundReactions, StrutProperties
│   ├── aerodynamics/                  # AerodynamicsSystem, Airfoils
│   ├── propulsion/                    # PropulsionSystem, PistonEngine
│   ├── environment/                   # EnvironmentSystem, AtmosphereModel, WindModel
│   ├── autopilot/                     # AutopilotSystem, PidController
│   ├── navigation/                    # RadioNavSystem, NavDatabase
│   ├── network/                       # NetworkSystem, UdpSocket
│   └── serial/                        # SerialInterfaceSystem, SerialProtocol
├── src/
│   ├── app/main.cpp                   # Main loop, dynamic aircraft catalog, input dispatch, rendering passes
│   └── graphics/
│       ├── GltfModel.cpp              # glTF 2.0 parser, Forward Kinematics, AutoAlignAndGround
│       └── Camera.cpp                 # Chase and Cockpit camera modes, orbit, zoom, eye-level tracking
└── tests/unit/                        # 15 CTest unit test suites (100% pass)
```

---

## 4. Keybindings & Interactive Controls

| Category | Key / Input | Action |
| :--- | :--- | :--- |
| **Flight Controls** | `Up` / `Down` Arrow | Pitch Down / Up (Elevator) |
| | `Left` / `Right` Arrow | Roll Left / Right (Ailerons) |
| | `Z` / `X` | Rudder Left / Right (Yaw) |
| | `F` / `Shift+F` | Extend / Retract Flaps ($0^\circ, 10^\circ, 20^\circ, 30^\circ$) |
| | `B` | Apply Wheel Brakes |
| **Throttle & Engine** | `1` / `2` | Throttle Down / Up |
| | `3` / `4` | Mixture Lean / Rich |
| | `U` | Toggle Retractable Landing Gear (Baron 58 / C-17) |
| **Aircraft Selection**| **`V`** | **Cycle Active Aircraft Model (C172 $\to$ B58 $\to$ E330 $\to$ C-17 $\to$ ...)** |
| **Camera Controls** | **`C`** | **Toggle Camera Mode (Chase External $\leftrightarrow$ Cockpit Internal)** |
| | **Right Mouse Drag** | **360° Orbit Look-Around (Pitch and Yaw in Chase or Cockpit)** |
| | **Mouse Wheel** | **Zoom In / Zoom Out ($0.35\times$ to $4.0\times$)** |
| **Position Presets** | `R` | Toggle Position (Runway Static Takeoff $\leftrightarrow$ 3000 ft Cruise) |
| **Cockpit Displays** | `H` | Cycle Display Mode (Glass PFD $\to$ HUD $\to$ Six-Pack $\to$ Off) |
| **Autopilot (AP)** | `A` | Toggle Autopilot Master (AP ON/OFF) |
| | `D` | Toggle Heading Select Mode (HDG SEL) |
| | `S` | Toggle Altitude Hold Mode (ALT HOLD) |
| | `,` / `.` | Adjust Heading Bug Left / Right |
| **Environment** | `W` | Cycle Weather Presets (Clear Calm $\to$ Standard $\to$ Crosswind $\to$ Gusty $\to$ Hot/High) |
| | `G` | Cycle Fog Mode (Auto $\to$ Off $\to$ Light $\to$ Moderate $\to$ Dense) |

---

## 5. Build, Test, and Execution Commands

Run from project root (`c:\Users\diego\Documents\Projetos\LokYS`):

### Build Project
```cmd
cmd /c "call ""C:\Program Files (x86)\Microsoft Visual Studio\2019\BuildTools\VC\Auxiliary\Build\vcvars64.bat"" && ninja -C build"
```
*(Or use `scratch\build.bat`)*

### Run All 15 Unit Test Suites
```powershell
Get-ChildItem build\*Tests.exe | ForEach-Object { & $_.FullName; if ($LASTEXITCODE -ne 0) { Write-Error "$($_.Name) failed" } }
```

### Run Flight Simulator
```powershell
.\build\LokYSFlightSimulator.exe
```

### Run Headless Sanity Test (120 frames)
```powershell
.\build\LokYSFlightSimulator.exe --test-frames=120
```

---

## 6. Next Steps & Future Roadmap

1. **Heavy Aircraft Flight Dynamics:**
   * Create a dedicated `HeavyTransport` archetype in `AircraftRegistry` tailored for large aircraft (e.g. C-17, 4 turbofan engines, 260,000 kg MTOW, high-lift slotted flaps) rather than falling back to the Baron 58 twin-piston profile.
2. **Liveries & Texture Variants:**
   * Support glTF material texture swaps or material overrides per aircraft slot.
3. **Cockpit Instrument Alignment:**
   * Allow docking the Glass Cockpit PFD or HUD seamlessly inside the 3D cockpit window space when in `CameraMode::Cockpit`.
4. **Airport Scenery Expansion:**
   * Additional regional airport runways and taxiway lighting definitions in `AirportDatabase.cpp`.
