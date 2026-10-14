# LokYS Flight Simulator — Project Checkpoint (Milestone M14)

**Timestamp:** 2026-10-14T01:36:00-03:00  
**Current Milestone:** M14 — Analog Six-Pack Avionics  
**Status:** COMPLETED & VERIFIED (All 7 Test Suites Passing, 0 Warnings, /W4 /WX /permissive-)  
**Target Architecture:** Native Windows x64 (MSVC 19.29 / Visual Studio 2019 BuildTools), Direct3D 12, Pure C++20 (`/std:c++latest`)  
**Design Philosophy:** Zero-Bloat, Zero Third-Party Runtime Dependencies (YSFlight Inspiration)

---

## 1. Executive Summary

LokYS is a native Windows C++20 flight simulation platform engineered under the YSFlight zero-bloat philosophy. The platform has progressed from initial architecture through complete 6-DOF aerodynamics, multi-point oleo landing gear mechanics, piston propulsion dynamics, binary glTF 2.0 articulated aircraft model rendering, and now a complete **Analog Six-Pack Avionics** subsystem with authentic pneumatic and gyroscopic sensor dynamics rendered procedurally via Direct3D 12 vector graphics.

All systems operate deterministically on a decoupled 120 Hz fixed-timestep physics scheduler with triple-buffered state snapshots for jitter-free visual interpolation.

---

## 2. Milestone Completion Status (M0 through M14)

| Milestone | Subsystem | Status | Key Deliverables |
| :--- | :--- | :--- | :--- |
| **M0** | Requirements & Baseline | **COMPLETED** | `REQUIREMENTS.md`, `ARCHITECTURE.md`, `DECISIONS.md`, `BUILD.md` |
| **M1** | Toolchain & Build | **COMPLETED** | CMake 3.20+, Ninja, MSVC `/W4 /WX /permissive-`, zero warning tolerance |
| **M2** | Win32 Platform | **COMPLETED** | Native Win32 window, message pump, raw OS handles, High-DPI Per-Monitor v2 awareness |
| **M3** | Direct3D 12 Core | **COMPLETED** | DXGI 1.6 factory, D3D12 device, command queues, swapchain, fence synchronization |
| **M4** | GPU Adapter Enumeration | **COMPLETED** | VRAM budget query, adapter feature levels (RTX 3050 Laptop / Intel UHD / RDNA2 targets) |
| **M5** | Aerospace Math Engine | **COMPLETED** | Quaternions, 3D vectors, 3x3 / 4x4 matrices, NED/Body/Wind frames, SI <-> Aviation units |
| **M6** | Simulation Scheduler | **COMPLETED** | Decoupled 120 Hz fixed-timestep accumulator, triple-buffered snapshot state |
| **M7** | 6-DOF Rigid-Body Dynamics | **COMPLETED** | Runge-Kutta 4th Order (RK4) integrator, inertia tensors, gravitational dynamics |
| **M8** | Aerodynamics Subsystem | **COMPLETED** | Cessna 172 polar curves (NASA CR-2144), stall breakaway hysteresis, control authority |
| **M9** | Input Engine & Hardware | **COMPLETED** | DirectInput/Win32 joystick polling, quadratic curves, keyboard flight controls |
| **M10** | Integrated Flight Prototype | **COMPLETED** | 3D rendering pipeline, runway & terrain, Cockpit/Chase cameras, takeoff/landing loop |
| **M11** | Ground Reactions & Struts | **COMPLETED** | Multi-point oleo struts, progressive stiffness, hydraulic damping, tire scrub friction |
| **M12** | Propulsion & Fuel | **COMPLETED** | Lycoming O-320 engine, McCauley propeller polars, rotational integrator, P-factor, fuel burn |
| **M13** | glTF 2.0 Aircraft Loading | **COMPLETED** | Spec-compliant `.glb` parser, zero-dep JSON parser, 12-node articulated scene graph, procedural model generator |
| **M14** | Analog Six-Pack Avionics | **COMPLETED** | 6 physical flight instruments (ASI, AI, ALT, TC, DG, VSI), sensor lag, D3D12 vector dials |

---

## 3. Deep-Dive: Milestone M14 (Analog Six-Pack Avionics)

### 3.1 Flight Instrument Physical & Sensor Models

1. **Airspeed Indicator (ASI):**
   - Pitot-static dynamic pressure: $q = \frac{1}{2} \rho V_{\text{TAS}}^2$
   - Compressibility-compensated indicated airspeed (IAS):
     $$V_{\text{IAS}} = \sqrt{\frac{2 q}{\rho_{\text{SL}}}}$$
   - Acoustic line impedance lag modeled via continuous low-pass filter ($\tau = 0.10\,\text{s}$).
   - Authentic Cessna 172 operating airspeed color arcs:
     - White Arc ($40\text{--}85\,\text{kts}$): Flap operating range ($V_{\text{so}}$ to $V_{\text{fe}}$)
     - Green Arc ($48\text{--}129\,\text{kts}$): Normal operating range ($V_{\text{s}}$ to $V_{\text{no}}$)
     - Yellow Arc ($129\text{--}163\,\text{kts}$): Caution range (smooth air only)
     - Red Radial Line ($163\,\text{kts}$): Never exceed speed ($V_{\text{ne}}$)

2. **Attitude Indicator (AI) / Artificial Horizon:**
   - 3-DOF Earth-vertical gyroscopic gimbal response ($\tau = 0.05\,\text{s}$).
   - Celestial sphere projection (sky blue `#3C82F6` and ground brown `#855723`).
   - Pitch ladder bars at $\pm 5^\circ, \pm 10^\circ, \pm 15^\circ, \pm 20^\circ$.
   - Bank index ticks at $10^\circ, 20^\circ, 30^\circ, 45^\circ, 60^\circ$.
   - Gimbal caging and tumbling limits at $\pm 85^\circ$ pitch to model mechanical stops.

3. **Altimeter (Aneroid Barometer):**
   - Tropospheric barometric lapse equation:
     $$h = \frac{T_0}{L} \left[1 - \left(\frac{P_{\text{ambient}}}{P_{\text{kollsman}}}\right)^{\frac{R \cdot L}{g}}\right]$$
   - Adjustable Kollsman subscale setting (default $29.92\,\text{inHg}$, adjustable via pilot input).
   - Static line pneumatic damping ($\tau = 0.15\,\text{s}$).
   - Triple concentric needle gearing:
     - $100\,\text{ft}$ pointer: $360^\circ / 1{,}000\,\text{ft}$
     - $1{,}000\,\text{ft}$ needle: $360^\circ / 10{,}000\,\text{ft}$
     - $10{,}000\,\text{ft}$ needle: $360^\circ / 100{,}000\,\text{ft}$

4. **Turn Coordinator:**
   - $30^\circ$ canted rate gyro equation sensing combined yaw and roll rates:
     $$\omega_{\text{sensed}} = r \cos(30^\circ) + p \sin(30^\circ)$$
   - Standard rate turn calibration ($3.0^\circ/\text{s} = 20.0^\circ$ aircraft symbol roll).
   - Inclinometer curved glass tube with fluid-damped ball:
     $$f_{y,\text{ball}} = g \sin\phi \cos\theta - a_{y,\text{kinematic}}$$
   - Correct physical separation of coordinated turns ($f_y = 0$), slips ($f_y > 0$, ball inside), and skids ($f_y < 0$, ball outside).

5. **Heading Indicator (Directional Gyro / DG):**
   - Vacuum horizontal gyro driving a rotating $360^\circ$ azimuth compass card.
   - Mechanical gyro precession drift modeled at $\approx 3^\circ$ per 15 minutes.
   - Pilot compass alignment control (`'B'`) to slave gyro back to true magnetic heading.
   - Pilot-adjustable heading bug (`','` and `'.'`).

6. **Vertical Speed Indicator (VSI):**
   - Calibrated metering capillary pneumatic delay equation:
     $$\frac{dVSI}{dt} = \frac{V_z - VSI}{\tau_{\text{vsi}}}$$
   - Authentic FAA capillary lag ($\tau_{\text{vsi}} = 2.50\,\text{s}$).
   - Dual logarithmic deflection scale up to $\pm 2{,}000\,\text{fpm}$.

---

### 3.2 Direct3D 12 Procedural Vector Rendering Pipeline

- **Architecture:** `AvionicsRenderer` ([`AvionicsRenderer.hpp`](file:///c:/Users/diego/Documents/Projetos/LokYS/include/LokYS/graphics/AvionicsRenderer.hpp), [`AvionicsRenderer.cpp`](file:///c:/Users/diego/Documents/Projetos/LokYS/src/graphics/AvionicsRenderer.cpp))
- **Geometry Generation:** Procedural 2D triangle-strip primitives (disks, arcs, bezels, needles, ticks, slip ball) uploaded dynamically per frame into a dedicated Direct3D 12 upload buffer.
- **Root Signature & Shaders:** Custom 2D orthographic vertex and pixel shaders with Premultiplied Alpha blending state (`D3D12_BLEND_ONE`, `D3D12_BLEND_INV_SRC_ALPHA`).
- **Layout:** Standard 2x3 Basic-T layout:
  - Top Row: ASI (Left), Attitude Indicator (Center), Altimeter (Right)
  - Bottom Row: Turn Coordinator (Left), Heading Indicator (Center), VSI (Right)
- **Display Modes:** Toggled between full panel overlay and clean flight view via `'H'`.

---

## 4. Deep-Dive: Milestone M13 (glTF 2.0 Binary Loader & Articulation)

- **Spec-Compliant Binary Parser (`GltfModel`):**
  - Reads `.glb` binary containers, parses 12-byte header (`magic=0x46546C67`, `version=2`, `length`), and extracts JSON (`0x4E4F534A`) and BIN (`0x004E4942`) chunks.
- **Zero-Dependency JSON Parser (`JsonParser`):**
  - Custom C++20 streaming recursive-descent parser supporting UTF-8, objects, arrays, numbers, strings, and booleans with zero third-party dependencies.
- **Articulated Scene Graph Kinematics:**
  - 12-node hierarchical structure:
    1. Root Aircraft Frame
    2. Fuselage
    3. Left Wing & Right Wing
    4. Left Aileron & Right Aileron (differential roll deflection $\pm 20^\circ$)
    5. Elevator (pitch deflection $\pm 25^\circ$)
    6. Rudder (yaw deflection $\pm 20^\circ$)
    7. Flaps (staged deflection $0^\circ, 10^\circ, 20^\circ, 30^\circ$)
    8. McCauley 2-Blade Propeller (continuous rotational integrator based on engine RPM)
    9. Cockpit & Canopy
    10. Nose Gear & Main Gear Struts
- **Procedural Model Generator:**
  - `AircraftModelGenerator::GenerateCessna172Glb()` creates `assets/models/cessna172.glb` automatically if missing.

---

## 5. Input Controls Quick Reference

| Key / Control | Function |
| :--- | :--- |
| **W / S** | Elevator Pitch Down / Pitch Up |
| **A / D** | Aileron Roll Left / Roll Right |
| **Q / E** | Rudder Yaw Left / Yaw Right |
| **Shift / Ctrl** | Engine Throttle Increase / Decrease |
| **F / V** | Flaps Deploy (+10°) / Retract (-10°) |
| **Space** | Wheel Brakes On / Off |
| **C** | Camera Perspective Toggle (Cockpit / Chase 3D) |
| **H** | **Analog Six-Pack Avionics Overlay Toggle (M14)** |
| **B** | **Directional Gyro / Compass Alignment (M14)** |
| **[ / ]** | **Altimeter Kollsman Subscale Decrement / Increment (M14)** |
| **, / .** | **Heading Indicator Heading Bug Decrement / Increment (M14)** |
| **M / N** | Fuel Mixture Rich / Lean |
| **P** | Simulation Pause / Resume |
| **O** | Single-Frame Step (while paused) |
| **R** | Reset Flight Simulation to Initial Runway State |

---

## 6. Verification & Test Suite Status

All unit test suites execute natively in **< 0.10s** with 100% pass rate:

```
Test project C:/Users/diego/Documents/Projetos/LokYS/build
    Start 1: AerospaceMathTests
1/7 Test #1: AerospaceMathTests ...............   Passed    0.01 sec
    Start 2: SixDofPhysicsTests
2/7 Test #2: SixDofPhysicsTests ...............   Passed    0.01 sec
    Start 3: AeroInputTests
3/7 Test #3: AeroInputTests ...................   Passed    0.01 sec
    Start 4: GroundReactionsTests
4/7 Test #4: GroundReactionsTests .............   Passed    0.01 sec
    Start 5: PropulsionTests
5/7 Test #5: PropulsionTests ..................   Passed    0.01 sec
    Start 6: GltfModelTests
6/7 Test #6: GltfModelTests ...................   Passed    0.01 sec
    Start 7: SixPackAvionicsTests
7/7 Test #7: SixPackAvionicsTests .............   Passed    0.01 sec

100% tests passed, 0 tests failed out of 7
Total Test time (real) = 0.08 sec
```

### Application Verification Command
```bash
.\build\LokYSFlightSimulator.exe --verify
```
**Output:**
```
LokYS Native Windows Flight Simulator [Milestone M14 Analog Six-Pack Avionics]
============================================================
Detected GPU: NVIDIA GeForce RTX 3050 6GB Laptop GPU
Dedicated VRAM: 6001 MB
Direct3D 12 Feature Probe: PASSED
Fixed-Timestep Simulation Scheduler: 120 Hz OPERATIONAL
6-DOF Rigid-Body Dynamics Engine (RK4): OPERATIONAL
Cessna 172 Aerodynamic Model (NASA CR-2144): OPERATIONAL
Multi-Point Oleo-Pneumatic Strut Suspension & Ground Reactions: OPERATIONAL
Lycoming O-320 & McCauley Propeller Dynamics: OPERATIONAL
Fuel Burn Rate & Mixture Control: OPERATIONAL
Propeller Torque Reaction, P-Factor & Gyroscopic Precession: OPERATIONAL
glTF 2.0 Binary Loader & Articulated Aircraft Kinematics: OPERATIONAL
Analog Six-Pack Avionics Cluster (ASI, AI, ALT, TC, DG, VSI): OPERATIONAL
Pitot-Static & Aneroid Barometric Pressure Simulation: OPERATIONAL
Gyroscopic Gimbal Dynamics & Mechanical Precession Drift: OPERATIONAL
Calibrated Metering Capillary Pneumatic Sensor Lag: OPERATIONAL
Direct3D 12 2D Flight Instrument Cluster Vector Pipeline: OPERATIONAL
============================================================
[M14 ANALOG SIX-PACK AVIONICS VERIFICATION EXIT 0]
```

---

## 7. Next Milestone: M15 (Spatial Audio Subsystem)

The upcoming milestone will implement 3D spatialized audio using native Windows XAudio2 and X3DAudio:
1. **XAudio2 & X3DAudio Pipeline Initialization:** Mastering voice, source voices, 3D audio listener/emitter coordinates.
2. **Dynamic Engine Sound Synthesis:** Synthesized multi-harmonic Lycoming O-320 engine audio pitch-shifted in real time by crankshaft RPM with load-dependent timbre.
3. **Aerodynamic Wind Rush:** Dynamic pressure $q$-driven bandpass filtered noise.
4. **Landing Gear Touchdown & Ground Roll:** Transient tire screech on touchdown and speed-dependent runway rumble.
5. **Perspective-Aware Cockpit Filtering:** Low-pass acoustic attenuation inside the cockpit vs. exterior chase camera.
