# LokYS Flight Simulator — Project Checkpoint (Milestone M15)

**Timestamp:** 2026-10-14T13:58:00-03:00  
**Current Milestone:** M15 — Spatial Audio Subsystem  
**Status:** COMPLETED & VERIFIED (All 8 Test Suites Passing, 0 Warnings, /W4 /WX /permissive-)  
**Target Architecture:** Native Windows x64 (MSVC 19.29 / Visual Studio 2019 BuildTools), Direct3D 12, XAudio2 / X3DAudio, Pure C++20 (`/std:c++latest`)  
**Design Philosophy:** Zero-Bloat, Zero Third-Party Runtime Dependencies (YSFlight Inspiration)

---

## 1. Executive Summary

Milestone M15 introduces a high-performance native Windows 3D spatial audio engine built strictly upon **XAudio2** and **X3DAudio**, completely bypassing bloated third-party audio middleware (FMOD, Wwise, OpenAL). The subsystem generates authentic, continuous procedural audio for the aircraft's Lycoming O-320 piston engine, McCauley propeller, aerodynamic boundary layer wind rush, runway rolling texture, and touchdown tire screech.

All sounds are synthesized procedurally into memory with zero disk reads during the simulation loop. The audio processing graph runs asynchronously on dedicated high-priority OS audio threads, completely decoupled from the 120 Hz fixed-timestep physics scheduler and the Direct3D 12 render loop.

---

## 2. Milestone Completion Status (M0 through M15)

| Milestone | Subsystem | Status | Key Deliverables |
| :--- | :--- | :--- | :--- |
| **M0** | Requirements & Baseline | **COMPLETED** | `REQUIREMENTS.md`, `ARCHITECTURE.md`, `DECISIONS.md`, `BUILD.md` |
| **M1** | Toolchain & Build | **COMPLETED** | CMake 3.20+, Ninja, MSVC `/W4 /WX /permissive-`, zero warning tolerance |
| **M2** | Win32 Platform | **COMPLETED** | Native Win32 window, message pump, raw OS handles, High-DPI Per-Monitor v2 awareness |
| **M3** | Direct3D 12 Core | **COMPLETED** | DXGI 1.6 factory, D3D12 device, command queues, swapchain, fence synchronization |
| **M4** | GPU Adapter Enumeration | **COMPLETED** | VRAM budget query, adapter feature levels (RTX 3050 Laptop / Intel UHD / RDNA2 targets) |
| **M5** | Aerospace Math Engine | **COMPLETED** | Quaternions, 3D vectors, 3x3 / 4x4 matrices, NED/Body/Wind frames, SI $\leftrightarrow$ Aviation units |
| **M6** | Simulation Scheduler | **COMPLETED** | Decoupled 120 Hz fixed-timestep accumulator, triple-buffered snapshot state |
| **M7** | 6-DOF Rigid-Body Dynamics | **COMPLETED** | Runge-Kutta 4th Order (RK4) integrator, inertia tensors, gravitational dynamics |
| **M8** | Aerodynamics Subsystem | **COMPLETED** | Cessna 172 polar curves (NASA CR-2144), stall breakaway hysteresis, control authority |
| **M9** | Input Engine & Hardware | **COMPLETED** | DirectInput/Win32 joystick polling, quadratic curves, keyboard flight controls |
| **M10** | Integrated Flight Prototype | **COMPLETED** | 3D rendering pipeline, runway & terrain, Cockpit/Chase cameras, takeoff/landing loop |
| **M11** | Ground Reactions & Struts | **COMPLETED** | Multi-point oleo struts, progressive stiffness, hydraulic damping, tire scrub friction |
| **M12** | Propulsion & Fuel | **COMPLETED** | Lycoming O-320 engine, McCauley propeller polars, rotational integrator, P-factor, fuel burn |
| **M13** | glTF 2.0 Aircraft Loading | **COMPLETED** | Spec-compliant `.glb` parser, zero-dep JSON parser, 12-node articulated scene graph, procedural model generator |
| **M14** | Analog Six-Pack Avionics | **COMPLETED** | 6 physical flight instruments (ASI, AI, ALT, TC, DG, VSI), sensor lag, D3D12 vector dials |
| **M15** | Spatial Audio Subsystem | **COMPLETED** | Native XAudio2/X3DAudio engine, Lycoming O-320 harmonic synthesis, aerodynamic wind, tire screech, cockpit low-pass filtering |

---

## 3. Deep-Dive: Milestone M15 (Spatial Audio Subsystem)

### 3.1 Architectural Principles & Low-Latency Threading Model

- **Zero Third-Party Dependencies:** Implemented directly on the Windows 10/11 SDK core audio platform (`xaudio2.lib`, `<xaudio2.h>`, `<x3daudio.h>`).
- **PIMPL Encapsulation:** The [`AudioSystem`](file:///c:/Users/diego/Documents/Projetos/LokYS/include/LokYS/audio/AudioSystem.hpp) class encapsulates all COM interfaces and platform audio headers via `std::unique_ptr<Impl>`, preventing header leakage and Windows macro contamination.
- **Asynchronous Processing:** XAudio2 manages hardware buffer mixing and streaming on internal multimedia real-time threads (MMCSS). Audio parameters (pitch, volume, filter cutoffs, spatial matrix) are pushed per-frame via non-blocking lockless updates.
- **Headless & Fallback Grace:** If the host environment lacks an active audio endpoint (e.g., automated CI servers, virtual test machines), `Initialize()` cleanly detects the hardware state, flags `m_available = false`, and allows all simulation and unit tests to run deterministically without throwing or crashing.

---

### 3.2 Sound Synthesis & Physical Modeling

#### 1. Lycoming O-320 Engine Synthesis & Dynamic Pitch Shifting
- **Acoustic Waveform Generation:**
  The 4-cylinder, 4-stroke boxer engine produces 2 power strokes per crankshaft revolution. At the 1200 RPM baseline ($20\,\text{Hz}$ rotation), the firing fundamental is $40\,\text{Hz}$.
  A seamless 1.0-second loop (44,100 samples) is synthesized mathematically from:
  - 8-harmonic Fourier series modeling the asymmetric blowdown pressure wave:
    $$s_{\text{pulse}}(t) = \sum_{k=1}^8 A_k \sin(2\pi \cdot k \cdot 40 \cdot t + \phi_k)$$
  - Sub-harmonic rotation pulse ($20\,\text{Hz}$) for crankshaft unbalance and intake manifold runner asymmetry.
  - Propeller blade passage slap ($40\,\text{Hz}$ sharp wavefront for the 2-blade McCauley fixed-pitch prop).
- **Pitch Modulation:**
  Real-time frequency ratio supplied to the voice resampler:
  $$\text{freqRatio} = \frac{\text{RPM}}{1200.0}$$
  - 600 RPM (Idle) $\rightarrow 0.50$
  - 2400 RPM (Cruise) $\rightarrow 2.00$
  - 2700 RPM (Redline) $\rightarrow 2.25$
- **Throttle Load Timbre:**
  Volume dynamically scales with both RPM and throttle plate opening ($V \propto \text{RPM} \times (1.0 + 0.25 \cdot \text{throttle})$).

#### 2. Aerodynamic Boundary Layer Wind Rush
- **Noise Generator:** Seamless pseudo-random pink noise generated via 3-pole digital integration (Kellet filter approximation) windowed for zero-click phase transition.
- **Dynamic Pressure Scaling:**
  $$q = \frac{1}{2} \rho V_{\text{TAS}}^2$$
  Volume scales monotonically with dynamic pressure relative to standard cruise dynamic pressure ($q_{\text{ref}} \approx 1900\,\text{Pa}$):
  $$V_{\text{wind}} = \left(\frac{q}{q_{\text{ref}}}\right)^{1.15}$$
- **Acoustic Airspeed Filtering:**
  Cutoff frequency rises from a low-frequency flutter ($500\,\text{Hz}$) when stationary up to a sharp tearing rush ($3500\,\text{Hz}\text{--}6000\,\text{Hz}$) at cruise and dive speeds:
  $$f_{\text{cutoff}} = 500 + 55 \cdot V_{\text{TAS}}\,\text{Hz}$$

#### 3. Ground Contact Reactions (Runway Rumble & Touchdown Screech)
- **Runway Rumble:** Continuous low-frequency structural vibration ($50\text{--}110\,\text{Hz}$) active only while one or more oleo struts are compressed on the asphalt. Volume and frequency scale with ground rolling velocity ($V_{\text{ground}}$). Strictly muted ($V = 0$) when airborne.
- **Touchdown Tire Screech:** Triggered transiently upon transition from airborne state to ground contact with forward ground speed $V_{\text{ground}} > 10\,\text{m/s}$. Synthesizes a downward tire-friction chirp ($2400\,\text{Hz} \rightarrow 1800\,\text{Hz}$) with immediate 10 ms attack and exponential decay over 350 ms.

#### 4. Perspective-Aware Cockpit Acoustic Filtering
- **Cockpit Interior (`CameraMode::Cockpit`):**
  The aircraft aluminium fuselage, windshield, and cabin upholstery attenuate high-frequency acoustic energy. The engine and wind voices engage a built-in XAudio2 low-pass filter with an $1100\,\text{Hz}$ cutoff, eliminating engine rasp and open-air turbulence.
- **Chase Camera (`CameraMode::Chase`):**
  Filter cutoff opens up to $8000\,\text{Hz}$ for uninhibited open-air propeller slap, exhaust crackle, and dynamic 3D spatial panning with Doppler shift.

---

### 3.3 3D Spatial Calculation & Channel Mapping

In Chase view, X3DAudio performs real-time geometric calculations between the camera listener and the aircraft emitter:
- **Listener:** Position, Forward, and Up vectors synchronized with the active camera.
- **Emitter:** Placed at aircraft center of mass and engine firewall.
- **DSP Settings:** Computes speaker channel matrices (`pMatrixCoefficients`) for 2.0 Stereo, 5.1, or 7.1 surround sound topologies and calculates velocity-dependent Doppler pitch multipliers ($0.5\text{--}2.0\times$).

---

## 4. Verification & Test Suite Status

All 8 automated unit test suites execute natively in **< 0.15s** with a 100% pass rate:

```
Test project C:/Users/diego/Documents/Projetos/LokYS/build
    Start 1: AerospaceMathTests
1/8 Test #1: AerospaceMathTests ...............   Passed    0.01 sec
    Start 2: SixDofPhysicsTests
2/8 Test #2: SixDofPhysicsTests ...............   Passed    0.01 sec
    Start 3: AeroInputTests
3/8 Test #3: AeroInputTests ...................   Passed    0.01 sec
    Start 4: GroundReactionsTests
4/8 Test #4: GroundReactionsTests .............   Passed    0.01 sec
    Start 5: PropulsionTests
5/8 Test #5: PropulsionTests ..................   Passed    0.01 sec
    Start 6: GltfModelTests
6/8 Test #6: GltfModelTests ...................   Passed    0.01 sec
    Start 7: SixPackAvionicsTests
7/8 Test #7: SixPackAvionicsTests .............   Passed    0.01 sec
    Start 8: SpatialAudioTests
8/8 Test #8: SpatialAudioTests ................   Passed    0.05 sec

100% tests passed, 0 tests failed out of 8
Total Test time (real) = 0.15 sec
```

### Application Verification Probe
```bash
.\build\LokYSFlightSimulator.exe --verify
```
**Output:**
```
LokYS Native Windows Flight Simulator [Milestone M15 Spatial Audio Subsystem]
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
XAudio2 & X3DAudio Spatial Sound Subsystem (Engine, Wind, Ground): OPERATIONAL
============================================================
[M15 SPATIAL AUDIO SUBSYSTEM VERIFICATION EXIT 0]
```

---

## 5. Input Controls Quick Reference

| Key / Control | Function |
| :--- | :--- |
| **W / S** | Elevator Pitch Down / Pitch Up |
| **A / D** | Aileron Roll Left / Roll Right |
| **Q / E** | Rudder Yaw Left / Yaw Right |
| **Shift / Ctrl** | Engine Throttle Increase / Decrease (Alters Engine Pitch & Volume) |
| **F / V** | Flaps Deploy (+10°) / Retract (-10°) |
| **Space** | Wheel Brakes On / Off |
| **C** | Camera Perspective Toggle (**Switches Cockpit Filtered $\leftrightarrow$ Chase 3D Audio**) |
| **H** | Analog Six-Pack Avionics Overlay Toggle |
| **B** | Directional Gyro / Compass Alignment |
| **[ / ]** | Altimeter Kollsman Subscale Decrement / Increment |
| **, / .** | Heading Indicator Heading Bug Decrement / Increment |
| **M / N** | Fuel Mixture Rich / Lean |
| **P** | Simulation Pause / Resume |
| **O** | Single-Frame Step (while paused) |
| **R** | Reset Flight Simulation (Runway $\leftrightarrow$ 3000ft Cruise) |

---

## 6. Next Milestone: M16 (Radio Navigation Systems)

According to [`ROADMAP.md`](file:///c:/Users/diego/Documents/Projetos/LokYS/docs/ROADMAP.md#L21), the upcoming milestone will implement navigational radio facilities:
1. **Navigational Facility Database:** VOR stations, DME transponders, and ILS runway localizer/glideslope transmitters with geographic coordinates and carrier frequencies.
2. **NAV1 / NAV2 Receiver Avionics:** VHF frequency tuning ($108.00\text{--}117.95\,\text{MHz}$), Morse code audio ident, signal reception line-of-sight cone and distance limits.
3. **Omni Bearing Selector (OBS) & Course Deviation Indicator (CDI):** VOR radial selection knob, TO/FROM flag resolution, angular deflection needle ($\pm 10^\circ$).
4. **Distance Measuring Equipment (DME):** Slant-range radar distance ($s = \sqrt{d_{\text{horiz}}^2 + \Delta h^2}$) and ground speed readout.
5. **Instrument Landing System (ILS):** Localizer lateral beam needle ($\pm 2.5^\circ$) and Glideslope vertical descent beam needle ($\pm 0.7^\circ$ centered on $3.0^\circ$ descent path).
