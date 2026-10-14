# LokYS Flight Simulator — Project Checkpoint (Milestone M16)

**Timestamp:** 2026-10-14T14:45:00-03:00  
**Current Milestone:** M16 — Radio Navigation Systems (VOR, ILS, DME, Markers)  
**Status:** COMPLETED & VERIFIED (All 9 CTest Suites Passing, 0 Warnings, `/W4 /WX /permissive-`)  
**Target Architecture:** Native Windows x64 (MSVC 19.29 / Visual Studio 2019 BuildTools), Direct3D 12, XAudio2 / X3DAudio, Pure C++20 (`/std:c++latest`)  
**Design Philosophy:** Zero-Bloat, Zero Third-Party Runtime Dependencies (YSFlight Inspiration)

---

## 1. Executive Summary

Milestone M16 introduces a fully functional, mathematically rigorous **VHF Radio Navigation & Precision Instrument Landing Subsystem** for the LokYS flight simulator. Built entirely from scratch with zero external dependencies, this subsystem models true aeronautical radio wave propagation, ground navigational facilities, dual cockpit radio receivers, electromechanical Course Deviation Indicators (CDI), Distance Measuring Equipment (DME), and spatial approach marker beacons.

The system seamlessly integrates with the 120 Hz fixed-timestep physics engine and the Direct3D 12 avionics vector pipeline, expanding the flight deck from the classic analog Six-Pack to an 8-instrument, 4-column balanced dashboard layout.

---

## 2. Milestone Completion Status (M0 through M16)

| Milestone | Subsystem | Status | Key Deliverables |
| :--- | :--- | :--- | :--- |
| **M0** | Requirements & Baseline | **COMPLETED** | `REQUIREMENTS.md`, `ARCHITECTURE.md`, `DECISIONS.md`, `BUILD.md` |
| **M1** | Toolchain & Build | **COMPLETED** | CMake 3.20+, Ninja, MSVC `/W4 /WX /permissive-`, zero warning tolerance |
| **M2** | Win32 Platform | **COMPLETED** | Native Win32 window, message pump, raw OS handles, High-DPI awareness |
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
| **M13** | glTF 2.0 Aircraft Loading | **COMPLETED** | Spec-compliant `.glb` parser, zero-dep JSON parser, 12-node articulated scene graph |
| **M14** | Analog Six-Pack Avionics | **COMPLETED** | 6 physical flight instruments (ASI, AI, ALT, TC, DG, VSI), sensor lag, D3D12 vector dials |
| **M15** | Spatial Audio Subsystem | **COMPLETED** | Native XAudio2/X3DAudio engine, Lycoming O-320 harmonic synthesis, aerodynamic wind, tire screech |
| **M16** | Radio Navigation Systems | **COMPLETED** | VOR radials, TO/FROM resolution, ILS Localizer & 3.0° Glideslope, DME radar slant range & closure rate, 3-marker beacons, 4-column D3D12 instrument panel |

---

## 3. Deep-Dive: Milestone M16 (Radio Navigation Systems)

### 3.1 Navigational Facility Database & Radio Horizon Propagation

The [`NavDatabase`](file:///c:/Users/diego/Documents/Projetos/LokYS/include/LokYS/navigation/NavDatabase.hpp) encapsulates airport and enroute navigation aids with geometric coordinates in the local NED navigation frame.

#### Line-of-Sight VHF Radio Horizon
VHF signals (108.00–117.95 MHz) are strictly limited by earth curvature. LokYS computes maximum theoretical line-of-sight distance using standard tropospheric refraction:
$$d_{\text{max}} = 1.23 \cdot \left(\sqrt{h_{\text{aircraft}}} + \sqrt{h_{\text{station}}}\right)\quad [\text{NM}]$$
where $h$ is elevation in feet above ground level. Signals beyond this horizon or beyond nominal station service volume (40 NM for terminal VORs, 25 NM for localizers) return a zero signal strength condition, causing the receiver to park the CDI needles at zero and raise the red NAV warning flag.

#### Regional Facility Catalog
- **`LOK` VOR/DME (114.50 MHz):** Positioned at $(X = 0, Y = 0, Z = 0)$ m (Runway 36 threshold center), $0.0^\circ$ magnetic variation, 40 NM range.
- **`ILOK` ILS Runway 36 (109.50 MHz):**
  - **Localizer Antenna:** $X = +1100.0$ m, $Y = 0.0$ m (100 m past departure end of runway), aligned to True Heading $000.0^\circ$, 25 NM service volume.
  - **Glideslope Antenna:** $X = -700.0$ m, $Y = +75.0$ m (300 m past South landing threshold, offset 75 m East), calibrated to standard $3.0^\circ$ descent angle, 10 NM service volume.
  - **DME Transponder:** Paired to runway threshold $(X = -1000.0$ m, $Y = 0.0$ m).
- **`PAO` VOR/DME (115.70 MHz):** Regional cross-radial station at $(X = -15000.0, Y = -12000.0, Z = -10.0)$ m.
- **`SJC` VOR/DME (114.10 MHz):** Regional cross-radial station at $(X = -25000.0, Y = +18000.0, Z = -15.0)$ m.
- **Approach Marker Beacons (Runway 36 Extended Centerline):**
  - **Outer Marker (OM):** $X = -7200.0$ m (3.9 NM from threshold), 400 Hz tone, Blue lamp.
  - **Middle Marker (MM):** $X = -1050.0$ m (0.57 NM from threshold), 1300 Hz tone, Amber lamp.
  - **Inner Marker (IM):** $X = -300.0$ m (Decision height / threshold vicinity), 3000 Hz tone, White lamp.

---

### 3.2 Dual Receiver Architecture & Pilot Controls

The [`RadioNavSystem`](file:///c:/Users/diego/Documents/Projetos/LokYS/include/LokYS/navigation/RadioNavSystem.hpp) models dual independent VHF receivers (NAV1 and NAV2) with standard general aviation controls:

- **Active / Standby Flip-Flop:** Pilot toggles active frequency with the `'T'` key.
- **Standby Frequency Tuning:** Pilot increments/decrements standby frequency in 50 kHz steps (108.00–117.95 MHz) with `'U'` and `'I'` keys.
- **Omni Bearing Selector (OBS):** Pilot rotates the azimuth card in $1.0^\circ$ steps with `'K'` and `'L'` keys.
- **Morse Audio Ident:** Pilot toggles station identification Morse audio with the `'Y'` key.

---

### 3.3 Instrument Calibration & Mathematical Algorithms

#### 1. VOR Radial & TO/FROM Resolution
For aircraft at NED position $\mathbf{p}_{\text{ac}}$ and station at $\mathbf{p}_{\text{vor}}$:
$$\theta_{\text{true}} = \operatorname{atan2}(y_{\text{ac}} - y_{\text{vor}}, x_{\text{ac}} - x_{\text{vor}}) \cdot \frac{180}{\pi}$$
$$\text{Radial}_{\text{mag}} = (\theta_{\text{true}} - \text{Var}_{\text{mag}}) \pmod{360^\circ}$$

Angular deviation relative to selected OBS course:
$$\Delta\theta = \text{Normalize180}(\text{Radial} - \text{OBS})$$

TO/FROM logic with an $8^\circ$ cone of ambiguity:
- If $|\Delta\theta| < 82^\circ \implies$ **FROM** flag active.
- If $|\Delta\theta| > 98^\circ \implies$ **TO** flag active.
- If $82^\circ \le |\Delta\theta| \le 98^\circ \implies$ **CONE OF AMBIGUITY** (Both TO/FROM drop; red NAV flag raises).

#### 2. CDI Deviation Scaling
Standard VOR sensitivity: **$1\text{ dot} = 2.0^\circ$ angular error** (full scale $\pm 5\text{ dots} = \pm 10.0^\circ$).
$$\text{Dots}_{\text{VOR}} = -\frac{\text{Deviation}^\circ}{2.0^\circ}$$

#### 3. ILS Localizer Deviation
Localizer sensitivity is 4 times sharper than VOR: **$1\text{ dot} = 0.5^\circ$ angular error** (full scale $\pm 5\text{ dots} = \pm 2.5^\circ$).
$$\Delta\theta_{\text{loc}} = \operatorname{atan2}(\text{crossTrack}, \text{alongTrack}) \cdot \frac{180}{\pi}$$
$$\text{Dots}_{\text{LOC}} = \frac{\Delta\theta_{\text{loc}}^\circ}{0.5^\circ}$$
Positive dots command fly-right; negative dots command fly-left toward runway centerline.

#### 4. ILS Glideslope Deviation
Glideslope sensitivity: **$1\text{ dot} = 0.14^\circ$ angular error** relative to the $3.0^\circ$ nominal glide path (full scale $\pm 5\text{ dots} = \pm 0.70^\circ$).
$$\gamma_{\text{actual}} = \operatorname{atan2}(h_{\text{antenna}}, d_{\text{horiz}}) \cdot \frac{180}{\pi}$$
$$\Delta\gamma = \gamma_{\text{actual}} - 3.0^\circ$$
$$\text{Dots}_{\text{GS}} = -\frac{\Delta\gamma^\circ}{0.14^\circ}$$
Above path ($\Delta\gamma > 0$) yields negative dots (fly-down command); below path yields positive dots (fly-up command).

#### 5. Distance Measuring Equipment (DME)
Computes true radar slant range:
$$R_{\text{slant}} = \frac{\sqrt{(x_{\text{ac}} - x_{\text{dme}})^2 + (y_{\text{ac}} - y_{\text{dme}})^2 + (z_{\text{ac}} - z_{\text{dme}})^2}}{1852.0}\quad [\text{NM}]$$

Closing ground speed and time-to-station (TTS):
$$V_{\text{close}} = -\frac{dR_{\text{slant}}}{dt} \cdot 3600\quad [\text{knots}]$$
$$\text{TTS} = \frac{R_{\text{slant}}}{V_{\text{close}}} \cdot 60\quad [\text{minutes}]$$

---

### 3.4 Direct3D 12 2D Vector Instrumentation Cluster

The [`AvionicsRenderer`](file:///c:/Users/diego/Documents/Projetos/LokYS/include/LokYS/graphics/AvionicsRenderer.hpp) expands the lower dashboard into a balanced 4-column by 2-row layout:

```
+--------------------+--------------------+--------------------+--------------------+
|  AIRSPEED (ASI)    |   ATTITUDE (AI)    |  ALTIMETER (ALT)   |     CDI / VOR      |
+--------------------+--------------------+--------------------+--------------------+
|  TURN COORD (TC)   |   DIR GYRO (DG)    |    VERT SPD (VSI)  |  RADIO / DME BOX   |
+--------------------+--------------------+--------------------+--------------------+
```

- **Instrument 7 (CDI):**
  - Rotating OBS azimuth card with 30° radial numerals and 5° tick marks.
  - 5-dot horizontal localizer/VOR scale and 5-dot vertical glideslope scale.
  - High-visibility vertical CDI needle and horizontal Glideslope needle.
  - Electromechanical TO and FROM triangular indicator flags.
  - Prominent red Barber-pole NAV and GS warning flags when out of reception or frequency mistuned.
- **Instrument 8 (Radio & DME Digital Panel):**
  - Twin amber active (`114.50`) and cyan standby (`109.50`) frequency LED readouts.
  - Real-time station identifier banner (e.g., `LOK` or `ILOK`).
  - DME radar digital display: Slant distance (`NM`), Ground speed (`KT`), and Time-To-Station (`MIN`).
  - Active Marker Beacon annunciator lamps: Blue `OM`, Amber `MM`, and White `IM`.

---

## 4. Automated Test Verification & Metrics

The test suite [`RadioNavTests.cpp`](file:///c:/Users/diego/Documents/Projetos/LokYS/tests/unit/RadioNavTests.cpp) verifies all mathematical and physical calculations across 7 suites:

1. `TestVorRadialCalculations`: North, East, South, West bearings and magnetic variation offsets.
2. `TestVorToFromAndAmbiguity`: Symmetrical TO/FROM resolution and 8° cone of ambiguity.
3. `TestCdiDeflectionScaling`: 1-dot ($2^\circ$), 5-dot ($\pm 10^\circ$), and saturation clipping.
4. `TestIlsLocalizerAndGlideslope`: 4x localizer sensitivity ($0.5^\circ/\text{dot}$) and $3.0^\circ$ glideslope error ($0.14^\circ/\text{dot}$).
5. `TestDmeSlantRange`: Direct 3D Pythagorean slant range, altitude bias, and horizon cutoff.
6. `TestMarkerBeacons`: Spatial cone containment for OM ($7200$ m), MM ($1050$ m), and IM ($300$ m).
7. `TestRadioNavSystemIntegration`: Complete end-to-end receiver frequency swap, OBS rotation, and database query.

### Test Execution Results
```
LokYS Native Windows Flight Simulator — Radio Nav Unit Tests
============================================================
[RUNNING] TestVorRadialCalculations...
[PASSED] TestVorRadialCalculations
[RUNNING] TestVorToFromAndAmbiguity...
[PASSED] TestVorToFromAndAmbiguity
[RUNNING] TestCdiDeflectionScaling...
[PASSED] TestCdiDeflectionScaling
[RUNNING] TestIlsLocalizerAndGlideslope...
[PASSED] TestIlsLocalizerAndGlideslope
[RUNNING] TestDmeSlantRange...
[PASSED] TestDmeSlantRange
[RUNNING] TestMarkerBeacons...
[PASSED] TestMarkerBeacons
[RUNNING] TestRadioNavSystemIntegration...
[PASSED] TestRadioNavSystemIntegration
============================================================
[ALL 7 RADIO NAVIGATION TEST SUITES PASSED (100%)]
```

### Full CTest Test Results
```
100% tests passed, 0 tests failed out of 9

Total Test time (real) =   0.24 sec
1/9 Test #1: AerospaceMathTests ...............   Passed    0.01 sec
2/9 Test #2: SixDofPhysicsTests ...............   Passed    0.01 sec
3/9 Test #3: AeroInputTests ...................   Passed    0.07 sec
4/9 Test #4: GroundReactionsTests .............   Passed    0.01 sec
5/9 Test #5: PropulsionTests ..................   Passed    0.01 sec
6/9 Test #6: GltfModelTests ...................   Passed    0.01 sec
7/9 Test #7: SixPackAvionicsTests .............   Passed    0.01 sec
8/9 Test #8: SpatialAudioTests ................   Passed    0.07 sec
9/9 Test #9: RadioNavTests ....................   Passed    0.01 sec
```

### Verification Probe Exit
```
LokYS Native Windows Flight Simulator [Milestone M16 Radio Navigation Systems]
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
VHF Radio Navigation & Instrument Landing Subsystem (VOR, ILS, DME, Markers): OPERATIONAL
============================================================
[M16 RADIO NAVIGATION SYSTEMS VERIFICATION EXIT 0]
ExitCode=0
```

---

## 5. Next Steps

With Milestone M16 fully completed and verified, the next milestone in sequence is:
- **Milestone M17: Serial Cockpit Hardware Interface:**
  - Asynchronous Win32 COM-port serial communication worker thread (`Overlapped I/O`).
  - Serial frame packet protocol with checksum validation for microcontrollers (Arduino / STM32 / Teensy).
  - Telemetry streaming out (airspeed, altitude, pitch/roll, heading, RPM, VOR/ILS needle positions).
  - Hardware switch and analog potentiometer input ingestion into `InputSystem`.
