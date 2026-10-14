# LokYS Flight Simulator — Project Checkpoint (Milestone M18)

**Timestamp:** 2026-10-14T15:20:00-03:00  
**Current Milestone:** M18 — Atmospheric & Weather Systems  
**Status:** COMPLETED & VERIFIED (All 11 CTest Suites Passing, 0 Warnings, `/W4 /WX /permissive- /utf-8 /std:c++latest`)  
**Target Architecture:** Native Windows x64 (MSVC 19.29 / Visual Studio 2019 BuildTools), Direct3D 12, XAudio2 / X3DAudio, Win32 Overlapped Serial I/O, Pure C++20  
**Design Philosophy:** Zero-Bloat, Zero Third-Party Runtime Dependencies (YSFlight Inspiration)

---

## 1. Executive Summary

Milestone M18 introduces a comprehensive, physically rigorous **Atmospheric & Weather Systems Engine** to the LokYS flight simulator. Built entirely from first principles in modern C++20 without external dependencies, this subsystem bridges the physical environment to the 120 Hz 6-DOF aircraft dynamics, Cessna 172 aerodynamics, propulsion, and analog avionics.

Key capabilities introduced in Milestone M18:
1. **International Standard Atmosphere (ISA) Continuous Lapse Model:** Continuous evaluation of temperature $T(h)$, static pressure $p(h)$, air density $\rho(h)$, speed of sound $a(h)$, Sutherland's dynamic viscosity $\mu(h)$, pressure altitude, and density altitude across the Troposphere ($0$ to $11,000$ m at $L = -0.0065$ K/m) and the isothermal Tropopause ($11,000$ to $20,000$ m at $T_{\text{tropo}} = 216.65$ K). Supports non-standard temperature offsets ($\Delta T_{\text{ISA}}$) and custom barometric altimeter subscale settings (QNH in Pa or inHg).
2. **Logarithmic Planetary Boundary Layer Wind Shear:** Realistically scales horizontal wind velocity according to the atmospheric boundary layer profile $U(z) = U_{\text{ref}} \frac{\ln(z/z_0)}{\ln(z_{\text{ref}}/z_0)}$ near the ground surface ($z \le 150$ m) over airport terrain roughness ($z_0 = 0.05$ m) with smooth cubic Hermite transition to geostrophic aloft winds.
3. **Runway Wind Vector Decomposition:** Automatically resolves instantaneous 3D horizontal wind vectors into runway-aligned headwind/tailwind and crosswind components relative to Runway 36 ($000^\circ$ true).
4. **Stochastic Dryden Turbulence Gust Spectral Filters:** Continuous Dryden filter equations driven by pseudo-random Gaussian white noise generate authentic longitudinal ($u_g$), lateral ($v_g$), and vertical ($w_g$) wind velocity perturbations, scaled by aircraft true airspeed and altitude. Includes localized Gaussian thermal updraft plumes with convective downdraft margins.
5. **Dynamic Aerodynamic Wind Coupling:** Couples environmental winds directly into the rigid body equations of motion:
   $$\mathbf{V}_{\text{rel,Body}} = \mathbf{V}_{\text{aircraft,Body}} - \mathbf{R}_{\text{NED}\to\text{Body}}(\mathbf{V}_{\text{wind,NED}})$$
   This authentic relative velocity feeds the NASA CR-2144 aerodynamic coefficient engine, producing natural crosswind crab angles, directional weathercocking, turbulent airspeed fluctuations, and stall margin changes.
6. **Weather Scenario Presets & Pilot Controls:** Features five distinct presets (`ClearCalm`, `StandardDay`, `CrosswindRunway36`, `GustyTurbulence`, `HotAndHigh`), dynamically switchable in real time via the `'W'` key with live telemetry in the application window title.

---

## 2. Milestone Completion Status (M0 through M18)

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
| **M16** | Radio Navigation Systems | **COMPLETED** | VOR radials, TO/FROM resolution, ILS Localizer & Glideslope, DME radar slant range, 3-marker beacons, 4-column avionics |
| **M17** | Serial Cockpit Hardware Interface | **COMPLETED** | Win32 COM overlapped async I/O, binary packet framing, CRC-16-CCITT, outbound telemetry streaming (~30 Hz), inbound 16-bit ADC control ingestion |
| **M18** | Atmospheric & Weather Systems | **COMPLETED** | International Standard Atmosphere (ISA) continuous lapse model, logarithmic boundary layer wind shear, Runway 36 headwind/crosswind decomposition, Dryden turbulence spectral filter, thermal updraft convection, relative aerodynamic velocity transformation, 5 weather presets, 'W' cycle key |

---

## 3. Deep-Dive: Milestone M18 Architecture

### 3.1 International Standard Atmosphere (ISA) Continuous Lapse Model

Implemented in [`AtmosphereModel`](file:///c:/Users/diego/Documents/Projetos/LokYS/include/LokYS/environment/AtmosphereModel.hpp) and [`AtmosphereModel.cpp`](file:///c:/Users/diego/Documents/Projetos/LokYS/src/environment/AtmosphereModel.cpp):

1. **Troposphere ($0 \le h \le 11,000$ m):**
   - Standard temperature lapse rate: $L = -0.0065$ K/m
   - Ambient temperature with non-standard day offset:
     $$T(h) = T_0 + L\cdot h + \Delta T_{\text{ISA}}$$
   - Barometric pressure equation:
     $$p(h) = P_{\text{QNH}} \cdot \left(\frac{T_0 + L\cdot h}{T_0}\right)^{-\frac{g_0}{L \cdot R}}$$
     where $g_0 = 9.80665\,\text{m/s}^2$ and $R = 287.05287\,\text{J/(kg}\cdot\text{K)}$.
2. **Tropopause ($11,000 < h \le 20,000$ m):**
   - Isothermal temperature: $T(h) = T_{\text{tropo}} + \Delta T_{\text{ISA}} = 216.65\,\text{K} + \Delta T_{\text{ISA}}$
   - Pressure decay:
     $$p(h) = p(11000) \cdot \exp\left(-\frac{g_0 (h - 11000)}{R \cdot T_{\text{tropo}}}\right)$$
3. **Ideal Gas Density & Thermodynamic Properties:**
   - Air density: $\rho(h) = \frac{p(h)}{R \cdot T(h)}$
   - Speed of sound: $a(h) = \sqrt{\gamma \cdot R \cdot T(h)}$, with $\gamma = 1.4$
   - Dynamic viscosity (Sutherland's Law):
     $$\mu(T) = \mu_0 \cdot \left(\frac{T}{T_0}\right)^{3/2} \frac{T_0 + S}{T + S}$$
     where $\mu_0 = 1.7894\times 10^{-5}\,\text{kg/(m}\cdot\text{s})$ and Sutherland's constant $S = 110.4\,\text{K}$.
4. **Altimetry Calculations:**
   - Pressure altitude: $h_p = \frac{T_0}{L} \left[\left(\frac{p}{P_0}\right)^{-\frac{L\cdot R}{g_0}} - 1\right]$
   - Density altitude: $h_\rho = \frac{T_0}{L} \left[\left(\frac{\rho}{\rho_0}\right)^{\frac{1}{\frac{g_0}{L\cdot R} - 1}} - 1\right]$

---

### 3.2 Planetary Boundary Layer Wind Shear & Runway Decomposition

Implemented in [`WindModel`](file:///c:/Users/diego/Documents/Projetos/LokYS/include/LokYS/environment/WindModel.hpp) and [`WindModel.cpp`](file:///c:/Users/diego/Documents/Projetos/LokYS/src/environment/WindModel.cpp):

- **Logarithmic Profile ($z \le z_{\text{shear}} = 150$ m):**
  $$U(z) = U_{\text{ref}} \frac{\ln(z / z_0)}{\ln(z_{\text{ref}} / z_0)}$$
  with $z_{\text{ref}} = 10.0$ m (standard anemometer mast) and $z_0 = 0.05$ m (airport runway / mown grass surface roughness).
- **Aloft Layer Transition ($150\,\text{m} < z < z_{\text{aloft}}$):**
  Uses smooth cubic Hermite blending:
  $$s = \frac{z - z_{\text{shear}}}{z_{\text{aloft}} - z_{\text{shear}}}, \quad \xi = 3s^2 - 2s^3$$
  $$U(z) = (1 - \xi) U_{\text{shear}} + \xi U_{\text{aloft}}$$
- **Directional Ekman Spiral / Aloft Veering:**
  The wind direction transitions smoothly from surface wind direction to aloft wind direction via shortest-path angular interpolation.
- **Runway 36 Vector Decomposition:**
  $$\psi_{\text{rel}} = \psi_{\text{from}} - \psi_{\text{rwy}}$$
  $$\text{Headwind} = U \cos(\psi_{\text{rel}})$$
  $$\text{Crosswind} = U \sin(\psi_{\text{rel}})$$
  where $\psi_{\text{rwy}} = 000^\circ$ (True North). Positive crosswind represents a crosswind blowing from the right.

---

### 3.3 Dryden Turbulence & Thermal Convection Simulation

Implemented in [`WindModel.cpp`](file:///c:/Users/diego/Documents/Projetos/LokYS/src/environment/WindModel.cpp):

- **Dryden Low-Altitude Gust Filter:**
  Maintains internal differential states for gust velocities driven by pseudo-random Gaussian noise $\eta_u, \eta_v, \eta_w \sim \mathcal{N}(0, 1)$:
  - Longitudinal gust:
    $$\dot{u}_g = -\frac{V_{\text{TAS}}}{L_u} u_g + \sigma_u \sqrt{\frac{2 V_{\text{TAS}}}{L_u}} \eta_u$$
  - Lateral gust:
    $$\dot{v}_g = -\frac{V_{\text{TAS}}}{L_v} v_g + \sigma_v \sqrt{\frac{2 V_{\text{TAS}}}{L_v}} \eta_v$$
  - Vertical gust:
    $$\dot{w}_g = -\frac{V_{\text{TAS}}}{L_w} w_g + \sigma_w \sqrt{\frac{2 V_{\text{TAS}}}{L_w}} \eta_w$$
  with scale lengths $L_w = \max(h, 3.0\,\text{m})$, $L_u = L_v = \frac{h}{(0.177 + 0.000823 h)^{1.2}}$, and turbulence intensity $\sigma_w = 0.1 \cdot U_{10}$, $\sigma_u = \sigma_v = \frac{\sigma_w}{(0.177 + 0.000823 h)^{0.4}}$.
- **Thermal Updraft Convection Plume:**
  Simulates localized spatial thermal updrafts modeled as vertical velocity profiles:
  $$w_{\text{thermal}}(r) = W_{\text{peak}} \exp\left(-\frac{r^2}{R_{\text{core}}^2}\right) \cdot \left[1 - 0.25 \frac{r^2}{R_{\text{core}}^2}\right]$$
  producing a strong upward core surrounded by a subtle descending annular downdraft ring.

---

### 3.4 Relative Aerodynamic Velocity Coupling

In [`src/app/main.cpp`](file:///c:/Users/diego/Documents/Projetos/LokYS/src/app/main.cpp), wind is coupled into the aircraft dynamics at every 120 Hz simulation tick:

```cpp
// 1. Advance atmospheric & wind state
environment.Update(state.positionNed, state.attitude, latestAeroOut.trueAirspeedMps, dt);
const auto& atmo = environment.GetAtmosphere();

// 2. Transform NED wind into aircraft Body frame (FRD) and compute relative wind
LokYS::Math::Vec3d relVelBody = environment.ComputeRelativeVelocityBody(state.velocityBody);

// 3. Evaluate aerodynamics with relative wind and dynamic air density & speed of sound
auto aeroOut = aero.Evaluate(relVelBody, state.angularVelocityBody, deflections, atmo.densityKgM3, atmo.speedOfSoundMps);

// 4. Evaluate propulsion with dynamic density and ambient pressure
auto propOut = propulsion.Evaluate(propCtrl, state.velocityBody, state.angularVelocityBody, atmo.densityKgM3, atmo.pressurePa, dt);

// 5. Update avionics pitot-static sensors with exact barometric pressure and density
avionics.Update(state, linearAccelBody, atmo.pressurePa, atmo.densityKgM3, dt);
```

---

### 3.5 Weather Scenario Presets

1. **Clear & Calm (`ClearCalm`):**
   - Wind: $0$ m/s, Turbulence: None, $\Delta T_{\text{ISA}} = 0$ K, $QNH = 101,325$ Pa.
2. **Standard Day (`StandardDay`):**
   - Wind: $8$ kt from $270^\circ$, Light turbulence, $QNH = 101,325$ Pa.
3. **Crosswind Runway 36 (`CrosswindRunway36`):**
   - Wind: $15$ kt from $060^\circ$ (producing $13.0$ kt right crosswind and $7.5$ kt headwind on Runway 36), Moderate boundary layer shear.
4. **Gusty Turbulence (`GustyTurbulence`):**
   - Wind: $20$ kt from $330^\circ$ aloft with severe Dryden turbulence gusts up to $30$ kt and active convective thermal plumes.
5. **Hot & High (`HotAndHigh`):**
   - Wind: $5$ kt from $180^\circ$, $\Delta T_{\text{ISA}} = +20.0$ K ($35^\circ$C at sea level), $QNH = 100,500$ Pa, producing high density altitude ($+2,400$ ft) and decreased engine power and lift margins.

---

## 4. Verification and Validation Results

### 4.1 Automated Unit Test Suite (`AtmosphereWeatherTests`)

7 comprehensive test suites were developed in `tests/unit/AtmosphereWeatherTests.cpp`:

| Test Suite | Conditions Tested | Status |
| :--- | :--- | :--- |
| `TestStandardSeaLevelIsa` | Evaluates $T_0=288.15$ K, $P_0=101,325$ Pa, $\rho_0=1.225$ kg/m$^3$, $a_0=340.29$ m/s at $h=0$ | **PASSED** |
| `TestTroposphereLapseAndTropopause` | Verifies $-6.5$ K/km lapse rate at $5000$ m and isothermal $216.65$ K tropopause behavior at $12,000$ m | **PASSED** |
| `TestNonStandardDayDensityAltitude` | Validates $\Delta T_{\text{ISA}} = +15$ K and $QNH = 99,500$ Pa yielding $\text{DA} > \text{PA}$ by over $1,500$ ft | **PASSED** |
| `TestSpeedOfSoundAndViscosity` | Tests Sutherland's dynamic viscosity and acoustic speed scaling with temperature | **PASSED** |
| `TestLogarithmicBoundaryLayerWindShear`| Verifies $U(10\text{m}) \approx 10$ m/s, $U(2\text{m}) < 8$ m/s, $U(100\text{m}) > 13$ m/s, and aloft free stream | **PASSED** |
| `TestRunwayWindDecomposition` | Validates pure crosswind ($090^\circ$), pure headwind ($360^\circ$), and $060^\circ$ angled resolution on Runway 36 | **PASSED** |
| `TestDrydenTurbulenceAndRelativeVelocity` | Tests continuous Dryden stochastic perturbation, spatial thermal updrafts, and relative wind transformation | **PASSED** |

### 4.2 CTest Full Suite Regression

All 11 automated test suites in the LokYS project pass with 100% success:

```
Test project C:/Users/diego/Documents/Projetos/LokYS/build
      Start  1: AerospaceMathTests
 1/11 Test  #1: AerospaceMathTests ...............   Passed    0.01 sec
      Start  2: SixDofPhysicsTests
 2/11 Test  #2: SixDofPhysicsTests ...............   Passed    0.01 sec
      Start  3: AeroInputTests
 3/11 Test  #3: AeroInputTests ...................   Passed    0.01 sec
      Start  4: GroundReactionsTests
 4/11 Test  #4: GroundReactionsTests .............   Passed    0.01 sec
      Start  5: PropulsionTests
 5/11 Test  #5: PropulsionTests ..................   Passed    0.01 sec
      Start  6: GltfModelTests
 6/11 Test  #6: GltfModelTests ...................   Passed    0.01 sec
      Start  7: SixPackAvionicsTests
 7/11 Test  #7: SixPackAvionicsTests .............   Passed    0.01 sec
      Start  8: SpatialAudioTests
 8/11 Test  #8: SpatialAudioTests ................   Passed    0.05 sec
      Start  9: RadioNavTests
 9/11 Test  #9: RadioNavTests ....................   Passed    0.01 sec
      Start 10: SerialInterfaceTests
10/11 Test #10: SerialInterfaceTests .............   Passed    0.01 sec
      Start 11: AtmosphereWeatherTests
11/11 Test #11: AtmosphereWeatherTests ...........   Passed    0.01 sec

100% tests passed, 0 tests failed out of 11
Total Test time (real) = 0.17 sec
```

### 4.3 CLI Verification Probe (`--verify`)

Executed `LokYSFlightSimulator.exe --verify`:

```
LokYS Native Windows Flight Simulator [Milestone M18 Atmospheric & Weather Systems]
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
Serial Cockpit Hardware Interface (Win32 COM Overlapped I/O & Microcontroller Ingestion): OPERATIONAL
Atmospheric & Weather Systems (ISA Lapse, Wind Shear, Dryden Turbulence, Runway Crosswinds): OPERATIONAL
============================================================
[M18 ATMOSPHERIC & WEATHER SYSTEMS VERIFICATION EXIT 0]
```

---

## 5. Next Steps

With Milestone M18 completed, the next milestone on the roadmap is:

**Milestone M19: Graphics Optimization & Visual FX**
- Directional Cascaded Shadow Maps (CSM) for direct sunlight shadowing across airframe, cockpit, and ground.
- Distance-based atmospheric fog and Rayleigh/Mie haze scattering driven by atmospheric moisture and density.
- Terrain quadtree continuous Level of Detail (LOD) and frustum culling.
- GPU instanced runway lights (PAPI, edge lights, centerline, threshold strobes) and visual weather effects (precipitation particles).
