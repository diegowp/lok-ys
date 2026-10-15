# Project Roadmap: Advanced Aerospace Physics & Multi-Aircraft Flight Dynamics

**Target System:** LokYS Native C++20 Flight Simulator  
**Protocol:** TOPA (Token-Optimized Aerospace Physics Architecture)  
**Baseline Compatibility:** MSVC x64 `/W4 /WX /permissive- /std:c++20`, Zero Regression across 17 Test Suites  

---

## 1. Executive Overview & Physical Objectives

This roadmap establishes the progressive implementation plan for upgrading the LokYS simulation core from linearized baseline aerodynamics to a first-principles, empirical aerospace physics engine. The model treats each aircraft archetype as an aerodynamically and mechanically distinct vehicle grounded in published flight test data (NASA CR-2144, manufacturer POHs, NACA wind tunnel polars, and Roskam flight dynamics).

```
===================================================================================================================
AIRCRAFT ARCHETYPE MATRIX & EMPIRICAL BASELINE
===================================================================================================================
Aircraft Archetype          Classification              Aero Reference Baseline             Key Physical Phenotypes
-------------------------------------------------------------------------------------------------------------------
Cessna 172SP Skyhawk        Single-Engine Piston (SEP)  NASA CR-2144 / Roskam Vol. VI       NACA 2412, Fixed Tricycle,
                            High-Wing GA Trainer        Lycoming O-320 / IO-360             Positive Dihedral (+1.73 deg)
-------------------------------------------------------------------------------------------------------------------
Beechcraft Baron 58         Multi-Engine Piston (MEP)   Beechcraft 58 POH / NACA 23016      NACA 23016.5/23010.5, Low Wing,
                            Twin High-Performance       2x Continental IO-550-C (600 hp)    Twin CSP, Critical Engine, Vmc
-------------------------------------------------------------------------------------------------------------------
Extra 330SC                 Unlimited Aerobatic         Walter Extra Specs / NACA 0012      NACA 0012 Symmetrical Airfoil,
                            Monoplane (+/-10G)          Lycoming AEIO-580-B1A (315 hp)      Taildragger, Zero Dihedral, CSP
===================================================================================================================
```

---

## 2. Phase-by-Phase Execution Roadmap

| Phase | Designation | Key Objectives & Deliverables | Acceptance Criteria | Status |
| :--- | :--- | :--- | :--- | :--- |
| **F1** | 6-DOF RK4 State-Dependent Integrator & Dynamic CG | • Evaluate forces per RK4 sub-step ($k_1..k_4$)<br>• Dynamic CG translation $\mathbf{r}_{cg}(t)$ as fuel drains<br>• Real-time parallel-axis tensor update $\mathbf{I}(t)$ | • Free-flight energy drift $< 10^{-5}$/min<br>• CG shift alters pitch trim equilibrium | **COMPLETED** |
| **F2** | Ground Effect, Propwash & Non-Linear Aerodynamics | • McCormick ground effect reduction $\phi(h/b)$<br>• Actuator disk propwash over empennage ($q_{tail}$)<br>• Coupled asymmetric post-stall & spin dynamics<br>• Prandtl-Glauert compressibility correction | • Measurable landing float flare increment<br>• Extra 330 prop-hang authority at $V=0$<br>• Asymmetric wing-drop on stall with rudder | **COMPLETED** |
| **F3** | Constant-Speed Governor & Discrete Multi-Engine | • Centrifugal CSP governor & blade pitch dynamics<br>• Discrete twin-engine simulation for Baron 58<br>• Prop feathering ($\beta \to 88^\circ$, drag drop 92%)<br>• Christen 801 inverted fuel/oil system (Extra 330) | • RPM stabilized under variable dive speed<br>• Baron 58 asymmetric yaw at $V_{mc} = 84$ kt<br>• Zero engine cut on Extra 330 inverted | **COMPLETED** |
| **F4** | Multi-Point Gear Kinematics & Taildragger Physics | • Archetype landing gear configurations (C172/B58/E330)<br>• Taildragger ground loop dynamic instability<br>• Steerable/castering tailwheel with friction limits<br>• Independent L/R differential toe brake channels | • C172/B58 tricycle taxi stability verified<br>• Extra 330 taildragger ground-loop divergence<br>• Differential braking produces zero-radius yaw | **COMPLETED** |
| **F5** | Verification Suite, Telemetry & Full Integration | • Dedicated test suite `LokYSAeroPhysicsTests.cpp`<br>• Real-time HUD/Avionics physics telemetry update<br>• Regression verification across all 17 existing suites<br>• Headless 240-frame deterministic benchmark | • 100% tests passing (18/18 test suites)<br>• Clean compile under `/W4 /WX /permissive-`<br>• Headless benchmark exit code 0 | **COMPLETED** |

---

## 3. Detailed Technical Breakdown & Tasks

### Phase F1: 6-DOF RK4 State-Dependent Integrator & Dynamic CG

- [x] **F1.1: Functional State Force Callback in RigidBody**
  - Refactor `RigidBody::IntegrateRK4` to receive a callable `ForceEvaluator(const RigidBodyState& state, f64 dt, Math::Vec3d& outF, Math::Vec3d& outM)`.
  - Re-evaluate aerodynamic, propulsive, and ground reaction forces at RK4 trial states $s_1, s_2, s_3$ instead of holding $t_0$ forces constant.
- [x] **F1.2: Dynamic Center of Gravity (CG) Position Tracking**
  - Compute total instantaneous mass:
    $$m(t) = m_{empty} + m_{fuel}(t) + m_{payload}$$
  - Compute instantaneous CG position in Body FRD coordinates:
    $$\mathbf{r}_{cg}(t) = \frac{m_{empty} \mathbf{r}_{empty} + m_{fuel}(t) \mathbf{r}_{tank} + m_{payload} \mathbf{r}_{payload}}{m(t)}$$
  - Shift aerodynamic neutral point and ground reaction contact moment arms relative to $\mathbf{r}_{cg}(t)$:
    $$\mathbf{M}_{cg} = \mathbf{M}_{ref} - (\mathbf{r}_{cg} - \mathbf{r}_{ref}) \times \mathbf{F}$$
- [x] **F1.3: Dynamic Inertia Tensor Transformation**
  - Apply Parallel Axis Theorem (Steiner's theorem) to update inertia tensor as CG shifts:
    $$\mathbf{I}_{cg}(t) = \mathbf{I}_{ref} - m(t) \cdot \left[ (\mathbf{d}^T \mathbf{d})\mathbf{E} - \mathbf{d}\mathbf{d}^T \right], \quad \mathbf{d} = \mathbf{r}_{cg}(t) - \mathbf{r}_{ref}$$

---

### Phase F2: Ground Effect, Propwash & Non-Linear Aerodynamics

- [x] **F2.1: Ground Effect Model (McCormick / Cheeseman-Bennett Formulation)**
  - Calculate normalized main wing height above terrain: $h/b = \max(h_{agl}, 0.1 \bar{c}) / b$.
  - Evaluate induced drag reduction factor:
    $$\phi\left(\frac{h}{b}\right) = \frac{16\left(\frac{h}{b}\right)^2}{1 + 16\left(\frac{h}{b}\right)^2}$$
  - Apply lift slope steepening and induced drag reduction:
    $$C_{Di,eff} = \phi(h/b) \cdot C_{Di}, \quad \Delta C_{L,ge} = C_L \left(\frac{1}{\phi(h/b)^{0.25}} - 1\right) \cdot 0.22$$
  - Implement downwash suppression at horizontal tail causing nose-down pitching moment on flare:
    $$\Delta C_{m,flare} = -0.05 \cdot C_L \cdot (1.0 - \phi(h/b))$$
- [x] **F2.2: Propeller Slipstream (Propwash) Impingement on Empennage**
  - Calculate axial slipstream velocity via Rankine-Froude momentum theory:
    $$V_{slip} = \sqrt{V_\infty^2 + \frac{2 T}{\rho A_{prop}}}$$
  - Blend local dynamic pressure at tailplane based on immersion fraction $k_{wash}$:
    $$q_{tail} = \frac{1}{2} \rho \left( k_{wash} V_{slip}^2 + (1 - k_{wash}) V_\infty^2 \right)$$
  - Scale elevator effectiveness ($C_{m\delta e}$) and rudder effectiveness ($C_{n\delta r}$) using $q_{tail}$ instead of free-stream $q_\infty$.
- [x] **F2.3: Coupled Asymmetric Post-Stall & Autorotation (Spin Dynamics)**
  - Implement spanwise wing-element strip stall detection:
    $$\alpha_{left} = \alpha + \frac{p \cdot (b/4)}{V_{tas}}, \quad \alpha_{right} = \alpha - \frac{p \cdot (b/4)}{V_{tas}}$$
  - If one wing stalls while the other remains unstalled, generate an asymmetric autorotative rolling moment:
    $$\Delta C_l = \frac{1}{2} C_{La} (\alpha_{left} - \alpha_{right}) \implies \text{Divergent roll into stall}$$
  - Couple yaw damping breakdown in post-stall regime to permit flat and steep spins for Extra 330SC.
- [x] **F2.4: High-Speed Compressibility Corrections**
  - Implement subsonic Prandtl-Glauert compressibility scaling for $M < 0.70$:
    $$C_{L,comp} = \frac{C_L}{\sqrt{1 - M^2}}, \quad C_{m,comp} = \frac{C_m}{\sqrt{1 - M^2}}$$
  - Model transonic wave drag divergence onset beyond $M_{crit}$.

---

### Phase F3: Constant-Speed Governor & Discrete Multi-Engine

- [x] **F3.1: Constant-Speed Propeller (CSP) Governor Subsystem**
  - Implement centrifugal flyweight governor differential equation:
    $$\frac{d\beta_{blade}}{dt} = K_{gov} \cdot (\text{RPM}_{engine} - \text{RPM}_{command})$$
  - Calculate blade pitch $\beta_{blade}$ bounded between fine pitch $\beta_{fine} \approx 12^\circ$ and coarse pitch $\beta_{coarse} \approx 35^\circ$.
  - Derive $C_t(J, \beta)$ and $C_p(J, \beta)$ as bivariate polynomial surfaces calibrated to Hartzell/McCauley CSP charts.
- [x] **F3.2: Discrete Twin-Engine Simulation for Beechcraft Baron 58**
  - Refactor propulsion model to instantiate two distinct engines:
    - Left Engine: Continental IO-550-C, lateral offset $Y = -1.83$ m, clockwise rotation.
    - Right Engine: Continental IO-550-C, lateral offset $Y = +1.83$ m, clockwise rotation.
  - Implement independent throttle, mixture, propeller pitch lever, and magneto controls for each engine.
  - Model Critical Engine aerodynamics: Since both engines rotate clockwise, the down-turning blade on the Right engine has a larger moment arm ($Y_{arm,R} = 1.83 + 0.12 D$) than the Left engine ($Y_{arm,L} = 1.83 - 0.12 D$), making Left Engine Failure significantly more critical.
- [x] **F3.3: Propeller Feathering Dynamics**
  - In the event of engine shutdown/failure, allow propeller feather command ($\beta_{blade} \to 88^\circ$).
  - Feathered propeller reduces windmill wind resistance by 92%, restoring single-engine climb capability above $V_{mc} = 84$ KIAS.
- [x] **F3.4: Christen 801 Inverted Fuel/Oil System (Extra 330SC)**
  - Implement negative-G detection in fuel pickup:
    - Cessna 172 & Baron 58: Float carburetor / un-valved fuel collector starves under $G_z < 0.0$ after 2.5 seconds.
    - Extra 330SC: Weighted aerobatic flop-tube and Christen 801 inverted oil system maintains uninterrupted fuel pressure and engine operation under sustained $-10G$.

---

### Phase F4: Multi-Point Gear Kinematics & Taildragger Physics

- [x] **F4.1: Archetype-Specific Landing Gear Configurations**
  - Define exact physical mounting coordinates, stroke, spring constant, and damping for each archetype:
    - **C172:** Tricycle with steerable nose oleo ($k=45\,\text{kN/m}$) and sprung-steel main legs ($k=65\,\text{kN/m}$).
    - **Baron 58:** Heavy tricycle oleo struts ($k=95\,\text{kN/m}$ mains, $k=60\,\text{kN/m}$ nose), wider track ($2.92$ m), retractable kinematic sequencing.
    - **Extra 330SC:** **Taildragger Configuration**:
      - Left Main: $X = +0.45$ m (forward of CG), $Y = -0.98$ m, $Z = 1.05$ m, composite spring-leaf ($k=55\,\text{kN/m}$).
      - Right Main: $X = +0.45$ m (forward of CG), $Y = +0.98$ m, $Z = 1.05$ m, composite spring-leaf ($k=55\,\text{kN/m}$).
      - Tailwheel: $X = -4.35$ m (aft), $Y = 0.00$ m, $Z = 0.40$ m, solid rubber tire with steerable spring link ($k=30\,\text{kN/m}$).
- [x] **F4.2: Taildragger Ground Loop Dynamic Instability**
  - Because main wheels are located ahead of CG ($x_{main} - x_{cg} > 0$), side friction forces during yaw induce destabilizing moments:
    $$\tau_{yaw,gear} = F_{y,main} \cdot (x_{main} - x_{cg}) > 0 \implies \text{Divergent ground loop tendency}$$
  - Require active rudder correction and steerable/lockable tailwheel mechanics during takeoff and rollout.
- [x] **F4.3: Independent Differential Toe Brakes**
  - Decouple left and right wheel brake pressure from serial controls and keyboard mapping:
    - Left pedal brake: retards left main wheel only, creating negative yaw moment (turns left).
    - Right pedal brake: retards right main wheel only, creating positive yaw moment (turns right).
  - Enables zero-radius pivot turns during low-speed ground taxiing.

---

### Phase F5: Verification Suite, Telemetry & Full Integration

- [x] **F5.1: Empirical Unit Test Suite (`LokYSAeroPhysicsTests.cpp`)**
  - Test suite asserting:
    1. Ground effect lift increase $\ge 8\%$ and induced drag reduction $\ge 35\%$ at $h/b = 0.15$.
    2. Propwash elevator authority at zero airspeed ($V_{tas} = 0, \text{RPM} = 2700 \implies M_{elevator} > 0$).
    3. Baron 58 asymmetric yaw moment with critical engine out at $84$ kt ($V_{mc}$).
    4. Extra 330SC roll rate $\ge 400^\circ/\text{s}$ at $140$ kt with full aileron deflection.
    5. Taildragger ground loop yaw divergence when rolling with $5^\circ$ initial crab angle without rudder correction.
- [x] **F5.2: Telemetry & HUD Physics Annunciators**
  - Expose Ground Effect factor ($\phi$), Propwash tail velocity ($V_{tail}$), CSP Blade Pitch ($\beta$), and Differential Brake status to window title bar and glass cockpit telemetry stream.
- [x] **F5.3: Regression & Headless Sanity Gate**
  - Verify 100% pass across all existing 17 test suites + new physics test suite (18/18 total).
  - Verify headless run `build\LokYSFlightSimulator.exe --headless --test-frames=240` exits cleanly with code 0.

---

## 4. Execution Protocol & Verification Standards

1. **Deterministic Timestep:** All equations remain anchored to the 120 Hz fixed-timestep physics scheduler ($\Delta t = 1/120$ s).
2. **Zero Allocation Policy:** Hot simulation loops must execute with 0 heap allocations (`malloc`, `new`, `std::vector` resize).
3. **Unit Discipline:** Internal calculations strictly use SI units (m, kg, s, rad, N, N*m, Pa). Conversions to knots, feet, fpm, inHg, or lbs occur only at UI/telemetry display boundaries.
4. **Compiler Standard:** MSVC 2019/2022 x64, C++20 standard (`/std:c++20`), warning level 4 with warnings as errors (`/W4 /WX /permissive-`).
