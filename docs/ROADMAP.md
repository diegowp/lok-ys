# Project Development Roadmap: LokYS Flight Simulator

| Milestone | Title | Status | Scope & Acceptance Criteria |
| :--- | :--- | :--- | :--- |
| **M0** | Requirements & Architectural Baseline | **COMPLETED** | Formalized `REQUIREMENTS.md`, `ARCHITECTURE.md`, `DECISIONS.md`, `BUILD.md`. |
| **M1** | Toolchain & Minimal Build Infrastructure | **COMPLETED** | `CMakeLists.txt`, `CMakePresets.json`, compiler warning discipline (`/W4 /WX /permissive-`), clean compile. |
| **M2** | Native Win32 Window & High-DPI Loop | **COMPLETED** | Native Win32 window class, message pump, raw OS handles, high-DPI scaling. |
| **M3** | Direct3D 12 Core Device Initialization | **COMPLETED** | DXGI factory, D3D12 device, command queues, swapchain, fences, clear screen. |
| **M4** | GPU Adapter Enumeration & Capability Probe | **COMPLETED** | Hardware detection (RTX 3050 Laptop / Intel UHD / RDNA2 targets), VRAM budget query. |
| **M5** | Math Engine, Units & Coordinate Systems | **COMPLETED** | Quaternions, 3D vectors, NED/Body/Wind transforms, SI <-> Aviation converters. |
| **M6** | Fixed-Timestep Simulation Scheduler | **COMPLETED** | Decoupled 120Hz accumulator, snapshot triple-buffer, pause/step controls. |
| **M7** | 6-DOF Rigid-Body Dynamics Engine | **COMPLETED** | RK4 numerical integrator, mass, inertia tensor, gravity trajectories. |
| **M8** | Aerodynamic Coefficient Engine | **COMPLETED** | Cessna 172 NASA CR-2144 polar curves ($C_L, C_D, C_Y, C_l, C_m, C_n$), dynamic pressure, stall breakaway, wind-to-body frame resolution. |
| **M9** | Input Engine & Hardware Abstraction | **COMPLETED** | Win32 joystick HID polling, deadzone calibration, quadratic response curves, keyboard flight controls, telemetry display. |
| **M10** | First Integrated Flyable Prototype | **COMPLETED** | Cessna 172 3D mesh, 2000m runway & terrain, Chase & Cockpit camera modes, D3D12 HLSL pipeline with depth testing, runway takeoff, flight & landing loop. |
| **M11** | Ground Reactions & Strut Physics | **COMPLETED** | Multi-point oleo strut kinematics (nose, left main, right main), progressive spring stiffness, hydraulic damping, tire scrub friction, nosewheel steering, wheel brakes, tailstrike & wingtip scrape protection. |
| **M12** | Propulsion & Fuel Subsystem | **COMPLETED** | Lycoming O-320 160hp piston engine dynamics, McCauley fixed-pitch prop polars ($C_t, C_p$), torque balance rotational integrator, Gagg-Farrar naturally aspirated altitude lapse, idle stop governor, mixture control, P-Factor, gyroscopic precession, prop torque reaction, fuel burn rate & dynamic mass. |
| **M13** | glTF 2.0 Aircraft Model Loading | PENDING | Exterior/interior meshes, PBR materials, articulated control surfaces. |
| **M14** | Analog Six-Pack Avionics | PENDING | ASI, altimeter, attitude gyro, turn coordinator, DG, VSI with sensor lag. |
| **M15** | Spatial Audio Subsystem | PENDING | XAudio2 3D listener, engine RPM audio pitch shifting, wind sound. |
| **M16** | Radio Navigation Systems | PENDING | VOR radial reception, DME distance, ILS localizer & glide slope needles. |
| **M17** | Serial Cockpit Hardware Interface | PENDING | Async COM-port telemetry stream, microcontroller switch/potentiometer input. |
| **M18** | Atmospheric & Weather Systems | PENDING | ISA atmospheric gradient, wind vector, thermal turbulence injection. |
| **M19** | Graphics Optimization & Visual FX | PENDING | Cascaded shadow maps, atmospheric fog, terrain LOD, instanced scenery. |
| **M20** | Multiplayer Network Prototype | PENDING | UDP snapshot replication, entity interpolation, dead reckoning. |
| **M21** | Glass Cockpit & Advanced Autopilot | PENDING | PID heading/altitude hold, flight director, digital flight display. |
| **M22** | Full Scenery & Multi-Aircraft Expansion | PENDING | Streaming geographic terrain tiles, multi-archetype flight models. |
