# Project Requirements Specification: LokYS Flight Simulator

**Project Name:** LokYS High-Performance Native Windows Flight Simulator  
**Status:** Approved (Milestone M0)  
**Document Revision:** 1.0  
**Target Platform:** Windows 10 / Windows 11 x64  

---

## 1. Executive Summary & Vision

LokYS is a native Windows flight simulation platform engineered for maximum runtime efficiency, physical credibility, and deterministic performance. It combines the low resource footprint and instant loading characteristic of YSFlight with modern Direct3D 12 graphics and a rigorous 6-DOF aerospace flight dynamics model. Heavy commercial general-purpose game engines (Unreal Engine, Unity, Godot) are strictly excluded in favor of purpose-built, zero-bloat native C++ systems.

---

## 2. Confirmed Project Requirements (Phase 0 Sign-Off)

### 2.1 Core Engineering Decisions

| Dimension | Confirmed Selection | Technical Specification |
| :--- | :--- | :--- |
| **Development Environment** | VS Code with MSVC & CMake | Visual Studio Build Tools toolset (MSVC v142/v143), CMake 3.20+, Ninja / MSBuild |
| **Language Standard** | **C++20** | `/std:c++20`, strict conformance (`/permissive-`), warning level 4 as errors (`/W4 /WX`) |
| **Executable Architecture** | **Modular Monolithic** | Single executable with decoupled subsystem interfaces, static internal linkage, LTO |
| **Initial Aircraft Archetype** | **General Aviation (GA)** | High-wing single-engine piston (Cessna 172 archetype), fixed-pitch propeller |
| **Flight Dynamics Fidelity** | **Enhanced 6-DOF Coefficient** | RK4 fixed-timestep integrator, tabulated aerodynamic polars ($C_L, C_D, C_Y, C_l, C_m, C_n$), dynamic pressure, stall envelope |
| **Graphics API & Fidelity** | **Direct3D 12 (Balanced Modern)** | Native D3D12, explicit resource state transitions, triple-buffering, directional sun, atmospheric scattering |
| **Terrain System** | **Handcrafted Local Airfield** | Local airfield with runway, taxiway, aprons, and elevation heightfield sampling |
| **Networking Scope** | **Offline Initially** | Single-player standalone simulation; network protocols deferred to future milestones |
| **Hardware Interfacing** | **USB Joysticks & HID** | Win32 Raw Input and DirectInput abstraction supporting flight sticks, throttles, rudder pedals, keyboard fallback |
| **Initial Milestone Target** | **Flyable Proof-of-Concept** | End-to-end integration: native window, D3D12 renderer, airfield, 6-DOF GA flight model, chase camera, flight controls |

---

## 3. Hardware Target Baseline

* **Host Platform:** Microsoft Windows 10/11 x64
* **Host CPU:** Intel Core i5-13450HX (10 Cores, 16 Threads)
* **Host Memory:** 16 GB Physical RAM
* **Primary Discrete GPU:** NVIDIA GeForce RTX 3050 Laptop GPU (6 GB VRAM)
* **Reference Architectural GPU:** AMD Radeon RX 6700 XT (RDNA 2, 12 GB VRAM baseline)
* **SDK / Compiler:** MSVC 19.29 (Build Tools v142), Windows 10 SDK 10.0.19041.0, DirectX 12 Native

---

## 4. Nonfunctional Requirements & Performance Budgets

* **Frame Rate:** Baseline 60 FPS @ 1920x1080; target 120 FPS in lightweight flight conditions.
* **Frame-Time Consistency:** 99th percentile frame times must not exceed 20ms under baseline flight conditions. Zero micro-stutters during camera motion.
* **Physics Determinism:** Physics executes at a fixed timestep (default 120 Hz) independent of presentation refresh rate. Variable render delta time must never enter integration.
* **Startup Latency:** Engine initialization to ready-to-fly state must complete in under 3.0 seconds.
* **Memory Footprint:** Baseline process working set capped below 500 MB for local airfield and single GA aircraft.
* **Zero Runtime Heap Allocation:** No dynamic allocations (`malloc`, `new`, `std::vector` resizing) in simulation or rendering hot paths.

---

## 5. Architectural Invariants & Constraints

1. **Unit Discipline:** Internal physics must compute strictly in SI units (kg, m, s, rad, N, N·m). Aviation units (knots, feet, inHg, lbs) are confined strictly to input and cockpit gauge presentation layers.
2. **Rotational Representation:** Orientation must be maintained as unit quaternions ($||q|| = 1.0$) with continuous renormalization at each integration step. Euler angles are strictly derived for display.
3. **Headless Execution:** FlightDynamicsSystem, PropulsionSystem, and AvionicsSystem must remain capable of executing without initializing the Direct3D 12 device.
4. **Camera-Relative Origin:** World coordinate positions must be converted to camera-relative coordinates before D3D12 vertex submission to eliminate 32-bit floating-point jitter over large distances.
