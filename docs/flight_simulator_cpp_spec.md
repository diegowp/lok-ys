# C++ Flight Simulator — Technical Specification & Task Context

## Role & Project Context
Act as a Senior C++ Game & Simulation Developer specializing in real-time flight dynamics, 3D rendering pipelines, HUD/avionics instrumentation, and responsive input/UI architectures. You are working on an existing Flight Simulator built in modern C++ (C++17/C++20).

## Objectives & Bug Fixes

### 1. Physics & Initial Aircraft State (Spawn Stationary)
- **Current Issue:** The aircraft spawns with non-zero initial velocity/thrust and begins moving immediately upon simulation start.
- **Required Fix:** Initialize the aircraft rigid body and flight dynamics model (FDM) in a completely stationary state on the runway/apron:
  - Set initial linear velocity ($\vec{v} = \vec{0}$) and angular velocity ($\vec{\omega} = \vec{0}$) vectors to zero.
  - Default throttle input to `0.0f` (idle thrust) and ensure wheel contact/ground friction forces are settled at frame `0`.
  - Initialize the parking brake state to `ACTIVE` on spawn so the aircraft remains held in place until explicitly released by the user.

### 2. Input Handling — Parking Brake State Toggle (`SPACE` Key)
- **Current Issue:** The parking brake currently behaves as a momentary hold action, requiring the user to continuously hold down the `SPACE` key to keep the brakes engaged.
- **Required Fix:** Refactor the parking brake input handler from a continuous key-down polling check to an edge-triggered state toggle (`ACTIVE` / `INACTIVE`):
  - Trigger state transitions only on the `KeyPressed` (rising edge) event for `SPACE` (ignoring key-repeat events).
  - Maintain a persistent boolean state (`bIsParkingBrakeEngaged`) inside the aircraft/braking subsystem and expose this status clearly to the HUD/cockpit indicator.

### 3. Avionics / HUD — Artificial Horizon (Attitude Indicator) Inversion Fix
- **Current Issue:** The Artificial Horizon (ADI / Attitude Director Indicator) renders inverted relative to the aircraft's actual pitch and/or roll attitude.
- **Required Fix:** Correct the pitch and bank angle transformations driving the Artificial Horizon UV offset/rotation matrix:
  - **Pitch Convention:** Pitching the aircraft nose UP (`+pitch`) must translate the horizon bar DOWN (revealing more sky/blue above the aircraft reference symbol), and pitching nose DOWN (`-pitch`) must translate the horizon bar UP.
  - **Roll/Bank Convention:** Banking the aircraft RIGHT (`+roll`) must rotate the horizon bar COUNTER-CLOCKWISE by the exact bank angle so the horizon remains parallel to the true world horizon.

## New Features & UI/UX Architecture

### 4. Pre-Flight Main Menu (Flight Setup & Environment Selection)
- Design and implement a pre-flight configuration screen before loading into the active simulation state (`GameState::MainMenu` -> `GameState::Simulation`):
  - **Aircraft Selection:** Selectable aircraft profiles (loading distinct aerodynamic parameters, mass properties, and 3D meshes).
  - **Airport / Spawn Location Selection:** Selectable airports/runways (setting initial world coordinates, elevation, and heading).
  - **Weather & Environment Presets:** Configurable weather conditions (e.g., Clear Skies, Overcast, Rain/Storm, Wind Speed/Direction, Time of Day/Visibility).

### 5. Modern, Intuitive UI/UX Design
- Overhaul the user interface with a clean, modern, and low-friction layout suited for flight simulation:
  - Clear visual hierarchy, responsive hover/active states, smooth transitions, and high-contrast typography for readability across resolutions.
  - Modular UI component design (e.g., using a hardware-accelerated UI layer or immediate/retained-mode GUI framework) cleanly decoupled from core simulation logic.

### 6. Global Application Settings & Input Binding Menu
- Implement a dedicated Settings/Options menu accessible from both the Main Menu and the in-flight Pause Menu:
  - **Input Action Mapping (Rebindable Controls):** Support dynamic remapping of actions and axes across **Joystick/HOTAS** (axes, deadzones, sensitivity curves, POV hats, buttons), **Keyboard** (key bindings and modifiers), and **Mouse** (camera look/sensitivity, cockpit interaction).
  - **Persistence:** Serialize and load user input profiles and general application settings (audio, graphics, control bindings) via a structured configuration file (e.g., JSON or INI).
