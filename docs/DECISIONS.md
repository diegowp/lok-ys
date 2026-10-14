# Architecture Decision Records (ADRs): LokYS

## ADR-001: Selection of Direct3D 12 Native over Game Engines
* **Status:** Accepted
* **Context:** The simulation requires ultra-fast startup (<3s), sub-500MB working set, and deterministic frame pacing matching the YSFlight philosophy.
* **Decision:** Build a native Windows Direct3D 12 engine instead of adopting Unreal Engine 5, Unity, or Godot.
* **Consequences:** Eliminates gigabytes of engine bloat and unwanted garbage collection/tick overhead; requires explicit manual resource barrier and descriptor heap governance.

## ADR-002: Modular Monolithic Architecture with Strict Layering
* **Status:** Accepted
* **Context:** Dynamic plugin architectures introduce virtual dispatch overhead, DLL boundary memory allocation issues across compilers, and complex runtime dependency resolution.
* **Decision:** Implement a modular monolithic executable with static subsystem linkage, clear C++ abstract interfaces, and Link-Time Optimization (LTO).
* **Consequences:** Maximizes inlining and compiler optimizations while enforcing subsystem isolation via header boundaries.

## ADR-003: Fixed-Timestep Physics with Triple-Buffered Snapshot Interpolation
* **Status:** Accepted
* **Context:** Coupling 6-DOF aerospace numerical integration directly to variable display refresh rates creates instability, numerical divergence during frame drops, and non-reproducible trajectories.
* **Decision:** Execute flight dynamics at a strict 120 Hz fixed timestep. Expose thread-safe state snapshots through a lockless triple buffer for asynchronous render-thread interpolation.
* **Consequences:** Ensures 100% deterministic flight physics across varying hardware; requires render-time interpolation of position and orientation quaternions.

## ADR-004: Floating-Origin / Camera-Relative Coordinate System
* **Status:** Accepted
* **Context:** 32-bit floating-point numbers lose precision past 10 kilometers, causing noticeable jitter on landing gear, cockpit instruments, and runway markings.
* **Decision:** Maintain global simulation positions as 64-bit doubles (`Vec3d`), translating coordinates relative to the active camera position before packing into 32-bit float vectors for GPU shaders.
* **Consequences:** Enables planetary/regional coordinate expansion without vertex jitter.

## ADR-005: Strict SI Units Internally with Boundary Converters
* **Status:** Accepted
* **Context:** Aviation systems traditionally use knots, feet, inches of mercury, and nautical miles, while physical differential equations require SI units.
* **Decision:** Internal engine state uses kilograms, meters, seconds, radians, and Newtons exclusively. Aviation units are restricted to input parsing and instrument display layers.
* **Consequences:** Eliminates silent unit-conversion bugs in aerodynamic equations.
