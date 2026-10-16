# Pillar 3: Architecture Decision Records (ADRs) — YSFlight Interoperability

**Target Architecture:** LokYS Native C++20 Flight Simulator  
**Protocol:** TOPA (Token-Optimized Aerospace Physics Architecture)  
**Document:** `docs/03_decisions.md`  
**Revision:** 1.0  

---

## ADR-006: Selection of Standard ZIP Container (`.lfs`, `.acft.zip`) via In-Memory Streaming

* **Status:** Accepted
* **Context:**
  Aviation simulators often suffer from mod directory chaos: loose text files, unorganized texture folders, and broken relative links. Distributing individual aircraft or sceneries across dozens of loose `.dat`, `.dnm`, `.srf`, and `.ist` files causes disk fragmentation, slow loading times, and complex user installation. Proprietary binary containers (such as `.pak` or encrypted archives) hinder community modding and break tooling interoperability.
* **Decision:**
  Adopt the standard PKZIP 2.0 container format with custom file extensions:
  - Aircraft: `.acft.zip` (or `.zip` with internal `manifest.json`).
  - Scenery: `.lfs` (LokYS Flight Scenery — standard ZIP internally).
  Implement in-memory stream extraction using a lightweight, zero-dependency C++20 Deflate/ZIP header parser (`ZipVfsReader`).
* **Consequences:**
  - Standard ZIP tooling (7-Zip, WinRAR, File Explorer) can inspect, modify, and create assets without specialized software.
  - One-click drag-and-drop installation into a `packages/` folder.
  - Zero extraction to temporary disk folders: archives are read directly into memory buffers, reducing loading times from seconds to milliseconds.

---

## ADR-007: Dual Text Payload & Binary Cache Inside Container

* **Status:** Accepted
* **Context:**
  YSFlight assets are exclusively ASCII text files. Text parsing ensures transparency, easy manual tuning in text editors, and Git diffability. However, repeatedly parsing hundreds of thousands of ASCII vertices in `.srf` and `.dnm` models incurs measurable CPU overhead at load time.
* **Decision:**
  Preserve the original ASCII text files (`.dat`, `.dnm`, `.srf`, `.fld`) as the authoritative source of truth inside containers. Support an optional binary cache chunk (`cache.bin` storing pre-triangulated vertex buffers and pre-computed aerodynamic polars).
  If `cache.bin` is present and matches the hash of the text files, load immediately via memory mapping. If absent or invalid, parse the text files and optionally generate the cache in memory.
* **Consequences:**
  - Maintains 100% human-readability and editing capability.
  - Sub-millisecond load times for cached assets.
  - Zero risk of version desynchronization.

---

## ADR-008: Coordinate Space Normalization at Ingestion Boundary

* **Status:** Accepted
* **Context:**
  YSFlight uses a right-handed system oriented with $+X$ to the right, $+Y$ upwards, and $+Z$ forward. Aircraft graphics in Direct3D 12 utilize a left-handed screen coordinate system. The LokYS physics core operates strictly in an aerospace Forward-Right-Down (**FRD**) body frame and North-East-Down (**NED**) navigation frame. Units in YSFlight are an ad-hoc mix of knots, km/h, feet, meters, pounds, and kilograms.
* **Decision:**
  Confine all coordinate conversions and unit normalizations strictly to the `AssetCore` parser boundary:
  - Positions and vectors: Transformed to FRD via $X_{\text{FRD}} = +Z_{\text{YS}}$, $Y_{\text{FRD}} = +X_{\text{YS}}$, $Z_{\text{FRD}} = -Y_{\text{YS}}$.
  - Polygon winding: Invert vertex ordering or adjust surface normal evaluation so triangles render with counter-clockwise front faces in Direct3D 12.
  - Physical quantities: Converted strictly into SI units ($\text{m}$, $\text{kg}$, $\text{s}$, $\text{rad}$, $\text{N}$, $\text{N}\cdot\text{m}$, $\text{W}$).
* **Consequences:**
  - The core 6-DOF physics engine (`RigidBody`, `FlightDynamicsSystem`, `AerodynamicsSystem`) remains 100% pure SI and agnostic to the legacy source.
  - Direct3D 12 rendering pipelines require no runtime coordinate flipping in vertex shaders.

---

## ADR-009: Monolithic Multi-Agent Ownership Boundaries

* **Status:** Accepted
* **Context:**
  Building an extensive interoperability layer, ZIP VFS, and standalone editors in a multi-agent development environment risks merge conflicts, header circularity, and duplicated parsing logic if boundaries are not formalized.
* **Decision:**
  Enforce strict, partitioned subsystem ownership across agent roles:
  1. **`Agent-Core`:** Owns `src/vfs/`, `src/math/CoordinateTransforms.hpp`, and common tokenizer primitives (`src/assets/AssetTokenizer.hpp`).
  2. **`Agent-Aircraft`:** Owns `src/assets/AircraftDatParser.cpp`, `src/assets/CockpitIstParser.cpp`, and integration into `AircraftRegistry`.
  3. **`Agent-Scenery`:** Owns `src/assets/SceneryFldParser.cpp`, `src/assets/SpawnStpParser.cpp`, and integration into `TerrainStreamingSystem`.
  4. **`Agent-Geometry`:** Owns `src/assets/DnmParser.cpp`, `src/assets/SrfParser.cpp`, and triangulation/vertex buffer compilation.
  5. **`Agent-Tools`:** Owns standalone editor executables (`tools/AircraftEditor/`, `tools/SceneryEditor/`) decoupled from the main simulator binary.
  6. **`Agent-QA`:** Owns automated round-trip tests in `tests/YsflightInterOpTests.cpp`.
* **Consequences:**
  - Zero header contamination or race conditions between concurrent agents.
  - Clear interface contracts verified by discrete unit tests.
