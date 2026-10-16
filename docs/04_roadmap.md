# Pillar 4: Multi-Agent Implementation Roadmap — YSFlight Interoperability & Tooling

**Target Architecture:** LokYS Native C++20 Flight Simulator  
**Protocol:** TOPA (Token-Optimized Aerospace Physics Architecture)  
**Document:** `docs/04_roadmap.md`  
**Revision:** 1.0  

---

## 1. Executive Roadmap Summary

This roadmap establishes the progressive implementation phases for delivering end-to-end YSFlight asset interoperability, ZIP-based packaged containers (`.lfs`, `.acft.zip`), and standalone creation tools for the LokYS simulation platform.

```
===================================================================================================
MULTI-AGENT EXECUTION TIMELINE & PHASING
===================================================================================================
Phase 1: Core Foundation & VFS           [ Agent-Core / Agent-Geometry ]
 ├─ Lightweight In-Memory ZIP Reader     (Zero-Disk Extraction VFS)
 ├─ Tokenizer Primitives & Math Bridges  (Coordinate & Unit Converters)
 └─ .SRF / .DNM Geometry Parsers         (Triangulation & Articulation Tree)
        │
Phase 2: Aircraft Ecosystem              [ Agent-Aircraft ]
 ├─ .DAT Flight Model Ingestion          (Aerodynamics, Mass, Engine, Gear)
 ├─ .IST Cockpit Instrument Ingestion    (Analogue Dial Positioning)
 └─ .acft.zip Package Bundler            (One-Click Aircraft Container)
        │
Phase 3: Scenery & World Pipeline        [ Agent-Scenery ]
 ├─ .FLD Terrain Heightfield Ingestion   (Elevation Grid & Surface Polygons)
 ├─ .STP Spawn Point Ingestion           (Runway / Airspawn Placement)
 └─ .lfs Unified Scenery Container       (Packaged Scenery with Ground Models)
        │
Phase 4: Standalone Tooling Suite        [ Agent-Tools ]
 ├─ Standalone Aircraft Editor           (Visual Hierarchy, Gizmos, Aero Graphs)
 └─ Standalone Scenery Editor            (3D Terrain Placer, Attribute Inspector)
        │
Phase 5: Verification & Quality Assur.   [ Agent-QA ]
 ├─ Automated Round-Trip Parser Tests    (Ingestion against local Ysflight/)
 └─ Headless 240-Frame Simulation Benchmark
===================================================================================================
```

---

## 2. Phase-by-Phase Deliverables & Acceptance Criteria

### Phase 1: Core Foundation, VFS & Geometry (`Agent-Core` & `Agent-Geometry`) — COMPLETED
- [x] **P1.1: In-Memory ZIP Virtual File System (`ZipVfsReader`)**
  - Implement zero-dependency C++20 PKZIP directory header parser.
  - Expose `ZipVfsReader::OpenFile(path)` returning a non-owning `std::span<const uint8_t>` or `std::string_view`.
  - In-memory Deflate inflation without writing temporary files to disk.
- [x] **P1.2: Robust Asset Tokenizer (`AssetTokenizer`)**
  - High-performance, zero-allocation line and token scanner.
  - Strips `#` and `REM` comments, handles inline units (`180HP`, `630kg`, `15deg`).
- [x] **P1.3: Surface Geometry Parser (`SrfParser`)**
  - Ingests `.srf` vertex arrays ($X, Y, Z$), face polygon indices, colors ($R, G, B$), and normals.
  - Decomposes non-planar or n-gon faces into triangle fans; produces Direct3D 12 vertex/index buffers.
- [x] **P1.4: Dynamic Articulated Model Parser (`DnmParser`)**
  - Ingests `.dnm` hierarchical scene graphs (`PCK` blocks, `CLA` animation classes, `STA` state keyframes, `POS` pivots).
  - Connects nodes directly to the LokYS articulated submesh scene graph solver.

### Phase 2: Aircraft Ecosystem & Flight Dynamics (`Agent-Aircraft`) — COMPLETED
- [x] **P2.1: Flight Model Parser (`AircraftDatParser`)**
  - Ingests `.dat` flight physics and maps to `AircraftConfig`, `AircraftMassConfig`, and `AeroDerivativesConfig`.
  - Computes lift curves, drag polars, stall angles, propulsion parameters, and gear geometry in SI units.
- [x] **P2.2: Cockpit Panel Parser (`CockpitIstParser`)**
  - Ingests `.ist` instrument panel definitions and maps coordinates to `SixPackGaugeRenderer`.
- [x] **P2.3: Aircraft Container Packager (`.acft.zip`)**
  - Support bundling and loading `.acft.zip` archives with `manifest.json`.

### Phase 3: Scenery & World Pipeline (`Agent-Scenery`)
- [ ] **P3.1: Terrain & Field Mesh Parser (`SceneryFldParser`)**
  - Ingests `.fld` files, parsing `TerrMesh` elevation blocks (`TMS`, `NBL`, `BLO`) into LokYS heightfields.
  - Ingests ground polygon vectors (runways, taxiways, aprons, water bodies).
  - Ingests placed ground object instances (`POS` + `TAG`) and links to `.srf`/`.dnm` visual meshes.
- [ ] **P3.2: Spawn & Starting Points Manager (`SpawnStpParser`)**
  - Ingests `.stp` files, mapping runway spawn points, carrier deck spots, and airborne waypoints to `AirportDatabase`.
- [ ] **P3.3: Unified Scenery Container Packager (`.lfs`)**
  - Container format combining terrain mesh, object instances, starting points, and embedded models into a single compressed `.lfs` file.

### Phase 4: Standalone Tooling Suite (`Agent-Tools`)
- [ ] **P4.1: Standalone Aircraft Editor**
  - Lightweight UI (Direct3D 12 / ImGui) for inspecting `.dnm` hierarchies, testing control surface travel, previewing `.ist` instrument panels, and tuning `.dat` polars.
  - Export to `.acft.zip`.
- [ ] **P4.2: Standalone Scenery Editor**
  - 3D viewport for loading `.fld` terrains, dragging-and-dropping 3D ground objects from `.objects.zip`, configuring spawn points, and one-click compiling to `.lfs`.

### Phase 5: Verification, Benchmarking & QA (`Agent-QA`)
- [ ] **P5.1: Ingestion Test Suite (`YsflightInterOpTests.cpp`)**
  - Automated unit tests validating ingestion of real assets from `Ysflight/aircraft/` (e.g., `cessna172r.dat`, `cessna172r.dnm`, `cessna172r.ist`) and `Ysflight/scenery/` (`small.fld`, `small.stp`).
  - Verifies mathematical identity of converted coordinates and bounding volumes.
- [ ] **P5.2: Headless Flight Simulation Benchmark**
  - Automated 240-frame headless flight run with an imported YSFlight aircraft spawning on an imported YSFlight field.
  - Strict tolerance: zero NaN/Inf, zero memory leaks, frame-time $< 1.0\,\text{ms}$ per physics tick.
