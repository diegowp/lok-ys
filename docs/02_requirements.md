# Pillar 2: Requirements Specification — YSFlight Interoperability & Standalone Tooling

**Target Architecture:** LokYS Native C++20 Flight Simulator  
**Protocol:** TOPA (Token-Optimized Aerospace Physics Architecture)  
**Document:** `docs/02_requirements.md`  
**Revision:** 1.0  

---

## 1. Functional Requirements: Legacy Asset Parsers

### 1.1 Ingestion of Legacy YSFlight Runtimes
- **REQ-PAR-01 (Loose File Support):** The engine shall parse uncompressed files directly from existing `Ysflight/` directory trees:
  - Aircraft folder (`aircraft/*.dat`, `*.dnm`, `*.srf`, `*.ist`, `aircraft.lst`).
  - Scenery folder (`scenery/*.fld`, `*.stp`, `*.yfs`, `scenery.lst`).
  - Ground objects folder (`ground/*.dat`, `*.dnm`, `*.srf`).
- **REQ-PAR-02 (Robust Tokenizer):**
  - Ignore trailing and inline comments initiated by `#` or `REM` (case-insensitive).
  - Support arbitrary whitespace (tabs, multiple spaces, Windows CRLF and UNIX LF line endings).
  - Support unit prefixes and suffixes attached directly to numbers without spaces (e.g., `180HP`, `630kg`, `15deg`, `250km/h`, `10000ft`, `0.0MACH`).
  - Fall back gracefully to sane aerodynamic defaults when encountering unmapped or obsolete legacy tags.
- **REQ-PAR-03 (Mesh Triangulation):**
  - Polygons in `.srf` with $>4$ vertices must be automatically decomposed into convex triangle fans.
  - Compute smooth or flat vertex normals when missing from `.srf` polygon definitions using the Newell algorithm for polygon normal evaluation.
  - Preserve vertex color attributes ($R, G, B \in [0, 255]$) and assign an opaque alpha ($A = 255$) by default.

---

## 2. Packaged Container Specifications (`.lfs`, `.acft.zip`, `.objects.zip`)

To modernise distribution and eliminate file clutter, LokYS defines structured ZIP containers:

### 2.1 Aircraft Package (`.acft.zip`)
Single compressed container representing an entire aircraft archetype:
```
my_aircraft.acft.zip
├── manifest.json            # Semantic metadata, author, version, entry paths
├── flight_model.dat         # Aerodynamic, mass, engine, and gear parameters
├── visual_model.dnm         # Articulated 3D mesh and animation hierarchy
├── cockpit_panel.ist        # Cockpit instrument layout script
├── collision.srf            # Simplified collision bounding geometry
├── cockpit_interior.srf     # High-detail cockpit interior visual mesh
└── textures/                # (Optional) PNG/DDS textures for modern PBR materials
```

**Manifest Schema (`manifest.json`):**
```json
{
  "formatVersion": 1,
  "type": "Aircraft",
  "identifier": "CESSNA_172R",
  "displayName": "Cessna 172R Skyhawk",
  "author": "LokYS Team",
  "files": {
    "flightModel": "flight_model.dat",
    "visualModel": "visual_model.dnm",
    "instrumentPanel": "cockpit_panel.ist",
    "collisionMesh": "collision.srf",
    "cockpitInterior": "cockpit_interior.srf"
  },
  "defaultLivery": "default"
}
```

### 2.2 Unified Scenery Package (`.lfs` — LokYS Flight Scenery)
Complete regional scenery archive packaged as a standard ZIP container with `.lfs` extension:
```
regional_airfield.lfs
├── scenery_manifest.json    # Scenery bounds, projection origin, regional airports
├── terrain_mesh.fld         # Base elevation grid, ground polygons, rivers, runways
├── spawn_points.stp         # Runway positions, parking ramps, carrier catapults, airspawns
├── nav_aids.json            # VOR, DME, NDB, and ILS localizer/glideslope transponders
└── objects/                 # Embedded 3D ground models (.srf / .dnm)
    ├── tower.dnm
    ├── hangar.srf
    └── runway_lights.srf
```

---

## 3. Standalone Editor Capabilities

To empower creators and bridge the legacy modding community with LokYS, two standalone tools shall share `AssetCore`:

```
===================================================================================================
STANDALONE TOOLING SUITE
===================================================================================================
                       [ Shared C++ AssetCore ]
                       (Parsers, VFS, Math, Exporters)
                                    │
            ┌───────────────────────┴───────────────────────┐
            ▼                                               ▼
[ Standalone Aircraft Editor ]                  [ Standalone Scenery Editor ]
 - Visual DNM Node Hierarchy Tree                - 3D Terrain Heightfield Visualizer
 - Pivot Offset & Rotation Gizmos                - Drag-and-Drop 3D Object Placement
 - Real-Time Animation Preview (Flaps, Gear)     - Instance Property Inspector (Tags, Lights)
 - DAT Aerodynamic Curve Visualizer & Tuner      - Runway & STP Spawn Point Editor
 - Interactive IST Instrument Gauge Placer       - One-Click .lfs Scenery Compilation
 - One-Click .acft.zip Export                    - Live Minimap & ILS Beam Visualizer
===================================================================================================
```

### 3.1 Aircraft Editor Requirements
- **Hierarchy Inspector:** Interactive tree view displaying all `PCK` nodes, parent-child relationships, and local coordinate frames.
- **Animation Debugger:** Sliders driving animation channels ($0.0 \dots 1.0$) for `CLA` classes (landing gear transit, flap deflection, elevator/aileron/rudder travel, spinning propeller).
- **Aerodynamics Tuner:** Graph plotting lift polar $C_L(\alpha)$, drag polar $C_D(\alpha)$, and pitching moment $C_m(\alpha)$ derived from `.dat` parameters, validating stability criteria ($C_{m_\alpha} < 0$).
- **Instrument Panel Preview:** WYSIWYG 2D canvas displaying `.ist` gauge placement with pilot eye camera alignment.

### 3.2 Scenery Editor Requirements
- **Terrain Sculpting & Import:** Load base `.fld` elevation blocks (`TerrMesh`), displaying 3D wireframe and solid elevation surfaces.
- **Object Library Palette:** Palette of loaded `.objects.zip` assets with thumbnail previews for click-to-place placement.
- **Instance Transform Manipulator:** 3-axis translation and yaw/pitch/roll rotation gizmo for placed objects, with automatic ground snapping.
- **Spawn Point Management:** Visual placement of runways, taxiways, and airborne start positions (`.stp`), with heading compass roses and altitude sliders.

---

## 4. Non-Functional & Performance Budgets

- **Ingestion Latency:** Complete loading and parsing of a complex aircraft package (`.acft.zip` with 15,000 vertices and 25 articulated nodes) must complete in $< 15\,\text{ms}$.
- **Zero Runtime Heap Allocation:** Mesh vertex and index buffers must be constructed via contiguous memory arenas during load phase, never reallocating inside the 120 Hz simulation loop.
- **Compression Efficiency:** Standard Deflate compression inside `.lfs` and `.acft.zip` must achieve $> 60\%$ file size reduction compared to raw ASCII `.fld` and `.dnm` files.
