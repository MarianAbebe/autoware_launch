# EXPERIMENTAL map_projector_info.yaml (tonight only)

**Status:** Experimental visualization aid — **not** production-correct.  
**File:** `../map_projector_info.yaml` (new file; nothing overwritten).

## What was generated

```yaml
projector_type: LocalCartesian
vertical_datum: WGS84
map_origin:
  latitude: 51.08080935188234
  longitude: -114.1331143659806
  altitude: 0.0
```

These numbers are the **first NovAtel `/novatel/oem7/fix` fix** used as the ENU origin in `intermediate/georef/georef_alignment_report.json` when aligning KISS→GNSS and writing `pointcloud_map_georeferenced.pcd`.

## Assumptions (explicit)

| # | Assumption | Risk if wrong |
|---|------------|---------------|
| 1 | Autoware `LocalCartesian` ≈ our georef ENU from that origin | OSM vs PCD can shift by meters (spherical ENU vs GeographicLib) |
| 2 | Using **first GNSS fix** as origin is acceptable for viz | Differs from any origin Carter used in Vector Map Builder |
| 3 | `vertical_datum: WGS84` | Loader assumes WGS84; altitude in YAML is ignored (forced to 0) |
| 4 | Visualization must load **georeferenced** PCD, not KISS-local PCD | Wrong cloud → OSM/PCD won’t overlap |
| 5 | `LocalCartesian` is OK for lanelet2_map_loader viz | Autoware notes `gnss_poser` may not support LocalCartesian |
| 6 | Ghost walls remain in PCD | Load success ≠ localization-ready |

## Why not MGRS / LocalCartesianUTM

- Full `finalMap.osm` spans **11UPS** and **11UQS** → MGRS forbidden.
- Georef frame was **ENU from lat/lon origin**, not UTM → `LocalCartesian` is the matching type, not `LocalCartesianUTM`.

## Experimental load layout (no overwrites)

Directory `experimental_viz_package/`:

| Autoware expected name | Points to |
|------------------------|-----------|
| `map_projector_info.yaml` | copy of experimental YAML |
| `pointcloud_map.pcd` | symlink → `../pointcloud_map_georeferenced.pcd` |
| `lanelet2_map.osm` | symlink → `../finalMap.osm` |

Originals kept: `pointcloud_map.pcd` (KISS local), `finalMap.osm`, georef PCD, KISS intermediates.

## How to try visualization (do not use sample-map path)

Point Autoware `map_path` at:

`/home/agxorin/autoware_data/maps/eng_loading_dock_map/experimental_viz_package`

Use a **separate** launch from the working sample logging-sim. If load fails, leave this directory as-is for tomorrow.

## What “success” means tonight

Autoware publishes `/map/pointcloud_map` and `/map/vector_map` and both appear in RViz in roughly the same place.  
**Does not** mean NDT-ready, GNSS-init correct, or production projector.

## Load-test results (2026-08-12 night)

Isolated loaders only (sample logging-sim not used). Logs: `intermediate/experimental_load_test/`.

| Component | Result |
|-----------|--------|
| `map_projector_info` → `/map/map_projector_info` | **SUCCESS** (`LocalCartesian`, origin as above, `scale_factor: 1.0`) |
| Georeferenced PCD → `/map/pointcloud_map` | **SUCCESS** (`frame_id: map`, 1,004,344 points) |
| `finalMap.osm` → `/map/vector_map` | **FAILED** — Lanelet2 parse errors (missing way nodes, empty ways, broken `traffic_light` regulatory elements). **Not** caused by the experimental YAML. |

**Overall:** partial — Autoware can load the experimental projector + georeferenced PCD together; Carter’s OSM needs repair/re-export before joint vector+PCD visualization.
