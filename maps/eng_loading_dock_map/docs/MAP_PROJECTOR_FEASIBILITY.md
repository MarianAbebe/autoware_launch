# map_projector_info.yaml — feasibility (NO)

**Date:** 2026-08-12  
**Decision:** **Do not generate `map_projector_info.yaml`.**  
**Scope:** Artifacts under `eng_loading_dock_map` only. Autoware / sensor kits / sample setup / PCD untouched.

## Artifacts inspected

| File | Role |
|------|------|
| `finalMap.osm` | Carter Lanelet2 (WGS84 `lat`/`lon` nodes) |
| `pointcloud_map.pcd` | KISS-ICP local odometry frame |
| Bag `/novatel/oem7/fix` | RTK-class `NavSatFix` (~51.081, −114.133) |
| Autoware `autoware_map_projection_loader` README + loader code | Official projector schema |

## 1. What coordinate system does `finalMap.osm` use?

- Standard OSM/Lanelet2 **geographic** encoding: every node has WGS84 **`lat` / `lon`**.
- **No** `local_x` / `local_y`, **no** `geoReference`, **no** embedded projector.
- Sampled bounds ≈ lat **[51.0664, 51.0831]**, lon **[−114.1526, −114.1160]** (Calgary).
- Autoware’s deprecated OSM fallback treats non-zero lat/lon maps as **MGRS** candidates — but that is not sufficient to write a correct YAML here (see below).

## 2. What does GNSS provide?

From `/novatel/oem7/fix` (status=2):

- Mean ≈ **51.0809°, −114.1327°**, alt ≈ **1094.4 m**
- Track lies **inside** the OSM bounding box
- Enough to locate the site on Earth and to compute MGRS/UTM **for those points**
- **Not** enough to prove which Autoware `projector_type` Carter authored against, or to place the **PCD** into that frame

## 3. Official Autoware `map_projector_info.yaml` requirements

Supported types (from loader + README):

| Type | Required fields |
|------|-----------------|
| `MGRS` | `vertical_datum`, `mgrs_grid` — map must lie in **one** 100 km MGRS grid |
| `LocalCartesianUTM` | `vertical_datum`, `map_origin.{latitude,longitude}` (altitude forced to 0 in loader) |
| `LocalCartesian` | same origin fields (GNSS poser support limited) |
| `TransverseMercator` | origin + optional `scale_factor` |
| `Local` | no geodesy (breaks GNSS localization) |

Expected map package:

```text
map_path/
  lanelet2_map.osm          # (name may be overridden)
  pointcloud_map.pcd        # MUST be in the same projected map frame
  map_projector_info.yaml
```

Lanelet2 loader projects OSM lat/lon into the frame from `/map/map_projector_info`. The PCD is **not** reprojected by Autoware; it must already match.

## 4. Decision: NO — cannot correctly complete the package

### Blocker A — OSM spans **two** MGRS grids (MGRS illegal for this file)

`GeoConvert -m` on OSM corners / GNSS:

| Location | MGRS | 100 km grid |
|----------|------|-------------|
| OSM SW | `11UPS99495…` | **11UPS** |
| OSM SE | `11UQS02058…` | **11UQS** |
| OSM NW | `11UPS99424…` | **11UPS** |
| OSM NE | `11UQS01986…` | **11UQS** |
| GNSS mean | `11UQS00829…` | **11UQS** |

Autoware README: *“It cannot be used with maps that span across two or more MGRS grids.”*

Therefore we **must not** write:

```yaml
projector_type: MGRS
mgrs_grid: 11UQS   # or 11UPS
```

That would be fabricating an invalid projector for Carter’s full OSM.

### Blocker B — Origin-based types need an **authoritative** origin we do not have

`LocalCartesianUTM` / `LocalCartesian` / `TransverseMercator` need a chosen `map_origin`.

- GNSS can suggest a lat/lon, but **which** origin Carter (or Vector Map Builder) used is **not** recorded in `finalMap.osm` or the bag.
- Picking GNSS mean / first fix would be an **arbitrary** choice, not a derived fact — forbidden by “do not guess.”

### Blocker C — PCD is not in any geodetic map frame

`pointcloud_map.pcd` XYZ spans roughly **[-76, 97] × [-54, 136] × [-7, 29] m** (KISS-ICP odometry).  
MGRS/UTM map frames for this site are **~1e5 m** scale. The cloud was never georeferenced to Lanelet2.

Even a perfect projector YAML would make Autoware project the OSM into MGRS/UTM while the PCD stays in a random local frame → **inconsistent map package** (vector map and pointcloud would not overlay; NDT/GNSS init would be wrong).

## What is still missing

1. **Carter’s actual projector choice + parameters** (from his Vector Map Builder / export settings), especially if not MGRS — e.g. `LocalCartesianUTM` origin used when building `finalMap.osm`.
2. **A pointcloud already expressed in that same projected frame** (georeferenced / GNSS-aligned rebuild), or a known rigid transform from KISS frame → map frame.
3. If MGRS is desired: either **crop/split** the OSM to a single 100 km grid (**11UQS** covers the loading-dock GNSS), or use a non-MGRS projector with a documented origin — still requires (1)+(2).

## Likely in Carter’s workflow?

Yes. Lanelet2 tools / Autoware Vector Map Builder normally fix projection when saving. Carter should have (or can re-export):

- projector type + `mgrs_grid` **or** `map_origin`
- optionally a PCD already built in that frame

Ask Carter for the `map_projector_info.yaml` (or VMB project settings) that matches `finalMap.osm`, and confirm how any prior pointcloud was georeferenced.

## Bottom line

| Question | Answer |
|----------|--------|
| Can we invent a YAML that “looks right”? | Technically yes — **we refuse** |
| Can we generate a **correct** projector from current artifacts alone? | **No** |
| Can we complete a consistent Autoware map package now? | **No** |

**No `map_projector_info.yaml` was written.**
