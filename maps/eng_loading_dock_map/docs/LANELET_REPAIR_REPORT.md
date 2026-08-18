# Lanelet2 repair report — `finalMap.osm` → `finalMap_repaired.osm`

**Date:** 2026-08-12  
**Original preserved:** `finalMap.osm` (untouched)  
**Repaired output:** `finalMap_repaired.osm`  
**Artifacts:** `intermediate/lanelet_repair/`  
**Not modified:** PCD, projector YAML, Autoware source, sensor kits

## Validation before repair

Custom integrity scan (same failure modes as Autoware loader): **31 errors**.

### Broken way geometry (14)

| Way ID | Issue |
|--------|-------|
| 10736, 22341, 22348, 22422, 39139, 39145, 39146, 54565, 54566, 57375, 57377 | Empty ways (0 nodes) |
| 56837, 56841, 56845 | Empty ways tagged `type=traffic_light` |

### Missing references (1)

| Way ID | Issue |
|--------|-------|
| 1511982565 | References nonexistent node `13817418538` |

### Disconnected / incomplete lanelets (12 messages / 7 lanelets)

| Lanelet ID | Issue |
|------------|-------|
| 10738 | No left border (only right + bad traffic-light reg) |
| 22342, 22349, 22423, 39147, 54567 | Empty members (no left/right) |
| 39140 | Only right border |

### Traffic light inconsistencies (4)

| RegulatoryElement ID | Issue |
|----------------------|-------|
| 57164, 57166 | `subtype=traffic_light` but no `refers` linestring (only `ref_line`) |
| 57376, 57378 | Empty traffic_light regulatory elements |

### Other categories

- Duplicate IDs: none found  
- Invalid relations (beyond above): none remaining after targeted fixes  
- Cascaded “Failed to get id 57164/57166” on lanelets 10558/10738: caused by invalid regs above

## Minimal fixes applied (29 actions)

| Action | Count | Intent |
|--------|------:|--------|
| Delete empty ways | 14 | Remove unloadable primitives; no redraw |
| Remove missing `nd` from way 1511982565 | 1 | Drop dangling ref |
| Delete degenerate way (&lt;2 pts) 1511982565 | 1 | After strip, not a valid linestring |
| Delete invalid traffic_light regulatory elements | 4 | Cannot invent missing signal geometry |
| Strip missing members from relations | 2 | Drop refs to deleted regs (e.g. lanelet 10558) |
| Delete broken lanelets | 7 | Cannot invent missing left/right borders |

**Road geometry preserved** for all intact lanelets/ways. Only empty/dangling/incomplete objects were removed. No centerlines were redrawn.

## Validation after repair

- Integrity scan: **0 errors**
- Autoware `autoware_lanelet2_map_loader`:

```
Succeeded to load lanelet2_map. Map is published.
```

- Topic `/map/vector_map` (`autoware_map_msgs/LaneletMapBin`) published, `frame_id: map`
- Harmless warn: OSM has no `format_version` (null) — does not block load (`allow_unsupported_version` path)

## Remaining / not auto-repairable (by design)

None blocking load. Intentionally **not** reconstructed:

1. Missing left/right borders for deleted lanelets (would require redrawing).  
2. Empty traffic-light geometries (would require surveyed signal positions).  
3. `format_version` tag (cosmetic; optional for Carter to add later).

Ask Carter to re-author those few lanelets/signals in Vector Map Builder if they are operationally needed.

## Files for tomorrow

| Path | Role |
|------|------|
| `finalMap.osm` | Original (Carter) |
| `finalMap_repaired.osm` | Autoware-loadable |
| `experimental_viz_package/lanelet2_map_repaired.osm` | Symlink for loader test |
| `intermediate/lanelet_repair/*.json` | Before/after validation + actions |
| `scripts/repair_lanelet_osm.py` | Reproducible repair |
