# Engineering Loading Dock map (first KISS-ICP pass)

- **`pointcloud_map.pcd`** — first Autoware-style PCD from `engg_loading_dock_01_gnss` via isolated KISS-ICP.
- **Frame:** KISS-ICP odometry (not georeferenced to GNSS/Lanelet2 yet).
- **Docs:** `docs/MAPPING_STEPS.md`
- **Quality:** `results/quality_deep.json`, preview `results/topdown_preview.png`
- **Venv:** `source venv/bin/activate` (does not affect Autoware)

Do not confuse with `sample-map-rosbag` — that setup is untouched.
