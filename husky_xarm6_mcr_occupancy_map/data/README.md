# Saved Octomap Files

This directory stores saved octomap files for offline NBV planner development.

## Workflow

### 1. Save Current Octomap (from Gazebo simulation)

While your simulation is running with the octomap being built:

```bash
ros2 launch husky_xarm6_mcr_occupancy_map save_octomap.launch.py
```

This will save the current octomap to `data/octomap_YYYYMMDD_HHMMSS.bt`

**Custom filename:**
```bash
ros2 launch husky_xarm6_mcr_occupancy_map save_octomap.launch.py \
    filename:=my_map.bt \
    octomap_topic:=/octomap_binary
```

### 2. Load Saved Octomap (for NBV development)

Load the most recent saved map without running Gazebo:

```bash
ros2 launch husky_xarm6_mcr_occupancy_map load_octomap.launch.py
```

**With specific file:**
```bash
ros2 launch husky_xarm6_mcr_occupancy_map load_octomap.launch.py \
    octomap_file:=$(ros2 pkg prefix husky_xarm6_mcr_occupancy_map)/share/husky_xarm6_mcr_occupancy_map/data/octomap_20260116_143022.bt
```

### 3. Run NBV Planner Demo with Loaded Map

Once the map is loaded and publishing on `/octomap_binary`:

```bash
ros2 launch husky_xarm6_mcr_nbv_planner octomap_interface_demo.launch.py \
    octomap_topic:=/octomap_binary \
    visualization_topic:=nbv_markers
```

## File Formats

- `.bt` - Binary compressed format (recommended, smaller file size)
- `.ot` - Uncompressed XML format (human-readable, larger)

## Tips

- Save maps at interesting exploration states (partially explored environments)
- The loader automatically finds and loads the most recent timestamped file
- Files are stored in `install/share/husky_xarm6_mcr_occupancy_map/data/`
- The loaded map is static - changes won't be reflected unless you reload
- Use `use_sim_time:=false` when loading maps standalone (no Gazebo)
