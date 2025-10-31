# Probabilistic Occupancy Mapping

## Overview

This package implements **probabilistic 3D occupancy mapping** for mobile robot navigation and planning. Unlike simple binary maps, it uses **Bayesian inference** to handle sensor uncertainty and fuse multiple observations over time.

## Algorithm: Log-Odds Occupancy Mapping

### Mathematical Foundation

Each voxel stores a **log-odds** value representing occupancy probability:

```
log_odds = log(P(occupied) / P(free))
```

**Why log-odds?**
- Efficient Bayesian updates (addition instead of multiplication)
- Numerically stable (avoids probabilities near 0 or 1)
- Natural representation of uncertainty

### Bayesian Update Rule

When a sensor observes a voxel, we update using Bayes rule in log-odds form:

```
log_odds_new = log_odds_old + log_odds_measurement - log_odds_prior
```

Where:
- `log_odds_measurement` = `log(P(occ|hit) / P(free|hit))` for occupied voxels
- `log_odds_measurement` = `log(P(occ|miss) / P(free|miss))` for free voxels
- `log_odds_prior` = `log(0.5 / 0.5) = 0` (uniform prior)

### Ray Casting

For each point in the point cloud:

1. **Cast ray** from sensor origin to hit point using 3D DDA (Digital Differential Analyzer)
2. **Free space update**: All voxels along the ray are likely free → decrease log-odds
3. **Occupied update**: The hit point voxel is likely occupied → increase log-odds

```
Sensor -----> [Free] [Free] [Free] [Occupied]
Origin                                 Hit Point
```

### Clamping

To prevent over-confidence from repeated observations:

```
log_odds = clamp(log_odds, log_odds_min, log_odds_max)
```

Default: `P_min = 0.12`, `P_max = 0.97`

### Classification

Voxels are classified based on probability thresholds:

- **Occupied**: `P(occ) > 0.7` (log-odds > 0.85)
- **Free**: `P(occ) < 0.3` (log-odds < -0.85)
- **Unknown**: `0.3 ≤ P(occ) ≤ 0.7` (uncertain)

## Implementation Details

### Data Structures

1. **Voxel Grid (Dict)**: `{(ix, iy, iz): log_odds_value}`
   - Sparse storage: only observed voxels are stored
   - Fast lookup: O(1) for queries
   - Memory efficient: scales with observed space, not total space

2. **Open3D Octree**: Hierarchical spatial structure
   - Efficient spatial queries
   - Visualization support
   - Multi-resolution representation

### Parameters

#### Sensor Model
- `prob_hit = 0.7`: Probability that a voxel is occupied given a hit
- `prob_miss = 0.4`: Probability that a voxel is occupied given ray passes through
- `prob_prior = 0.5`: Prior probability (uniform)

#### Thresholds
- `occupied_threshold = 0.7`: Classification threshold for occupied
- `free_threshold = 0.3`: Classification threshold for free
- `prob_min = 0.12`: Minimum clamping probability
- `prob_max = 0.97`: Maximum clamping probability

#### Map Configuration
- `resolution = 0.1`: Voxel size in meters (10 cm)
- `max_depth = 16`: Maximum octree depth
- `downsample_voxel = 0.1`: Downsample input clouds for speed

### Computational Complexity

- **Ray casting**: O(n) per point, where n = ray length / resolution
- **Update**: O(1) per voxel (dict lookup + addition)
- **Total per cloud**: O(M × R) where M = points, R = avg ray length

## Multi-Resolution Mapping for Large Environments

### Problem: Apple Orchard Scale

- **Large area**: 10+ hectares (100,000+ m²)
- **Sparse observations**: Most space is free/unknown
- **Variable detail**: Fine resolution near trees, coarse elsewhere

### Solution: Adaptive Resolution

The octree naturally supports multi-resolution:

```
Root (coarse)
├── Quadrant 1 (medium)
│   ├── Subquadrant 1.1 (fine) ← Tree detail
│   └── Subquadrant 1.2 (medium)
└── Quadrant 2 (coarse) ← Open space
```

### Implementation Strategy

1. **Depth-based resolution**:
   ```python
   # Near trees: max depth (fine detail)
   max_depth = 16  # resolution = 0.1 m
   
   # Open areas: lower depth (coarse)
   depth = 8  # resolution = 1.0 m
   ```

2. **Pruning homogeneous regions**:
   - If all 8 child voxels have similar occupancy → collapse to parent
   - Saves memory in uniform areas (sky, ground, empty space)

3. **Region-of-interest (ROI) mapping**:
   ```python
   def should_refine(voxel_key):
       # Check if near tree or high-interest area
       if near_tree(voxel_key):
           return True  # Fine resolution
       return False  # Coarse resolution
   ```

### Future Extensions

For orchard-scale mapping, consider:

1. **Submaps**: Divide orchard into tiles, load on-demand
2. **Hierarchical updates**: Update coarse levels first, refine locally
3. **Disk-based storage**: Stream maps from/to disk for large areas
4. **Probabilistic pruning**: Remove low-confidence distant voxels

## Comparison: Deterministic vs Probabilistic

| Feature | Deterministic (Old) | Probabilistic (New) |
|---------|---------------------|---------------------|
| **Representation** | Binary sets (free/occ) | Log-odds probabilities |
| **Uncertainty** | None | Quantified via probability |
| **Conflicting obs** | Last wins | Bayesian fusion |
| **Confidence** | Binary | Continuous (0-1) |
| **Sensor noise** | Not handled | Modeled via prob_hit/miss |
| **Memory** | Set storage | Dict storage (similar) |
| **Updates** | Overwrite | Incremental fusion |
| **Multi-resolution** | Not supported | Native via octree |

## Usage

### Basic Launch

```bash
ros2 launch husky_xarm6_mcr_planning occupancy_map.launch.py
```

### Custom Parameters

```bash
ros2 launch husky_xarm6_mcr_planning occupancy_map.launch.py \
  resolution:=0.05 \
  prob_hit:=0.8 \
  prob_miss:=0.3 \
  occupied_threshold:=0.8
```

### For Large Sparse Environments (Orchard)

```bash
ros2 launch husky_xarm6_mcr_planning occupancy_map.launch.py \
  resolution:=0.2 \
  max_depth:=12 \
  downsample_voxel:=0.3
```

## Visualization

The node publishes three marker types:

1. **Occupied voxels** (blue, opaque): High confidence obstacles
2. **Free voxels** (green, transparent): Confirmed free space
3. **Unknown voxels** (yellow, transparent): Unexplored regions

Topics:
- `/occupancy_map/markers` - MarkerArray with all voxel types
- `/occupancy_map/point_cloud` - Accumulated observations

## Information Gain for NBV Planning

The map provides `compute_information_gain()` for next-best-view planning:

```python
# Cast rays through camera frustum
gain = occupancy_map.compute_information_gain(candidate_pose)

# Higher gain = more unknown voxels visible
# Use for viewpoint selection in NBV planner
```

## References

1. **Octomap**: Hornung et al., "OctoMap: An Efficient Probabilistic 3D Mapping Framework", Autonomous Robots, 2013
2. **Probabilistic Robotics**: Thrun, Burgard, Fox, Chapter 9 (Occupancy Grid Mapping)
3. **Log-odds**: Moravec, Elfes, "High Resolution Maps from Wide Angle Sonar", IEEE ICRA, 1985

## Performance Tips

### For Real-Time Operation
- Increase `downsample_voxel` to reduce points
- Decrease `max_depth` for coarser resolution
- Limit marker publishing to occupied voxels only

### For High Accuracy
- Decrease `resolution` for finer detail
- Tune `prob_hit` and `prob_miss` to your sensor
- Increase `max_depth` for more detail levels

### For Large Environments
- Use coarse `resolution` in open areas
- Implement ROI-based refinement
- Consider tiled submaps for memory efficiency
