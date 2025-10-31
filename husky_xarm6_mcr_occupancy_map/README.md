# Husky xArm6 Occupancy Mapping

## Overview

This package provides **octomap-based 3D occupancy mapping** with MoveIt2 integration for the Husky xArm6 mobile manipulator. It wraps the `octomap` library with thread-safe access and provides ROS2 nodes for real-time mapping.

## Architecture

### Design Philosophy

The architecture follows MoveIt2's occupancy mapping design but is **framework-agnostic**:

```
┌─────────────────────────────────────────────────────────┐
│                   Applications                           │
│  ┌──────────────────┐      ┌──────────────────────┐    │
│  │  MoveIt2         │      │  Standalone          │    │
│  │  Planning Scene  │      │  Navigation          │    │
│  └──────────────────┘      └──────────────────────┘    │
└─────────────────────────────────────────────────────────┘
                           ▲
                           │ octomap_msgs
                           │
┌─────────────────────────────────────────────────────────┐
│              OccupancyMapMonitor                         │
│  ┌──────────────────────────────────────────────┐      │
│  │  Coordinates updates, manages lifecycle      │      │
│  └──────────────────────────────────────────────┘      │
│          ▲                    ▲                          │
│          │                    │                          │
│  ┌───────────────┐    ┌───────────────┐                │
│  │PointCloud     │    │ Depth Image   │                │
│  │Updater        │    │ Updater       │                │
│  └───────────────┘    └───────────────┘                │
│          │                    │                          │
│          └────────────────────┘                          │
│                     ▼                                    │
│         ┌───────────────────────┐                       │
│         │  OccupancyMapTree     │                       │
│         │  (Thread-safe octree) │                       │
│         └───────────────────────┘                       │
└─────────────────────────────────────────────────────────┘
                           │
                           ▼
                    ┌─────────────┐
                    │  Visualizer │
                    │  (RViz)     │
                    └─────────────┘
```

### Key Components

#### 1. **OccupancyMapTree** (Core)
- **Purpose**: Thread-safe wrapper around `octomap::OcTree`
- **Features**:
  - Shared/exclusive locking (`std::shared_mutex`)
  - Update callbacks for subscribers
  - Inherits full octomap functionality
  - Framework-agnostic (no ROS dependencies)

```cpp
// Thread-safe access
auto lock = tree->reading();  // RAII read lock
size_t occupied = tree->getNumOccupiedNodes();

tree->lockWrite();
tree->insertRay(...);  // Modify octree
tree->unlockWrite();
tree->triggerUpdateCallback();  // Notify subscribers
```

#### 2. **OccupancyMapMonitor** (Coordinator)
- **Purpose**: Manages octree and sensor updaters
- **Responsibilities**:
  - Owns the `OccupancyMapTree`
  - Coordinates multiple sensor updaters
  - Provides save/load functionality
  - Thread-safe map access
  - Lifecycle management (start/stop)

```cpp
// Create monitor
auto monitor = std::make_shared<OccupancyMapMonitor>(node, tf_buffer, params);

// Add sensor updaters
auto pc_updater = std::make_shared<PointCloudUpdater>(...);
monitor->addUpdater(pc_updater);

// Start processing
monitor->startMonitor();
```

#### 3. **OccupancyMapUpdater** (Base Class)
- **Purpose**: Abstract interface for sensor-specific updaters
- **Pattern**: Strategy pattern for different sensor types
- **Implementations**:
  - `PointCloudUpdater`: Processes `sensor_msgs/PointCloud2`
  - `DepthImageUpdater`: Processes depth images (future)
  - Custom updaters: Extensible

```cpp
class OccupancyMapUpdater {
  virtual void start() = 0;   // Begin processing
  virtual void stop() = 0;    // Stop processing  
  virtual std::string getType() const = 0;
};
```

#### 4. **PointCloudUpdater** (Concrete Implementation)
- **Purpose**: Update octree from point cloud messages
- **Features**:
  - Subscribes to `sensor_msgs/PointCloud2`
  - Transforms points to map frame (TF2)
  - Ray casting: sensor origin → hit points
  - Configurable filtering (range, ground plane)
  - Downsampling for performance

**Algorithm**:
```
For each PointCloud2 message:
  1. Transform points to map_frame using TF2
  2. Filter points (range, ground plane)
  3. For each point:
     a. Cast ray from sensor origin to point
     b. insertRay() updates free space + occupied endpoint
  4. Trigger update callback
```

#### 5. **OccupancyMapVisualizer** (Debugging)
- **Purpose**: Publish RViz markers for visualization
- **Features**:
  - Occupied voxels (blue cubes)
  - Optional free voxels (green, transparent)
  - Updates on map changes
  - Lightweight for debugging

---

## MoveIt2 Integration

### How It Plugs Into MoveIt Planning Scene

MoveIt2 expects `octomap_msgs/msg/Octomap` messages on a topic. The planning scene subscribes to this and updates collision checking.

**Integration Flow**:
```
PointCloud2 → PointCloudUpdater → OccupancyMapTree
                                        ↓
                              Update callback triggered
                                        ↓
                           Serialize to octomap_msgs::Octomap
                                        ↓
                              Publish on /octomap_binary
                                        ↓
                          MoveIt PlanningScene subscribes
                                        ↓
                         Collision world updated
```

**Key Files**:
1. `moveit_octomap_server.cpp`: Publishes `octomap_msgs/Octomap`
2. `occupancy_map_server.cpp`: Standalone version (no MoveIt dependency)

**MoveIt Configuration** (in `move_group` launch):
```yaml
# sensors_3d.yaml
sensors:
  - sensor_plugin: occupancy_map_monitor/PointCloudOctomapUpdater
    point_cloud_topic: /firefly_left/points2
    max_range: 5.0
    padding_scale: 1.0
    padding_offset: 0.03
    filtered_cloud_topic: filtered_cloud
```

---

## Usage

### 1. Standalone Occupancy Mapping

Just build the map and visualize:

```bash
ros2 launch husky_xarm6_mcr_occupancy_map occupancy_map.launch.py
```

**Publishes**:
- `/occupancy_map/markers` - Visualization markers
- `/octomap_binary` - Binary octomap message

### 2. MoveIt Integration

Connect to MoveIt planning scene:

```bash
ros2 launch husky_xarm6_mcr_occupancy_map moveit_octomap.launch.py
```

**Parameters**:
- `pointcloud_topic`: Input point cloud (default: `/firefly_left/points2`)
- `map_frame`: Fixed frame (default: `map`)
- `resolution`: Voxel size (default: `0.1` m)
- `max_range`: Maximum sensor range (default: `5.0` m)
- `prob_hit`: Hit probability (default: `0.7`)
- `prob_miss`: Miss probability (default: `0.4`)

---

## Parameters

### Octomap Configuration

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `resolution` | double | 0.1 | Voxel size (m) |
| `map_frame` | string | "map" | Fixed frame |
| `max_range` | double | 5.0 | Max sensor range (m) |
| `min_range` | double | 0.1 | Min sensor range (m) |
| `prob_hit` | double | 0.7 | P(occupied \| hit) |
| `prob_miss` | double | 0.4 | P(occupied \| miss) |
| `clamp_min` | double | 0.1192 | Min log-odds clamp |
| `clamp_max` | double | 0.971 | Max log-odds clamp |

### Filtering

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `filter_ground` | bool | false | Remove ground plane |
| `ground_filter_distance` | double | 0.04 | Distance to ground (m) |
| `ground_filter_angle` | double | 0.15 | Angle tolerance (rad) |

### Performance

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `pointcloud_downsample` | double | 0.05 | Downsample voxel size (m) |
| `update_rate` | double | 1.0 | Visualization update (Hz) |

---

## API Reference

### C++ API

```cpp
#include <husky_xarm6_mcr_occupancy_map/occupancy_map_monitor.hpp>

// Create parameters
OccupancyMapParameters params;
params.resolution = 0.1;
params.map_frame = "map";
params.max_range = 5.0;

// Create monitor
auto monitor = std::make_shared<OccupancyMapMonitor>(node, tf_buffer, params);

// Add pointcloud updater
auto updater = std::make_shared<PointCloudUpdater>(
    node, tf_buffer, "/firefly_left/points2", params);
monitor->addUpdater(updater);

// Set callback for map updates
monitor->setUpdateCallback([]() {
    // Called whenever map changes
    // Publish to MoveIt, update visualization, etc.
});

// Start processing
monitor->startMonitor();

// Access octree (thread-safe)
auto tree = monitor->getOcTreePtr();
{
    auto lock = tree->reading();  // RAII lock
    size_t occupied = tree->getNumOccupiedNodes();
}

// Save/load
monitor->saveMap("/path/to/map.bt");
monitor->loadMap("/path/to/map.bt");
```

---

## File Structure

```
husky_xarm6_mcr_occupancy_map/
├── include/husky_xarm6_mcr_occupancy_map/
│   ├── occupancy_map_tree.hpp          # Thread-safe octree wrapper
│   ├── occupancy_map_monitor.hpp       # Map coordinator
│   ├── occupancy_map_updater.hpp       # Base updater class
│   ├── pointcloud_updater.hpp          # PointCloud2 updater
│   └── occupancy_map_visualizer.hpp    # RViz visualization
├── src/
│   ├── occupancy_map_tree.cpp          # Core implementation
│   ├── occupancy_map_monitor.cpp       # Monitor logic
│   ├── occupancy_map_updater.cpp       # Base updater
│   ├── pointcloud_updater.cpp          # PointCloud processing
│   ├── occupancy_map_visualizer.cpp    # Markers
│   ├── occupancy_map_server.cpp        # Standalone node
│   └── moveit_octomap_server.cpp       # MoveIt integration node
├── launch/
│   ├── occupancy_map.launch.py         # Standalone launch
│   └── moveit_octomap.launch.py        # MoveIt integration
├── config/
│   └── occupancy_map_params.yaml       # Default parameters
└── CMakeLists.txt
```

---

## Comparison to MoveIt's Implementation

| Feature | MoveIt (moveit_ros_occupancy_map_monitor) | This Package |
|---------|-------------------------------------------|--------------|
| **Core** | Uses `octomap::OcTree` | Wraps `octomap::OcTree` with same interface |
| **Thread Safety** | `OccMapTree` with `shared_mutex` | `OccupancyMapTree` with `shared_mutex` (identical) |
| **Plugin System** | Pluginlib for sensor types | Direct instantiation (simpler) |
| **Middleware** | `MiddlewareHandle` abstraction | Direct ROS2 integration |
| **MoveIt Integration** | Built-in to `move_group` | External node publishing `octomap_msgs` |
| **Flexibility** | Tightly coupled to MoveIt | Standalone + MoveIt compatible |
| **Complexity** | ~3000 LOC | ~1500 LOC (simplified) |

---

## Thread Safety

All map access is **thread-safe**:

```cpp
// Multiple threads can read simultaneously
void thread1() {
    auto lock = tree->reading();  // Shared lock
    octomap::point3d p(0, 0, 0);
    auto node = tree->search(p);
}

void thread2() {
    auto lock = tree->reading();  // Another shared lock (OK)
    size_t occupied = tree->getNumOccupiedNodes();
}

// Only one thread can write
void writer_thread() {
    auto lock = tree->writing();  // Exclusive lock
    tree->insertRay(...);         // Modify octree
}  // Lock released automatically (RAII)
```

---

## Performance Tuning

### For Real-Time Operation
- Increase `resolution` (coarser voxels)
- Enable `pointcloud_downsample`
- Reduce `max_range`
- Lower visualization `update_rate`

### For High Accuracy
- Decrease `resolution` (finer voxels)
- Disable downsampling
- Tune `prob_hit` / `prob_miss` for your sensor
- Increase `max_range`

---

## Future Extensions

1. **Multi-Resolution**: Adaptive octree depth based on region
2. **Temporal Decay**: Forget old observations over time
3. **Ground Plane Removal**: RANSAC-based filtering
4. **Multiple Sensors**: Fusion of multiple point clouds
5. **Semantic Mapping**: Object-level occupancy

---

## References

1. **Octomap**: Hornung et al., "OctoMap: An Efficient Probabilistic 3D Mapping Framework", Autonomous Robots, 2013
2. **MoveIt2 Occupancy Mapping**: https://github.com/moveit/moveit2/tree/main/moveit_ros/occupancy_map_monitor
3. **ROS Octomap**: http://wiki.ros.org/octomap

---

## License

MIT
