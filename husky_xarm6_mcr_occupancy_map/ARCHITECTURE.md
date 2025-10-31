# Architecture Diagram

## Complete System Architecture

```
┌────────────────────────────────────────────────────────────────────┐
│                        GAZEBO SIMULATION                            │
│                                                                      │
│   ┌──────────────┐        ┌──────────────┐                         │
│   │  Husky Base  │        │  xArm6       │                         │
│   │              │────────│  Manipulator │                         │
│   └──────────────┘        └──────────────┘                         │
│          │                                                          │
│          │                                                          │
│   ┌──────▼──────────────────────────────────┐                      │
│   │   Multi-Camera Rig (Firefly x3)        │                      │
│   │   - firefly_left/points2                │                      │
│   │   - firefly_center/points2              │                      │
│   │   - firefly_right/points2               │                      │
│   └─────────────────────────────────────────┘                      │
└────────────────────────────────────────────────────────────────────┘
                            │
                            │ sensor_msgs/PointCloud2
                            │
                            ▼
┌────────────────────────────────────────────────────────────────────┐
│              HUSKY_XARM6_MCR_OCCUPANCY_MAP PACKAGE                  │
│                                                                      │
│   ┌──────────────────────────────────────────────────────────┐    │
│   │          occupancy_map_server (ROS2 Node)                 │    │
│   │                                                            │    │
│   │   ┌────────────────────────────────────────────────┐     │    │
│   │   │      OccupancyMapMonitor                       │     │    │
│   │   │  (Coordinator)                                 │     │    │
│   │   │                                                 │     │    │
│   │   │  ┌─────────────────────────────────────┐      │     │    │
│   │   │  │   PointCloudUpdater                 │      │     │    │
│   │   │  │   - Subscribes to point cloud       │      │     │    │
│   │   │  │   - Transforms to map frame (TF2)   │      │     │    │
│   │   │  │   - Filters (range, ground)         │      │     │    │
│   │   │  │   - Calls insertRay()              │      │     │    │
│   │   │  └─────────────────────────────────────┘      │     │    │
│   │   │                    │                           │     │    │
│   │   │                    ▼                           │     │    │
│   │   │  ┌─────────────────────────────────────┐      │     │    │
│   │   │  │   OccupancyMapTree                  │      │     │    │
│   │   │  │   (Thread-safe octomap wrapper)     │      │     │    │
│   │   │  │                                      │      │     │    │
│   │   │  │   Inherits: octomap::OcTree         │      │     │    │
│   │   │  │   Adds:                             │      │     │    │
│   │   │  │   - std::shared_mutex (R/W locks)   │      │     │    │
│   │   │  │   - Update callbacks                │      │     │    │
│   │   │  │   - MoveIt compatibility            │      │     │    │
│   │   │  └─────────────────────────────────────┘      │     │    │
│   │   │                    │                           │     │    │
│   │   │                    │ Triggers callback         │     │    │
│   │   │                    ▼                           │     │    │
│   │   │  ┌─────────────────────────────────────┐      │     │    │
│   │   │  │   Update Callback                   │      │     │    │
│   │   │  │   - Serialize octree                │      │     │    │
│   │   │  │   - Publish octomap_msgs            │      │     │    │
│   │   │  │   - Trigger visualization           │      │     │    │
│   │   │  └─────────────────────────────────────┘      │     │    │
│   │   └────────────────────────────────────────────────┘     │    │
│   └──────────────────────────────────────────────────────────┘    │
│                            │                                        │
│                            │ publishes                             │
│                            ▼                                        │
│   ┌──────────────────────────────────────────────────────────┐    │
│   │            OccupancyMapVisualizer                         │    │
│   │            (Optional, for debugging)                      │    │
│   └──────────────────────────────────────────────────────────┘    │
└────────────────────────────────────────────────────────────────────┘
                            │
                ┌───────────┴───────────┐
                │                       │
                ▼                       ▼
    octomap_msgs/Octomap    visualization_msgs/MarkerArray
    /octomap_binary         /occupancy_map/markers
                │                       │
                │                       │
                ▼                       ▼
┌───────────────────────────┐   ┌─────────────────┐
│  MoveIt2 move_group       │   │     RViz2       │
│                            │   │  Visualization  │
│  ┌──────────────────────┐ │   │                 │
│  │  Planning Scene      │ │   │  - Blue cubes   │
│  │  - Collision world   │ │   │    (occupied)   │
│  │  - Octomap layer     │ │   │  - Green cubes  │
│  │  - Updates from      │ │   │    (free)       │
│  │    /octomap_binary   │ │   └─────────────────┘
│  └──────────────────────┘ │
│                            │
│  ┌──────────────────────┐ │
│  │  Motion Planning     │ │
│  │  - Collision check   │ │
│  │    against octree    │ │
│  │  - Avoids obstacles  │ │
│  └──────────────────────┘ │
└───────────────────────────┘
```

## Data Flow Diagram

```
SENSOR DATA → PROCESSING → STORAGE → DISTRIBUTION
────────────────────────────────────────────────

1. Point Cloud Arrival
   sensor_msgs/PointCloud2
   {header: {frame_id: "firefly_left_camera_optical_frame"}}
                │
                ▼
2. PointCloudUpdater::pointCloudCallback()
   - Lookup TF: camera_frame → map_frame
   - Parse PointCloud2 to vector<point3d>
   - Apply transform to each point
   - Get sensor origin from transform
                │
                ▼
3. Ray Casting
   for each point:
     octomap::point3d origin = sensor_origin
     octomap::point3d endpoint = transformed_point
     
     tree->lockWrite()
     tree->insertRay(origin, endpoint)
       ↓
       Inside octree:
       - Mark voxels along ray as FREE
       - Mark endpoint voxel as OCCUPIED
       - Update probabilities (log-odds)
     tree->unlockWrite()
                │
                ▼
4. Update Notification
   tree->triggerUpdateCallback()
                │
                ▼
5. Serialization
   octomap_msgs::Octomap msg
   tree->lockRead()
   octomap::binaryMapToMsgData(*tree, msg.data)
   tree->unlockRead()
                │
                ▼
6. Publishing
   ┌───────────┴────────────┐
   │                        │
   ▼                        ▼
octomap_pub->publish()   marker_pub->publish()
                │                        │
                ▼                        ▼
         MoveIt Scene              RViz Markers
```

## Thread Safety Model

```
CONCURRENT ACCESS TO OCTREE
───────────────────────────

Reader Thread 1                 Reader Thread 2
(Collision Check)               (Visualization)
     │                               │
     │ tree->lockRead()              │ tree->lockRead()
     ├─────────────────────────────┤
     │  SHARED READ ACCESS          │
     │  Both can read simultaneously│
     │  No blocking                 │
     ├─────────────────────────────┤
     │ tree->unlockRead()           │ tree->unlockRead()
     │                               │


Writer Thread
(Sensor Update)
     │
     │ tree->lockWrite()
     ├───────────────────────────
     │  EXCLUSIVE WRITE ACCESS
     │  No other readers or writers
     │  tree->insertRay(...)
     ├───────────────────────────
     │ tree->unlockWrite()
     │ tree->triggerUpdateCallback()
     │


RAII Helper Usage:
──────────────────

void someFunction() {
    auto lock = tree->reading();  // Acquires shared lock
    
    // Do read operations
    size_t n = tree->getNumOccupiedNodes();
    
}  // Lock automatically released (RAII)


void updateFunction() {
    auto lock = tree->writing();  // Acquires exclusive lock
    
    // Modify octree
    tree->insertRay(...);
    
}  // Lock automatically released (RAII)
```

## MoveIt Integration Flow

```
OCCUPANCY MAP → MOVEIT PLANNING SCENE
──────────────────────────────────────

Step 1: Your Node Publishes
────────────────────────────
occupancy_map_server
     │
     │ publishes @ 1 Hz
     ▼
/octomap_binary (octomap_msgs/Octomap)
{
  header: {frame_id: "map"}
  binary: true
  id: "OcTree"
  resolution: 0.1
  data: [binary octree data]
}


Step 2: MoveIt Subscribes
──────────────────────────
move_group node (MoveIt)
     │
     │ subscribes to /octomap_binary
     ▼
planning_scene_monitor
     │
     ▼
OccMapTree (inside MoveIt)
     │
     ▼
collision_detection::World
     │
     ▼
Added to Planning Scene


Step 3: Motion Planning Uses It
────────────────────────────────
plan_kinematic_path service called
     │
     ▼
ompl_interface::PlanningContext
     │
     ▼
collision_detection::CollisionEnv
     │
     ▼
Check robot against OccMapTree
     │
     ├─ Occupied voxels → Collision
     └─ Free voxels → Valid


Step 4: Trajectory Execution
─────────────────────────────
execute_trajectory service
     │
     ▼
Continuous collision checking
     │
     ▼
If octree updated (new obstacles):
  - E-stop if collision detected
  - Replan if needed
```

## File Dependency Graph

```
HEADERS
───────

occupancy_map_tree.hpp
    │
    │ (inherits)
    ▼
octomap::OcTree (external)


occupancy_map_updater.hpp
    │
    │ (depends on)
    ▼
occupancy_map_tree.hpp


pointcloud_updater.hpp
    │
    │ (inherits)
    ▼
occupancy_map_updater.hpp


occupancy_map_monitor.hpp
    │
    │ (depends on)
    ├─ occupancy_map_tree.hpp
    └─ occupancy_map_updater.hpp


occupancy_map_visualizer.hpp
    │
    │ (depends on)
    └─ occupancy_map_tree.hpp


IMPLEMENTATIONS
───────────────

*.cpp files implement their respective headers

EXECUTABLES
───────────

occupancy_map_server.cpp
    │
    │ (uses)
    ├─ occupancy_map_monitor
    ├─ pointcloud_updater
    └─ occupancy_map_visualizer


moveit_octomap_server.cpp
    │
    │ (uses)
    ├─ occupancy_map_monitor
    ├─ pointcloud_updater
    └─ octomap_msgs (for publishing)
```

## Summary

This architecture provides:
1. **Thread-safe** octree access for concurrent operations
2. **Framework-agnostic** core that works with any planning system
3. **MoveIt2 integration** via standard octomap_msgs
4. **Extensible** updater system for different sensor types
5. **Lightweight** visualization for debugging

The design follows established patterns from MoveIt2 while remaining simpler and more focused on your specific use case.
