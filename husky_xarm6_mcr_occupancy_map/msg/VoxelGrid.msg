# Simple voxel grid representation for NBV planning
# This format is easy to extend with semantic labels later

std_msgs/Header header

# Grid parameters
float64 resolution          # voxel size in meters
float64 origin_x            # grid origin
float64 origin_y
float64 origin_z

# Bounding box
float64 min_x
float64 min_y
float64 min_z
float64 max_x
float64 max_y
float64 max_z

# Voxel data (parallel arrays)
# For each occupied/free voxel, store its center position and state
float32[] voxel_x           # x coordinates of voxel centers
float32[] voxel_y           # y coordinates of voxel centers
float32[] voxel_z           # z coordinates of voxel centers
int8[] voxel_state          # -1=free, 0=unknown, 1=occupied

# Future: add semantic labels
# uint8[] voxel_label        # semantic class ID for each voxel
