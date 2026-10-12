# CogProj Spatial Ore Message
# Represents a detected/tracked ore object with associated physical 3D coordinates.

int32 track_id
int32 class_id
string class_name
float32 confidence

# 3D Position in SLAM map frame (valid only if is_localized == true)
geometry_msgs/Point position_map

# 3D Position in robot base frame (base_footprint / base_link)
geometry_msgs/Point position_base

# Estimated distance from camera/robot in meters
float32 estimated_distance

# Localization validity flag
# If false, position_map and position_base are NOT valid (zeroed), and no fabricated coordinates are emitted.
bool is_localized

# Detailed diagnostic status of localization
# Examples: "LOCALIZED", "UNCALIBRATED_CAMERA", "NO_CAMERA_INFO", "NO_LIDAR_SCAN", "NO_LIDAR_POINTS", "TF_LOOKUP_FAILED", "STALE_SENSOR_DATA", "OUT_OF_RANGE"
string localization_status
