# CogProj Spatial Ore Array Message
# Array of spatially analyzed ore objects for a single observation frame.

std_msgs/Header header

# List of spatial ore estimates
SpatialOre[] ores

# Aggregate counts for introspection
uint32 localized_count
uint32 unlocalized_count
