int32 id
geometry_msgs/Point position
geometry_msgs/Vector3 velocity

# Time to collision (TTC) in seconds.
# If negative or very large, object is not expected to collide.
float64 ttc

# Bounding box
stereo_perception_msgs/BoundingBox bounding_box

# Bounding box marker for visualization
visualization_msgs/Marker bb_marker
