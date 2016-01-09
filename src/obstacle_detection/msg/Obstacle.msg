
# If it is believed an obstacle is the same as an already published one (or
# its properties changed!) it has the same id
uint32 unique_id

# Position and facing direction of object
geometry_msgs/Pose pose

# Every object has a circular "stop-zone" with a radius
float32 radius

# Movement speed directly in pose direction
float32 velocity
