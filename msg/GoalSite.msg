# The ID of the visual marker coupled with the
# goal position. Currently between 0 and 1024.
#  0-999          4x4 ArUCO Tags with ID ranging from 0 to 999.
# (0-999, 0-999)  A pair of ArUCO tags symbolizing a gate, IDs are forward facing left to right.
#  1002           No visual marker at site.
uint16                       marker_count
geometry_msgs/PoseStamped[]  marker_poses
uint16[]                     marker_ids