int32 x               # top-left pixel of the bounding box in the warped board image
int32 y               # top-left pixel of the bounding box
int32 width           # bounding box width in pixels
int32 height          # bounding box height in pixels

float32 confidence    # 0–1, for colour-based detection you can just set 1.0

geometry_msgs/Pose pose  # pose of the cup in the world frame (for MoveIt etc.)
geometry_msgs/Pose drop_pose
