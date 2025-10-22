# DiceResult.msg

# Bounding box of the detected dice: (x, y, width, height)
int32 x
int32 y
int32 width
int32 height

int32 dice_number  # Detected dice number (1-6)
float32 confidence # YOLO detection confidence (0.0 to 1.0)

geometry_msgs/Pose pose  # Pose of the dice in world frame
