### Run detector:

ros2 run perception_cv aruco_cv --ros-args -p show_image:=false

ros2 run perception_cv aruco_cv --ros-args -p show_image:=true

### Run fake image pub:

ros2 run perception_cv fake_camera \
    --ros-args -p image_path:=/home/mtrn/MTRN4231_Project/images/baord2.png

ros2 run perception_cv fake_camera \
    --ros-args -p image_path:=/home/lachlan/diceCV/diceCVros2/images/board.jpg

This will publish the topic:
`/camera/camera/color/image_raw`

Note: Need to change image path