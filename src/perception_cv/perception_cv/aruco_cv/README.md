### Run detector:

```bash
ros2 run perception_cv aruco_cv --ros-args -p show_image:=false

ros2 run perception_cv aruco_cv --ros-args -p show_image:=true
```

### Run fake image pub:

```bash
ros2 run perception_cv fake_camera \
    --ros-args -p image_path:=/home/mtrn/MTRN4231_Project/images/baord2.png
```

This will publish the topic:
`/camera/camera/color/image_raw`

Note: Need to change image path

### For warped board:

```bash
ros2 run perception_cv fake_camera \
    --ros-args \
    -p image_path:=/home/mtrn/MTRN4231_Project/images/warp2.png \
    -p topic:=board/warped_image
```