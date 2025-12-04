# MTRN4231: Dice-Playing Robot Casino System

# 1. Table of Contents

- [2. Project Overview](#2-project-overview)
- [3. System Architecture](#3-system-architecture)
- [4. Technical Components](#4-technical-components)
- [5. Installation and Setup](#5-installation-and-setup)
- [6. Running the System](#6-running-the-system)
- [7. Results and Demonstration](#7-results-and-demonstration)
- [8. Discussion and Future Work](#8-discussion-and-future-work)
- [9. Contributors and Roles](#9-contributors-and-roles)
- [10. Repository Structure](#10-repository-structure)
- [11. References and Acknowledgements](#11-references-and-acknowledgements)

---

# 2. Project Overview

## 2.1 Problem Statement and Customer
Casinos continue to rely on human-operated table games. This results in high operational costs, inconsistent gameplay speed, and increased risk of human error. At the same time, players seek fast, transparent, and beginner-friendly gaming experiences, while casinos demand compact, reliable, and regulation-compliant automated solutions. There is currently a gap in the market for a small-footprint, visually verifiable, automated dice game suitable for high-density casino floors.
   
This project demonstrates the automation of the a game called "Odds & Evens Dice Game" as a proof-of-concept for introducing low-cost, high-turnover robotic casino games.

| Category | Requirement ID | Description |
|----------|----------------|-------------|
| Functional | F1 | Fully automated dice rolling and return to cup |
|  | F2 | Automatic evaluation of bets and payout calculation |
| | F3 | Visual confirmation of physical dice results |
| | F4 | Real-time display of game status, results, and betting windows |
| | F5 | Compatibility with standard casino betting interfaces (e.g. chips) |
| Performance| P1 | Short time from game start to dice roll |
|  | P2 | Low latency vision pipeline |
| | P3 | True randomness |
|| P4 | Transparent operation — dice visible throughout game cycle |
|| P5 | Pick-up accuracy and repeatability |
| User Experience | UX1 | Simple, intuitive gameplay suitable for novice users |
|| UX2 | Clear, readable on-screen instructions and outcomes |
|| UX3 | Visual feedback for game events |
|| UX4 | Accessible interface |


## 2.2 Robot Functionality Summary

At startup the game frontend is launched. The perception pipeline start by identifying the 4 ArUco that mark the corners of the game board. From this, it begins identfying the dice, cup, and player states. The board state should begin as seen below: with the dice in the cup and placed in the designated play area - which is the area covered in green felt - and the player sections should be clear of aruco markers and chips.

![System Overview](docs/media/aruco_raw.png)

The players should now place their ArUco marker ID's in a designated player section. The systems perception will automatically detect the player markers and populate the front end with player information.With our players in, they may being placing bets. Once again the systems perception will automatiaclly detect the bets and populate the front end based on the bets placed on the board. Once bets are placed, our players make their prediction on the dice outcome through the front end. Once all players have made a prediction the start button will become availible. 

![System Overview](docs/media/front_end.png)

After the start button is pressed, the robot will move to a designated 1st position and open the gripper. It will then move to pick up the cup based on the perception pipelines prediction of cup position and orientation. The robot will pick up the dice cup, lift it slightly make a roll and the place the cup back down in it's starting position. The robot will now return back to the designated 1st position. Now the yolo model works to identify the dice outcome which is then reported to the front end, confirming the results of the round and allowing dealers to handle payout. 

![System Overview](docs/media/game_state_3.jpg)

The robot now moves to each die, picking it up and placing it back in the cup, before returning to the home position.  

## 2.3 Demo Video
[![Watch the Demo](https://img.icons8.com/fluency/240/youtube-play.png)](https://drive.google.com/file/d/1irMjWqZdTdLUCx5GPTzw7JCPZO1h40ZF/view?usp=drive_link)

above is a sped up video showing the system performing 1 round of gameplay.  

---

# 3. System Architecture

## 3.1 ROS2 Node Graph

> TODO: Include a diagram of your ROS2 nodes, topics, services and actions (e.g. screenshot from `rqt_graph` or a custom schematic).

![ROS2 Node Graph](docs/diagrams/ros2_graph.png)

Brief description:

> TODO: One paragraph summarising the overall data flow (e.g. perception → brain → motion planning → execution → visualisation).

## 3.2 Package-Level Architecture

> TODO: Provide a package-level diagram showing how ROS2 packages interact (topic connections, service interactions, shared utilities).


![Package Architecture](docs/diagrams/package_architecture.png)

Describe, in a few bullet points:

- `perception_cv`: TODO – vision and board/dice/cup/player detection.
- `brain`: TODO – game logic, sequencing and coordination.
- `moveit_path_planner`: TODO – motion planning and execution for UR5e.
- `casino_dashboard` / web UI: TODO – external visualisation and user interaction.
- Any other relevant packages.

## 3.3 Behaviour Tree / State Machine

> TODO: Include a behaviour-tree or state-machine diagram describing the closed-loop behaviour across one full game round.
![Behaviour-tree of one full game close-loop](docs/diagrams/BehaviorTree.png)


## 3.4 Node Summaries

| Node Name                              | Package              | Role / Description                                                                                                                                                          | Key Topics / Interfaces                                                                                           |
|----------------------------------------|----------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------|
| `brain`                                | `brain`              | Central game controller. Runs the round state machine, coordinates perception, motion planning and gripper actions, and exposes a simple interface to the dashboard.      | Sub: `/dice_results` (DiceResults), `/cup_result` (CupResult), `/players` (Players) <br> Action client: `/moveit_path_plan` (Movement) <br> Service clients: `/gripper_cmd`, `/start_round` |
| `cartesian_move_demo` | `moveit_path_planner` | MoveIt-based motion planning and execution server for the UR5e. Handles both Cartesian and joint goals, applies joint constraints and collision objects, and executes plans. | Action server: `/moveit_path_plan` (custom_interface/Movement) <br> Uses MoveGroupInterface for `ur_manipulator` and PlanningScene collision objects                                      |
| `gripper_server`                       | `gripper`            | Gripper controller. Receives open/close requests and drives the physical end-effector (via microcontroller / IO) with success feedback.                  | Service server: `/gripper_cmd` (GripperCmd)                                                                       |                           |
| `dice_cv`                        | `perception_cv`      | Detects dice in the camera image, classifies their face values, and estimates 3D poses in the board/world frame for use by the brain and MoveIt planner.                  | Sub: `/camera/color/image_raw` <br> Pub: `/dice_results` (DiceResults) <br> Optional RViz markers (e.g. `/dice_markers`) |
| `cup_cv`                         | `perception_cv`      | Segments the cup using colour / HSV masks and blob geometry, estimates its centroid and orientation, and publishes a stable pose for grasping and placement.             | Sub: `/camera/color/image_raw` <br> Pub: `/cup_result` (CupResult) <br> Optional RViz markers                          |
| `aruco_cv`                       | `perception_cv`      | Detects ArUco markers on the game board, estimates the board pose, and maintains the board frame used for pixel-to-board and board-to-world transforms.                  | Sub: `/camera/color/image_raw` <br> Pub: board pose / calibration topic (e.g. `/board_pose`) <br> Broadcasts TF `world → board` |
| `player_cv`                      | `perception_cv`      | Identifies player ids using aruco detection, and chip locations using colour masking. Constructs and publishes a structured list of players and bets to the brain and dashboard.  | Sub: `/camera/color/image_raw` <br> Pub: `/players` (Players)                                                       |
| `rosbridge_websocket`                  | `rosbridge_server`   | Bridges ROS2 topics, services and actions to the web dashboard, enabling live game state, dice results and round status to be shown in the browser UI.                   | WebSocket server for dashboard <br> Forwards topics such as `/dice_results`, `/cup_result`, `/players`, game status  |


## 3.5 Custom Messages and Interfaces

---

### 3.5.1 Custom Messages

#### `DiceResult.msg`

| Field | Type | Description |
|-------|--------|-------------|
| `x, y, width, height` | `int32` | 2D pixel bounding box of the die |
| `dice_number` | `int32` | Detected die face value (1–6) |
| `confidence` | `float32` | YOLO detection confidence |
| `pose` | `geometry_msgs/Pose` | Die pose in the world frame |

**Usage:** Published by `dice_cv`; consumed by `brain` to compute outcomes and generate pick poses.

---

#### `DiceResults.msg`

| Field | Type | Description |
|-------|--------|-------------|
| `dice` | `DiceResult[]` | All detected dice in a frame or round |

**Usage:** Published on `/dice_results`; consumed by `brain` and the dashboard.

---

#### `CupResult.msg`

| Field | Type | Description |
|-------|--------|-------------|
| `x, y, width, height` | `int32` | Cup bounding box in warped board frame |
| `confidence` | `float32` | Detection confidence |
| `pose` | `geometry_msgs/Pose` | Cup pose in the world frame |
| `drop_pose` | `geometry_msgs/Pose` | Predefined cup drop location |

**Usage:** Published by `cup_cv`; consumed by `brain` and passed to MoveIt.

---

#### `Player.msg`

| Field | Type | Description |
|-------|--------|-------------|
| `player_id` | `string` | Logical identifier (ArUco ID) |
| `position` | `int32` | Table position (1–3) |
| `bet_is_odd` | `bool` | `true` = odd, `false` = even |
| `bet_colors` | `string[]` | Chip colours |
| `bet_x`, `bet_y` | `int32[]` | Chip centroids (pixels) |
| `bet_diameter_px` | `int32[]` | Chip diameters (pixels) |

**Usage:** Published by `player_cv` and used by `brain` and UI.

---

#### `Players.msg`

| Field | Type | Description |
|-------|--------|-------------|
| `players` | `Player[]` | List of detected players and their bets |

**Usage:** Shared between `player_cv`, `brain`, and UI.

---

#### `RoundResult.msg`

| Field | Type | Description |
|-------|--------|-------------|
| `is_odd` | `bool` | Whether round result is odd |
| `is_complete` | `bool` | Whether round has finished |
| `dice_results` | `DiceResults` | Final dice state |

**Usage:** Published at end of round for UI updates and payout calculation.

---

### 3.5.2 Custom Actions

#### `Movement.action`

| Section | Fields / Meaning |
|---------|------------------|
| **Goal** | `string command`, `float64[] positions`, `string constraints_identifier` |
| **Result** | `bool success` |
| **Feedback** | `string status` |

**Usage:** Interface between `brain` and `moveit_path_planner` for UR5e movements.

---

### 3.5.3 Custom Services

#### `StartRound.srv`

| Component | Fields |
|-----------|--------|
| **Request** | `bool start` |
| **Response** | `bool accepted`, `string message` |

**Usage:** Dashboard → `brain` round trigger.

---

#### `GripperCmd.srv`

| Component | Fields |
|-----------|--------|
| **Request** | `int32 width` |
| **Response** | `bool success`, `string message` |

**Usage:** Low-level gripper command from `brain` to `gripper`.

---

#### `ResetGripperCmd.srv`

| Component | Fields |
|-----------|--------|
| **Request** | `bool reset_gripper` |
| **Response** | `bool success`, `string message` |

**Usage:** Re-home the gripper on startup or recovery.

---

# 4. Technical Components

## 4.1 Computer Vision Pipeline

The perception stack is split into separate ROS2 nodes for board localisation, cup detection, dice detection and player / chip detection. Each node subscribes to a colour camera stream and publishes geometric information (poses, chip locations, bets) that the `brain` node uses to drive the game.

### 4.1.1 ArUco Board Detection (`aruco_cv`)

The ArUco node establishes the **`board_frame`**, which is the common reference for every other CV module and for motion planning. All dice, cup and player coordinates are ultimately expressed relative to this frame.

#### Purpose in the system

- Detect the four ArUco markers attached to the corners of the physical board.
- Estimate the board’s pose relative to the camera and world.
- Produce the **rectified, top-down board image** used by all other vision pipelines.
- Publish simple RViz markers so the board alignment can be checked easily.

#### Inputs & Setup

- **RGB image** from the depth camera  
  (`/camera/camera/color/image_raw`, configurable via `color_topic`)
- **Camera intrinsics** from `/camera/camera/aligned_depth_to_color/camera_info`
- **Processing throttle**: the node only processes every **4th frame** to keep CPU usage low.

The TF tree produced is:

```
world
└── camera_frame
    └── board_frame
```

#### Raw Camera View

![Raw ArUco detection](docs/media/aruco_raw.png)

#### Detecting the board corners

Each frame:

1. The image is converted to BGR via `CvBridge`.
2. The node runs ArUco detection using the `DICT_4X4_250` family.
3. It extracts the centroids of markers **0, 1, 2, 3**, representing TL, TR, BR, BL.
4. If fewer than 4 markers are visible, the frame is skipped. The last valid pose is retained.

#### Estimating the board pose

Using the four detected centroids:

- Compute the **board centre** in pixel space.
- Using camera intrinsics and a fixed height, back-project to metric `(x, y)`.
- Estimate the **board yaw** from the average direction of its edges, yielding a smooth and stable orientation estimate.

The transform is published as:

- Parent: **`camera_frame`**  
- Child: **`board_frame`**

with translation `(x, y, CAM_HEIGHT_M)` and yaw-derived rotation.

#### Warped top‑down board image

The node computes a homography using the board corners, mapping them to a target space matching the physical board size:

- Destination dimensions: **`BOARD_W_MM × BOARD_H_MM`**
- Result: **1 pixel = 1 mm** in the warped image

Published on:

- **`board/warped_image`**

#### Warped Board View

![Warped board](docs/media/board_warped.png)

#### RViz markers

Corner points are transformed into the warped image and then into metric board coordinates.  
For each (`TL`, `TR`, `BR`, `BL`), the node publishes:

- A cube marker in `board_frame`
- A text label above the cube

This visualisation confirms stable detection and consistent alignment.

#### Downstream use

Other CV modules rely on the board frame:

- **dice_cv** — converts dice detections into world-frame picking poses
- **cup_cv** — locates the cup base and orientation
- **player_cv** — assigns chips to the correct betting zones

If detection fails, the transform is not updated, preventing jumps in downstream components.

---


### 4.1.2 Dice Detection (`dice_cv`)

The dice detection pipeline operates on the **rectified, top‑down board image** produced by the ArUco node. Its job is to detect dice reliably, estimate their orientation, and output usable 3D poses for the robot.


#### Purpose in the system

- Detect all dice visible on the board.
- Classify each die’s face value (1–6).
- Estimate the die’s in‑plane rotation (yaw).
- Convert pixel positions into **board-frame** and **world-frame** coordinates.
- Publish structured detections and visual markers used by the robot and UI.


#### Inputs and outputs

##### **Subscribed**
- `board/warped_image` (Image) — top‑down view of the board.

##### **Published**
- `dice_results` (DiceResults) — world-frame poses of dice.
- `dice_markers` (MarkerArray) — board-frame RViz cubes and labels.
- `dice_ee_goal_markers` (MarkerArray) — end-effector goal visualisations.

##### **Model**
- Loads a custom YOLO model (`best.pt`) from `perception_cv/dice_cv/weights`.

##### **TF**
Uses the existing TF chain:

```
world
└── camera_frame
    └── board_frame
```


#### Annotated Dice Detections

The warped-board view includes:

- Rotated bounding boxes  
- Dice value + confidence  
- Orientation arrow  
- Crop region rectangle  

![Dice detection on warped board](docs/media/dice_board_annotated.png)


#### Processing overview

##### 1. **Pre‑processing and crop**
- Incoming warped image is converted via `CvBridge`.
- A rectangular crop is taken from the **upper half** of the board where dice land.
- YOLO inference runs only on this crop (faster, fewer false positives).
- The crop region is drawn in green on the final debug image.


##### 2. **YOLO inference**
Each YOLO detection produces a bounding box, class index, and confidence.

- Detections below **0.55 confidence** are ignored.
- The node keeps track of:
  - `dice_count`
  - `total_sum` (sum of all detected die values)
- These are drawn in the debug overlay.


#### Rotation estimation (simple contour method)

For each YOLO bounding box:

1. Extract the die region from the full warped image.  
2. Convert to grayscale, apply Otsu thresholding.  
3. Find external contours; select the largest.  
4. Fit a `minAreaRect` to determine the die’s rotation angle.  
5. Convert OpenCV’s clockwise angle into a usable yaw.

If anything fails (no contour, noisy threshold), yaw defaults to **0°**, and the frame continues safely.

A rotated bounding box is redrawn around the die for clarity.


#### Converting pixels → board and world

##### **Board-frame coordinates**
```
pixel_to_board_coords(
    x_px = centre_x,
    y_px = centre_y,
    img_w, img_h,
    z_offset = DICE_HALF_HEIGHT (0.03m)
)
```

Returns `(x_m, y_m, z_m)` in **metres** inside the `board_frame`.

##### **World-frame pose**
```
pixel_to_world_pose(..., yaw_rad, tf_buffer)
```

- Uses camera intrinsics + `world → camera_frame` TF.  
- Outputs a full `PoseStamped` in the `world` frame.  
- If TF lookup fails, that dice detection is skipped.

##### **Yaw correction for MoveIt**
The robot’s tool convention requires a 180° flip around the Y-axis:

```
pose.orientation = orientation ⨉ quaternion_from_euler(0, π, 0)
```

This ensures the die is approached correctly.


#### Publishing results

##### **World-frame**
The node fills a `DiceResult` message for each detection:

- Pixel bbox  
- `dice_number`, `confidence`  
- Corrected world-frame pose  

All `DiceResult`s are packed into a `DiceResults` message and published.


#### Board-frame visualisation

For each detection, a simplified board-frame version is created:

- Position: `(x_m, y_m, z_m)`
- Orientation: yaw-only quaternion
- Passed to `publish_dice_markers()`

This function:

- Clears old markers using `DELETEALL`
- Publishes:
  - A **red cube** at the dice location
  - A **white text label** showing the dice value just above it

These markers provide intuitive feedback in RViz.


#### End-effector goal markers

A world-frame `PoseStamped` is created for each die.  
`visualise_pose_in_rviz()` publishes small axis markers to show where the robot will approach from.

These markers appear on the topic:

- `dice_ee_goal_markers`


#### Debug view and GUI

The node maintains an OpenCV window `"Dice Recognition"`.

Overlays include:

- Crop region  
- Rotated bounding boxes  
- Dice number + confidence  
- Dice count & total sum  

A small timer ensures the GUI stays responsive even when no new frames arrive.

This makes it easy to verify that:

- YOLO detects only the intended dice region  
- Rotations look stable  
- World-frame poses are reasonable before being passed to the `brain`

---


### 4.1.3 Cup Detection (`cup_cv`)

The cup detection pipeline works on the same rectified board image as the dice node and produces a single, stable pick pose for the cup. It combines a simple colour mask with geometry-based reasoning to estimate both the base position of the cup and the direction the gripper should approach from.

#### Role in the system

- Find the yellow cup on the warped board image.
- Estimate the pose of the cup base in both the `board_frame` and `world` frames.
- Infer a consistent pickup direction for the UR5e gripper.
- Publish a `CupResult` message that the `brain` and motion planner can use directly.

#### Inputs and outputs

- **Subscribed topics**
  - `Image` on **`board/warped_image`** – the top-down board view produced by the ArUco node.
- **Publishers**
  - `CupResult` on **`cup_result`** – world-frame pose and bounding box of the cup.
  - `MarkerArray` on **`cup_markers`** – board-frame 3D box for RViz.
  - `MarkerArray` on **`cup_ee_goal_markers`** – end-effector target pose for the gripper.
- **TF**
  - Uses a `tf2_ros.Buffer` and `TransformListener` to access the `world → camera_frame → board_frame` transforms.

#### Colour mask and cropping

To make detection simple and robust, the node only looks at the region of the board where the cup is expected to appear and applies a yellow colour mask.

- The incoming warped board image is cropped to the upper central region:
  - A horizontal padding of about 110 px on each side.
  - Only the top half of the warped board (similar to the dice node).
- The cropped region is converted to HSV and thresholded using a fixed yellow range.
- A small morphological open/close operation removes isolated noise and fills gaps.
- The resulting binary mask is shown in a separate debug window for tuning.

**Cup colour mask**:

![Cup colour mask](docs/media/cup_mask.png)

#### Selecting the cup blob

- All external contours in the mask are extracted.
- The node keeps only the largest contour above a minimum area threshold (`MIN_CONTOUR_AREA`) so that small specks of noise are ignored.
- If no valid contour is found:
  - An empty `CupResult` is published.
  - The current frame is shown for debugging and the callback returns early.

Once a valid contour is found, it is treated as the visible footprint of the cup in the cropped image.

#### Orientation and base footprint

Rather than relying only on the raw contour, the node enforces a known footprint for the cup:

- A rotated bounding box (`cv2.minAreaRect`) is fitted to the contour.
- Among the rectangle’s edges, the node chooses the long edges and biases towards those that are more vertical in the image. This stabilises the orientation when the box becomes nearly square.
- Using this long edge and a known physical footprint (approximately 10×4 cm), the node reconstructs a clean base rectangle that:
  - Shares the leftmost long edge of the fitted box.
  - Extends inward across the board by the known width of the cup base.

The centroid of this reconstructed base rectangle is used as the cup base position in the cropped image.

#### Visual annotations on the warped board

For each frame, the node draws several overlays on the full warped board image:

- **Green box** – the original rotated bounding box around the detected yellow blob.
- **Yellow box** – the reconstructed base rectangle that enforces the correct cup footprint.
- **Red dot** – the centroid of the reconstructed base, interpreted as the cup base.
- **Blue arrow** – the approach direction for the gripper, derived from the estimated yaw.

All of these are drawn in full-image coordinates (after accounting for the crop offset), giving an intuitive view of what the node thinks the cup pose and pickup direction are.

**Cup detection on warped board**:

![Cup detection with centroid and pickup direction](docs/media/cup_board_annotated.png)

#### Converting to board and world poses

Once the base centroid has been found in full-image pixel coordinates:

- The node calls `pixel_to_board_coords(...)` with a small height offset (`CUP_HALF_HEIGHT = 0.06` m) to obtain `(x, y, z)` in the `board_frame`.
- It also calls `pixel_to_world_pose(...)` with the same pixel centre, height offset and yaw angle to obtain a `PoseStamped` in the `world` frame.
- If TF lookup fails, the node logs a warning and skips publishing a cup pose for that frame.

#### Publishing `CupResult`

The main world-frame output is a `CupResult` message:

- 2D bounding box fields (`x`, `y`, `width`, `height`) describe the cup in full warped-image pixels.
- `confidence` is set to 1.0 (the node assumes exactly one cup is present once a valid blob is found).
- `pose` holds the final world-frame pose that the UR5e end-effector should move to above the cup.
- `drop_pose` is initially copied from the same world pose and can be used as a starting point for where the cup will be placed.

Before publishing, the node adjusts this pose to better match the end-effector frame and tool geometry:

- A local offset of about 14 cm along the cup’s local X-axis is applied so that the wrist target sits behind the cup rather than at its centre.
- An additional fixed rotation is applied to align the gripper with the cup axis using a pre-defined Euler rotation (`π/2, π, π/2`).

These adjustments mean that downstream components (the `brain` and motion planner) can treat `cup_result.pose` as a ready-to-use end-effector goal.

#### Board-frame markers and RViz view

For visualisation in `board_frame`:

- The node builds a simplified `CupResult` with:
  - Position `(x_m, y_m, z_m)` in `board_frame`.
  - Orientation constructed from the estimated yaw.
- It publishes a `MarkerArray` on `cup_markers` that contains:
  - A yellow 3D box that roughly matches the cup dimensions (10×4×8 cm) and is positioned so that the base sits on the board surface.

To visualise the end-effector goal, the node also:

- Wraps the world pose in a `PoseStamped` with `frame_id = "world"`.
- Calls `visualise_pose_in_rviz(...)` with `cup_ee_goal_markers` as the publisher, drawing an axes marker at the intended approach pose above the cup.

#### Debugging and GUI

- An OpenCV window named `Cup Detection` shows the main warped-board view with all annotations (bounding boxes, centroid, and approach arrow).
- A separate window (`Cup Mask`) shows the current yellow mask, which is useful for tuning the HSV thresholds.
- A small timer (`gui_timer`) keeps the GUI responsive by regularly calling `cv2.waitKey(1)` even when new images are not arriving.

Together, the colour mask, geometric reconstruction and RViz markers make it easy to see how the system is interpreting the cup’s position and orientation before the UR5e commits to a pick-up motion.

---

### 4.1.4 Player and Bet Detection (`player_node`)

The player detection pipeline is responsible for mapping physical players and their chips on the board to a structured `Players` message that the `brain` and dashboard can use. It runs on the warped board image, reuses the same coordinate system and divides the table into three logical zones, one per player.

#### Role in the system

- Detect which player positions (1–3) are currently active using ArUco markers.
- Segment and classify coloured chips in the betting area.
- Assign chips to players based on their horizontal zone.
- Publish a `Players` message containing chip locations, colours and diameters for each player.
- Provide simple RViz markers so player positions can be checked at a glance.

#### Inputs and outputs

- **Subscribed topics**
  - `Image` on **`board/warped_image`** – top-down board view produced by the ArUco node.
- **Publishers**
  - `Players` on **`players`** – list of players and their detected chips/bets.
  - `MarkerArray` on **`player_markers`** – cube + text markers in `board_frame` for each player.

#### Annotated player view

The main debug view shows:
- The ArUco marker bounding boxes and player labels (`P1`, `P2`, `P3`).
- Blue rectangular “zones” corresponding to each player’s betting area.
- Circles around detected chips, coloured by chip colour and annotated with size and zone.

This is captured in a warped-board image like:

![Player detection and zones](docs/media/player_board_annotated.png)

#### Cropping and player zones

Players and chips are only expected in the lower half of the warped board image. To simplify processing and avoid interference with dice and cup graphics:

- The node crops the **bottom half** of the warped image, with a horizontal padding of about 100 px on each side.
- This cropped region defines the “player area” where ArUco tags and chips are searched for.
- The horizontal span of the player area is split evenly into **three zones** (left, middle, right). Each zone corresponds to one logical player position:
  - Zone 1 → Player position 1 (left)
  - Zone 2 → Player position 2 (centre)
  - Zone 3 → Player position 3 (right)

On the debug image, the overall player area is outlined, and each zone is shown as a separate rectangle with a `Zone 1/2/3` label at the top.

#### Player detection with ArUco markers

Players are identified using ArUco markers placed near each player’s seat:

- The node runs ArUco detection (`DICT_4X4_250`) on the cropped player region.
- For each detected marker:
  - The marker centre is converted back into full-image coordinates using the crop offsets.
  - The x-position is compared to the pre-defined zone bounds to determine which of the three positions (1–3) this marker belongs to.
  - The marker centre is converted to metric board coordinates using `pixel_to_board_coords(...)`.
  - A `Player` message is created with:
    - `player_id` set to the numeric ArUco ID.
    - `position` set to 1, 2 or 3 based on the zone.
    - Empty arrays for chip information (`bet_colors`, `bet_x`, `bet_y`, `bet_diameter_px`) that will be filled later.
- Simple RViz markers are created for each player:
  - A small green cube at the player’s board-frame position.
  - A text label above the cube, e.g. “Player 23”.

On the debug image, each marker location is annotated with a label `P{position}`, so you can see at a glance which real-world marker is mapped to which logical seat.

#### Chip detection and assignment to players

After players are located, the node looks for circular chips within the same player area:

- The cropped region is converted to HSV colour space.
- For each colour class (blue, green, red, white), a dedicated HSV range is applied to create a mask.
- A small morphological close and median blur clean up the mask.
- Contours are detected in each mask, and only those that look like valid chips are kept, using:
  - Approximate diameter limits (between `LOWER_LIMIT` and `UPPER_LIMIT` pixels).
  - An area check consistent with that diameter.
  - A solidity check to reject irregular or fragmented blobs.
- For each accepted contour:
  - The centre `(cx, cy)` and diameter are computed from the minimum enclosing circle.
  - The global image coordinates are reconstructed using the crop offsets.
  - The x-position is used to determine which zone (1–3) the chip belongs to.

Chips are then assigned to players:

- For each chip, the node finds the `Player` whose `position` matches the chip’s zone.
- The chip’s properties are appended to that player’s arrays:
  - `bet_colors` (e.g. `"blue"`, `"red"`, `"green"`, `"white"`)
  - `bet_x`, `bet_y` (pixel centroids in the warped image)
  - `bet_diameter_px` (measured chip size in pixels)

On the debug image, each chip is drawn as a coloured circle with a short label containing its colour, approximate diameter and zone (e.g. `blue (36px) Z2`).

#### Published messages and markers

At the end of each frame:

- A `Players` message containing all active `Player` entries is published on `/players`. This exposes, for each logical seat:
  - The ArUco-derived `player_id` and table position.
  - A list of chip colours and pixel locations that the `brain` and dashboard can interpret as bets.
- A `MarkerArray` with cubes and text labels for each player is published on `/player_markers`, rendered in the `board_frame` for RViz.

These outputs give the rest of the system a clean, geometry-aware view of “who is sitting where” and “which chips are in front of which player”, without needing to re-run any image processing.

#### Debugging and GUI

- An OpenCV window named `Player Detection` shows the warped board with:
  - The player region outline.
  - The three zone rectangles.
  - ArUco-based player positions (`P1`, `P2`, `P3`).
  - Coloured chip overlays.
- A small timer (`gui_timer`) keeps the window responsive by calling `cv2.waitKey(1)` periodically, even if new images are not arriving.

This view makes it easy to verify that:
- Players are being assigned to the correct seat positions.
- Chips fall inside the expected zones.
- The mapping from physical layout → warped image → board frame is behaving as intended before the `brain` uses these detections to drive game logic.


## 4.2 Custom End-Effector

> TODO: Describe and document your custom end-effector.

Include:

- One or more photos/renders:  
  `![End Effector Photo](docs/media/end_effector.jpg)`
- Summary of mechanical design and function (e.g. cup gripper, dice rake, etc.).
- Assembly or exploded-view drawing (link to drawings/STEP files if relevant).
- Control overview: how the end-effector is actuated (e.g. via Teensy, IO, vacuum).
- ROS integration: topics / services / actions used to control it.
  

The end effector is a linear gripper designed to reliably manipulate both the dice and the cup. The gripper is actuated using a rack-and-pinion mechanism, with a central pinion gear driving two opposing, gear-profiled gripper fingers that function as linear racks. This configuration ensures symmetric finger motion and consistent gripping force.

A key advantage of the linear gripper design is its passive self-centring behaviour. As long as the object lies between the open fingers, the closing motion naturally guides it toward the centre of the gripper. This makes the system highly tolerant to positional misalignment between the gripper and the object, improving robustness, repeatability, and overall reliability during operation.

Actuation is controlled using a **Teensy 4.1 microcontroller**, which drives a **DSS-PO5 Standard Servo** to position the gripper with precise and repeatable motion. 

The Brain node commands the gripper via a dedicated gripper server using a service-based interface. Each service call specifies the desired target position for the servo motor.

Upon receiving a request, the gripper server transmits the position command to the Teensy 4.1 microcontroller over a serial connection. The Teensy then generates the appropriate PWM signal to drive the servo to the requested position. Once actuated, the servo provides sufficient holding torque to securely grasp and lift both the dice and the cup during operation.
## 4.3 System Visualisation

> TODO: Explain how your system is visualised.

Include:

- RViz configuration (TF tree, markers for dice/cup/players, planned trajectories).
- Any custom markers / MarkerArrays used (e.g. dice pose arrows, text overlays).
- How the web dashboard / GUI (if used) reflects robot state and game status.
- Screenshots or GIFs:  
  `![RViz Visualisation](docs/media/rviz_view.png)`

## 4.4 Closed-Loop Operation

> TODO: Describe your feedback and adaptation mechanisms.

Discuss:

- What sensor feedback is used (vision, robot state, error signals).
- How stale or inconsistent data is handled.
- Examples of adaptive behaviour (e.g. re-detect dice if occluded, replan if path fails).
- Timing considerations (latency, update rates, synchronisation).

---

# 5. Installation and Setup

## 5.1 Requirements

The system has a number of requirements which must first be met before setup can begin. 

### Ros2
Fistly, the system relies on Ros2 Humble Hawksbill. [HERE](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) is the guide for installing Ros2 Huble Hawksbill via ubuntu debain packages.

### Moveit
All path planning and executing with the Ur5e is done via Moveit. An installation guide can be found [HERE](https://moveit.ai/install-moveit2/source/).

### RealSense
To make use of the RealSense camera we require the appropriate ros wrapper. Follow the installation guide at in the ReadMe [HERE](https://github.com/realsenseai/realsense-ros?tab=readme-ov-file#installation-on-ubuntu).

### NodeJS and NPM
The front-end requires NodeJS to operate and npm to handle dependencies. Installing NodeJS should install npm with it. On Debian-based systems, you can use the NodeSource repository or Node Version Manager (NVM). For the NodeSource method, first update your system and install dependencies:
```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y curl
```
Then, use the setup script for Node.js 18:
```bash
curl -fsSL https://deb.nodesource.com/setup_18.x | sudo -E bash -
sudo apt install -y nodejs
```
### Front-End
** TO BE DONE AFTER CLONING INTO REPO **
To ensure all front end dependencies are installed, run the follwing commands from the base of the repository. 
```bash
cd frontend
npm install
```

### Additional Back-End
The back end requires a couple more packages. These include some extra drivers as well as python libraries for the vision and and end-effector code. 
```bash
pip install -r requirements.txt

sudo apt install ros-humble-rosbridge-server

sudo apt install ros-humble-message-filters

sudo apt install ros-humble-tf-transformations
```

## 5.2 Workspace Setup and Cloning the Repo
Navigate to your desired workspace directory and clone the repository:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```
```bash
git clone https://github.com/LachlanWallbridge/MTRN4231_Project.git
cd MTRN4231_Project

colcon build --symlink-install
source install/setup.bash
```

## 5.3 Hardware Setup

> TODO: Describe the hardware required and how it should be connected.

Include subsections such as:

- **UR5e**: connection via Ethernet, IP address, external control setup, required URCaps.
- **Camera**: mounting position, USB/Ethernet connection, intrinsic/extrinsic calibration files.
- **End-effector controller (Teensy, etc.)**: USB port, baud rate, firmware location.
- Any other sensors or devices.

## 5.4 Configuration and Calibration

> TODO: Document configuration files and calibration procedures.

Include:

- Paths to configuration YAML files (e.g. MoveIt, controller configs, camera parameters).
- Where hand–eye calibration is stored and how it is loaded (even if assumed to be present).
- Any tunable parameters for perception thresholds, planning margins, etc.

---

# 6. Running the System

## 6.1 One-Command Launch

> TODO: Provide the single command that launches the full system (no manual sequencing).

Example placeholder:

```bash
# Make sure that you are in the MTRN4231_Project workspace
cd launch
./launch_all_urdf.sh
```

Briefly describe what this launch file does (starts perception, `brain`, MoveIt, visualisation, dashboard, etc.).

### 6.2 Example Commands

> TODO: List commonly used commands for development, debugging or partial launches.

Examples:

```bash
# Connect to camera
cd scripts
./realsense.sh
```

```bash
# Launch ur_robot_driver, ur_moveit_config, Moveit_path_planner_server (cartesian+ joints)
ros2 launch linear_gripper_visualiser display.launch.py
```

```bash
# Launch only gripper server
ros2 run gripper gripper_server
```

```bash
# Launch only perception stack
ros2 launch perception_cv perception.launch.py
```

```bash
# Launch only RosBridgeServer to communicate with backend
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

```bash
# Launch only the brain
ros2 run brain brain_node
```

```bash
# Launch only backend
cd $WS/backend && uvicorn api:app --reload --port 8000
```

```bash
# Launch only frontend
cd $WS/frontend && npm start
```

(Replace with your actual package / launch file names.)

## 6.3 Expected Behaviour

> TODO: Describe what a user should see when the system is running correctly.

once the shell script is executed in the terminal
```bash
# Make sure that you are in the MTRN4231_Project workspace
cd launch
./launch_all_urdf.sh
```
There will be 8 new terminals appeared on the screen with the following titles:
## Terminal & Process Overview

| **Terminal** | **Title** | **Role / Description** |
|--------------|-----------|-------------------------|
| **1** | `RealSense Camera` | Runs the `realsense_interface` node responsible for streaming RGB-D data to all perception modules. |
| **2** | `Real Robot` | Launches the real UR5e stack: `ur_robot_driver`, `ur_moveit_config` (with `gripper.xacro`), and the `cartesian_path_planner` action server for executing robot motions. |
| **3** | `Gripper Server` | Provides gripper control services, receiving open/close commands from the Brain node and driving the physical gripper hardware. |
| **4** | `Perception` | Hosts all perception nodes: ArUco board detection, dice CV, cup CV, and player CV. Publishes structured detections to the Brain. |
| **5** | `Rosbridge` | Runs `rosbridge_websocket` to enable communication between ROS 2 and the backend/frontend dashboard using WebSockets and JSON. |
| **6** | `Brain Node` | The main game orchestrator. Coordinates perception, MoveIt motion planning, and gripper operations. Maintains round state and communicates with the UI. |
| **7** | `Backend` | Backend server for game logic, APIs, and routing data between rosbridge and the frontend dashboard. |
| **8** | `Frontend` | Web frontend UI that displays the game state, dice results, player info, and robot actions in real-time. |

## Additional Interfaces

| **Component** | **Role / Description** |
|---------------|-------------------------|
| **RViz** | Visualises the UR5e robot, TF tree, board frame, cup/dice poses, and all perception markers. |
| **Cup Mask Image** | Live OpenCV debugging window showing the cup segmentation mask. |
| **Dice Detection Image** | Live window displaying detected dice, bounding boxes, classification, and pose overlays. |
| **Player Detection Image** | Visualises player identification, markers, and filtered chip detections. |
| **Frontend UI** | Accessible at **http://localhost:3000/** — opens in a new browser tab for interactive game monitoring. |



## 6.4 Troubleshooting

### **No camera images are received**

If RViz or the CV windows do not show camera images:

- Check the **RealSense Camera terminal** for messages like *"camera not found"* or failed device initialization.
- Use `lsusb` to confirm the RealSense camera is detected by the system.
- If the camera does not appear, unplug and reconnect it.  
  Ensure the camera is plugged into a **USB 3.0 port** (blue port). Avoid hubs if possible.

---

### **Game cannot be started from the frontend**

If pressing **Start Round** does not begin the game:

- Make sure all players have selected **Odd** or **Even**.
- Confirm that CV image windows (cup, players, board) are updating.  
  If they are frozen, adjust board position, move the robot out of the camera's view, or improve lighting conditions.

If the player marker appears **glitchy or unstable** in the frontend:

- Replace the player tag with a **non-reflective ArUco marker**, as shiny surfaces cause jittery detection.

If the round starts but the robot does not move:

- Check the **Brain Node terminal** for messages such as “move action failed”.
- Check the **Real Robot terminal** to determine whether the issue is an **execution timeout** or a **catastrophic error**.
- For timeouts, restart the system and ensure the UR5e teach pendant is set to allow ROS control by pressing *Play* when prompted.
- For catastrophic errors, reposition the board lower closer to player to avoid robot getting too close to the back collision wall.

---

### **CV detection limitations and object tracking issues**

If dice or cup are not being detected or disappear in the interface:

- Ensure the dice and cup stay within the **green area**.  
  The CV system can only detect objects inside this region.
- If dice roll outside the carpet during shaking, manually place them back within the green area.
- Apply the same rule for the cup; it must remain within the detectable region.
- If dice or the cup are too far from the robot's workspace, **MoveIt path planning will fail**.  
  Adjust the board or objects so they remain reachable by the robot.

---

### **Gripper does not respond**

If the gripper does not open or close:

- Use the **VSCode Serial Monitor** to check that the selected serial port matches the one defined in the gripper server code.
- Verify the **power supply** to the gripper controller and servo driver, ensuring stable voltage is being provided.

---

---

# 7. Results and Demonstration
 
The system successfully implements autonomous dice rolling, pickup, and return to cup — in line with requirement F1. Through the front-end we see the system evaluating bets and handling payout logic as required by F2. The front end displays dice results and confirms the parity for communication of the game outcome, fulfilling F3. The robot's joint states and end effector, as well as the position and orientation of the board, dice, and cup, are all visualised in RVIZ. This ensures we have met requirement F4. Player bets are made using chips, matching requirement F5.

Over a period of 8 games, 4 movement errors were seen. In each game cycle, 10 movements occur. Thus, 76/80 movements were seen to be successful. During these 8 games, dice pickup position was seen to be accurate to around 0.75 mm. These results indicate success in achieving requirement P5 — Pick-up accuracy and repeatability.

![Results Summary](docs/media/Movement_actions_chart.png)

The overall time for a game cycle was seen to be 90 seconds on average. The dice phase of the cycle was consistently below 20 seconds, with the remaining 70 seconds dedicated to placing the dice back into the cup and returning to the home position. This allows for quick gameplay through fast dice rolling and then adequate time after a dice result for the players to handle payout and place new bets. Thus the metric P1, Short time from game start to dice roll, has been met.

Throughout a game cycle, the handling of the dice is open and easy to see. This fulfills the need for transparent operation as outlined by P4. No pattern to dice outcome was observed throughout multiple rounds of gameplay. The final dice position after a roll is also highly variable. These indicate that there is true randomness in the dice rolls as desired by P3.

When the ArUco markers at the corner of the board are not occluded to the camera’s vision, the system consistently outputs image processing results in under a second. This meets the requirement outlined by P2 for a low latency vision pipeline.

The front end provides clear feedback about the state of the game. This includes player records, balances, bets, and prediction, in addition to informing the users of when the system is waiting for input and when it is waiting for the start game button to be pressed. After a game the dice outcome is clearly shown and player records and balances are updated. The frontend is thus seen to provide clear on-screen instructions and visual feedback in line with UX2 and UX3.

Users have provided positive feedback on the front end and the game experience. Users have been able to play after a quick and minimal onboarding. These indicate achievement of UX1 and UX4.

---

# 8. Discussion and Future Work

> TODO: Reflect on engineering challenges and potential improvements.

Include:

- Major technical challenges (e.g. calibration drift, latency, path planning collisions) and how you addressed them.
- Design trade-offs you made (e.g. algorithm choice vs. runtime, robustness vs. complexity).
- Ideas for “Version 2.0”:
  - More advanced game logic or betting options.
  - Improved perception (multi-view, better models, tracking).
  - More expressive visualisation or user controls.
  - Hardware improvements (faster gripper, safer interactions).
- What makes your approach distinctive or effective compared to a naive baseline.

---

# 9. Contributors and Roles

![System Overview](docs/media/Team.jpg)


| Name            | zID       | Primary Roles                              |
|-----------------|-----------|--------------------------------------------|
| Lachlan Wallbridge       | z5xxxxxxx | Vision, Web dashboard, visualisation       |
| Kanokpop Juemjutitam      | z5382229 | Motion planning, MoveIt integration        |
| Samuel Maron       | z5351169 | Hardware, end-effector, UR5e integration, visualisation   |


---

# 10. Repository Structure

> TODO: Briefly describe the folder structure of your repository.

Example (edit to match your repo):

```text
.
├── README.md
├── src/
│   ├── brain/                # High-level game logic and coordination
│   ├── perception_cv/        # Computer vision utilities and transforms
│   │   ├── dice_cv/              # Dice detection node/package
│   │   ├── cup_cv/               # Cup detection node/package
│   │   └── player_cv/            # Player / chip detection
│   └── moveit_path_planner/  # UR5e motion planning and execution
├── docs/
│   ├── diagrams/             # Architecture, state machine, node graphs
│   ├── figures/              # Results plots, tables
│   └── media/                # Photos, GIFs, demo videos (or thumbnails)
├── launch/                   # Top-level and helper launch files
├── config/                   # YAML configuration (MoveIt, controllers, etc.)
├── scripts/                  # Helper scripts (bash, Python)
└── requirements.txt          # Python dependencies
```

---

# 11. References and Acknowledgements

## 11.1 References

> TODO: Credit any external libraries, tutorials or prior codebases.

Examples:

- ROS2 and MoveIt2 documentation.
- YOLO / Ultralytics documentation and model zoo.
- Tutorials, blog posts or example repos you adapted.

(Provide links or citations as appropriate.)

https://github.com/N3VERS4YDIE/dice-recognition


## 11.2 Acknowledgements

> TODO: Acknowledge people and organisations who helped.

Examples:

- Course staff and demonstrators for support and guidance.
- Other student groups who shared insights or debugging help.
- Any external organisations or tools (e.g. UR, Intel RealSense, etc.).
