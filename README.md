# MTRN4231: Dice-Playing Robot Casino System

## 1. Table of Contents

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

## 2. Project Overview

### 2.1 Problem Statement and Customer

> TODO: Briefly describe the task or problem your system solves, including the intended “customer” or end-user (e.g. casino operator, teaching lab, demonstrator, etc.).

### 2.2 Robot Functionality Summary

> TODO: Summarise the full cycle of the robot (from game initialisation, bet placement, dice rolling, perception, decision making, motion planning, to result visualisation).

Suggested prompts:

- What does the robot perceive?
- What actions does it perform on the environment?
- How does it close the loop between sensing, planning and acting?

### 2.3 Demo Video

> TODO: Insert a short video (10–30 s) showing one complete, closed-loop game cycle.

For example:

[![System Demo](docs/media/demo_thumbnail.png)](https://link.to/your/demo/video)

(Add a short caption describing what the viewer should look for.)

---

## 3. System Architecture

### 3.1 ROS2 Node Graph

> TODO: Include a diagram of your ROS2 nodes, topics, services and actions (e.g. screenshot from `rqt_graph` or a custom schematic).

Example placeholder:

![ROS2 Node Graph](docs/diagrams/ros2_graph.png)

Brief description:

> TODO: One paragraph summarising the overall data flow (e.g. perception → brain → motion planning → execution → visualisation).

### 3.2 Package-Level Architecture

> TODO: Provide a package-level diagram showing how ROS2 packages interact (topic connections, service interactions, shared utilities).

Example placeholder:

![Package Architecture](docs/diagrams/package_architecture.png)

Describe, in a few bullet points:

- `perception_cv`: TODO – vision and board/dice/cup/player detection.
- `brain`: TODO – game logic, sequencing and coordination.
- `moveit_path_planner`: TODO – motion planning and execution for UR5e.
- `casino_dashboard` / web UI: TODO – external visualisation and user interaction.
- Any other relevant packages.

### 3.3 Behaviour Tree / State Machine

> TODO: Include a behaviour-tree or state-machine diagram describing the closed-loop behaviour across one full game round.

Example placeholder:

![State Machine](docs/diagrams/state_machine.png)

Suggested states to document (adjust as needed):

- `Idle`
- `WaitForPlayers`
- `PlaceBets`
- `RollDice`
- `PerceiveBoard`
- `EvaluateOutcome`
- `AnnounceResults`
- `ResetRound`

### 3.4 Node Summaries

> TODO: Briefly describe each node and its responsibilities.

Example table (edit to match your actual node set):

| Node Name           | Package         | Role / Description                                                   | Key Topics / Interfaces                                   |
|---------------------|-----------------|---------------------------------------------------------------------|-----------------------------------------------------------|
| `brain`             | `brain`         | Orchestrates game flow, manages state machine and high-level logic. | Subscribes: `/dice_results`, `/cup_result`, etc.          |
| `dice_detector`     | `dice_cv`       | Detects dice poses and values from camera images.                   | Subscribes: `/camera/color/image_raw`; Publishes: `/dice_results` |
| `cup_detector`      | `cup_cv`        | Detects the cup pose and orientation.                               | Subscribes: `/camera/color/image_raw`; Publishes: `/cup_result` |
| `player_detector`   | `player_cv`     | Identifies players and their bets from chips/markers.               | Publishes: `/players`                                    |
| `moveit_path_planner` | `moveit_path_planner` | Plans UR5e motions for cup/dice manipulation.                       | Actions: `/moveit_path_plan`                             |
| `rosbridge_websocket` | `rosbridge_server` | Bridges ROS2 topics to the web dashboard.                           | WebSocket interface to dashboard                          |

(Add/remove rows as required.)

### 3.5 Custom Messages and Interfaces

> TODO: List and briefly explain any custom message, service or action types you defined.

Example format:

- `custom_interface/msg/DiceResult.msg`
  - **Fields:** `x`, `y`, `width`, `height`, `dice_number`, `confidence`, `pose`
  - **Usage:** Published by `dice_detector`, consumed by `brain` and visualisation.
- `custom_interface/msg/CupResult.msg`
  - **Fields:** TODO
  - **Usage:** TODO
- `custom_interface/action/Movement.action`
  - **Purpose:** TODO – coordinate motion planning and execution.

---

## 4. Technical Components

### 4.1 Computer Vision Pipeline

> TODO: Describe your vision pipeline and how it contributes to the task.

Suggested structure:

- Input source(s) (camera model, resolution, frame rate).
- Pre-processing (e.g. undistortion, cropping, colour-space conversion).
- Detection methods (YOLO, ArUco, HSV masks, blob detection, etc.).
- Pose estimation (pixel-to-board, board-to-world transforms).
- Filtering / tracking (e.g. confidence thresholds, area filtering, temporal smoothing).
- How vision outputs are fed into the `brain` or planner.

### 4.2 Custom End-Effector

> TODO: Describe and document your custom end-effector.

Include:

- One or more photos/renders:  
  `![End Effector Photo](docs/media/end_effector.jpg)`
- Summary of mechanical design and function (e.g. cup gripper, dice rake, etc.).
- Assembly or exploded-view drawing (link to drawings/STEP files if relevant).
- Control overview: how the end-effector is actuated (e.g. via Teensy, IO, vacuum).
- ROS integration: topics / services / actions used to control it.

### 4.3 System Visualisation

> TODO: Explain how your system is visualised.

Include:

- RViz configuration (TF tree, markers for dice/cup/players, planned trajectories).
- Any custom markers / MarkerArrays used (e.g. dice pose arrows, text overlays).
- How the web dashboard / GUI (if used) reflects robot state and game status.
- Screenshots or GIFs:  
  `![RViz Visualisation](docs/media/rviz_view.png)`

### 4.4 Closed-Loop Operation

> TODO: Describe your feedback and adaptation mechanisms.

Discuss:

- What sensor feedback is used (vision, robot state, error signals).
- How stale or inconsistent data is handled.
- Examples of adaptive behaviour (e.g. re-detect dice if occluded, replan if path fails).
- Timing considerations (latency, update rates, synchronisation).

---

## 5. Installation and Setup

> TODO: Provide clear, step-by-step instructions to reproduce your development environment and run the system on a fresh machine.

### 5.1 Requirements

```bash
pip install -r requirements.txt

sudo apt install ros-humble-rosbridge-server

sudo apt install ros-humble-message-filters

sudo apt install ros-humble-tf-transformations
```

(Add any additional ROS packages, Python dependencies or system libraries here.)

### 5.2 Workspace Setup

> TODO: Describe how to clone and build the ROS2 workspace.

Suggested steps (edit as needed):

1. Create and initialise the workspace:
   ```bash
   mkdir -p ~/mtrn4231_ws/src
   cd ~/mtrn4231_ws/src
   git clone https://github.com/your-org/your-repo.git
   cd ..
   rosdep install --from-paths src --ignore-src -r -y
   colcon build
   source install/setup.bash
   ```
2. Mention any required branches or tags.
3. Note any environment variables that must be set (e.g. `UR_TYPE`, `ROS_DOMAIN_ID`, etc.).

### 5.3 Hardware Setup

> TODO: Describe the hardware required and how it should be connected.

Include subsections such as:

- **UR5e**: connection via Ethernet, IP address, external control setup, required URCaps.
- **Camera**: mounting position, USB/Ethernet connection, intrinsic/extrinsic calibration files.
- **End-effector controller (Teensy, etc.)**: USB port, baud rate, firmware location.
- Any other sensors or devices.

### 5.4 Configuration and Calibration

> TODO: Document configuration files and calibration procedures.

Include:

- Paths to configuration YAML files (e.g. MoveIt, controller configs, camera parameters).
- Where hand–eye calibration is stored and how it is loaded (even if assumed to be present).
- Any tunable parameters for perception thresholds, planning margins, etc.

---

## 6. Running the System

### 6.1 One-Command Launch

> TODO: Provide the single command that launches the full system (no manual sequencing).

Example placeholder:

```bash
ros2 launch mtrn4231_casino bringup.launch.py
```

Briefly describe what this launch file does (starts perception, `brain`, MoveIt, visualisation, dashboard, etc.).

### 6.2 Example Commands

> TODO: List commonly used commands for development, debugging or partial launches.

Examples:

```bash
# Launch only perception stack
ros2 launch perception_cv perception.launch.py

# Launch only the brain and planner
ros2 launch brain brain.launch.py
```

(Replace with your actual package / launch file names.)

### 6.3 Expected Behaviour

> TODO: Describe what a user should see when the system is running correctly.

For example:

- RViz shows the UR5e model, board frame and detected dice/cup markers.
- Web dashboard shows current bets, dice results and round status.
- The robot moves through the game sequence without manual intervention.

### 6.4 Troubleshooting

> TODO: Provide brief troubleshooting tips.

Examples:

- What to check if no camera images are received.
- What to check if TF frames are missing or misaligned.
- Common build issues and how to fix them.
- Known limitations or required workarounds.

---

## 7. Results and Demonstration

> TODO: Summarise how the system performs against its design goals.

Suggested structure:

- Quantitative metrics:
  - Dice value detection accuracy (e.g. % correct over N trials).
  - Pose/placement accuracy (e.g. mean/STD position error).
  - Round completion time, success rate.
- Qualitative observations:
  - Robustness to lighting changes or occlusions.
  - Behaviour when bets are invalid or camera data is noisy.
- Figures and media:
  - Plots or tables of results (accuracy, timing).
  - Photos or GIFs showing successful rounds and edge cases.

Example placeholder for a results figure:

![Results Summary](docs/figures/results_summary.png)

---

## 8. Discussion and Future Work

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

## 9. Contributors and Roles

> TODO: List team members and their primary responsibilities.

Example table:

| Name            | zID       | Primary Roles                              |
|-----------------|-----------|--------------------------------------------|
| Student A       | z5xxxxxxx | Vision, Web dashboard, visualisation       |
| Student B       | z5xxxxxxx | Motion planning, MoveIt integration        |
| Student C       | z5xxxxxxx | Hardware, end-effector, UR5e integration   |

(Add/remove rows as necessary.)

---

## 10. Repository Structure

> TODO: Briefly describe the folder structure of your repository.

Example (edit to match your repo):

```text
.
├── README.md
├── src/
│   ├── brain/                # High-level game logic and coordination
│   ├── perception_cv/        # Computer vision utilities and transforms
│   ├── dice_cv/              # Dice detection node/package
│   ├── cup_cv/               # Cup detection node/package
│   ├── player_cv/            # Player / chip detection
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

## 11. References and Acknowledgements

### 11.1 References

> TODO: Credit any external libraries, tutorials or prior codebases.

Examples:

- ROS2 and MoveIt2 documentation.
- YOLO / Ultralytics documentation and model zoo.
- Tutorials, blog posts or example repos you adapted.

(Provide links or citations as appropriate.)

### 11.2 Acknowledgements

> TODO: Acknowledge people and organisations who helped.

Examples:

- Course staff and demonstrators for support and guidance.
- Other student groups who shared insights or debugging help.
- Any external organisations or tools (e.g. UR, Intel RealSense, etc.).
