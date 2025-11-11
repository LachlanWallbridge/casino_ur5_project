# 🧠 Brain Node — Dice Game Orchestrator

The **Brain Node** coordinates the dice-playing robot by handling game logic, movement requests, and round control.

It interfaces with:
- `/dice_results` topic (`custom_interface/msg/DiceResults`)
- `/moveit_path_plan` action (`custom_interface/action/Movement`)
- `/start_round` service (`custom_interface/srv/StartRound`, optional)

---

## 🚀 Overview

| Component | Type | Description |
|------------|------|-------------|
| **MoveIt Path Planner** | Action Client | Sends motion goals for Cartesian or joint moves. |
| **Dice Detector** | Subscriber | Receives dice detections (pose, value, confidence). |
| **Round Control** | Service or Manual | Starts each round either via service call or console input. |

Each round:
1. Waits for dice to appear.  
2. Logs detected values and total (odd/even).  
3. Picks up all dice until none remain.  
4. Returns the arm to the home position.

---

## ⚙️ Parameters

| Name | Type | Default | Description |
|------|------|----------|-------------|
| `manual_start` | `bool` | `false` | Enables manual round start from the console instead of waiting for `/start_round` service calls. |

---

## 🧩 Usage

### Default (Service-based Start)

In this mode, the node exposes a `/start_round` service.  
A frontend or another node can trigger rounds by calling it.

```bash
ros2 run your_package brain_node
```

Then, from another terminal:

```bash
ros2 service call /start_round custom_interface/srv/StartRound "{start: true}"
```

---

### Manual Start (Console)

To manually trigger rounds using console input instead of the service:

```bash
ros2 run your_package brain_node --ros-args -p manual_start:=true
```

Then press **ENTER** in the terminal to begin a round.

This mode is ideal for quick local testing — it runs a background thread for input without interrupting ROS2 callbacks.

---

## 🧱 Topics and Interfaces

| Interface | Type | Direction | Description |
|------------|------|------------|-------------|
| `/dice_results` | `custom_interface/msg/DiceResults` | **Subscriber** | Receives the latest detected dice and their poses. |
| `/moveit_path_plan` | `custom_interface/action/Movement` | **Action Client** | Sends movement goals to the MoveIt action server. |
| `/start_round` | `custom_interface/srv/StartRound` | **Service Server** *(optional)* | Trigger a new game round (disabled if `manual_start=true`). |

---

## 🧩 Example Round Log

```
[INFO] [brain]: 🎯 StartRound service received. Starting a new round...
[INFO] [brain]: Waiting for dice to appear...
[INFO] [brain]: 🎲 Dice rolled: [3, 5] (Total=8, EVEN)
[INFO] [brain]: Picking up dice 3 at (0.310, 0.220, 0.040) (confidence=0.95)
[INFO] [brain]: ✅ MoveIt action succeeded.
[INFO] [brain]: ✅ Round complete. All dice removed.
```

---

## 🧰 Tips

- Use `manual_start` for debugging when no frontend is running.  
- Ensure `/moveit_path_plan` action server is available before starting.  
- Confirm the `custom_interface` package is properly built and sourced.

---

**Maintainer:** Lachlan Wallbridge  
**Package:** `brain_node`  
**ROS2 Distro:** Humble
