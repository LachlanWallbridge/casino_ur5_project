ros2 action send_goal /moveit_path_plan custom_interface/action/Movement "{command: 'joint', positions: [-1.3, 1.57, -1.83, -1.57, 0.0, 0.0], constraints_identifier: 'NONE'}" --feedback

ros2 action send_goal /moveit_path_plan custom_interface/action/Movement "{command: 'cartesian', positions: [0.4, 0.1, 0.3, 0.0, 1.57, 0.0], constraints_identifier: 'NONE'}"

ros2 action send_goal /moveit_path_plan custom_interface/action/Movement "{command: 'cartesian', positions: [0.6, 0.1, 0.3, -1.57, 0.0, 0.0], constraints_identifier: 'NONE'}"