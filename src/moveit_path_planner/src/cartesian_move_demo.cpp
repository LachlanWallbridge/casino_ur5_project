// #include <memory>
// #include <string>
// #include <vector>

// #include "rclcpp/rclcpp.hpp"
// #include "geometry_msgs/msg/pose.hpp"
// #include "moveit/move_group_interface/move_group_interface.h"
// #include "moveit/planning_scene_interface/planning_scene_interface.h"
// #include "moveit_msgs/msg/constraints.h"
// #include "moveit_msgs/msg/joint_constraint.h"
// #include "moveit_msgs/msg/orientation_constraint.h"
// #include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
// #include "custom_interface/srv/movement_request.hpp"
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// #include <tf2/LinearMath/Matrix3x3.h> 
// #include <tf2_eigen/tf2_eigen.hpp>



// class MoveitPathPlanningServer {
// public:
//   MoveitPathPlanningServer(const rclcpp::Node::SharedPtr& node)
//   : node_(node)
//   {
//     using namespace std::placeholders;

//     RCLCPP_INFO(node_->get_logger(), "Starting MoveIt Path Planning Server...");

//     // Initialize MoveGroupInterface with proper parameters
//     // base_link -> tool0
//     move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
//       node_, 
//       "ur_manipulator"
//     );

//     // Configure planner parameters
//     node_->declare_parameter("planning_time", 20.0);

//     // IMPORTANT!!!!!!!!!!!!!!!!!
//     // Refrain urself from loosing the tolerance, if the planning is slow, it's probably not the fault of the tolerance
//     // Why?
//     // If you increase it to 0.05, there will be a max goal displacement of 0.05m in all axis, that's a nightmare to tune
//     node_->declare_parameter("goal_joint_tolerance", 0.001);
//     node_->declare_parameter("goal_position_tolerance", 0.001);  
//     node_->declare_parameter("goal_orientation_tolerance", 0.001);  

//     setupCollisionObjects();

//     // Apply parameters
//     move_group_->setPlanningTime(node_->get_parameter("planning_time").as_double());
//     move_group_->setGoalJointTolerance(node_->get_parameter("goal_joint_tolerance").as_double());
//     move_group_->setGoalPositionTolerance(node_->get_parameter("goal_position_tolerance").as_double());
//     move_group_->setGoalOrientationTolerance(node_->get_parameter("goal_orientation_tolerance").as_double());
//     // move_group_->setPlannerId("RRTConnect");
//     // move_group_->setPlannerId("BKPIECEkConfigDefault");
//     move_group_->setPlannerId("LBKPIECEkConfigDefault");

//     service_ = node_->create_service<custom_interface::srv::MovementRequest>(
//       "/moveit_path_plan",
//       std::bind(&MoveitPathPlanningServer::handle_request, this, _1, _2)
//     );
//   }

// 	moveit_msgs::msg::Constraints set_constraint(const std::string& constraint_str) {
// 		moveit_msgs::msg::Constraints constraints;
	
// 		if (str_contains(constraint_str, ORIEN)) {
// 			RCLCPP_INFO(node_->get_logger(), "should not be triggered.");
// 			constraints.orientation_constraints.push_back(set_orientation_constraint(constraint_str));
// 		}

// 		// Joint Constraints

// 		// Elbow -> no less than 60
// 		//       -> no more than 150
// 		// Joint constraint for elbow (50° to 145°)

// 		if (str_contains(constraint_str, ELBOW)) {
// 			moveit_msgs::msg::JointConstraint elbow_constraint;
// 			elbow_constraint.joint_name = "elbow_joint";
			
// 			// Convert degrees to radians
// 			const double min_angle = 60.0 * M_PI / 180.0;   // 60° in radians (~1.0472)
// 			const double max_angle = 150.0 * M_PI / 180.0;  // 150° in radians (~2.6179)
			
// 			// Calculate midpoint (105° which is between 60° and 150°)
// 			const double midpoint = (min_angle + max_angle) / 2.0;  // ~1.8326 radians
			
// 			// Set constraints
// 			elbow_constraint.position = midpoint;
// 			elbow_constraint.tolerance_below = 1.0;  // ~0.7854 radians (45°)
// 			elbow_constraint.tolerance_above = max_angle - midpoint;  // ~0.7854 radians (45°)
// 			elbow_constraint.weight = 1.0;
			
// 			RCLCPP_INFO(node_->get_logger(), "elbow constraint implemented.");
			
// 			constraints.joint_constraints.push_back(elbow_constraint);
// 		}

// 		if (str_contains(constraint_str, WRIST_1) || str_contains(constraint_str, FULL)) {
// 			moveit_msgs::msg::JointConstraint wrist_constraint;
// 			wrist_constraint.joint_name = "wrist_1_joint";
			
// 			// Convert degrees to radians (-71° to -218°)
// 			const double min_angle = 218.0 * M_PI / 180.0;  // ≈ -3.8048 radians
// 			const double mid_angle = 144.5 * M_PI / 180.0;  // Midpoint (-144.5° ≈ -2.5220 rad)
// 			const double max_angle = 71.0 * M_PI / 180.0;   // ≈ -1.2392 radians
		
// 			// Configure constraint
// 			wrist_constraint.position = mid_angle;           // Center of range
// 			wrist_constraint.tolerance_below = 1.7;
// 			// wrist_constraint.tolerance_below = 0.01;
// 			// wrist_constraint.tolerance_above = 0.01;
// 			wrist_constraint.tolerance_above = 1.2828;
// 			wrist_constraint.weight = 1.0;
			
// 			constraints.joint_constraints.push_back(wrist_constraint);
// 		}

//     if (str_contains(constraint_str, FULL)) {
//       moveit_msgs::msg::JointConstraint elbow_constraint;
// 			elbow_constraint.joint_name = "elbow_joint";
			
// 			// Convert degrees to radians
// 			const double min_angle = 20.0 * M_PI / 180.0;   // 60° in radians (~1.0472)
// 			const double max_angle = 150.0 * M_PI / 180.0;  // 150° in radians (~2.6179)
			
// 			// Calculate midpoint (105° which is between 60° and 150°)
// 			const double midpoint = (min_angle + max_angle) / 2.0;  // ~1.8326 radians
			
// 			// Set constraints
// 			elbow_constraint.position = midpoint;
// 			elbow_constraint.tolerance_below = 1.0;  // ~0.7854 radians (45°)
// 			elbow_constraint.tolerance_above = max_angle - midpoint;  // ~0.7854 radians (45°)
// 			elbow_constraint.weight = 1.0;
			
// 			RCLCPP_INFO(node_->get_logger(), "elbow constraint implemented.");
			
// 			constraints.joint_constraints.push_back(elbow_constraint);

//     }

//     if (str_contains(constraint_str, FULL)) {
//       moveit_msgs::msg::JointConstraint shoulder_pan_constraint;
// 			shoulder_pan_constraint.joint_name = "shoulder_pan_joint";
			
// 			// Convert degrees to radians
// 			const double min_angle = -45.0 * M_PI / 180.0;   // 60° in radians (~1.0472)
// 			const double max_angle = 90.0 * M_PI / 180.0;  // 150° in radians (~2.6179)
			
// 			// Calculate midpoint (105° which is between 60° and 150°)
// 			const double midpoint = (min_angle + max_angle) / 2.0;  // ~1.8326 radians
			
// 			// Set constraints
// 			shoulder_pan_constraint.position = midpoint;
// 			shoulder_pan_constraint.tolerance_below = 1.0;  // ~0.7854 radians (45°)
// 			shoulder_pan_constraint.tolerance_above = max_angle - midpoint;  // ~0.7854 radians (45°)
// 			shoulder_pan_constraint.weight = 1.0;
			
// 			RCLCPP_INFO(node_->get_logger(), "eshoulder pan constraint implemented.");
			
// 			constraints.joint_constraints.push_back(shoulder_pan_constraint);

//     }



// 		return constraints;
// 	}

// 	// -1.389
// 	//- -3.691542764703268

// 	// actual value: 3.361864

// 	// -3.926776

// 	// -1.244382

// 	moveit_msgs::msg::OrientationConstraint set_orientation_constraint(const std::string& orien_constraint_str) {
// 		moveit_msgs::msg::OrientationConstraint orientation_constraint;

// 		// Common setup for all orientations
// 		orientation_constraint.header.frame_id = move_group_->getPlanningFrame();
// 		orientation_constraint.link_name = move_group_->getEndEffectorLink();
// 		orientation_constraint.absolute_x_axis_tolerance = 0.001;
// 		orientation_constraint.absolute_y_axis_tolerance = 0.001;
// 		orientation_constraint.absolute_z_axis_tolerance = 0.001;
// 		orientation_constraint.weight = 1;
	
// 		// Set orientation based on input parameter
// 		tf2::Quaternion q;
		
// 		if (str_contains(orien_constraint_str, DOWN)) {
// 			// Facing downward: Z-axis pointing down (Roll=0, Pitch=π, Yaw=0)
// 			q.setRPY(0, M_PI, 0);
// 		} 
// 		else if (str_contains(orien_constraint_str, LEFT)) {
// 			// Facing left: X-axis pointing left (Roll=0, Pitch=0, Yaw=π/2)
// 			q.setRPY(0, 0, M_PI_2);
// 		}
// 		else if (str_contains(orien_constraint_str, RIGHT)) {
// 			// Facing right: X-axis pointing right (Roll=0, Pitch=0, Yaw=-π/2)
// 			q.setRPY(0, 0, -M_PI_2);
// 		}
// 		else if (str_contains(orien_constraint_str, UP)) {
// 			// Facing upward: Z-axis pointing up (Roll=0, Pitch=0, Yaw=0)
// 			q.setRPY(0, 0, 0);
// 		}
	
// 		orientation_constraint.orientation = tf2::toMsg(q);

// 		return orientation_constraint;
// 	}

//   void handle_request(
//     const std::shared_ptr<custom_interface::srv::MovementRequest::Request> request,
//     std::shared_ptr<custom_interface::srv::MovementRequest::Response> response)
// {
//     RCLCPP_INFO(node_->get_logger(), "Received MoveIt path planning request. Command: %s", request->command.c_str());

//     if (request->positions.size() != 6) {
//         RCLCPP_ERROR(node_->get_logger(), "Expected 6 position elements, got %zu", request->positions.size());
//         response->success = false;
//         return;
//     }

//     // Clear previous targets and constraints
//     move_group_->clearPoseTargets();
//     move_group_->clearPathConstraints();

//     // Handle different command types
//     bool target_set = false;
//     if (request->command == "cartesian") {
//         target_set = set_cartesian_target(request->positions);
//     } 
//     else if (request->command == "joint") {
//         target_set = set_joint_target(request->positions);
//     }
//     else {
//         RCLCPP_ERROR(node_->get_logger(), "Invalid command: %s", request->command.c_str());
//         response->success = false;
//         return;
//     }

//     if (!target_set) {
//         response->success = false;
//         return;

//     }

//     // Apply constraints if specified
//     if (request->constraints_identifier != NONE) {
//         move_group_->setPathConstraints(set_constraint(request->constraints_identifier));
//     }

//     // move_group_->setStartStateToCurrentState();

//     // Common planning and execution logic
//     response->success = plan_and_execute();
// }

//   void setupCollisionObjects() {
//     std::string frame_id = "world";
//     moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

//     planning_scene_interface.applyCollisionObject(generateCollisionObject(2.4, 0.04, 3.0, 0.70, -0.60, 0.5, frame_id, "backWall"));
//     planning_scene_interface.applyCollisionObject(generateCollisionObject(0.04, 2.4, 3.0, -0.30, 0.25, 0.5, frame_id, "sideWall"));
//     planning_scene_interface.applyCollisionObject(generateCollisionObject(3, 3, 0.01, 0.85, 0.25, -0.10, frame_id, "table"));
//     planning_scene_interface.applyCollisionObject(generateCollisionObject(2.4, 2.4, 0.04, 0.85, 0.25, 1.5, frame_id, "ceiling"));
//   }

//   auto generateCollisionObject(float sx, float sy, float sz, float x, float y, float z, const std::string& frame_id, const std::string& id) -> moveit_msgs::msg::CollisionObject {
//     moveit_msgs::msg::CollisionObject collision_object;
//     collision_object.header.frame_id = frame_id;
//     collision_object.id = id;

//     shape_msgs::msg::SolidPrimitive primitive;
//     primitive.type = primitive.BOX;
//     primitive.dimensions = {sx, sy, sz};

//     geometry_msgs::msg::Pose box_pose;
//     box_pose.orientation.w = 1.0;
//     box_pose.position.x = x;
//     box_pose.position.y = y;
//     box_pose.position.z = z;

//     collision_object.primitives.push_back(primitive);
//     collision_object.primitive_poses.push_back(box_pose);
//     collision_object.operation = collision_object.ADD;

//     return collision_object;
//   }

//   bool str_contains(const std::string& str, const std::string& substr) {
//     if (substr.empty()) return false;
//     return str.find(substr) != std::string::npos;
//   }

// private:
//   bool set_cartesian_target(const std::vector<double>& positions) {
//     try {
//         // geometry_msgs::msg::Pose target_pose = create_pose_from_positions(positions);
//         // move_group_->setPoseTarget(target_pose);
//         target_pose_ = create_pose_from_positions(positions);
//         return true;
//     } catch (const std::exception& e) {
//         RCLCPP_ERROR(node_->get_logger(), "Failed to set Cartesian target: %s", e.what());
//         return false;
//     }
//   }

//   bool set_joint_target(const std::vector<double>& positions) {
//     try {
//         auto joint_targets = create_joint_map_from_positions(positions);
//         move_group_->setJointValueTarget(joint_targets);
//         return true;
//     } catch (const std::exception& e) {
//         RCLCPP_ERROR(node_->get_logger(), "Failed to set joint target: %s", e.what());
//         return false;
//     }
//   }

//   geometry_msgs::msg::Pose create_pose_from_positions(const std::vector<double>& positions) {
//     tf2::Quaternion q;
//     q.setRPY(positions[3], positions[4], positions[5]);
//     q.normalize();
    
//     geometry_msgs::msg::Pose target_pose;
//     target_pose.position.x = positions[0];
//     target_pose.position.y = positions[1];
//     target_pose.position.z = positions[2];
//     target_pose.orientation = tf2::toMsg(q);
    
//     return target_pose;
//   }

//   std::map<std::string, double> create_joint_map_from_positions(const std::vector<double>& positions) {
//     return {
//         {"shoulder_pan_joint", positions[5]},
//         {"shoulder_lift_joint", positions[0]},
//         {"elbow_joint", positions[1]},
//         {"wrist_1_joint", positions[2]},
//         {"wrist_2_joint", positions[3]},
//         {"wrist_3_joint", positions[4]}
//     };
//   }

//   bool plan_and_execute()
//   {
//       // Ensure MoveIt sees the current state
//       move_group_->setStartStateToCurrentState();
      
//       if (!wait_for_robot_state(5.0)) {

//         return false;
//       }

//       std::vector<geometry_msgs::msg::Pose> waypoints;
//       geometry_msgs::msg::Pose start_pose = move_group_->getCurrentPose().pose;
//       // auto current_state = move_group_->getCurrentState(10.0);

//       // --- Get the current end-effector pose properly ---
//       // geometry_msgs::msg::Pose start_pose;
//       // const std::string end_effector_link = move_group_->getEndEffectorLink();
//       // const Eigen::Isometry3d& ee_transform =
//       //   current_state->getGlobalLinkTransform(end_effector_link);
//       // tf2::convert(ee_transform, start_pose);

//       waypoints.push_back(start_pose);
//       waypoints.push_back(target_pose_);

//       moveit_msgs::msg::RobotTrajectory trajectory;
//       const double eef_step = 0.01;       // 1 cm resolution
//       const double jump_threshold = 0.0;

//       double fraction = move_group_->computeCartesianPath(
//           waypoints, eef_step, jump_threshold, trajectory, false);

//       if (fraction < 0.90) {
//           RCLCPP_ERROR(node_->get_logger(), "Cartesian Path Planning Failed with %f fraction.", fraction);
//           move_group_->clearPathConstraints();
//           move_group_->clearPoseTargets();
//           return false;
//       }

//       // Scale velocity and acceleration
//       for (auto &point : trajectory.joint_trajectory.points) {
//           for (auto &v : point.velocities) v *= speed_scale_;
//           for (auto &a : point.accelerations) a *= speed_scale_ * speed_scale_;

//           double t = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9;
//           t /= speed_scale_;
//           point.time_from_start.sec = static_cast<int32_t>(floor(t));
//           point.time_from_start.nanosec = static_cast<uint32_t>((t - floor(t)) * 1e9);
//       }

//       moveit::planning_interface::MoveGroupInterface::Plan plan;
//       plan.trajectory_ = trajectory;

//       move_group_->execute(plan);
//       return true;
//   }


//   bool wait_for_robot_state(double timeout = 5.0)
//   {
//       RCLCPP_INFO(node_->get_logger(), "Waiting for a valid robot state...");
//       rclcpp::Time start = node_->now();
//       rclcpp::Rate rate(10); // 10 Hz polling

//       while (rclcpp::ok()) {
//           auto current_state = move_group_->getCurrentState();
//           if (current_state && current_state->satisfiesBounds()) {
//               RCLCPP_INFO(node_->get_logger(), "Robot state ready.");
//               return true;
//           }

//           if ((node_->now() - start).seconds() > timeout) {
//               RCLCPP_WARN(node_->get_logger(), "Timeout waiting for robot state. Proceeding anyway...");
//               return false; // or true if you want to ignore freshness
//           }

//           rate.sleep();
//       }

//       return false;
//   }




//   rclcpp::Node::SharedPtr node_;
//   std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
//   rclcpp::Service<custom_interface::srv::MovementRequest>::SharedPtr service_;

//   // rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
//   // sensor_msgs::msg::JointState::SharedPtr latest_joint_state_;

//   const int NO_CONSTRAINT = 0;
//   const int ORIENTATION_CONSTRAINT_DOWN = 1;
//   const int ORIENTATION_CONSTRAINT_LEFT = 2;
//   const std::string DOWN = "DOWN";
//   const std::string UP =  "UP";
//   const std::string LEFT = "LEFT";
//   const std::string RIGHT = "RIGHT";
//   const std::string NONE = "NONE";
//   const std::string VER_ELBOW = "LEFT_ELBOW";
//   const std::string ELBOW = "ELBOW";
//   const std::string ORIEN = "ORIEN";
//   const std::string WRIST_1= "WRIST1";
//   const std::string FULL = "FULL";
//   double speed_scale_ = 0.2;
//   geometry_msgs::msg::Pose target_pose_;
// };

// int main(int argc, char** argv)
// {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<rclcpp::Node>("moveit_path_planning_server");
//   MoveitPathPlanningServer server(node);
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }

// // birds-eye for camera horizontal
// // ros2 service call /moveit_path_plan custom_interface/srv/MovementRequest "{positions: [0.822, 0.183, 0.856, 0.0, 3.14, 0.0]}"

// // birds-eye for camera vertical
// // ros2 service call /moveit_path_plan custom_interface/srv/MovementRequest "{positions: [0.64, 0.174, 1.041, -1.56, -0.0, -1.571]}"

// // ros2 service call /moveit_path_plan custom_interface/srv/MovementRequest "{positions: [0.44, 0.174, 0.841, -1.56, -0.0, -1.571]}"
// // can't push past 0.75

// // ros2 service call /moveit_path_plan custom_interface/srv/MovementRequest "{positions: [0.44, 0.16, 0.83,-1.687, 0, -1.61]}"

// // ros2 service call /moveit_path_plan custom_interface/srv/MovementRequest "{positions: [0.471,0.149, 1.044, -1.978, 0.058, -1.549]}"
// // Object 1: X=1.248m, Y=-0.042m, Z=1.067m

// // Home pose Joint
// // ros2 service call /moveit_path_plan custom_interface/srv/MovementRequest "{command: 'joint', positions: [-1.3, 1.57, -1.83, -1.57, 0, 0], constraints_identifier: '0'}"


// //ros2 service call /moveit_path_plan custom_interface/srv/MovementRequest "{command: 'cartesian', positions: [0.64, 0.174, 1.041, -1.56, -0.0, -1.571], constraints_identifier: 'FULL'}"



#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit_msgs/msg/constraints.h"
#include "moveit_msgs/msg/joint_constraint.h"
#include "moveit_msgs/msg/orientation_constraint.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "custom_interface/action/movement.hpp"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_eigen/tf2_eigen.hpp>

class MoveitPathPlanningServer
{
public:
  using Movement = custom_interface::action::Movement;
  using GoalHandleMovement = rclcpp_action::ServerGoalHandle<Movement>;

  explicit MoveitPathPlanningServer(const rclcpp::Node::SharedPtr& node)
    : node_(node)
  {
    RCLCPP_INFO(node_->get_logger(), "Starting MoveIt Path Planning Action Server...");

    // Initialize MoveGroupInterface with provided node
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "ur_manipulator");

    // Declare parameters
    node_->declare_parameter("planning_time", 20.0);
    node_->declare_parameter("goal_joint_tolerance", 0.001);
    node_->declare_parameter("goal_position_tolerance", 0.001);
    node_->declare_parameter("goal_orientation_tolerance", 0.001);

    // Apply MoveIt parameters
    move_group_->setPlanningTime(node_->get_parameter("planning_time").as_double());
    move_group_->setGoalJointTolerance(node_->get_parameter("goal_joint_tolerance").as_double());
    move_group_->setGoalPositionTolerance(node_->get_parameter("goal_position_tolerance").as_double());
    move_group_->setGoalOrientationTolerance(node_->get_parameter("goal_orientation_tolerance").as_double());
    move_group_->setPlannerId("LBKPIECEkConfigDefault");

    setupCollisionObjects();

    // Initialize Action Server
    action_server_ = rclcpp_action::create_server<Movement>(
      node_,
      "moveit_path_plan",
      std::bind(&MoveitPathPlanningServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&MoveitPathPlanningServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&MoveitPathPlanningServer::handle_accepted, this, std::placeholders::_1)
    );
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Server<Movement>::SharedPtr action_server_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  geometry_msgs::msg::Pose target_pose_;
  double speed_scale_ = 0.075;

  const std::string VER_ELBOW = "LEFT_ELBOW";
  const std::string ELBOW = "ELBOW";
  const std::string ORIEN = "ORIEN";
  const std::string WRIST_1 = "WRIST1";
  const std::string FULL = "FULL";

  // ======== Action Server Callbacks ========

  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const Movement::Goal> goal)
  {
    RCLCPP_INFO(node_->get_logger(), "Received goal request: %s", goal->command.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleMovement> goal_handle)
  {
    RCLCPP_INFO(node_->get_logger(), "Received cancel request.");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleMovement> goal_handle)
  {
    std::thread{std::bind(&MoveitPathPlanningServer::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  // ======== Execution Logic ========

  void execute(const std::shared_ptr<GoalHandleMovement> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Movement::Feedback>();
    auto result = std::make_shared<Movement::Result>();

    RCLCPP_INFO(node_->get_logger(), "Executing goal: %s", goal->command.c_str());
    feedback->status = "Processing target...";
    goal_handle->publish_feedback(feedback);

    if (goal->positions.size() != 6) {
      RCLCPP_ERROR(node_->get_logger(), "Expected 6 position elements, got %zu", goal->positions.size());
      result->success = false;
      goal_handle->abort(result);
      return;
    }

    move_group_->clearPoseTargets();
    move_group_->clearPathConstraints();

    bool target_set = false;
    if (goal->command == "cartesian") {
      target_set = set_cartesian_target(goal->positions);
    } else if (goal->command == "joint") {
      target_set = set_joint_target(goal->positions);
    }

    if (!target_set) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to set target.");
      result->success = false;
      goal_handle->abort(result);
      return;
    }

    if (goal->constraints_identifier != "NONE") {
      move_group_->setPathConstraints(set_constraint(goal->constraints_identifier));
    }

    feedback->status = "Planning and executing...";
    goal_handle->publish_feedback(feedback);

    bool success = plan_and_execute();

    if (success) {
      feedback->status = "Execution complete.";
      goal_handle->publish_feedback(feedback);
      result->success = true;
      goal_handle->succeed(result);
    } else {
      feedback->status = "Execution failed.";
      goal_handle->publish_feedback(feedback);
      result->success = false;
      goal_handle->abort(result);
    }
  }

  // ======== Helper Functions ========

  bool set_cartesian_target(const std::vector<double>& positions)
  {
    tf2::Quaternion q;
    q.setRPY(positions[3], positions[4], positions[5]);
    q.normalize();

    target_pose_.position.x = positions[0];
    target_pose_.position.y = positions[1];
    target_pose_.position.z = positions[2];
    target_pose_.orientation = tf2::toMsg(q);
    return true;
  }

  bool set_joint_target(const std::vector<double>& positions)
  {
    std::map<std::string, double> joint_targets = {
      {"shoulder_pan_joint", positions[5]},
      {"shoulder_lift_joint", positions[0]},
      {"elbow_joint", positions[1]},
      {"wrist_1_joint", positions[2]},
      {"wrist_2_joint", positions[3]},
      {"wrist_3_joint", positions[4]}
    };
    move_group_->setJointValueTarget(joint_targets);
    return true;
  }

  moveit_msgs::msg::Constraints set_constraint(const std::string& constraint_str)
  {
    moveit_msgs::msg::Constraints constraints;

    if (str_contains(constraint_str, WRIST_1) || str_contains(constraint_str, FULL)) {
      moveit_msgs::msg::JointConstraint wrist_constraint;
      wrist_constraint.joint_name = "wrist_1_joint";
      const double mid_angle = 144.5 * M_PI / 180.0;
      wrist_constraint.position = mid_angle;
      wrist_constraint.tolerance_below = 1.7;
      wrist_constraint.tolerance_above = 1.2828;
      wrist_constraint.weight = 1.0;
      constraints.joint_constraints.push_back(wrist_constraint);
    }

    if (str_contains(constraint_str, FULL)) {
      moveit_msgs::msg::JointConstraint elbow_constraint;
      elbow_constraint.joint_name = "elbow_joint";
      const double min_angle = 20.0 * M_PI / 180.0;
      const double max_angle = 150.0 * M_PI / 180.0;
      const double midpoint = (min_angle + max_angle) / 2.0;
      elbow_constraint.position = midpoint;
      elbow_constraint.tolerance_below = 1.0;
      elbow_constraint.tolerance_above = max_angle - midpoint;
      elbow_constraint.weight = 1.0;
      RCLCPP_INFO(node_->get_logger(), "Elbow constraint applied.");
      constraints.joint_constraints.push_back(elbow_constraint);
    }

    return constraints;
  }

  bool plan_and_execute()
  {
    move_group_->setStartStateToCurrentState();

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(move_group_->getCurrentPose().pose);
    waypoints.push_back(target_pose_);

    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = move_group_->computeCartesianPath(waypoints, 0.01, 0.0, trajectory, false);

    if (fraction < 0.90) {
      RCLCPP_ERROR(node_->get_logger(), "Cartesian Path failed: %.2f", fraction);
      return false;
    }

    for (auto &point : trajectory.joint_trajectory.points) {
      for (auto &v : point.velocities) v *= speed_scale_;
      for (auto &a : point.accelerations) a *= speed_scale_ * speed_scale_;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    move_group_->execute(plan);
    return true;
  }

  void setupCollisionObjects()
  {
    std::string frame_id = "world";
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    planning_scene_interface.applyCollisionObject(generateCollisionObject(2.4, 0.04, 1.0, 0.85, -0.30, 0.5, frame_id, "backWall"));
    planning_scene_interface.applyCollisionObject(generateCollisionObject(0.04, 1.2, 1.0, -0.30, 0.25, 0.5, frame_id, "sideWall"));
    planning_scene_interface.applyCollisionObject(generateCollisionObject(2.4, 2.4, 0.01, 0.85, 0.25, 0.013, frame_id, "table"));
    planning_scene_interface.applyCollisionObject(generateCollisionObject(2.4, 2.4, 0.04, 0.85, 0.25, 1.2, frame_id, "ceiling"));
  }

  moveit_msgs::msg::CollisionObject generateCollisionObject(float sx, float sy, float sz, float x, float y, float z,
                                                            const std::string& frame_id, const std::string& id)
  {
    moveit_msgs::msg::CollisionObject obj;
    obj.header.frame_id = frame_id;
    obj.id = id;
    shape_msgs::msg::SolidPrimitive prim;
    prim.type = prim.BOX;
    prim.dimensions = {sx, sy, sz};

    geometry_msgs::msg::Pose pose;
    pose.orientation.w = 1.0;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;

    obj.primitives.push_back(prim);
    obj.primitive_poses.push_back(pose);
    obj.operation = obj.ADD;
    return obj;
  }

  bool str_contains(const std::string& str, const std::string& substr)
  {
    if (substr.empty()) return false;
    return str.find(substr) != std::string::npos;
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // ✅ Create a shared node and pass it to the server
  auto node = std::make_shared<rclcpp::Node>("moveit_path_planning_action_server_node");
  auto server = std::make_shared<MoveitPathPlanningServer>(node);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
