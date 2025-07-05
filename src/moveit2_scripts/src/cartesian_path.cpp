// MoveIt includes for robot motion planning and control
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// MoveIt message types for visualization and communication
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

// Logger for outputting information during execution
static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

int main(int argc, char **argv) {
  // Initialize ROS2 communication
  rclcpp::init(argc, argv);
  
  // Set up node options to automatically declare parameters
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  
  // Create a ROS2 node for the MoveIt interface
  auto move_group_node =
      rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

  // Create an executor to handle ROS2 callbacks in a separate thread
  // This is necessary to keep the node responsive while planning and executing motions
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // Define the planning group name for the UR robot arm
  // This corresponds to the group defined in the robot's SRDF file
  // SOURCE: /home/shubham-0802/ros2_ws_UR_MoveIt/src/universal_robot_ros2/test_moveit_config/config/name.srdf
  // - Line 22: <group name="ur_manipulator">
  // - Line 23:   <chain base_link="base_link" tip_link="tool0"/>
  // - Line 24: </group>
  // This planning group includes all joints from base_link to tool0 (the entire UR arm)
  static const std::string PLANNING_GROUP_ARM = "ur_manipulator";

  // Create MoveIt interface for controlling the robot arm
  // This interface handles motion planning, collision checking, and execution
  moveit::planning_interface::MoveGroupInterface move_group_arm(
      move_group_node, PLANNING_GROUP_ARM);

  // Get the joint model group for the arm to access joint information
  // This allows us to work with joint positions and constraints
  const moveit::core::JointModelGroup *joint_model_group_arm =
      move_group_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);

  // Get Current State
  // Retrieve the current robot state with a 10-second timeout
  // This is important to know where the robot is before planning new motions
  moveit::core::RobotStatePtr current_state_arm =
      move_group_arm.getCurrentState(10);

  // Create a vector to store current joint positions
  // This will be used to modify specific joints while keeping others unchanged
  std::vector<double> joint_group_positions_arm;
  current_state_arm->copyJointGroupPositions(joint_model_group_arm,
                                             joint_group_positions_arm);

  // Set the start state to current state before planning any motion
  // This ensures the planner knows where the robot currently is
  move_group_arm.setStartStateToCurrentState();

  // ============================================================================
  // PHASE 1: GO TO HOME POSITION
  // ============================================================================
  RCLCPP_INFO(LOGGER, "Going Home");

  // Define home position joint values (in radians)
  // These values position the robot in a safe, accessible configuration
  // joint_group_positions_arm[0] = 0.00;  // Shoulder Pan (base rotation)
  joint_group_positions_arm[1] = -2.50; // Shoulder Lift (moves arm up/down)
  joint_group_positions_arm[2] = 1.50;  // Elbow (bends the arm)
  joint_group_positions_arm[3] = -1.50; // Wrist 1 (first wrist rotation)
  joint_group_positions_arm[4] = -1.55; // Wrist 2 (second wrist rotation)
  // joint_group_positions_arm[5] = 0.00;  // Wrist 3 (end-effector rotation)

  // Set the target joint positions for the home configuration
  move_group_arm.setJointValueTarget(joint_group_positions_arm);

  // Plan the motion from current position to home position
  moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
  bool success_arm = (move_group_arm.plan(my_plan_arm) ==
                      moveit::core::MoveItErrorCode::SUCCESS);

  // Execute the planned motion to move to home position
  move_group_arm.execute(my_plan_arm);

  // ============================================================================
  // PHASE 2: MOVE TO PREGRASP POSITION
  // ============================================================================
  RCLCPP_INFO(LOGGER, "Pregrasp Position");

  // Define the pregrasp pose in Cartesian space
  // This positions the end-effector above the target object
  geometry_msgs::msg::Pose target_pose1;
  
  // Set orientation (quaternion) - pointing downward
  // x=-1.0 means the gripper is oriented downward (180° rotation around x-axis)
  target_pose1.orientation.x = -1.0;
  target_pose1.orientation.y = 0.00;
  target_pose1.orientation.z = 0.00;
  target_pose1.orientation.w = 0.00;
  
  // Set position coordinates (in meters) - above the target object
  target_pose1.position.x = 0.343; // Forward/backward position
  target_pose1.position.y = 0.132; // Left/right position  
  target_pose1.position.z = 0.264; // Height above the target
  
  // Set this pose as the target for motion planning
  move_group_arm.setPoseTarget(target_pose1);

  // Plan the motion to the pregrasp position
  success_arm = (move_group_arm.plan(my_plan_arm) ==
                 moveit::core::MoveItErrorCode::SUCCESS);

  // Execute the motion to pregrasp position
  move_group_arm.execute(my_plan_arm);

  // ============================================================================
  // PHASE 3: CARTESIAN APPROACH TO OBJECT
  // ============================================================================
  // Move closer to the target object using Cartesian path planning
  // This ensures the end-effector follows a straight line path in Cartesian space
  RCLCPP_INFO(LOGGER, "Approach to object!");

  // Create a vector of waypoints for the approach motion
  // Waypoints define intermediate poses the end-effector should pass through
  std::vector<geometry_msgs::msg::Pose> approach_waypoints;
  
  // First waypoint: Move down 3cm from pregrasp position
  target_pose1.position.z -= 0.03; // Move down by 3cm
  approach_waypoints.push_back(target_pose1);

  // Second waypoint: Move down another 3cm (total 6cm approach)
  target_pose1.position.z -= 0.03; // Move down by another 3cm
  approach_waypoints.push_back(target_pose1);

  // Configure Cartesian path planning parameters
  moveit_msgs::msg::RobotTrajectory trajectory_approach;
  
  // Jump threshold: Maximum allowed "jump" in joint space between consecutive points
  // 0.0 means no jumps are allowed (strictest setting for smooth motion)
  const double jump_threshold = 0.0;
  
  // End-effector step: Maximum distance between consecutive points on the Cartesian path
  // Smaller values = smoother path but more computation time
  // 0.01m = 1cm resolution
  const double eef_step = 0.01;

  // Compute the Cartesian path through the waypoints
  // Returns the fraction of the path that was successfully computed (0.0 to 1.0)
  // fraction = 1.0 means the entire path was computed successfully
  double fraction = move_group_arm.computeCartesianPath(
      approach_waypoints, eef_step, jump_threshold, trajectory_approach);

  // Execute the computed Cartesian trajectory
  // This moves the robot along the planned straight-line path
  move_group_arm.execute(trajectory_approach);

  // ============================================================================
  // PHASE 4: CARTESIAN RETREAT FROM OBJECT
  // ============================================================================
  // Move away from the object using Cartesian path planning
  // This simulates lifting an object after grasping
  RCLCPP_INFO(LOGGER, "Retreat from object!");

  // Create a vector of waypoints for the retreat motion
  // This will reverse the approach motion
  std::vector<geometry_msgs::msg::Pose> retreat_waypoints;
  
  // First waypoint: Move up 3cm from current position
  target_pose1.position.z += 0.03; // Move up by 3cm
  retreat_waypoints.push_back(target_pose1);

  // Second waypoint: Move up another 3cm (total 6cm retreat, back to pregrasp height)
  target_pose1.position.z += 0.03; // Move up by another 3cm
  retreat_waypoints.push_back(target_pose1);

  // Create trajectory message for the retreat motion
  moveit_msgs::msg::RobotTrajectory trajectory_retreat;

  // Compute the Cartesian path for retreat motion
  // Uses the same parameters as approach (eef_step and jump_threshold)
  fraction = move_group_arm.computeCartesianPath(
      retreat_waypoints, eef_step, jump_threshold, trajectory_retreat);

  // Execute the computed retreat trajectory
  // This moves the robot back along a straight-line path
  move_group_arm.execute(trajectory_retreat);

  // Cleanup and shutdown ROS2
  rclcpp::shutdown();
  return 0;
}