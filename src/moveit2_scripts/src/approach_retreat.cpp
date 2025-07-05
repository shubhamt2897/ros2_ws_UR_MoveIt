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
  // This corresponds to the group defined in the robot's SRDF file which is part of official Universal Robots ROS2 package
  // SOURCE: /home/shubham-0802/ros2_ws_UR_MoveIt/src/universal_robot_ros2/test_moveit_config/config/name.srdf
  // - Line 22: <group name="ur_manipulator">
  // - Line 23:   <chain base_link="base_link" tip_link="tool0"/>
  // - Line 24: </group>
  // This planning group includes all joints from base_link to tool0 (the entire UR arm)
  static const std::string PLANNING_GROUP_ARM = "ur_manipulator";

  // Create MoveIt interface for controlling the robot arm
  // This interface handles motion planning, collision checking, and execution
  //
  // CONSTRUCTOR PARAMETERS EXPLAINED:
  // Parameter 1: move_group_node (rclcpp::Node::SharedPtr)
  //   - This is the ROS2 node that was created above
  //   - Provides the communication interface to ROS2 topics, services, and actions
  //   - MoveGroupInterface uses this node to communicate with the move_group server
  //
  // Parameter 2: PLANNING_GROUP_ARM (const std::string&)
  //   - Value: "ur_manipulator" 
  //   - This is the planning group name defined in the robot's SRDF file
  //   - Source file: /home/shubham-0802/ros2_ws_UR_MoveIt/src/universal_robot_ros2/test_moveit_config/config/name.srdf line 22
  //   - SRDF definition: <group name="ur_manipulator">
  //                        <chain base_link="base_link" tip_link="tool0"/>
  //                      </group>
  //   - This defines which joints/links are part of this planning group
  //   - The chain goes from base_link to tool0, including all UR robot arm joints
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

  // ============================================================================
  // PHASE 1: GO TO HOME POSITION
  // ============================================================================
  // Set the start state to current state before planning any motion
  // This ensures the planner knows where the robot currently is
  move_group_arm.setStartStateToCurrentState();
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
  // Update robot state after the previous motion
  RCLCPP_INFO(LOGGER, "Pregrasp Position");
  current_state_arm = move_group_arm.getCurrentState(10);
  current_state_arm->copyJointGroupPositions(joint_model_group_arm,
                                             joint_group_positions_arm);

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
  // PHASE 3: APPROACH THE OBJECT
  // ============================================================================
  // Move closer to the target object for grasping
  RCLCPP_INFO(LOGGER, "Approach to object");
  current_state_arm = move_group_arm.getCurrentState(10);
  current_state_arm->copyJointGroupPositions(joint_model_group_arm,
                                             joint_group_positions_arm);

  // Define the approach distance (how much to move down toward the object)
  float delta = 0.04; // 4 cm approach distance
  
  // Keep the same orientation as pregrasp (pointing downward)
  target_pose1.orientation.x = -1.0;
  target_pose1.orientation.y = 0.00;
  target_pose1.orientation.z = 0.00;
  target_pose1.orientation.w = 0.00;
  
  // Keep X and Y positions the same, only move down in Z
  target_pose1.position.x = 0.343;
  target_pose1.position.y = 0.132;
  target_pose1.position.z = target_pose1.position.z - delta; // Move down by delta
  
  move_group_arm.setPoseTarget(target_pose1);

  // Plan and execute the approach motion
  success_arm = (move_group_arm.plan(my_plan_arm) ==
                 moveit::core::MoveItErrorCode::SUCCESS);

  move_group_arm.execute(my_plan_arm);

  // ============================================================================
  // PHASE 4: RETREAT FROM THE OBJECT
  // ============================================================================
  // Move away from the object (simulating lifting after grasping)
  RCLCPP_INFO(LOGGER, "Retreat from object");
  current_state_arm = move_group_arm.getCurrentState(10);
  current_state_arm->copyJointGroupPositions(joint_model_group_arm,
                                             joint_group_positions_arm);

  // Keep the same orientation (still pointing downward)
  target_pose1.orientation.x = -1.0;
  target_pose1.orientation.y = 0.00;
  target_pose1.orientation.z = 0.00;
  target_pose1.orientation.w = 0.00;
  
  // Keep X and Y positions the same, move up in Z direction
  target_pose1.position.x = 0.343;
  target_pose1.position.y = 0.132;
  target_pose1.position.z = target_pose1.position.z + delta; // Move up by delta (same distance as approach)
  
  move_group_arm.setPoseTarget(target_pose1);

  // Plan and execute the retreat motion
  success_arm = (move_group_arm.plan(my_plan_arm) ==
                 moveit::core::MoveItErrorCode::SUCCESS);

  move_group_arm.execute(my_plan_arm);

  // Cleanup and shutdown ROS2
  rclcpp::shutdown();
  return 0;
}