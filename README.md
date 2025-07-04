# ROS2 Universal Robot MoveIt Workspace

A ROS2 workspace containing Universal Robot configurations with MoveIt motion planning and custom motion scripts.

## 📋 What's in this Repository

This repository contains source code and configurations for:
- **Custom MoveIt Scripts**: C++ implementations for cartesian path planning and approach/retreat motions
- **Robot Descriptions**: URDF/Xacro files for UR robots and Robotiq gripper
- **MoveIt Configurations**: Planning configurations and semantic descriptions
- **Launch Files**: ROS2 launch files for motion planning scripts
- **Simulation Support**: Gazebo integration files and custom launch scripts

## 📁 Repository Structure

```
src/
├── moveit2_scripts/                    # Custom MoveIt motion planning scripts
│   ├── src/
│   │   ├── approach_retreat.cpp        # Approach and retreat motion implementation
│   │   └── cartesian_path.cpp          # Cartesian path planning implementation
│   ├── launch/                         # Launch files for motion scripts
│   ├── CMakeLists.txt                  # Build configuration
│   └── package.xml                     # Package metadata
└── universal_robot_ros2/               # UR robot packages (submodule)
    ├── Universal_Robots_ROS2_Description/     # Robot URDF models and meshes
    ├── Universal_Robots_ROS2_Gazebo_Simulation/ # Gazebo simulation files
    ├── moveit2/                        # MoveIt framework
    ├── robotiq_85_gripper/             # Robotiq gripper descriptions and drivers
    ├── test_moveit_config/             # MoveIt configuration files
    └── launch_ur3e_*.sh               # Custom launch scripts
```


## 📚 Learning Resources
Based on tutorials from [The Construct Robotics Institute](https://www.theconstructsim.com)
