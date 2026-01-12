# Fanuc CRX-10iA – MoveIt Task 1 (ROS 2 Humble)

## Overview
This repository contains **Task 1** of the DexSent Robotics assignment.  
The objective is to set up the **Fanuc CRX-10iA** robot in **ROS 2 Humble**, configure **MoveIt 2**, integrate **ros2_control**, and execute motion using a real **FollowJointTrajectory** action server.  

> Gripper integration is **partially completed**.

---

## What Has Been Completed (Task 1)

### Robot Description (URDF)
- Fanuc CRX-10iA 6-DOF robot model loaded
- All joints defined with limits and axes
- Correct TF chain up to `tool0`
- Visual and collision meshes configured

### ros2_control Integration
- `ros2_control` block embedded in the URDF
- Hardware simulated using `mock_components/GenericSystem`
- Each joint exposes:
  - `position` command interface
  - `position` and `velocity` state interfaces

### Controllers
- `joint_state_broadcaster`
- `joint_trajectory_controller`
- Controllers successfully loaded and activated

### FollowJointTrajectory Action
- Action server available at:
  `/joint_trajectory_controller/follow_joint_trajectory`
- Verified using:
  `ros2 action info /joint_trajectory_controller/follow_joint_trajectory`
- MoveIt sends trajectories directly to this controller (real execution)

### MoveIt 2 Setup
- MoveIt configuration generated via MoveIt Setup Assistant
- Planning group: `arm`
- `moveit_simple_controller_manager` configured
- Controllers correctly mapped to FollowJointTrajectory
- RViz MotionPlanning panel fully functional

### Motion Planning & Execution
- Interactive marker used to define goal poses
- Motion planned and executed from RViz
- Robot state updates correctly during execution

### Robotiq 2F-85 Gripper (Partial)
- Separate description package created: `robotiq_2f_85_description`
- Gripper URDF and meshes imported
- Folder structure prepared for integration
- Gripper not yet attached to `tool0`
- No gripper controller configured (left intentionally for later)

---

## Repository Structure

fanuc-crx10ia-moveit-task1/
├── moveit_configs/
│   ├── config/
│   ├── launch/
│   ├── package.xml
│   └── CMakeLists.txt
└── src/
    ├── fanuc_crx10ia_description/
    │   ├── urdf/
    │   ├── meshes/
    │   ├── launch/
    │   └── config/
    └── robotiq_2f_85_description/
        ├── urdf/
        ├── meshes/
        └── launch/

Note: build/, install/, and log/ directories are intentionally excluded.

---

## Requirements
- Ubuntu 22.04
- ROS 2 Humble
- MoveIt 2
- colcon
- RViz2

---

## How to Run

### 1. Create Workspace
mkdir -p ~/dexsent_ws/src  
cd ~/dexsent_ws  

### 2. Clone Repository
cd src  
git clone https://github.com/KashifAlam407/fanuc-crx10ia-moveit-task1.git  

### 3. Build
cd ~/dexsent_ws  
colcon build  
source install/setup.bash  

### 4. Launch Robot and Controllers
ros2 launch fanuc_crx10ia_description control.launch.py  

Verify controllers:
ros2 control list_controllers  

### 5. Launch MoveIt and RViz
ros2 launch moveit_configs demo.launch.py  

---

## How to Test Motion Execution
1. Open RViz → MotionPlanning panel  
2. Select planning group: `arm`  
3. Drag the interactive marker  
4. Click **Plan**  
5. Click **Execute**  

The trajectory is executed using the real  
`FollowJointTrajectory` action server.

---

## Notes
- Focus is on system integration, not grasping logic
- Partial gripper setup is intentional and documented
- Task 1 requirements are satisfied

---

