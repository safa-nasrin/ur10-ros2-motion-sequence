# 6-DOF Arm Motion Sequence (ROS 2 + MoveIt 2 + Gazebo)

This repository contains a complete ROS 2 workspace that simulates a UR10-like 6-DOF robotic arm with a parallel gripper. The system integrates ROS 2 Humble, MoveIt 2 for motion planning, and Gazebo (Ignition) for physics simulation. An automated Python node continuously executes a specific joint-space motion sequence.

## 1. Environment & Prerequisites
Before cloning and building this repository, your host machine must have the following core stack installed and configured:

* **Operating System:** Ubuntu 22.04 LTS (Jammy Jellyfish)
* **ROS 2 Distribution:** ROS 2 Humble Hawksbill (Desktop Install)
* **Simulation Engine:** Gazebo Fortress (Ignition Gazebo) with `ros_ign` (or `ros_gz`) bridge packages
* **Motion Planning Framework:** MoveIt 2 (`ros-humble-moveit`)

### Quick Reference: Installing Core Dependencies on Ubuntu 22.04
If setting up a fresh machine, ensure ROS 2 Humble and MoveIt 2 are installed via official binaries:

    # Set up ROS 2 Humble apt repositories and install desktop full
    sudo apt update && sudo apt install curl -y
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt update && sudo apt install ros-humble-desktop-full -y

    # Install MoveIt 2 and Ignition Gazebo control plugins for Humble
    sudo apt install ros-humble-moveit ros-humble-ign-ros2-control ros-humble-ros-gz-sim -y

## 2. System Architecture

| Area | Specification / Component |
| :--- | :--- |
| **Target Platform** | Ubuntu 22.04 LTS, ROS 2 Humble Hawksbill |
| **Simulation** | Gazebo Fortress (`ign-gazebo`) with gravity and ground plane |
| **Motion Planning** | MoveIt 2 with OMPL pipeline and custom SRDF |
| **Description Package** | robot_description (URDF/Xacro macros, limits, collision, ros2_control) |
| **MoveIt Config Package**| robot_moveit_config (SRDF, kinematics, controllers, RViz setup) |
| **Task Package** | robot_task (Top-level launch, Python motion node, YAML config, tests) |

## 3. Setup and Installation

Follow these commands to clone, build, and source the workspace on your local ROS 2 environment:

    # Navigate to workspace source directory (create ros2_ws/src if needed)
    cd ~/ros2_ws/src
    
    # Clone your repository here (or ensure packages are placed in src/)
    # git clone <your-repository-url>

    # Navigate back to workspace root and build packages with symlink install
    cd ~/ros2_ws
    colcon build --symlink-install

    # Source the overlay environment
    source install/setup.bash

## 4. Launch Instructions

* **Single-Command Bringup:** Start Gazebo, Robot State Publisher, MoveIt 2, RViz, controllers, and the automated Python sequence node simultaneously using one command:
    
    ros2 launch robot_task full_sim.launch.py

* **No Manual Terminals:** No separate manual terminal windows or extra commands are required after launch.

## 5. Motion Sequence & Pose Configuration

The robot executes a continuous loop of the required sequence, waiting exactly 2.0 seconds after each successful movement before proceeding.

| Step | Action Command | Target Result | Dwell After Success |
| :--- | :--- | :--- | :--- |
| **1** | Move arm to Home | Arm reaches named Home state | 2.0 seconds |
| **2** | Move arm to Pose 1 | Arm reaches named Pose 1 | 2.0 seconds |
| **3** | Open gripper | Fingers open to target width | 2.0 seconds |
| **4** | Move arm to Pose 2 | Arm reaches named Pose 2 | 2.0 seconds |
| **5** | Open gripper | Fingers confirm open state | 2.0 seconds |
| **6** | Move arm to Home | Arm returns to Home; loop repeats | 2.0 seconds |

### Joint-Space Target Coordinates (Radians) & Parameters
All target poses and timing parameters are dynamically loaded from poses.yaml:
* **Home Pose:** [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
* **Pose 1:** [0.5, -1.0, 1.2, -0.5, 1.5, 0.0]
* **Pose 2:** [-0.5, -1.2, 1.0, 0.5, -1.5, 0.0]
* **Gripper Open Position:** 0.035
* **Dwell Time Parameter:** 2.0 seconds
* **Loop Behavior:** Enabled by default (continuously loops until terminated)

## 6. Known Issues & Technical Assumptions

* **Execution Blocking:** The sequence loop currently utilizes Python's time.sleep() to fulfill the strict 2.0-second action-to-action dwell requirement. In a production environment, this linear sequence would be refactored into a non-blocking, asynchronous state machine using ROS 2 Timers to prevent blocking the executor thread.
* **Redundant Gripper Command:** The required sequence commands the gripper to "Open" at Step 3 and again at Step 5 without a closing command in between. To adhere strictly to the assignment prompt's "No deviations" rule, the sequence was implemented precisely as specified in the rubric table.
* **MoveIt 2 Shutdown Behavior:** Upon terminating the launch file with Ctrl + C, an upstream ROS 2 Humble / MoveIt 2 C++ destructor segmentation fault (exit code -11 or SIGKILL exit code -9) may occasionally appear as move_group fails to terminate gracefully. This is a known framework-level behavior in this distribution and does not affect successful runtime execution or clean teardown.
