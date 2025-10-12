**Turtlebot3 Trajectory Tracking Simulation**

Setup and Launch
Build and source workspace:
```bash
cd ~/ROS/turtletrack_ws
colcon build
source install/setup.bash
ros2 launch turtlebot3 simulation.launch.py
```
Launch the Turtlebot3 simulation:
```bash
ros2 launch turtlebot3 simulation.launch.py
```

Launch the trajectory controller:
```bash
ros2 launch trajectory_controller trajectory_follow.launch.py
```

**Project Overview**
This project implements trajectory tracking for a differential drive robot (Turtlebot3) using ROS2 in simulation.

**File Descriptions :**

**simulation.launch.py:** Launches the Turtlebot3 Gazebo simulation environment.

**trajectory_follow.launch.py:** Starts the controller node to make the robot follow predefined trajectories.

**trajectory_controller.py:** Contains the main control logic to track the trajectory by computing velocity commands.

**trajectory_generation.py:** Generates smooth trajectories based on predefined waypoints.

**path_smoother.py:** Implements path smoothing algorithms for smoother robot motion.

**smoothed_path_visualizer.py:** ROS node for visualizing the smoothed trajectory.

**waypoints.yaml:** Defines the set of waypoints for the robot to follow during the simulation.

**robot_state_publisher.launch.py:** Publishes the robot state information in ROS.

**ros_ign_bridge.launch.py:** Bridges ROS2 and Ignition Gazebo topics for communication.

**Design Logic**
The robot's waypoints are defined in the waypoints.yaml file.

trajectory_generation.py creates smooth trajectories from these waypoints.

path_smoother.py improves trajectory smoothness to avoid abrupt motions.

trajectory_controller.py uses feedback control to follow the generated trajectory precisely.

Launch files modularize simulation and control components to allow easy testing.

**Extending to a Real Robot**
Replace simulation launch files with hardware-specific launch files.

Integrate real sensor data inputs in place of simulated data.

Tune controller parameters based on real robot dynamics and latency.

Maintain ROS2 interface for modular software reusability.
