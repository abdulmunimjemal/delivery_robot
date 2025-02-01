# Autonomous Delivery Robot Simulation

This final project simulates an autonomous delivery robot in a hotel environment using ROS 2, Gazebo, and various ROS packages. The simulation includes robot deployment, object spawning (hotel, tables), mapping, and navigation. The system leverages the TurtleBot3 model along with the Nav2 stack for autonomous navigation.

## System Architecture

The system is composed of the following main modules:

### A. Environment Simulation (Gazebo)
- **Gazebo Server (gzserver)**: Runs the physics engine and world simulation.
- **Gazebo Client (gzclient)**: Provides a GUI for visualizing the simulation.

### B. Robot Deployment
- **State Publisher**: Publishes the robot’s URDF model and joint states.
- **TurtleBot3 Spawner**: Spawns the TurtleBot3 robot in the simulated environment.

### C. Object Spawning
- **Hotel Model Spawner**: Loads the predefined hotel world (`hotel.sdf`).
- **Table Spawners**: Adds multiple tables at specific locations in the world.

### D. Mapping & Navigation
- **SLAM Toolbox (Optional)**: Used for mapping if required.
- **Nav2 (Navigation Stack 2)**: Enables path planning and autonomous movement using a pre-built map (`hotel_world.yaml`) and robot navigation parameters (`tb3_nav_params.yaml`).

## Prerequisites

To set up and run this project, you need the following:

- **ROS 2**: Install ROS 2 humble.
- **TurtleBot3 Packages**: Install the necessary TurtleBot3 ROS 2 packages.
- **Gazebo**: Make sure Gazebo is installed and properly configured for ROS 2.
- System Stack: Ubuntu 22.04 + ROS2 Humble + Gazebo 

If you are using **VS Code**, ensure you have the following environment variables set up before launching the simulation.

```
export QT_QPA_PLATFORM=xcb
unset GTK_PATH
source /usr/share/gazebo/setup.bash
```

## Installation

1. Clone this repository to your ROS 2 workspace:

   ```
   cd ~/ros2_ws/src
   git clone https://github.com/abdulmunimjemal/delivery_robot.git
   ```

2. Install the dependencies:

   ```
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup
   sudo apt install -y ros-humble-slam-toolbox
   ```

3. Build the workspace:

   ```
   colcon build
   ```

4. Source the setup script:

   ```
   source ~/ros2_ws/install/setup.bash
   ```

## Launch Instructions

1. **Start the simulation**:

   You can start the Gazebo simulation with the TurtleBot3 model, hotel world, and tables as follows:

   ```
   ros2 launch delivery_robot hotel_waiter.launch.py
   ```

   This will launch the following components:
   - **Gazebo Server and Client**: For simulating the environment and visualizing the robot.
   - **TurtleBot3 Robot**: Spawned at the specified coordinates.
   - **Hotel Model**: Loaded into the environment.
   - **Tables**: Spawned at predefined locations.

2. **Mapping (Optional)**:

   If you need to perform mapping, you can launch the SLAM Toolbox with the following command:

   ```
   ros2 launch delivery_robot mapping.launch.py
   ```

3. **Navigation**:

   The robot will use the Nav2 stack to autonomously navigate within the hotel environment using a pre-built map and the navigation parameters. This can be started as part of the main launch:

   ```
   ros2 launch delivery_robot navigation.launch.py
   ```

4. **RViz Visualization**:

   You can visualize the robot's status and navigation in RViz using the following command:

   ```
   ros2 run rviz2 rviz2 -d $(ros2 pkg prefix delivery_robot)/config/tb3_nav.rviz
   ```

## File Structure

```
├── CMakeLists.txt
├── config/
│   ├── hotel_world.pgm
│   ├── hotel_world.yaml
│   └── tb3_nav_params.yaml
├── delivery_robot/
│   ├── hotel_waiter_multi_button.py
│   └── spawn_entity.py
├── launch/
│   ├── navigation.launch.py
│   └── online_async_launch.py
├── models/
│   └── table/
│       └── model.sdf
├── urdf/
│   └── turtlebot3.urdf
└── world/
    ├── hotel/
    │   └── model.sdf
    └── maze/
        └── model.sdf
```

## Running the Robot

1. **Spawn the Robot**:
   The robot can be spawned into the Gazebo simulation at specified coordinates (`x_pose`, `y_pose`) using the launch files provided.

2. **Map and Navigate**:
   The robot uses a pre-built map (`hotel_world.yaml`) and a configured navigation stack (`tb3_nav_params.yaml`) to autonomously navigate the environment.

## Contributing

Feel free to open issues or contribute to the project by submitting pull requests.

## Authors

1. Abdulmunim Jundurahman - UGR/8625/14
2. Elizabeth Yonas - UGR/6912/14
3. Eyob Deresse - UGR/6771/14