# UGVC 2024 Workspace

This is the official workspace of [UGV-DTU](https://sites.google.com/dtu.ac.in/ugvdtu) for [ICTMC UGVC 2024](https://ugvc.conferences.ekb.eg/), organized into 3 branches:

- [**ros_workspace**](https://github.com/thekartikwalia/ugvc2024_ws/tree/ros_workspace): ROS packages for robot control and simulation.
- [**react_gui**](https://github.com/thekartikwalia/ugvc2024_ws/tree/react_gui): React frontend for the GUI for controlling the rover.
- [**arduino_sensors**](https://github.com/thekartikwalia/ugvc2024_ws/tree/arduino_sensors): Arduino code for interfacing with various sensors.

## Cloning the Repository

To clone the repository with all branches:

```bash
git clone https://github.com/Kartik-Walia/ugvc2024_ws
cd ugvc2024_ws
git fetch --all
```

## Switching between Branches

```bash
# Switch to the ROS workspace branch
git checkout ros_workspace 

# Switch to the React GUI branch
git checkout react_gui 

# Switch to the Arduino sensors branch
git checkout arduino_sensors 
```

For detailed information and specific branch-related instructions, please refer to the README file within each branch after checking it out.

<!-- catkin_make
source devel/setup.bash -->

<!-- To automate sourcing on new terminal windows, add this line to your `~/.bashrc`:

```bash
echo "source ~/ugvc2024_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
``` -->

<!-- ## How to Run

```bash
catkin_make
source devel/setup.bash
roslaunch <package> <node>
``` -->

<!-- ## Workspace Breakdown

This workspace is divided into nine packages, each responsible for a specific task:

- **black_box_detection**: Detects and reads numbers on a black box in the final stage.
- **launch**: Manages the launch of various sections and task nodes.
- **metal_detection**: Processes input from a metal detector for further use.
- **obstacle_detection_avoidance**: Uses RPLidar input to detect and avoid obstacles.
- **ocr**: Reads digits from detected black boxes.
- **robot_simulation**: Simulates the robot in Rviz.
- **terrain_traversing**: Performs spiral-like motion in the first stage to find metal.
- **waypoint_navigation_gps**: Uses GPS and IMU data to navigate to specified GPS coordinates.
- **waypoint_navigation_imu**: Uses IMU data to navigate in regions where GPS is unavailable. -->

## Final Rover
<img src="./Rover.png" alt="drawing" width="400"/>
