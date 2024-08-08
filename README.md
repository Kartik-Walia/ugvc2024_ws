# UGV Software - ROS Noetic Workspace

## How to Use This Repository

```bash
catkin_make
source devel/setup.bash
```

To automate sourcing on new terminal windows, add this line to your `~/.bashrc`:

```bash
echo "source ~/ugvc2024_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## How to Run

```bash
catkin_make
source devel/setup.bash
roslaunch <package_name> <node_name>
```

## Workspace Breakdown

This workspace is divided into folowing packages, each responsible for a specific task:

- **black_box_detection**: Detects and reads numbers on a black box in the final stage.
- **launch**: Manages the launch of various sections and task nodes.
- **metal_detection**: Processes input from a metal detector for further use.
- **obstacle_detection_avoidance**: Uses RPLidar input to detect and avoid obstacles.
- **ocr**: Reads digits from detected black boxes.
- **robot_simulation**: Simulates the robot in Rviz.
- **terrain_traversing**: Performs spiral-like motion in the first stage to find metal.
- **waypoint_navigation_gps**: Uses GPS and IMU data to navigate to specified GPS coordinates.
- **waypoint_navigation_imu**: Uses IMU data to navigate in regions where GPS is unavailable.