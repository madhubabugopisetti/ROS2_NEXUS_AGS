# ROS2_NEXUS_AGS

## Installation
- ROS2 JAZZY: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
- GAZEBO HARMONIC: https://gazebosim.org/docs/harmonic/install_ubuntu
- ROS–Gazebo bridge: ``` sudo apt install ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz ```
- OTHERS: ``` sudo apt install python3-colcon-common-extensions ros-jazzy-joint-state-publisher ros-jazzy-joint-state-publisher-gui ```
- CHECKING: ``` sudo apt install liburdfdom-tools ```

### Kill terminals: 
```
pkill -f ros2
pkill -f gazebo
pkill -f gz
pkill -f rqt
pkill -f rviz
pkill -f nav2
pkill -f slam_toolbox
```

### BUILD
```
cd ~/ros2_nexus_ags_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
clear
```


# GOAL 1: Render model in Gazebo

### STEP 1: Creating a workspace
```
mkdir -p ~/ros2_nexus_ags_ws/src
cd ~/ros2_nexus_ags_ws/src
source /opt/ros/jazzy/setup.bash
ros2 pkg create ags_description --build-type ament_cmake
```

### STEP 2: Creating folders structure
ags_description/
    ├── config/
        ├── 
    ├── launch/
        ├── gazebo_rviz.launch.py
    ├── urdf/
        ├── ags.xacro
    ├── worlds/
        ├── world.sdf
    ├── rviz/
        ├── ags.rviz

```
cd ~/ros2_nexus_ags_ws/src
mkdir -p ags_description/{config,launch,urdf,worlds,rviz}
touch ags_description/worlds/world.sdf
touch ags_description/launch/gazebo_rviz.launch.py
touch ags_description/rviz/ags.rviz
touch ags_description/urdf/ags.xacro
```

### STEP 3: Create world and render in gazebo
- Add world, walls in world.sdf
- Add gazebo node in gazebo_rviz.launch.py
- Terminal 1: ros2 launch ags_description gazebo_rviz.launch.py
- ![world](./src/images/image.png)