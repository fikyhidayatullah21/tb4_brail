This project is for my midterm exam in the Localization and Mapping/RE702 course. The goal is for turltlebot4 to move to point A, activate the bell once, then move to point B and activate the bell twice.

---


# How To Build

## Create Folder Workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

## Clone this repo
git clone https://github.com/fikyhidayatullah21/tb4_brail.git

## Install package and dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

## Build the package
cd ~/ros2_ws
colcon build

---

# How To Connect from PC to Turtle
## Via Ethernet
ssh ubuntu@192.168.185.3
## Via WiFi
ssh ubuntu@your_robot_ip

---

# How To Use Mapping Mode
## Mapping Launch
ros2 launch turtlebot4_navigation slam.launch.py

## Rviz Run
ros2 launch turtlebot4_viz view_navigation.launch.py  # This is Jazzy
ros2 launch turtlebot4_viz view_robot.launch.py       # This is Humble

## Control Robot Via Teleop Keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true

---

# How To Run Nav2
## Localization Launch
cd ~/ros2_ws
source install/setup.bash
ros2 launch tb4_brail localization.launch.py

## Navigation Launch
cd ~/ros2_ws
source install/setup.bash
ros2 launch tb4_brail uts_nav.launch.py

## Rviz Run
ros2 launch turtlebot4_viz view_navigation.launch.py  # This is  Jazzy
ros2 launch turtlebot4_viz view_robot.launch.py       # This is Humble

## Run the send goal Point A and Point B
cd ~/ros2_ws
source install/setup.bash
ros2 run tb4_brail tb4_brail_node

