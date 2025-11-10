------------------------------
The project flow 
------------------------------
phase 1-
Terminal 1
# Source ROS 2 Humble and your workspace overlay
source /opt/ros/humble/setup.bash
source ~/ws_moveit2/install/setup.bash   # Replace with your workspace path if different
# Launch MoveIt Task Constructor demo
ros2 launch moveit2_tutorials mtc_demo.launch.py

Terminal 2
# Source ROS 2 Humble and your workspace overlay
source /opt/ros/humble/setup.bash
source ~/ws_moveit2/install/setup.bash   # Replace with your workspace path if different
# Launch the Pick-and-Place demo
ros2 launch moveit2_tutorials pick_place_demo.launch.py

phase 2-
# Terminal 3 — start server and watch
source /opt/ros/humble/setup.bash
source ~/ws_moveit2/install/setup.bash

# Run server (foreground)
ros2 run panda_keyboard_control moveit_service_action

#Terminal 4
source /opt/ros/humble/setup.bash
source ~/ws_moveit2/install/setup.bash

ros2 service list | grep -E "moveit_command|move_command" || true
ros2 service call /moveit_command panda_interfaces/srv/MoveCommand "{command: 'home'}"

phase 3-
# Terminal 5
source /opt/ros/humble/setup.bash
source ~/ws_moveit2/install/setup.bash

ros2 action list | grep pick || true
ros2 action send_goal /pick_place panda_interfaces/action/PickPlace "{object_name: 'cube'}" --feedback

phase 4-
#Terminal 6 
source ~/ws_moveit2/install/setup.bash
ros2 run panda_keyboard_control keyboard_control

#Terminal 7
source ~/ws_moveit2/install/setup.bash
ros2 topic echo /panda_command