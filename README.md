# Stream_Motion_Controller
This repository contains the C++ controller for controlling the Fanuc robots using the Stream Motion interface.


## Instructions
1. Modify the configuration file `path_to_repo/config/user_config.json`.
2. Launch the controller node:
```
roslaunch stmotion_controller controller_node.launch
```
3. Launch your task planning file:
```
roslaunch stmotion_controller controller_usage_example.launch
# OR
rosrun stmotion_controller controller_usage_example.py
```
