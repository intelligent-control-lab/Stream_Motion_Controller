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


## Citation
If you find this repository helpful, please kindly cite our works.
```
@INPROCEEDINGS{9863251,
      author={Liu, Ruixuan and Chen, Rui and Sun, Yifan and Zhao, Yu and Liu, Changliu},
      booktitle={2022 IEEE/ASME International Conference on Advanced Intelligent Mechatronics (AIM)}, 
      title={Jerk-bounded Position Controller with Real-Time Task Modification for Interactive Industrial Robots}, 
      year={2022},
      volume={},
      number={},
      pages={1771-1778},
      doi={10.1109/AIM52237.2022.9863251}
}

@article{liu2022safe,
      title={Safe Interactive Industrial Robots using Jerk-based Safe Set Algorithm}, 
      author={Ruixuan Liu and Rui Chen and Changliu Liu},
      journal={arXiv preprint arXiv:2204.03038},
      year={2022}
}
```