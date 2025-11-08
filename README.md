# Homework 2: Control your Robot
:construction_worker: **Claudia Adamo**, *P38000---*

:construction_worker: **Emanuele Fusco**, *P380000338*

:construction_worker: **Giacomo Ricco**, *P380000354*

:construction_worker: **Francesco Romano**, *P38000---*

## :video_game: Kinematic Control
Open a ROS 2 workspace in a terminal and build the needed packages:
```sh
colcon build --packages-select iiwa_description iiwa_bringup ros2_kdl_package
source install/setup.bash
```

Then, launch the iiwa robot in Rviz:
```sh
ros2 launch iiwa_bringup iiwa.launch.py command_interface:="velocity" robot_controller:="velocity_controller"
```

Now, open another ROS 2 workspace and launch the `ros2_kdl_node` with its `param.yaml` and specify the type of the controller you want to adopt by using the `cmd_interface` argument, via the command:
```sh
ros2 launch ros2_kdl_package ros2_kdl.launch.py cmd_interface:=velocity_ctrl
```

to control with a simple proportional velocity controller, or with: 
```sh
ros2 launch ros2_kdl_package ros2_kdl.launch.py cmd_interface:=velocity_ctrl_null
```
to control with the velocity controller with the extra term given by the projector.

> :mag:**Note:** by default the controller is set on `position`.
