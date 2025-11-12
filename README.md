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
> 💡**Tip:** to improve the build, first run this line:
> ```sh
> rm -r build/ log/ install/
> ```
> and then the `colcon build` and `source` command.

Then, launch the iiwa robot in Rviz:
```sh
ros2 launch iiwa_bringup iiwa.launch.py command_interface:="velocity" robot_controller:="velocity_controller"
```

Now, open another ROS 2 workspace, launch the `ros2_kdl_node` with its `param.yaml` and specify the type of the controller you want to adopt by using the `cmd_interface` and `ctrl` arguments:
* **position controller**: 
```sh
ros2 launch ros2_kdl_package ros2_kdl.launch.py cmd_interface:=position
```
* **base velocity controller**: 
```sh
ros2 launch ros2_kdl_package ros2_kdl.launch.py cmd_interface:=velocity ctrl:=velocity_ctrl
```
* **velocity controller** with the extra term given by the **projector**:
```sh
ros2 launch ros2_kdl_package ros2_kdl.launch.py cmd_interface:=velocity ctrl:=velocity_ctrl_null
```

> :mag:**Note:** the default controller is set on `position`.

### Client-Server version (point 1c)
Open a ROS 2 workspace in a terminal and build the needed packages:
```sh
colcon build --packages-select iiwa_description iiwa_bringup ros2_kdl_package ros2_kdl_action_interface
source install/setup.bash
```
Then, launch the iiwa robot in Rviz:
```sh
ros2 launch iiwa_bringup iiwa.launch.py command_interface:="velocity" robot_controller:="velocity_controller"
```

Now, open another ROS 2 workspace, launch the server `ros2_kdl_node` through the `node` argument and specify the type of velocity controller by filling the `cmd_interface` and `ctrl` arguments:
```sh
ros2 launch ros2_kdl_package ros2_kdl.launch.py cmd_interface:=velocity ctrl:=velocity_ctrl|velocity_ctrl_null node:=server
```

Finally, in another terminal launch the client node:
```sh
ros2 launch ros2_kdl_package ros2_kdl.launch.py cmd_interface:=velocity ctrl:=velocity_ctrl|velocity_ctrl_null node:=client
```
> :mag:**Note:** by deafault the position controller and the server node are selected.

## :earth_africa: Gazebo world
Open a ROS 2 workspace in a terminal and build the needed packages:
```sh
colcon build --packages-select iiwa_description iiwa_bringup ros2_kdl_package ros2_kdl_action_interface aruco aruco_ros aruco_msgs
source install/setup.bash
```

Add the world path to `GZ_SIM_RESOURCE_PATH` by modifying the `.bashrc` file as follows:
```sh
echo 'export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/ros2_ws/src/ros2_iiwa/iiwa_description/gazebo/models' >> ~/.bashrc
source ~/.bashrc
```
>:white_check_mark:**Test:** to check if all is done properly you can run `echo $GZ_SIM_RESOURCE_PATH` and it should show the path `/home/user/ros2_ws/src/ros2_iiwa/iiwa_description/gazebo/models`.

Therefore, launch the iiwa robot in the new Gazebo world by setting `use_sim:="true"` as follows:
```sh
ros2 launch iiwa_bringup iiwa.launch.py command_interface:="velocity" robot_controller:="velocity_controller" use_sim:="true"
```

In another terminal, the Aruco tag can be detected by the camera by running the following command:
```sh
ros2 run rqt_image_view rqt_image_view
```
and by selecting the topic `/aruco_node/result`.

### Following Aruco tag (not working properly)
Launch the iiwa robot in gazebo by setting `use_sim:="true"` as before:
```sh
ros2 launch iiwa_bringup iiwa.launch.py command_interface:="velocity" robot_controller:="velocity_controller" use_sim:="true"
```
In another terminal, launch the `ros2_kdl_node` with `ctrl:=vision_ctrl`:
```sh
ros2 launch ros2_kdl_package ros2_kdl.launch.py cmd_interface:=velocity ctrl:=vision_ctrl
```
