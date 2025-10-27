# Robotics-Lab Homework 1
================================

----------------------
1. Environment setup 
----------------------
Source your ROS 2 installation (example with Humble):

  source /opt/ros/humble/setup.bash
    
-----------------------Install dependencies-----------------------

If you haven’t initialized rosdep yet:

  sudo rosdep init
  rosdep update

Then, from the root of your workspace (ros2_ws):

  sudo apt update
  rosdep install --from-paths src --ignore-src -r -y

-----------------------Build the workspace-----------------------

  colcon build
  source install/setup.bash

----------------------
2. Running simulations
----------------------

RViz:

  ros2 launch armando_description armando_display.launch.py

Gazebo:

To avoid mesh-loading issues, export the resource path:

 export GZ_SIM_RESOURCE_PATH=$HOME/ros2_ws/install/armando_description/share:$GZ_SIM_RESOURCE_PATH

Then launch Gazebo:

 ros2 launch armando_gazebo armando_world.launch.py

You can switch controller types by adding the argument:

 controller_type:=trajectory
            or
 controller_type:=position

Example:

  ros2 launch armando_gazebo armando_world.launch.py controller_type:=trajectory

-----------------------Controllers-----------------------

To control the robot manually with arm_controller_node, open another terminal sourced in install/setup.bash and do:

  ros2 run armando_controller arm_controller_node

You can specify the same controller type used in Gazebo:

  ros2 run armando_controller arm_controller_node --ros-args -p controller_type:=trajectory
  or
  ros2 run armando_controller arm_controller_node --ros-args -p controller_type:=position

-----------------------Camera-----------------------

To run the camera view, open another terminal sourced in install/setup.bash and do:

  ros2 run rqt_image_view rqt_image_view
    
To see something with camera u must start Gazebo simulation before.

-----------------------Useful commands--------------------------

If after opening Gazebo with `armando_world.launch.py`, it is possible at run time to:

* Show all the loaded controllers:


  ros2 control list_controllers
 

* Activate/deactivate a controller with:

  
  ros2 control switch_controllers --activate <controller_name>

  ros2 control switch_controllers --deactivate <controller_name>
  

* Unload a controller and load another one (must be inactive first):

  
  ros2 control unload_controller <controller_name>
   
  ros2 control load_controller <new_controller_name>
  
------------------------------------------------------------------

Tip: Always run

  source install/setup.bash

after building your workspace with colcon build, and every time you open a new terminal.
