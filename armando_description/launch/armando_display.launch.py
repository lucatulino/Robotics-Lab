from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    arm_description_path = get_package_share_directory('armando_description')
    urdf_path = os.path.join(arm_description_path, "urdf", "arm.urdf.xacro") 
    rviz_config = os.path.join(arm_description_path, "config", "rvizconfig.rviz")


    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f"URDF file not found: {urdf_path}")

    with open(urdf_path, 'r') as infp:
        robot_description_config = infp.read()

    robot_description = {'robot_description': robot_description_config}


    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

  
    joint_state_publisher_node_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
    )
    
 
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
    )

    nodes_to_start = [
        robot_state_publisher_node,
        joint_state_publisher_node_gui,
        rviz_node,
    ]

    return LaunchDescription(nodes_to_start)

