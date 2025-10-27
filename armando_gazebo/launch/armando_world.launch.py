from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    gui_arg = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Set to true to enable Gazebo GUI'
    )
    description_pkg_arg = DeclareLaunchArgument(
        'description_pkg', default_value='armando_description',
        description='Package containing the robot description'
    )
    urdf_path_arg = DeclareLaunchArgument(
        'urdf_path', default_value='urdf/arm.urdf.xacro',
        description='Path to the URDF file relative to package root'
    )
    controller_type_arg = DeclareLaunchArgument(
        'controller_type', default_value='position',
        description='Type of controller to spawn: "position" or "trajectory"'
    )


    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={
            'gui': LaunchConfiguration('gui'),
            'pause': 'true',
            'gz_args': ['empty.sdf']
        }.items()
    )


    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'description.launch.py'])
        ),
        launch_arguments={
            'urdf_package': LaunchConfiguration('description_pkg'),
            'urdf_package_path': LaunchConfiguration('urdf_path'),
        }.items()
    )


    spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        name='urdf_spawner',
        arguments=[
            '-topic', '/robot_description',
            '-entity', 'armando',
            '-z', '0.0',
            '-unpause'
        ],
        output='screen'
    )


    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )


    def spawn_controller(context, *args, **kwargs):
        controller_type = LaunchConfiguration('controller_type').perform(context)
        if controller_type == 'position':
            return [Node(
                package="controller_manager",
                executable="spawner",
                arguments=["position_controller", "--controller-manager", "/controller_manager"]
            )]
        else:
            return [Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"]
            )]


    delay_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity_node,
            on_exit=[joint_state_broadcaster],
        )
    )

    delay_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity_node,
            on_exit=[OpaqueFunction(function=spawn_controller)]
        )
    )


    gz_ros_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera@sensor_msgs/msg/Image[ignition.msgs.Image'
        ],
        output='screen'
    )


    return LaunchDescription([
        gui_arg,
        description_pkg_arg,
        urdf_path_arg,
        controller_type_arg,
        gazebo_launch,
        description_launch,
        spawn_entity_node,
        delay_joint_state_broadcaster,
        delay_controller,
        gz_ros_bridge
    ])

