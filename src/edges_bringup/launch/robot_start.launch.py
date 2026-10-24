from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    ld = LaunchDescription()

    urdf_path = PathJoinSubstitution(
        [FindPackageShare("edges_description"), "urdf", "robot.urdf.xacro"]
    )

    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare("edges_description"), "rviz", "rviz_config.rviz"]
    )

    world_path = PathJoinSubstitution(
        [FindPackageShare("edges_description"), "worlds", "walls.world"]
    )

    start_robot_description = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": Command(["xacro ", urdf_path])}],
    )

    start_joint_state = Node(
        package="joint_state_publisher_gui", executable="joint_state_publisher_gui"
    )

    start_rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_path],
    )

    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={"gz_args": [world_path, " -r"], "gz_version": "6"}.items(),
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-world",
            "walls_world",
            "-topic",
            "robot_description",
            "-name",
            "edges_robot",
            "-Y",
            "0.7853981634",
        ],
    )

    start_cmd_vel_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
        ],
    )

    ld.add_action(start_robot_description)
    # ld.add_action(start_joint_state)
    # ld.add_action(start_rviz)
    ld.add_action(start_gazebo)
    ld.add_action(spawn_robot)
    ld.add_action(start_cmd_vel_bridge)

    return ld
