import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    rname = LaunchConfiguration("rname")
    enable_foot_contact_bridge = LaunchConfiguration("enable_foot_contact_bridge")
    foot_contact_threshold = LaunchConfiguration("foot_contact_threshold")
    foot_contact_debug = LaunchConfiguration("foot_contact_debug")

    wname = "earth"
    robot_name = ParameterValue(Command(["echo -n ", rname]), value_type=str)
    ros_namespace = ParameterValue(Command(["echo -n ", "/", rname, "_gazebo"]), value_type=str)
    gazebo_model_name_value = Command(["echo -n ", rname, "_gazebo"])
    gazebo_model_name = ParameterValue(gazebo_model_name_value, value_type=str)

    robot_description = ParameterValue(
        Command([
            "xacro ",
            Command(["echo -n ", Command(["ros2 pkg prefix ", rname, "_description"])]),
            "/share/", rname, "_description/xacro/robot.xacro"
        ]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            # "verbose": "true",
            # "pause": "true",  # Not Available
            "world": os.path.join(get_package_share_directory("rl_sar"), "worlds", wname + ".world"),
        }.items(),
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "/robot_description",
            "-entity", gazebo_model_name_value,
            "-z", "1.0",
        ],
        output="screen",
    )

    joint_state_broadcaster_node = Node(
        package="controller_manager",
        executable='spawner.py' if os.environ.get('ROS_DISTRO', '') == 'foxy' else 'spawner',
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    robot_joint_controller_node = Node(
        package="controller_manager",
        executable='spawner.py' if os.environ.get('ROS_DISTRO', '') == 'foxy' else 'spawner',
        arguments=["robot_joint_controller"],
        output="screen",
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'deadzone': 0.1,
            'autorepeat_rate': 0.0,
        }],
    )

    param_node = Node(
        package="demo_nodes_cpp",
        executable="parameter_blackboard",
        name="param_node",
        parameters=[{
            "robot_name": robot_name,
            "gazebo_model_name": gazebo_model_name,
        }],
    )

    foot_contact_bridge_node = Node(
        package="rl_sar",
        executable="run_foot_contact_bridge.py",
        name="foot_contact_bridge",
        output="screen",
        condition=IfCondition(enable_foot_contact_bridge),
        arguments=[
            "--threshold", foot_contact_threshold,
            "--debug", foot_contact_debug,
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "rname",
            description="Robot name; this workspace supports go2",
            default_value=TextSubstitution(text=""),
        ),
        DeclareLaunchArgument(
            "enable_foot_contact_bridge",
            description="Start the Gazebo contact-force to /rl_sar/foot_contacts bridge for FTNet policies.",
            default_value=TextSubstitution(text="false"),
        ),
        DeclareLaunchArgument(
            "foot_contact_threshold",
            description="Contact force threshold in Newtons for the FTNet foot contact bridge.",
            default_value=TextSubstitution(text="1.0"),
        ),
        DeclareLaunchArgument(
            "foot_contact_debug",
            description="Print FTNet foot contact bridge force/contact values once per second.",
            default_value=TextSubstitution(text="false"),
        ),
        robot_state_publisher_node,
        gazebo,
        spawn_entity,
        joint_state_broadcaster_node,
        # robot_joint_controller_node,  # Spawn in rl_sim.cpp
        joy_node,
        param_node,
        foot_contact_bridge_node,
    ])
