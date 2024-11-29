from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
import os
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from pathlib import Path


def generate_launch_description():
    bumperbot_description_dir = get_package_share_directory("bumperbot_description")
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(
            bumperbot_description_dir,
            "urdf",
            "bumperbot.urdf.xacro",
        ),
        description="Absolute path to robot urdf file",
    )
    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]),
        value_type=str)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": robot_description}
            
        ]
    )

    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[

            str(Path(bumperbot_description_dir).parent.resolve()),
            ],
    )

    gazebo = IncludeLaunchDescription(PythonLaunchDescriptionSource([
        os.path.join(get_package_share_directory("ros_gz_sim"), "launch"),
        "/gz_sim.launch.py"
        ]),
        launch_arguments=[
            ("gz_args", [" -v 4", " -r"," empty.sdf"]),
                
        ]
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "bumperbot",
            "-topic",
            "robot_description",
        ],
        output="screen",
    )
    return LaunchDescription([
        model_arg,
        gazebo_resource_path,
        gazebo,
        gz_spawn_entity,
        robot_state_publisher,
    ])