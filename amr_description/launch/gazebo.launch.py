import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    amr_description_dir = get_package_share_directory("amr_description")


    model_arg = DeclareLaunchArgument(name="model",default_value=os.path.join(amr_description_dir,"urdf","amr.urdf.xacro"), description="Absolute path to robot urdf file")



    # Use xacro to process the model
    robot_description = ParameterValue(Command([
            "xacro ", 
            LaunchConfiguration("model")
            ]),
            value_type=str
    )


    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable = "robot_state_publisher",
        parameters=[{"robot_description":robot_description}]
    )

    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCEPATH",
        value=[
            str(Path(amr_description_dir).parent.resolve())
        ]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory("ros_gz_sim"),"launch"),"/gz_sim.launch.py"]),
            launch_arguments={
                    "gz_args": f"-v 4 -r {os.path.join(amr_description_dir, 'worlds', 'stairs.sdf')}"
                    }.items()
        )

    gz_spawn_entity = Node(
        package = "ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic","robot_description",
                   "-name","amr"]

    )

    return LaunchDescription([
        model_arg,
        robot_state_publisher_node,
        gazebo_resource_path,
        gazebo,
        gz_spawn_entity
    ])
