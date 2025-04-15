import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue



def generate_launch_description():
    
    amr_description_dir = get_package_share_directory("amr_description")

    model_arg = DeclareLaunchArgument(name="model1",default_value=os.path.join(
                                            amr_description_dir,"urdf","amr.urdf.xacro"
                                            ), 
                                        description="Absolute path to robot urdf file")




    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model1")]), 
        value_type=str)

    #rviz_config_path = os.path.join(get_package_share_directory("amr_description"), 'rviz', 'view_robot.rviz')

    # Define nodes
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        
    )

#name="rviz2",#arguments=['-d', rviz_config_path]

    return LaunchDescription([
        model_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])