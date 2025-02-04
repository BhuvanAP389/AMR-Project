from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    wheel_radius_front_left_arg = DeclareLaunchArgument(
        "wheel_radius_front_left",
        default_value = "0.065"
    )

    wheel_radius_middle_left_arg = DeclareLaunchArgument(
        "wheel_radius_middle_left",
        default_value = "0.055"
    )

    wheel_radius_rear_left_arg = DeclareLaunchArgument(
        "wheel_radius_rear_left",
        default_value = "0.055"
    )

    wheel_radius_front_right_arg = DeclareLaunchArgument(
        "wheel_radius_front_right",
        default_value = "0.065"
    )

    wheel_radius_middle_right_arg = DeclareLaunchArgument(
        "wheel_radius_middle_right",
        default_value = "0.055"
    )

    wheel_radius_rear_right_arg = DeclareLaunchArgument(
        "wheel_radius_rear_right",
        default_value = "0.055"
    )

    wheel_separation_arg = DeclareLaunchArgument(
        "wheel_separation",
        default_value = "0.233"
    )

    wheel_radius_front_left = LaunchConfiguration("wheel_radius_front_left")
    wheel_radius_middle_left = LaunchConfiguration("wheel_radius_middle_left")
    wheel_radius_rear_left = LaunchConfiguration("wheel_radius_rear_left")     
    wheel_radius_front_right = LaunchConfiguration("wheel_radius_front_right")
    wheel_radius_middle_right = LaunchConfiguration("wheel_radius_middle_right")
    wheel_radius_rear_right = LaunchConfiguration("wheel_radius_rear_right")
    wheel_separation = LaunchConfiguration("wheel_separation")

                                                   
                                                   
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager"
        ]
    )

    simple_velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "simple_velocity_controller",
            "--controller-manager",
            "/controller_manager"
        ]
    )

    delivery_box_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "delivery_box_controller",
            "--controller-manager",
            "/controller_manager"
        ]
    )

    motor_controller_py = Node(
        package="amr_controller",
        executable="motor_controller.py",
        parameters = [{"wheel_radius_front_left": wheel_radius_front_left,
                       "wheel_radius_middle_left":wheel_radius_middle_left,
                       "wheel_radius_rear_left": wheel_radius_rear_left,
                       "wheel_radius_front_right": wheel_radius_front_right,
                       "wheel_radius_middle_right": wheel_radius_middle_right,
                       "wheel_radius_rear_right": wheel_radius_rear_right,
                       "wheel_separation": wheel_separation
                       }]
    )


    return LaunchDescription([
        wheel_radius_front_left_arg,
        wheel_radius_middle_left_arg,
        wheel_radius_rear_left_arg,
        wheel_radius_front_right_arg,
        wheel_radius_middle_right_arg,
        wheel_radius_rear_right_arg,
        wheel_separation_arg,
        joint_state_broadcaster_spawner,
        simple_velocity_controller_spawner,
        delivery_box_controller_spawner,
        motor_controller_py,

        
    ])