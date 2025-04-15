from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

def noisy_controller(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration("use_sim_time")
    wheel_radius_middle_left = float(LaunchConfiguration("wheel_radius_middle_left").perform(context))
    wheel_radius_middle_right = float(LaunchConfiguration("wheel_radius_middle_right").perform(context))
    wheel_separation = float(LaunchConfiguration("wheel_separation").perform(context))
    wheel_radius_middle_left_error = float(LaunchConfiguration("wheel_radius_middle_left_error").perform(context))
    wheel_radius_middle_right_error = float(LaunchConfiguration("wheel_radius_middle_right_error").perform(context))
    wheel_separation_error = float(LaunchConfiguration("wheel_separation_error").perform(context))
    
    noisy_controller_py = Node(
        package="amr_controller",
        executable="noisy_controller.py",
        parameters=[{"wheel_radius_middle_left" : wheel_radius_middle_left + wheel_radius_middle_left_error,
                    "wheel_radius_middle_right": wheel_radius_middle_right + wheel_radius_middle_right_error,
                    "wheel_separation": wheel_separation + wheel_separation_error,
                    "use_sim_time": use_sim_time}],
    )

    return [
        noisy_controller_py
    ]

def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
    )

    wheel_radius_front_left_arg = DeclareLaunchArgument(
        "wheel_radius_front_left",
        default_value = "0.065",
    )

    wheel_radius_middle_left_arg = DeclareLaunchArgument(
        "wheel_radius_middle_left",
        default_value = "0.055",
    )

    wheel_radius_rear_left_arg = DeclareLaunchArgument(
        "wheel_radius_rear_left",
        default_value = "0.055",
    )

    wheel_radius_front_right_arg = DeclareLaunchArgument(
        "wheel_radius_front_right",
        default_value = "0.065",
    )

    wheel_radius_middle_right_arg = DeclareLaunchArgument(
        "wheel_radius_middle_right",
        default_value = "0.055",
    )

    wheel_radius_rear_right_arg = DeclareLaunchArgument(
        "wheel_radius_rear_right",
        default_value = "0.055",
    )

    wheel_separation_arg = DeclareLaunchArgument(
        "wheel_separation",
        default_value = "0.233",
    )

    wheel_separation_front_middle_arg = DeclareLaunchArgument(
        "wheel_separation_front_middle",
        default_value = "0.155",
    )

    wheel_separation_middle_back_arg = DeclareLaunchArgument(
        "wheel_separation_middle_back",
        default_value = "0.130362",
    )

    wheel_radius_middle_left_error_arg = DeclareLaunchArgument(
        "wheel_radius_middle_left_error",
        default_value="0.005",
    )

    wheel_radius_middle_right_error_arg = DeclareLaunchArgument(
        "wheel_radius_middle_right_error",
        default_value="0.005",
    )

    wheel_separation_error_arg = DeclareLaunchArgument(
        "wheel_separation_error",
        default_value="0.02",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    wheel_radius_front_left = LaunchConfiguration("wheel_radius_front_left")
    wheel_radius_middle_left = LaunchConfiguration("wheel_radius_middle_left")
    wheel_radius_rear_left = LaunchConfiguration("wheel_radius_rear_left")     
    wheel_radius_front_right = LaunchConfiguration("wheel_radius_front_right")
    wheel_radius_middle_right = LaunchConfiguration("wheel_radius_middle_right")
    wheel_radius_rear_right = LaunchConfiguration("wheel_radius_rear_right")
    wheel_separation = LaunchConfiguration("wheel_separation")
    wheel_separation_front_middle = LaunchConfiguration("wheel_separation_front_middle")
    wheel_separation_middle_back = LaunchConfiguration("wheel_separation_middle_back")

                                                   
                                                   
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager"
        ],
    )

    simple_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "simple_position_controller",
            "--controller-manager",
            "/controller_manager"
        ],
    )

    simple_velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "simple_velocity_controller",
            "--controller-manager",
            "/controller_manager"
        ],
    )

    delivery_box_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "delivery_box_controller",
            "--controller-manager",
            "/controller_manager"
        ],
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
                       "wheel_separation": wheel_separation,
                       "wheel_separation_front_middle": wheel_separation_front_middle ,
                       "wheel_separation_middle_back": wheel_separation_middle_back
                       }],
    )

    noisy_controller_launch = OpaqueFunction(function=noisy_controller)

    return LaunchDescription([
        use_sim_time_arg,
        wheel_radius_front_left_arg,
        wheel_radius_middle_left_arg,
        wheel_radius_rear_left_arg,
        wheel_radius_front_right_arg,
        wheel_radius_middle_right_arg,
        wheel_radius_rear_right_arg,
        wheel_separation_arg,
        wheel_separation_front_middle_arg,
        wheel_separation_middle_back_arg,
        wheel_radius_middle_left_error_arg,
        wheel_radius_middle_right_error_arg,
        wheel_separation_error_arg,
        joint_state_broadcaster_spawner,
        simple_position_controller_spawner,
        simple_velocity_controller_spawner,
        delivery_box_controller_spawner,
        motor_controller_py,
        noisy_controller_launch,


        
    ])