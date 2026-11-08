from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gui = LaunchConfiguration("gui")
    start_simulation = LaunchConfiguration("start_simulation")
    auto_demo = LaunchConfiguration("auto_demo")
    auto_duration_s = LaunchConfiguration("auto_duration_s")
    params_file = LaunchConfiguration("params_file")
    rviz = LaunchConfiguration("rviz")
    world_file = PathJoinSubstitution([
        FindPackageShare("safe_obstacle_avoidance"), "worlds", "ellipsoid_obstacle_world.sdf"
    ])
    bridge_file = PathJoinSubstitution([
        FindPackageShare("safe_obstacle_avoidance"), "config", "ros_gz_bridge.yaml"
    ])
    default_params = PathJoinSubstitution([
        FindPackageShare("safe_obstacle_avoidance"), "config", "params.yaml"
    ])

    run_gui = IfCondition(PythonExpression([
        "'", start_simulation, "' == 'true' and '", gui, "' == 'true'"
    ]))
    run_headless = IfCondition(PythonExpression([
        "'", start_simulation, "' == 'true' and '", gui, "' == 'false'"
    ]))

    run_auto_demo = IfCondition(PythonExpression([
        "'", start_simulation, "' == 'true' and '", auto_demo, "' == 'true'"
    ]))

    gazebo_gui = ExecuteProcess(
        cmd=["gz", "sim", "-r", world_file],
        condition=run_gui,
        output="screen",
        emulate_tty=True,
    )
    gazebo_headless = ExecuteProcess(
        cmd=["gz", "sim", "-s", "-r", world_file],
        condition=run_headless,
        output="screen",
        emulate_tty=True,
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        output="screen",
        parameters=[{"config_file": bridge_file}],
    )
    generator = Node(
        package="safe_obstacle_avoidance",
        executable="safe_set_generator",
        name="safe_set_generator",
        output="screen",
        parameters=[params_file, {"use_sim_time": True}],
    )
    controller = Node(
        package="safe_obstacle_avoidance",
        executable="cbf_controller",
        name="cbf_controller",
        output="screen",
        parameters=[params_file, {"use_sim_time": True}],
    )

    visualizer = Node(
        package="safe_obstacle_avoidance",
        executable="cbf_visualization",
        name="cbf_visualization",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )
    rviz_process = ExecuteProcess(
        cmd=["rviz2", "-d", rviz_config],
        condition=IfCondition(rviz),
        output="screen",
        emulate_tty=True,
    )
    reference_publisher = Node(
        package="safe_obstacle_avoidance",
        executable="reference_cmd_publisher",
        name="demo_reference_cmd_publisher",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "linear_speed": 0.15,
            "angular_speed": 0.0,
            "duration_s": ParameterValue(auto_duration_s, value_type=float),
            "max_path_length": 5.0,
        }],
        condition=run_auto_demo,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "gui", default_value="true", description="Run Gazebo GUI; false starts server-only"
        ),
        DeclareLaunchArgument(
            "start_simulation", default_value="true", description="Launch the bundled Gazebo world"
        ),
        DeclareLaunchArgument(
            "auto_demo", default_value="true",
            description="Publish a bounded forward reference command in the bundled simulation"
        ),
        DeclareLaunchArgument(
            "auto_duration_s", default_value="30.0",
            description="Maximum simulated seconds for the automatic reference command"
        ),
        DeclareLaunchArgument(
            "params_file", default_value=default_params, description="ROS 2 parameter YAML"
        ),
        gazebo_gui,
        gazebo_headless,
        TimerAction(period=3.0, actions=[bridge, generator, controller, reference_publisher, visualizer]),\n        TimerAction(period=5.0, actions=[rviz_process]),
    ])
