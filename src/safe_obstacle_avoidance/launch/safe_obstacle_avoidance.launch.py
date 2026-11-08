from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")
    start_simulation = LaunchConfiguration("start_simulation")
    auto_demo = LaunchConfiguration("auto_demo")
    goal_x = LaunchConfiguration("goal_x")
    goal_y = LaunchConfiguration("goal_y")
    auto_duration_s = LaunchConfiguration("auto_duration_s")
    params_file = LaunchConfiguration("params_file")

    share = FindPackageShare("safe_obstacle_avoidance")
    world_file = PathJoinSubstitution([share, "worlds", "ellipsoid_obstacle_world.sdf"])
    bridge_file = PathJoinSubstitution([share, "config", "ros_gz_bridge.yaml"])
    default_params = PathJoinSubstitution([share, "config", "params.yaml"])
    rviz_config = PathJoinSubstitution([share, "config", "cbf_visualization.rviz"])

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
        condition=run_gui, output="screen", emulate_tty=True,
    )
    gazebo_headless = ExecuteProcess(
        cmd=["gz", "sim", "-s", "-r", world_file],
        condition=run_headless, output="screen", emulate_tty=True,
    )
    bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge",
        name="ros_gz_bridge", output="screen",
        parameters=[{"config_file": bridge_file}],
    )
    generator = Node(
        package="safe_obstacle_avoidance", executable="safe_set_generator",
        name="safe_set_generator", output="screen",
        parameters=[params_file, {"use_sim_time": True}],
    )
    controller = Node(
        package="safe_obstacle_avoidance", executable="cbf_controller",
        name="cbf_controller", output="screen",
        parameters=[params_file, {"use_sim_time": True}],
    )
    reactive_navigator = Node(
        package="safe_obstacle_avoidance", executable="reactive_goal_navigator",
        name="reactive_goal_navigator", output="screen",
        parameters=[{"use_sim_time": True,
                     "goal_x": ParameterValue(goal_x, value_type=float),
                     "goal_y": ParameterValue(goal_y, value_type=float)}],
        condition=run_auto_demo,
    )
    reference_publisher = Node(
        package="safe_obstacle_avoidance", executable="reference_cmd_publisher",
        name="demo_reference_cmd_publisher", output="screen",
        parameters=[{
            "use_sim_time": True,
            "linear_speed": 0.15,
            "angular_speed": 0.0,
            "duration_s": ParameterValue(auto_duration_s, value_type=float),
            "max_path_length": 5.0,
        }],
        condition=IfCondition("false"),  # retained only for manual backwards-compatibility
    )
    visualizer = Node(
        package="safe_obstacle_avoidance", executable="cbf_visualization",
        name="cbf_visualization", output="screen",
        parameters=[{"use_sim_time": True}],
    )
    rviz_process = ExecuteProcess(
        cmd=["rviz2", "-d", rviz_config, "--ros-args", "-p", "use_sim_time:=true"],
        condition=IfCondition(rviz), output="screen", emulate_tty=True,
    )

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true",
                              description="Run Gazebo GUI; false starts server-only"),
        DeclareLaunchArgument("rviz", default_value="true",
                              description="Launch RViz2 visualization window"),
        DeclareLaunchArgument("start_simulation", default_value="true",
                              description="Launch the bundled Gazebo world"),
        DeclareLaunchArgument("auto_demo", default_value="true",
                              description="Publish a bounded forward reference command"),
        DeclareLaunchArgument("goal_x", default_value="5.5", description="Demo goal x in odom frame"),
        DeclareLaunchArgument("goal_y", default_value="0.0", description="Demo goal y in odom frame"),
        DeclareLaunchArgument("auto_duration_s", default_value="30.0",
                              description="Maximum simulated seconds for the automatic command"),
        DeclareLaunchArgument("params_file", default_value=default_params,
                              description="ROS 2 parameter YAML"),
        gazebo_gui,
        gazebo_headless,
        TimerAction(period=3.0, actions=[
            bridge, generator, controller, reactive_navigator, visualizer
        ]),
        TimerAction(period=5.0, actions=[rviz_process]),
    ])
