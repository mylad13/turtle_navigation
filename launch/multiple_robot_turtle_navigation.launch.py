import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def generate_launch_description() -> None:
    # Get the launch and rviz directories
    turtle_nav2_bringup_dir = get_package_share_directory("turtle_navigation")

    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    nav2_bringup_launch_dir = os.path.join(nav2_bringup_dir, "launch")

    rviz_config_dir = os.path.join(turtle_nav2_bringup_dir, "rviz2", "rviz_navigation_namespaced.rviz")


    # Common settings
    use_sim_time = LaunchConfiguration("use_sim_time", default="True")
    map_yaml_file = LaunchConfiguration("map")
    default_bt_xml_filename = LaunchConfiguration("default_bt_xml_filename")
    autostart = LaunchConfiguration("autostart")
    rviz_config_file = LaunchConfiguration("rviz_config")
    use_rviz = LaunchConfiguration("use_rviz")
    log_settings = LaunchConfiguration("log_settings", default="true")

    declare_robot1_name_cmd = DeclareLaunchArgument(
        "robot1_name",
        default_value="explorer1",
        description="Name for robot1"
    )
    declare_robot2_name_cmd = DeclareLaunchArgument(
        "robot2_name",
        default_value="explorer2",
        description="Name for robot2"
    )
    declare_robot3_name_cmd = DeclareLaunchArgument(
        "robot3_name",
        default_value="rescuer1",
        description="Name for robot3"
    )
    # Names and poses of the robots
    robots = [
        {"name": LaunchConfiguration("robot1_name")},
        {"name": LaunchConfiguration("robot2_name")},
        {"name": LaunchConfiguration("robot3_name")},]

    declare_robot_params_file_cmd = DeclareLaunchArgument(
        "robot_params_file",
        default_value=os.path.join(
            turtle_nav2_bringup_dir, "params", "nav2_multirobot_params.yaml"
        ),
        description="Full path to the ROS2 parameters file to use for robot launched nodes",
    )

    declare_bt_xml_cmd = DeclareLaunchArgument(
        "default_bt_xml_filename",
        default_value=os.path.join(
            get_package_share_directory("nav2_bt_navigator"), "behavior_trees", "navigate_w_replanning_and_recovery.xml"
        ),
        description="Full path to the behavior tree xml file to use",
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart", default_value="True", description="Automatically startup the stacks"
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        "rviz_config", default_value=rviz_config_dir, description="Full path to the RVIZ config file to use."
    )

    declare_use_rviz_cmd = DeclareLaunchArgument("use_rviz", default_value="True", description="Whether to start RVIZ")

    # Define commands for launching the navigation instances
    nav_instances_cmds = []
    for robot in robots:
        robot_name = robot["name"]
        params_file = LaunchConfiguration("robot_params_file")

        param_substitutions = {
            "voxel_layer.scan.topic": [robot_name, "/scan"],
            "global_costmap.obstacle_layer.scan.topic": [robot_name, "/scan"]
        }

        configured_params = RewrittenYaml(
            source_file=params_file,
            param_rewrites=param_substitutions,
            convert_types=True)
        
        group = GroupAction(
            [
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(nav2_bringup_launch_dir, "rviz_launch.py")),
                    condition=IfCondition(use_rviz),
                    launch_arguments={
                        "namespace": robot_name,
                        "use_namespace": "True",
                        "rviz_config": rviz_config_file,
                    }.items(),
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(turtle_nav2_bringup_dir, "launch", "turtle_navigation_individual.launch.py")
                    ),
                    launch_arguments={
                        "namespace": robot_name,
                        "use_namespace": "True",
                        "map": [robot_name, "/map"],
                        "use_sim_time": use_sim_time,
                        "params_file": configured_params,
                        "default_bt_xml_filename": default_bt_xml_filename,
                        "autostart": autostart,
                        "use_rviz": "False",
                        "use_simulator": "False",
                        "headless": "False",
                    }.items(),
                ),

                LogInfo(condition=IfCondition(log_settings), msg=["Launching ", robot_name]),
                LogInfo(condition=IfCondition(log_settings), msg=[robot_name, " map yaml: ", map_yaml_file]),
                LogInfo(condition=IfCondition(log_settings), msg=[robot_name, " params yaml: ", configured_params]),
                LogInfo(condition=IfCondition(log_settings), msg=[robot_name, " behavior tree xml: ", default_bt_xml_filename]),
                LogInfo(condition=IfCondition(log_settings), msg=[robot_name, " rviz config file: ", rviz_config_file]),
                LogInfo(condition=IfCondition(log_settings), msg=[robot_name, " autostart: ", autostart]),
            ]
        )

        nav_instances_cmds.append(group)

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_robot_params_file_cmd)
    ld.add_action(declare_bt_xml_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_robot1_name_cmd)
    ld.add_action(declare_robot2_name_cmd)
    ld.add_action(declare_robot3_name_cmd)

    for simulation_instance_cmd in nav_instances_cmds:
        ld.add_action(simulation_instance_cmd)

    return ld
