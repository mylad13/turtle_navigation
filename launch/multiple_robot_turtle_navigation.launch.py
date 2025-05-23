import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution, PythonExpression
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
    map_dir = LaunchConfiguration("map_dir")
    default_bt_xml_filename = LaunchConfiguration("default_bt_xml_filename")
    autostart = LaunchConfiguration("autostart")
    rviz_config_file = LaunchConfiguration("rviz_config")
    use_rviz = LaunchConfiguration("use_rviz")
    log_settings = LaunchConfiguration("log_settings", default="true")

    declare_map_dir_cmd = DeclareLaunchArgument(
        "map_dir",
        default_value=os.path.join(turtle_nav2_bringup_dir, 'maps'),
        description="Directory where map files are stored"
    )

    declare_robot1_name_cmd = DeclareLaunchArgument(
        "robot1_name",
        default_value="explorer1",
        description="Name for robot1"
    )
    declare_robot1_type_cmd = DeclareLaunchArgument(
        "robot1_type",
        default_value="explorer",
        description="Type for robot1"
    )
    declare_robot2_name_cmd = DeclareLaunchArgument(
        "robot2_name",
        default_value="explorer2",
        description="Name for robot2"
    )
    declare_robot2_type_cmd = DeclareLaunchArgument(
        "robot2_type",
        default_value="explorer",
        description="Type for robot2"
    )
    declare_robot3_name_cmd = DeclareLaunchArgument(
        "robot3_name",
        default_value="rescuer1",
        description="Name for robot3"
    )
    declare_robot3_type_cmd = DeclareLaunchArgument(
        "robot3_type",
        default_value="rescuer",
        description="Type for robot3"
    )
    # Names and poses of the robots
    robots = [
        {"name": LaunchConfiguration("robot1_name"), "type": LaunchConfiguration("robot1_type")},
        {"name": LaunchConfiguration("robot2_name"), "type": LaunchConfiguration("robot2_type")},
        {"name": LaunchConfiguration("robot3_name"), "type": LaunchConfiguration("robot3_type")}
        ]

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
        robot_type = robot["type"]
        params_file = LaunchConfiguration("robot_params_file")
        scan_topic = PathJoinSubstitution([
            TextSubstitution(text='/'),
            robot_name,
            TextSubstitution(text='scan')
        ])
        # --- Define parameter rewrites for EXPLORER type ---
        explorer_param_rewrites = {
            "local_costmap.local_costmap.ros__parameters.voxel_layer.scan.topic": scan_topic,
            "global_costmap.global_costmap.ros__parameters.obstacle_layer.scan.topic": scan_topic,
            "controller_server.ros__parameters.FollowPath.max_vel_x": "1.5",
            "controller_server.ros__parameters.FollowPath.max_speed_xy": "1.5"
        }
        explorer_configured_params = RewrittenYaml(
            source_file=params_file,
            param_rewrites=explorer_param_rewrites,
            convert_types=True
        )

        # --- Define parameter rewrites for RESCUER type ---
        # Adjust these paths based on your actual nav2_multirobot_params.yaml structure
        rescuer_param_rewrites = {
            "local_costmap.local_costmap.ros__parameters.voxel_layer.scan.topic": scan_topic,
            "global_costmap.global_costmap.ros__parameters.obstacle_layer.scan.topic": scan_topic,
            "controller_server.ros__parameters.FollowPath.max_vel_x": "0.25",
            "controller_server.ros__parameters.FollowPath.max_speed_xy": "0.25"
        }
        rescuer_configured_params = RewrittenYaml(
            source_file=params_file,
            param_rewrites=rescuer_param_rewrites,
            convert_types=True
        )
        
        # --- Common launch arguments for the included file ---
        map_yaml_file = PathJoinSubstitution([map_dir, robot_name, "map.yaml"])
        common_launch_args = {
            "namespace": robot_name,
            "use_namespace": "True",
            "map": map_yaml_file,
            "use_sim_time": use_sim_time,
            "default_bt_xml_filename": default_bt_xml_filename,
            "autostart": autostart,
            "use_rviz": "False",
            "use_simulator": "False",
            "headless": "False",
        }

        group = GroupAction(
            [
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(nav2_bringup_launch_dir, "rviz_launch.py")),
                    condition=IfCondition(use_rviz),
                    launch_arguments={
                        "namespace": robot_name,
                        "use_namespace": "True",
                        "rviz_config": rviz_config_file,
                        "use_sim_time": use_sim_time,
                    }.items(),
                ),
                # --- Include navigation stack IF type is 'explorer' ---
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(turtle_nav2_bringup_dir, "launch", "turtle_navigation_individual.launch.py")
                    ),
                    condition=IfCondition(PythonExpression(["'", robot_type, "' == 'explorer'"])),
                    launch_arguments={**common_launch_args, 'params_file': explorer_configured_params}.items()
                ),

                # --- Include navigation stack IF type is 'rescuer' ---
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(turtle_nav2_bringup_dir, "launch", "turtle_navigation_individual.launch.py")
                    ),
                    # Condition checks if the resolved robot_type equals 'rescuer'
                    condition=IfCondition(PythonExpression(["'", robot_type, "' == 'rescuer'"])),
                    # Pass common args + the rescuer-specific params
                    launch_arguments={**common_launch_args, 'params_file': rescuer_configured_params}.items()
                ),

                LogInfo(condition=IfCondition(log_settings), msg=["Launching ", robot_name]),
                LogInfo(condition=IfCondition(log_settings), msg=[robot_name, " map yaml: ", map_yaml_file]),
                LogInfo(condition=IfCondition(log_settings), msg=[robot_name, " behavior tree xml: ", default_bt_xml_filename]),
                LogInfo(condition=IfCondition(log_settings), msg=[robot_name, " rviz config file: ", rviz_config_file]),
                LogInfo(condition=IfCondition(log_settings), msg=[robot_name, " autostart: ", autostart]),
            ]
        )

        nav_instances_cmds.append(group)

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_map_dir_cmd)
    ld.add_action(declare_robot_params_file_cmd)
    ld.add_action(declare_bt_xml_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_robot1_name_cmd)
    ld.add_action(declare_robot1_type_cmd)
    ld.add_action(declare_robot2_name_cmd)
    ld.add_action(declare_robot2_type_cmd)
    ld.add_action(declare_robot3_name_cmd)
    ld.add_action(declare_robot3_type_cmd)

    for simulation_instance_cmd in nav_instances_cmds:
        ld.add_action(simulation_instance_cmd)

    return ld
