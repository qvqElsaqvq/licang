# 导入库
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.actions import TimerAction

def generate_launch_description():
    """launch内容描述函数，由ros2 launch 扫描调用"""

    if_rviz = True
    if_sim = False
    if_map = False  # 只启动雷达驱动、point-lio

    point_lio_launch = get_package_share_directory("point_lio_cxr")
    robot_bring_up_path = get_package_share_directory("robot_bring_up")
    init_transform_path = get_package_share_directory("map_odom_pub")
    obstacle_segmentation_path = get_package_share_directory("obstacle_segmentation")
    robot_description_path = get_package_share_directory("robot_description")
    livox_driver_path = get_package_share_directory("livox_ros_driver2")
    yaml_path = os.path.join(robot_bring_up_path, "config", "robot.yaml")
    nav2_bringup_dir = get_package_share_directory("nav2_bringup") #nav2_bringup功能包

    param_if_map = LaunchConfiguration("if_map", default=if_map)
    declare_if_map = DeclareLaunchArgument(
        "if_map",
        default_value=param_if_map,
        description="Whether to run map",
    )
    param_yaml_path = LaunchConfiguration("params_file", default=yaml_path)
    declare_yaml_path = DeclareLaunchArgument(
        "params_file",
        default_value=param_yaml_path,
        description="Full path to the configuration file to load",
    )
    param_launch_rviz = LaunchConfiguration("launch_rviz", default=if_rviz)
    declare_launch_rviz = DeclareLaunchArgument(
        "launch_rviz",
        default_value=param_launch_rviz,
        description="Whether to run rviz",
    )
    param_rviz_config_dir = LaunchConfiguration(
        "rviz_config_dir",
        default=os.path.join(nav2_bringup_dir, "rviz", "nav2_default_view.rviz"),
    )
    declare_rviz_config_dir = DeclareLaunchArgument(
        "rviz_config_dir",
        default_value=param_rviz_config_dir,
        description="Full path to the rviz config file to load",
    )
    param_launch_gazebo = LaunchConfiguration("launch_gazebo", default=if_sim)
    declare_launch_gazebo = DeclareLaunchArgument(
        "launch_gazebo",
        default_value=param_launch_gazebo,
        description="Whether to run gazebo",
    )
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [robot_description_path, "/launch", "/robot_display_rviz2.launch.py"]
        ),
        launch_arguments={
            "launch_rviz": param_launch_rviz,
            "launch_gazebo": param_launch_gazebo,
        }.items(),
    )
    livox_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [livox_driver_path, "/launch_ROS2", "/msg_MID360_launch.py"]
        ),
    )
    point_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [point_lio_launch, "/launch", "/pointlio.launch.py"]
        ),
        launch_arguments={
            "params_file": param_yaml_path,
            "launch_rviz": param_launch_rviz,
        }.items(),
    )
    init_transform_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [init_transform_path, "/launch", "/transform_pub.launch.py"]
        ),
        # launch_arguments={
        #     "param_dir": param_yaml_path,
        # }.items(),
        #condition=UnlessCondition(param_if_map),
    )
    obstacle_segmentation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [obstacle_segmentation_path, "/launch", "/obstacle_segmentation_autostart.launch.py"]
        ),
        launch_arguments={
            "params_file": param_yaml_path,
        }.items(),
        condition=UnlessCondition(param_if_map),
    )
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [robot_bring_up_path, "/launch", "/bringup_launch.py"]
        ),
        launch_arguments={
            "params_file": param_yaml_path,
            "use_sim_time": param_launch_gazebo,
        }.items(),
        condition=UnlessCondition(param_if_map),
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", param_rviz_config_dir],
        parameters=[{"use_sim_time": param_launch_gazebo}],
        output="screen",
        condition=IfCondition(param_launch_rviz),
    )

    # 创建LaunchDescription对象launch_description,用于描述launch文件
    list = [declare_launch_gazebo,
            declare_yaml_path,
            declare_rviz_config_dir,
            declare_launch_rviz,
            declare_if_map,
            livox_driver_launch,
            robot_description_launch,
            rviz_node,
            TimerAction(
                period=15.0,
                actions=[
                    point_lio_launch
                ],
            ),
            TimerAction(
                period=10.0,
                actions=[
                    navigation_launch,
                    obstacle_segmentation_launch
                    #init_transform_launch
                ],
            )
            ]


    # 返回让ROS2根据launch描述执行节点
    return LaunchDescription(list)
