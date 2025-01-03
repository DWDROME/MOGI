"""
文件名: gazebo_world.launch.py
描述: 配置 ROS 2 的 Launch 文件，用于启动 Gazebo 仿真环境。
    - 提供 Gazebo 世界文件的路径和名称。
    - 添加自定义 Gazebo 模型路径。
    - 启动 Gazebo 仿真并加载指定的世界文件。
作者: ZiYang Chen
日期: 2025-01-03
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution


def generate_launch_description():
    """
    定义 Launch 文件描述，用于加载 Gazebo 世界文件并启动仿真环境。
    """
    # 定义参数：Gazebo 世界文件的名称
    world_arg = DeclareLaunchArgument(
        'world', default_value='world.sdf',
        description='Name of the Gazebo world file to load'
    )

    # 获取 bme_gazebo_basics 和 ros_gz_sim 包的共享目录路径
    pkg_bme_gazebo_basics = get_package_share_directory('bme_gazebo_basics')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # 自定义 Gazebo 模型的路径
    gazebo_models_path = "/home/david/gazebo_models"

    # 更新 GZ_SIM_RESOURCE_PATH 环境变量，确保 Gazebo 可以找到自定义模型
    if "GZ_SIM_RESOURCE_PATH" in os.environ:
        os.environ["GZ_SIM_RESOURCE_PATH"] += os.pathsep + gazebo_models_path
    else:
        os.environ["GZ_SIM_RESOURCE_PATH"] = gazebo_models_path

    # 包含 Gazebo 仿真启动文件，设置启动参数
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'),
        ),
        launch_arguments={
            'gz_args': [
                PathJoinSubstitution([
                    pkg_bme_gazebo_basics,
                    'worlds',
                    LaunchConfiguration('world')
                ]),
                TextSubstitution(text=' -r -v -v1')  # 仿真启动参数
            ],
            'on_exit_shutdown': 'true'  # 仿真退出时关闭进程
        }.items()
    )

    # 创建 LaunchDescription 对象并添加参数和动作
    launch_description = LaunchDescription()
    launch_description.add_action(world_arg)
    launch_description.add_action(gazebo_launch)

    return launch_description
