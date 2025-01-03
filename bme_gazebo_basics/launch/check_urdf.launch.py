"""
文件名: check_urdf.launch.py
描述: 配置 ROS 2 的 Launch 文件，用于加载和显示 URDF 模型。
    - 提供 RViz 配置文件路径。
    - 加载指定的 URDF 模型。
    - 支持通过参数启用或禁用关节状态发布器的 GUI。
作者: ZiYang Chen
日期: 2025-01-03
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    定义 Launch 文件内容，加载 URDF 模型并启动相关工具。
    """
    
    # 获取 bme_gazebo_basics 包的路径，用于定位共享资源
    pkg_bme_gazebo_basics = FindPackageShare('bme_gazebo_basics')
    
    # 设置默认的 RViz 配置文件路径
    default_rviz_config_path = PathJoinSubstitution([pkg_bme_gazebo_basics, 'rviz', 'urdf.rviz'])

    # 定义参数：控制关节状态发布器的 GUI
    gui_arg = DeclareLaunchArgument(
        name='gui', 
        default_value='true', 
        choices=['true', 'false'],
        description='Enable or disable the joint_state_publisher_gui'
    )
    
    # 定义参数：RViz 配置文件路径
    rviz_arg = DeclareLaunchArgument(
        name='rvizconfig', 
        default_value=default_rviz_config_path,
        description='Path to the RViz configuration file'
    )

    # 定义参数：URDF 模型文件名
    model_arg = DeclareLaunchArgument(
        'model', 
        default_value='mogi_bot.urdf',
        description='Name of the URDF model file to load'
    )

    # 加载 URDF 模型，并通过 urdf_launch 包的 display.launch.py 启动
    urdf = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'display.launch.py']),
        launch_arguments={
            'urdf_package': 'bme_gazebo_basics',  # 模型所在包
            'urdf_package_path': PathJoinSubstitution(['urdf', LaunchConfiguration('model')]),  # 模型文件路径
            'rviz_config': LaunchConfiguration('rvizconfig'),  # RViz 配置路径
            'jsp_gui': LaunchConfiguration('gui')  # GUI 开关
        }.items()
    )

    # 返回包含所有参数和加载动作的 LaunchDescription
    launch_description = LaunchDescription([
        gui_arg,
        rviz_arg,
        model_arg,
        urdf
    ])

    return launch_description
