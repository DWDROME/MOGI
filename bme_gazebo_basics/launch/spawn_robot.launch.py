"""
文件名: [文件名]
描述: 定义 ROS 2 的 Launch 文件，用于加载 Gazebo 世界文件、显示 RViz 可视化以及运行相关节点。
    功能包括：
    - 加载 Gazebo 世界文件和 URDF 模型。
    - 启动 RViz 进行可视化。
    - 启动机器人状态发布器和模拟时间支持。
    - 启动 ros_gz_bridge 实现 ROS 2 和 Gazebo 消息桥接。
    - 启动轨迹服务器节点。
作者: ZiYang Chen
日期: 2025-01-03
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    定义 Launch 文件描述，用于加载和启动 Gazebo 世界文件、URDF 模型以及相关节点。
    """

    # 获取 bme_gazebo_basics 包的路径，用于定位共享资源
    pkg_bme_gazebo_basics = get_package_share_directory('bme_gazebo_basics')

    # 设置 GZ_SIM_RESOURCE_PATH 环境变量，确保 Gazebo 可以找到模型资源
    gazebo_models_path, _ = os.path.split(pkg_bme_gazebo_basics)
    os.environ["GZ_SIM_RESOURCE_PATH"] += os.pathsep + gazebo_models_path

    # 定义参数：是否启动 RViz
    rviz_launch_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Open RViz.'
    )

    # 定义参数：Gazebo 世界文件名称
    world_arg = DeclareLaunchArgument(
        'world', default_value='world.sdf',
        description='Name of the Gazebo world file to load'
    )

    # 定义参数：URDF 模型文件名称
    model_arg = DeclareLaunchArgument(
        'model', default_value='mogi_bot.urdf',
        description='Name of the URDF description to load'
    )

    # 定位 URDF 文件路径
    urdf_file_path = PathJoinSubstitution([
        pkg_bme_gazebo_basics,
        "urdf",
        LaunchConfiguration('model')
    ])

    # 加载 Gazebo 世界文件的 Launch 文件
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_bme_gazebo_basics, 'launch', 'world.launch.py'])
        ),
        launch_arguments={'world': LaunchConfiguration('world')}.items()
    )

    # 定义 RViz 节点
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_bme_gazebo_basics, 'rviz', 'rviz.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[{'use_sim_time': True}]
    )

    # 定义 Gazebo 中加载 URDF 模型的节点
    spawn_urdf_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "my_robot",
            "-topic", "robot_description",
            "-x", "0.0", "-y", "0.0", "-z", "0.5", "-Y", "0.0"
        ],
        output="screen",
        parameters=[{'use_sim_time': True}]
    )

    # 定义机器人状态发布器节点
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': Command(['xacro', ' ', urdf_file_path]),
            'use_sim_time': True}
        ],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
    )

    # 定义 ROS-Gazebo 消息桥接节点
    gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry",
            "/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model",
            "/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V"
        ],
        output="screen",
        parameters=[
            {'use_sim_time': True},
        ]
    )

    # 定义轨迹服务器节点
    trajectory_node = Node(
        package='mogi_trajectory_server',
        executable='mogi_trajectory_server',
        name='mogi_trajectory_server',
    )

    # 创建并返回 LaunchDescription
    launch_description = LaunchDescription()

    # 添加所有定义的参数和节点
    launch_description.add_action(rviz_launch_arg)
    launch_description.add_action(world_arg)
    launch_description.add_action(model_arg)
    launch_description.add_action(world_launch)
    launch_description.add_action(rviz_node)
    launch_description.add_action(spawn_urdf_node)
    launch_description.add_action(robot_state_publisher_node)
    launch_description.add_action(gz_bridge_node)
    launch_description.add_action(trajectory_node)
    
    return launch_description
