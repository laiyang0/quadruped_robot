import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import TimerAction
def print_red(text):
    print(f"\033[31m{text}\033[0m")
# 运行阶段
def launch_setup(context, *args, **kwargs):
    package_description = context.launch_configurations['pkg_description']  #获取package_description
    init_height = context.launch_configurations['height']

    pkg_path = os.path.join(get_package_share_directory(package_description))
    controllers_yaml_path = os.path.join(get_package_share_directory(package_description),'config','robot_control.yaml')

    xacro_file = os.path.join(pkg_path, 'xacro', 'robot.xacro')
    urdf_path=xacro.process_file(xacro_file, mappings={'DEBUG': 'false','GAZEBO': 'true'}).toxml()

    world_path="/home/laiyang/quadruped_robot/world/empty.sdf"
    
    print_red(f"Package description: {package_description}")
    print_red(f"Init height: {init_height}")
    print_red(f"pkg_path: {pkg_path}")
    print_red(f"controllers_yaml_path:{controllers_yaml_path}")
    print_red(f"world_path:{world_path}")
    rviz_config_file = os.path.join(get_package_share_directory(package_description), "config", "visualize_urdf.rviz")

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_ocs2',
        output='screen',
        arguments=["-d", rviz_config_file],
        parameters=[{'use_sim_time': True}]
    )

    #发布机器人的状态“/robot_description”
    robot_state_publisher = Node(    
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {
                'publish_frequency': 20.0,
                'use_tf_static': True,
                'robot_description': urdf_path ,
                'ignore_timestamp': True
            }
        ],
    )
    # 启动现代 Gazebo
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': f'-r {world_path}'}.items()  # -r 表示启动后立即运行
    )
    #gazebo_harmonic通过接收robot_state_publisher发布的"/robot_description"生成模型
    # 相当于在这里生成了仿真环境下的机器人模型
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-name','quadruped_robot',
                    '-topic', 'robot_description',
                    # '-file', urdf_path, # 你的 URDF 绝对路径
                   'robot', '-allow_renaming', 'true', '-x',"0.0" ,'-y',"0.0",'-z', '0.4'], #设置机器人的初始坐标
    )
    ros_gz_bridge=Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                    '/model/quadruped_robot/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose',],
                        # '/model/quadruped_robot/link/base/pose@geometry_msgs/msg/PoseStamped@gz.msgs.Pose'],
            output='screen'
        )

    joint_state_publisher = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager"],
    )

    imu_sensor_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["imu_sensor_broadcaster",
                   "--controller-manager", "/controller_manager"],
    )
    #这个是用于实机控制使用
    # control_node = Node(
    #     package='controller_manager',
    #     executable='ros2_control_node',
    #     parameters=[
    #         {'robot_description': urdf_path},  # URDF 内容
    #         controllers_yaml_path,             # ← YAML 在这里被传入
    #     ],
    #     output='both',
    # )
    #该节点向指定的controller_manager发送请求，加载并激活quadruped_robot_controller控制器
    quadruped_robot_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["quadruped_robot_controller", "--controller-manager", "/controller_manager"],
    )
    #启动控制器
    spawn_joint_state = RegisterEventHandler(
        OnProcessExit(
            target_action=gz_spawn_entity,
            on_exit=[joint_state_publisher],
        )
    )

    spawn_imu = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_publisher,
            on_exit=[imu_sensor_broadcaster],
        )
    )

    spawn_quadruped = RegisterEventHandler(
        OnProcessExit(
            target_action=imu_sensor_broadcaster,
            on_exit=[quadruped_robot_controller],
        )
    )
    # spawn_controllers = RegisterEventHandler(   
    #     OnProcessExit(
    #         target_action=gz_spawn_entity,
    #         on_exit=[
    #             joint_state_publisher ,
    #             imu_sensor_broadcaster,
    #             quadruped_robot_controller
    #         ],
    #     )
    # )
    delayed_gazebo = TimerAction(
        period=1.0,
        actions=[gz_sim, gz_spawn_entity]
        )
    return [
        rviz2,
        ros_gz_bridge,
        robot_state_publisher,
            delayed_gazebo,
            # gz_sim,
            # gz_spawn_entity, 
            # spawn_controllers
            spawn_joint_state,
            spawn_imu,
            spawn_quadruped,
            ]


#在使用ros2 launch时，launch系统会调用该文件的generate_launch_description函数
# 解析阶段
def generate_launch_description():
    #这里声明的参数可以在启动的时候通过命令行进行修改，比如：ros2 launch quadruped_robot_controller gazebo_sim.launch.py pkg_description:=laiyang height:=1.0
    #将pkg_description参数的默认值go2_description修改为laiyang，将height参数的默认值0.5修改为1.0
    pkg_description = DeclareLaunchArgument(         
        'pkg_description',
        default_value='go2_description',
        description='package for robot description'
    )

    height = DeclareLaunchArgument(
        'height',
        default_value='0.5',
        description='Init height in simulation'
    )

    #OpaqueFunction：延迟执行的钩子，可以用来在启动时执行任意 Python 函数，这里用于启动launch_setup
    #LaunchDescription 交给 launch 系统执行
    return LaunchDescription([
        pkg_description,
        height,
        OpaqueFunction(function=launch_setup),
    ])