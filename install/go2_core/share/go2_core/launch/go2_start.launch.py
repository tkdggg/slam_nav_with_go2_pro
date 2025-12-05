from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
import os

def generate_launch_description():

    #获取各功能包
    go2_driver_pkg = get_package_share_directory("go2_driver")
    go2_core_pkg = get_package_share_directory("go2_core")
    go2_slam_pkg = get_package_share_directory("go2_slam")
    go2_perception_pkg = get_package_share_directory("go2_perception")
    go2_navigation2_pkg = get_package_share_directory("go2_navigation2")

    # 添加启动开关
    use_slamtoolbox = DeclareLaunchArgument(
        name="use_slamtoolbox",
        default_value="true"
    )

    use_nav2 = DeclareLaunchArgument(
        name="use_nav2",
        default_value="true"
    )

    use_rviz = DeclareLaunchArgument(
        name="use_rviz",
        default_value="false"
    )

    use_message = DeclareLaunchArgument(
        name="use_message",
        default_value="true"
    )

    # 里程计融合imu
    go2_robot_localization = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(go2_core_pkg, "launch", "go2_robot_localization.launch.py")
            )
        )

    # 导航
    go2_nav2_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(go2_navigation2_pkg, "launch", "go2_nav2.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration('use_nav2'))
        )
    
    # # Message Test 发布机器人baselink pose和 接收目标pose
    message_test_pkg = get_package_share_directory('message_test')
    message_test_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(message_test_pkg, 'launch', 'message_test_run.launch.py' )
        ),
        condition=IfCondition(LaunchConfiguration('use_message'))
    )

    # 启动驱动包
    go2_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(go2_driver_pkg, "launch", "driver.launch.py")
        )   
    )

    # 点云处理
    go2_pointcloud_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(go2_perception_pkg, "launch", "go2_pointcloud.launch.py")
            )
        )

    # slam-toolbox 配置
    go2_slamtoolbox_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(go2_slam_pkg, "launch", "go2_slamtoolbox.launch.py")  # go2_slamtoolbox.launch.py :建图模式   go2_slamtoolbox_loco.launch.py :定位模式（加载已有地图）
            ),
            condition=IfCondition(LaunchConfiguration('use_slamtoolbox'))
        )

    # 包含rviz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(go2_core_pkg, "rviz2", "display.rviz")],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    return LaunchDescription([
        go2_driver_launch,
        use_nav2,  # LaunchArgument（参数声明）必须在使用它的节点/launch之前
        go2_nav2_launch,
        use_rviz,
        rviz_node,
        go2_robot_localization,
        go2_pointcloud_launch,
        use_slamtoolbox,
        go2_slamtoolbox_launch,
        use_message,
        message_test_launch,
        
    ])
