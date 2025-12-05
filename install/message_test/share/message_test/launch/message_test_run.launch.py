from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    此函数定义了要启动的一系列节点。
    """
    return LaunchDescription([
        
        # 1. 启动 exe_node
        Node(
            package='message_test',         # 你的功能包名
            executable='exe_node',          # entry_points 中定义的第一个可执行名称
            name='exe_processor_node',      # 可选：给节点一个实例名
            output='screen',                # 将节点的输出打印到控制台
            emulate_tty=True                # 确保在容器/远程环境中正确显示日志
        ),

        # 2. 启动 pub_pos_base_link_node
        Node(
            package='message_test',
            executable='pub_pos_base_link_node', # entry_points 中定义的第二个可执行名称
            name='base_link_publisher',
            output='screen',
            emulate_tty=True
        )
    ])