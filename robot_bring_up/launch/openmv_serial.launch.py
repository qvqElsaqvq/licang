# 虽然只有一个节点，也没有参数，但是为了程序寄了之后自动重启，还是需要一个launch文件
# 导入库
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """launch内容描述函数，由ros2 launch 扫描调用"""
    
    node_01 = Node(
        package="robot_serial",
        executable="openmv_serial_node",
        output="screen",
        name="openmv_serial_node",
        respawn=True # 重启
    )
    # 创建LaunchDescription对象launch_description,用于描述launch文件
    launch_description = LaunchDescription(
        [node_01]
    )
    # 返回让ROS2根据launch描述执行节点
    return launch_description
