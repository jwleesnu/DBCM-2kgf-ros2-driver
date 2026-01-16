from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 이 값을 필요에 맞게 직접 수정하세요.
    port = '/dev/ttyUSB0'
    baud = 115200

    node = Node(
        package='load_cell_publisher',
        executable='load_cell_publisher',
        name='load_cell_publisher',
        output='screen',
        parameters=[{
            'port': port,
            'baud': baud,
        }],
    )

    return LaunchDescription([node])

