from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='openwrt_ros2_diagnoster',
            namespace='openwrt_ros2_diagnoster',
            executable='router_diagnoster',
            name='router_diagnoster',
            parameters=[
                {
                    'router_host': '192.168.1.1',
                    'router_username': 'root',
                    'router_password': '123',
                    'cpu_critical_level': .5,
                    'free_memory_critical_level': 5.0,
                    'reliability_critical_level': 20.0,
                },
            ],
        ),
        # Node(
        #     package='openwrt_ros2_diagnoster',
        #     namespace='lallaololo',
        #     executable='router_diagnoster',
        #     name='xz_kto'
        # ),
    ])