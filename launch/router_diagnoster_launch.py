from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='openwrt_ros2_diagnoster',
            namespace='openwrt_ros2_diagnoster',
            executable='router_diagnoster',
            name='router_diagnoster'
        ),
        # Node(
        #     package='turtlesim',
        #     namespace='turtlesim2',
        #     executable='turtlesim_node',
        #     name='sim'
        # ),
        # Node(
        #     package='turtlesim',
        #     executable='mimic',
        #     name='mimic',
        #     remappings=[
        #         ('/input/pose', '/turtlesim1/turtle1/pose'),
        #         ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
        #     ]
        # )
    ])