from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    cmd_node = Node(
        package='command',
        executable='command_publisher',
        name='command_publisher_node',
        output='screen',
        )
    imu_node = Node(
        package='command',
        executable='imu_reader',
        name='imu_node_reader',
        output= 'screen'
    )
    nn_node = Node(
        package='command',
        executable='nn_controller',
        name='nn_controller_node',
        output= 'screen'
    )
    return LaunchDescription([
        cmd_node, 
        imu_node,
        nn_node,
        RegisterEventHandler(
            OnProcessExit(
                target_action=cmd_node,
                on_exit=[Shutdown()]
            )
        )
    ])
