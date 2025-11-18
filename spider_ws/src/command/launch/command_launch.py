from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    cmd_node = Node(
        package='command',
        executable='command_publisher',
        name='command_publisher_node',
        output='screen'
    )
    ui_node = Node(
        package='command',
        executable='ui_controller',
        name='ui_controller_node',
        output='screen'
    )
    return LaunchDescription([
        cmd_node, 
        ui_node,
        RegisterEventHandler(
            OnProcessExit(
                target_action=cmd_node,
                on_exit=[Shutdown()]
            )
        ),
        # ExecuteProcess(
        #     cmd=['ros2', 'bag', 'record', '-a', '-o', 'session1'],
        #     output='screen'
        # )
    ])
