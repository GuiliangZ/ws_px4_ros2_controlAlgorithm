
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_dir = get_package_share_directory('px4_offboard_mpc')
    # bash_script_path = os.path.join(package_dir, 'scripts', 'TerminatorScript.sh')
    return LaunchDescription([
        # ExecuteProcess(cmd=['bash', bash_script_path], output='screen'),
        Node(
            package='px4_offboard_mpc',
            namespace='px4_offboard_mpc',
            executable='offboard_safety_layer',
            name='offboard_safety_layer'
        ),
        Node(
            package='px4_offboard_mpc',
            namespace='px4_offboard_mpc',
            executable='processes',
            name='processes',
            prefix='gnome-terminal --'
        ),
        Node(
            package='px4_offboard_mpc',
            namespace='px4_offboard_mpc',
            executable='traj_control_scvx',
            name='traj_control_scvx',
        ),
        Node(
            package='px4_offboard_mpc',
            namespace='px4_offboard_mpc',
            executable='px4_attitude',
            name='px4_attitude'
        ),
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', [os.path.join(package_dir, 'visualize.rviz')]]
        )
    ])
