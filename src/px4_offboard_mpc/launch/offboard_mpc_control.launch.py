
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
            executable='px4_offboard_safeLayer',
            name='px4_offboard_safeLayer'
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
            executable='px4_offboard_scvx',
            name='px4_offboard_scvx',
        ),
        Node(
            package='px4_offboard_mpc',
            namespace='px4_offboard_mpc',
            executable='px4_attitude_plot',
            name='px4_attitude_plot'
        ),
        Node(
            package='px4_offboard_mpc',
            namespace='px4_offboard_mpc',
            executable='scvx_class',
            name='scvx_class'
        ),
        Node(
            package='px4_offboard_mpc',
            namespace='px4_offboard_mpc',
            executable='px4_attitude_plot',
            name='px4_attitude_plot'
        ),
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', [os.path.join(package_dir, 'visualize.rviz')]]
        )
    ])
