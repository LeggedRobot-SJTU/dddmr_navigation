import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('dddmr_docking')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'apriltag_inference.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    the_yaml = os.path.join(
        get_package_share_directory('dddmr_docking'),
        'config',
        'apriltag_inference.yaml'
    )

    apriltag_labelling_node = Node(
        package='dddmr_docking',
        executable='dddmr_docking',
        name='dddmr_docking',
        parameters = [the_yaml],
        output='screen'
    )


    return LaunchDescription([
        rviz_node,
        apriltag_labelling_node
    ])
