import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('dddmr_docking')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'apriltag_labelling.rviz')

    '''
    Assume /odom is being published
    Assume /camera/camera/color/image_raw and /camera/camera/color/camera_info are published
    '''

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
        'apriltag_labelling.yaml'
    )

    apriltag_labelling_node = Node(
        package='dddmr_docking',
        executable='apriltag_labelling_node',
        name='apriltag_labelling_node',
        parameters = [the_yaml],
        output='screen'
    )


    return LaunchDescription([
        rviz_node,
        apriltag_labelling_node
    ])
