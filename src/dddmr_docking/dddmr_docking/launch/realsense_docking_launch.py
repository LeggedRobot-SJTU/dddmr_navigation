import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('dddmr_docking')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'realsense_example.rviz')

    '''
    Assume /odom is being published
    Assume /camera/camera/color/image_raw and /camera/camera/color/camera_info are published
    '''

    b2c_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='b2c_tf_node',
        # arguments: x, y, z, yaw, pitch, roll, parent_frame_id, child_frame_id
        arguments=['0.25', '0', '0', '0', '0', '0', 'base_link', 'camera_link'],
        output='screen'
    )

    t2cpp_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='t2cpp_tf_node',
        # arguments: x, y, z, yaw, pitch, roll, parent_frame_id, child_frame_id
        arguments=['-0.4', '0', '0', '0', '0', '0', 'tag', 'charging_parking_point'],
        output='screen'
    )

    cpp2left_pivot_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='cpp2left_pivot_tf_node',
        # arguments: x, y, z, yaw, pitch, roll, parent_frame_id, child_frame_id
        arguments=['-2.0', '0.5', '0', '0', '0', '0', 'charging_parking_point', 'charging_left_pivot_point'],
        output='screen'
    )

    cpp2right_pivot_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='cpp2right_pivot_tf_node',
        # arguments: x, y, z, yaw, pitch, roll, parent_frame_id, child_frame_id
        arguments=['-2.0', '-0.5', '0', '0', '0', '0', 'charging_parking_point', 'charging_right_pivot_point'],
        output='screen'
    )

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
        'realsense_docking_params.yaml'
    )

    dddmr_docking_node = Node(
        package='dddmr_docking',
        executable='docking_node',
        name='docking_node',
        parameters = [the_yaml],
        remappings=[
          ('tag_pose', 'camera1/tag_pose')
        ],
        output='screen'
    )


    return LaunchDescription([
        b2c_tf_node,
        t2cpp_tf_node,
        cpp2left_pivot_tf_node,
        cpp2right_pivot_tf_node,
        rviz_node,
        dddmr_docking_node
    ])
