import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('dddmr_docking')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'realsense_example.rviz')

    cmd2odom_node = Node(
        package='dddmr_cmd2odom_simulator',
        executable='cmd2odom_simulator_node',
        name='cmd2odom_simulator_node',
        output='screen'
    )

    b2c_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_to_base_link_tf',
        # arguments: x, y, z, yaw, pitch, roll, parent_frame_id, child_frame_id
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'camera_link'],
        output='screen'
    )

    t2cpp_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_to_base_link_tf',
        # arguments: x, y, z, yaw, pitch, roll, parent_frame_id, child_frame_id
        arguments=['-0.5', '0', '0', '0', '0', '0', 'tag', 'charging_parking_point'],
        output='screen'
    )

    cpp2left_pivot_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_to_base_link_tf',
        # arguments: x, y, z, yaw, pitch, roll, parent_frame_id, child_frame_id
        arguments=['-2.0', '0.5', '0', '0', '0', '0', 'charging_parking_point', 'charging_left_pivot_point'],
        output='screen'
    )

    cpp2right_pivot_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_to_base_link_tf',
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
        cmd2odom_node,
        b2c_tf_node,
        t2cpp_tf_node,
        cpp2left_pivot_tf_node,
        cpp2right_pivot_tf_node,
        rviz_node,
        dddmr_docking_node
    ])
