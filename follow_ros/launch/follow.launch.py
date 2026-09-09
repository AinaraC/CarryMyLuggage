import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_follow_ros = get_package_share_directory('follow_ros')
    pkg_rb1_gazebo = get_package_share_directory('rb1_gazebo')
    pkg_upo_laser = get_package_share_directory('upo_laser_people_detector')

    safety_distance_arg = DeclareLaunchArgument(
        'safety_distance',
        default_value='1.0',
        description='Distance to keep from detected person'
    )

    simulator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rb1_gazebo, 'launch', 'leon_home.launch.py')
        )
    )

    upo_laser_node = Node(
        package='upo_laser_people_detector',
        executable='lasermodelnode',
        name='lasermodelhost',
        parameters=[{
            'model_file': '/home/ubuntu/ros2_ws/src/upo_laser_people_detector/upo_laser_people_detector/models/LFE-PPN.onnx',
            'laser_topic': '/scan',
            'output_topic': '/detected_people',
            'marker_topic': '/detected_people_markers',
            'use_sim_time': True

        }],
        output='screen'
    )

    follow_node = Node(
        package='follow_ros',
        executable='follow_node',
        name='follow_node',
        parameters=[{
            'safety_distance': LaunchConfiguration('safety_distance'),
            'use_sim_time': True
        }],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(safety_distance_arg)
    ld.add_action(simulator_launch)
    ld.add_action(upo_laser_node)
    ld.add_action(follow_node)

    return ld
