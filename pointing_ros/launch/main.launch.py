import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    gz_resource_path = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        os.path.join(get_package_share_directory("rb1_description"), "..") + ":" +
        os.path.join(get_package_share_directory("rb1_gazebo"), "models"))
    # Simulador en Gazebo
    tiago_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rb1_gazebo'),
                'launch',
                'leon_home.launch.py',
            )
        ),
    )

    # YOLO Bringup
    yolo_detection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('yolo_bringup'),
                'launch',
                'yolov12.launch.py',
            )
        ),
        launch_arguments={
            'model': 'yolo12m.pt',
            'input_image_topic': '/camera/rgb/image_raw',
            'device': 'cuda:0',
            'threshold': '0.5',
            'image_reliability': '2', # Best effort
            'namespace': 'yolo_detection',

        }.items(),
    )

    yolo_pose_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('yolo_bringup'),
                'launch',
                'yolov11.launch.py',
            )
        ),
        launch_arguments={
            'model': 'yolo11m-pose.pt',
            'input_image_topic': '/camera/rgb/image_raw',
            'device': 'cuda:0',
            'threshold': '0.5',
            'image_reliability': '2',
            'namespace': 'yolo_pose',
        }.items(),
    )

    pointing_node = Node(
        package='pointing_ros',
        executable='pointing_node',
        name='pointing_node',
        output='screen'
    )

    return LaunchDescription([
        gz_resource_path,
        tiago_gazebo_launch,
        yolo_pose_launch,
        yolo_detection_launch,
        pointing_node
    ])
