from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # URDF 파일 경로 지정
    urdf_file = os.path.join(
        get_package_share_directory('haechi_description'),
        'urdf',
        'haechi_robot.urdf'
    )

    return LaunchDescription([
        # Robot State Publisher 실행
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': open(urdf_file, 'r').read()}],
        ),
        # RViz 실행
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        ),
    ])
