from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('fanuc_crx10ia_description')

    urdf_path = os.path.join(pkg_share, 'urdf', 'crx10ial.urdf')
    controllers_path = os.path.join(pkg_share, 'config', 'controllers.yaml')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        ),

        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                {'robot_description': robot_description},
                controllers_path
            ],
            output='screen'
        ),

	Node(
	    package='controller_manager',
	    executable='spawner',
	    arguments=[
		'joint_state_broadcaster',
		'--controller-manager', '/controller_manager',
		'--activate'
	    ],
	    output='screen'
	),


	Node(
	    package='controller_manager',
	    executable='spawner',
	    arguments=[
		'joint_trajectory_controller',
		'--controller-manager', '/controller_manager',
		'--activate'
	    ],
    	    output='screen'
),

    ])
