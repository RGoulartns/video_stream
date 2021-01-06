from launch import LaunchDescription
from launch_ros.actions import Node
from os.path import expanduser

maps_param_path = expanduser("~/MAPS/maps_parameters.yaml")

def generate_launch_description():
	return LaunchDescription([
		Node(
			package='video_stream',
			namespace='cam_front',
			executable='video_stream_node',
			name='video_stream_node',
			output='screen',
			parameters=[maps_param_path],
		),
		
		Node(
			package='video_stream',
			namespace='cam_rear',
			executable='video_stream_node',
			name='video_stream_node',
			output='screen',
			parameters=[maps_param_path],
		),			
	])
