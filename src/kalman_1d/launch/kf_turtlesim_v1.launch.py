from launch import LaunchDescription

import launch.actions
import launch_ros.actions
from launch.actions import ExecuteProcess

def generate_launch_description():
	x = 0.		# initial position mean
	P = 20.**2	# initial position variance
	v = 0.3	# turtle's speed
	R = 1.		# sensor/measurement variance
	Q = 0.001	# process variance
	dt = 0.02	# time interval between measurements in seconds
	
	return LaunchDescription([
        	launch_ros.actions.Node( # initialize turtlesim simulation
			package='turtlesim',
			#namespace='turtlesim',
			executable='turtlesim_node',
			output='screen',
			name='turtlesim'),
		ExecuteProcess( # teleport main turtle to x=0
			cmd=[[
				'ros2 service call /turtle1/teleport_absolute turtlesim/srv/TeleportAbsolute "{x: 0, y: 5.544445 , theta: 0.0}"', # hard coded y
			]],
			shell=True
		),	
		launch_ros.actions.Node( # initialize node that simulate a noisy sensor
			package='noisy_sensors',
			#namespace='turtlesim1',
			executable='noisy_pose_turtlesim',
			name='noisy_pose_turtlesim',
			parameters=[
				{'R': R},
				{'dt': dt}
			]
		), 
		launch_ros.actions.Node( # initialize node that filters noisy sensor with Kalman Filter
			package='kalman_1d',
			#namespace='turtlesim1',
			executable='kf_1d_turtlesim_v1',
			name='kf_pose_turtlesim',
			parameters=[
				{'x': v},
				{'P': v},
				{'v': v},
				{'R': R},
				{'Q': Q},
				{'dt': dt}
			]
		),
		launch_ros.actions.Node(  # spawns turtle to reflect noisy sensor
			package='turtlesim_addon',
			#namespace='turtlesim1',
			executable='visualization_turtle_turtlesim',
			name='noisy_turtle1',
			parameters=[
				{'turtle_name': 'noisy'},
				{'y_offset': 1}
			]
		),
		launch_ros.actions.Node(  # spawns turtle to reflect filtered measurements
			package='turtlesim_addon',
			#namespace='turtlesim1',
			executable='visualization_turtle_turtlesim',
			name='kf_turtle1',
			parameters=[
				{'turtle_name': 'kf'},
				{'y_offset': -1}
			]
		),
		launch_ros.actions.Node( # move main turtle
			package='turtlesim_addon',
			#namespace='turtlesim1',
			executable='move_turtle_turtlesim',
			name='move_turtle',
			parameters=[
				{'v': v},
				{'Q': Q}
			]
		),
		launch_ros.actions.Node( # initialize node that teleports turtle to x=0 when ir reaches the end of space
			package='turtlesim_addon',
			#namespace='turtlesim',
			executable='teleport_service_turtlesim',
			output='screen',
			name='teleport_service_turtlesim'),				
	])



