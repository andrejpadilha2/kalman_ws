from launch import LaunchDescription

import launch.actions
import launch_ros.actions
from launch.actions import ExecuteProcess

def generate_launch_description():
	x = 0.		# initial position mean
	P = 11.**2	# initial position variance ( (11m)^2 )
	v = 0.3	# turtle's speed
	R = 1.		# sensor/measurement variance
	Q = .01	# process variance
	dt = 0.02	# time interval between measurements in seconds
	
	return LaunchDescription([
        	launch_ros.actions.Node( # initialize turtlesim simulation
			package='turtlesim',
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
			executable='noisy_pose_in_x_turtlesim',
			name='noisy_pose',
			parameters=[
				{'R': R},
				{'dt': dt}
			]
		), 
		launch_ros.actions.Node( # initialize node that filters noisy sensor with Kalman Filter
			package='kalman_1d',
			executable='kf_1d_turtlesim_v1',
			name='kf_pose',
			parameters=[
				{'x': x},
				{'P': P},
				{'v': v},
				{'R': R},
				{'Q': Q},
				{'dt': dt}
			]
		),
		launch_ros.actions.Node(  # spawns turtle to reflect noisy sensor
			package='turtlesim_addon',
			executable='visualization_turtle_in_x_turtlesim',
			name='noisy_visualization_turtle1',
			parameters=[
				{'turtle_name': 'noisy'},
				{'y_this_turtle': 6.544445} # where in y-axis this turtle will live
			]
		),
		launch_ros.actions.Node(  # spawns turtle to reflect filtered measurements
			package='turtlesim_addon',
			executable='visualization_turtle_in_x_turtlesim',
			name='kf_visualization_turtle1',
			parameters=[
				{'turtle_name': 'kf'},
				{'y_this_turtle': 4.544445} # where in y-axis this turtle will live
			]
		),
		launch_ros.actions.Node( # move main turtle
			package='turtlesim_addon',
			executable='move_turtle_in_x_turtlesim',
			name='move_turtle',
			parameters=[
				{'v': v},
				{'Q': Q}
			]
		),
		launch_ros.actions.Node( # initialize node that teleports turtle to x=0 when ir reaches the end of space
			package='turtlesim_addon',
			executable='teleport_service_in_x_turtlesim',
			output='screen',
			name='teleport_service_turtlesim'),				
	])

