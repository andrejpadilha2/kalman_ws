from launch import LaunchDescription

import launch.actions
import launch_ros.actions
from launch.actions import ExecuteProcess

import numpy as np

def generate_launch_description():
	# SIMULATION PARAMETERS
	v = 0.3				# real turtle's speed
	Q_var = .01				# real process variance
	R_var = 1.				# sensor variance (there would be one for each sensor)
	dt = 0.02				# time interval
	
	
	return LaunchDescription([
        	launch_ros.actions.Node( 		# initialize turtlesim simulation
			package='turtlesim',
			executable='turtlesim_node',
			output='screen',
			name='turtlesim'),
		ExecuteProcess( 			# teleport main turtle to x=0
			cmd=[[
				'ros2 service call /turtle1/teleport_absolute turtlesim/srv/TeleportAbsolute "{x: 0, y: 5.544445 , theta: 0.0}"', # hard coded y
			]],
			shell=True
		),	
		launch_ros.actions.Node( 		# initialize node that simulate a noisy sensor
			package='noisy_sensors',
			executable='noisy_pose_turtlesim',
			name='noisy_pose',
			parameters=[
				{'R': R_var},
				{'dt': dt}
			]
		), 
		launch_ros.actions.Node( 		# initialize node that filters noisy sensor with Kalman Filter
			package='kalman_filter_nd',
			executable='kf_nd_turtlesim_v0',
			name='kf_pose',
			parameters=[
				{'Q_var': Q_var},
				{'R_var': R_var},
				{'dt': dt},
			]
		),
		launch_ros.actions.Node(  		# spawns turtle to reflect noisy sensor
			package='turtlesim_addon',
			executable='visualization_turtle_turtlesim',
			name='noisy_visualization_turtle1',
			parameters=[
				{'turtle_name': 'noisy'},
				{'y_this_turtle': 6.544445} # where in y-axis this turtle will live
			]
		),
		launch_ros.actions.Node(  		# spawns turtle to reflect filtered measurements
			package='turtlesim_addon',
			executable='visualization_turtle_turtlesim',
			name='kf_visualization_turtle1',
			parameters=[
				{'turtle_name': 'kf'},
				{'y_this_turtle': 4.544445} # where in y-axis this turtle will live
			]
		),
		launch_ros.actions.Node( 		# move main turtle
			package='turtlesim_addon',
			executable='move_turtle_turtlesim',
			name='move_turtle',
			parameters=[
				{'v': v},
				{'Q': Q_var}
			]
		),
		launch_ros.actions.Node( 		# initialize node that teleports turtle to x=0 when ir reaches the end of space
			package='turtlesim_addon',
			executable='teleport_service_turtlesim',
			output='screen',
			name='teleport_service_turtlesim'),				
	])

