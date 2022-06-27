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
	sensor_data_rate = 50			# in Hz
	dt = 1/sensor_data_rate		# time interval
	
	
	return LaunchDescription([
        	launch_ros.actions.Node( 		# initialize turtlesim simulation
			package='turtlesim',
			executable='turtlesim_node',
			output='screen',
			name='turtlesim'),
		ExecuteProcess( 			# teleport main turtle to x=0
			cmd=[[
				'ros2 service call /turtle1/teleport_absolute turtlesim/srv/TeleportAbsolute "{x: 0, y: 0 , theta: 0.0}"', # hard coded y
			]],
			shell=True
		),	
		launch_ros.actions.Node( 		# initialize node that simulates noisy sensor 1
			package='noisy_sensors',
			executable='noisy_pose_in_xy_turtlesim',
			name='noisy_pose',
			parameters=[
				{'R': R_var},
				{'dt': dt},
				{'topic_name': '/turtle1/noisy_pose'},
			]
		), 
		launch_ros.actions.Node( 		# initialize node that simulates noisy sensor 2
			package='noisy_sensors',
			executable='noisy_pose_in_xy_turtlesim',
			name='noisy_pose_2',
			parameters=[
				{'R': R_var},
				{'dt': dt},
				{'topic_name': '/turtle1/noisy2_pose'},
			]
		), 
		launch_ros.actions.Node( 		# initialize node that filters noisy sensor with Kalman Filter
			package='kalman_filter_nd',
			executable='kf_sensor_fusion_turtlesim_in_xy_v0',
			name='kf_pose',
			parameters=[
				{'Q_var': Q_var},
				{'R_var': R_var},
				{'dt': dt},
			]
		),
		launch_ros.actions.Node(  		# spawns turtle to reflect noisy sensor 1
			package='turtlesim_addon',
			executable='visualization_turtle_in_xy_turtlesim',
			name='noisy_visualization_turtle1',
			parameters=[
				{'turtle_name': 'noisy'},
				{'spawn_location': [0., 0., 0.]},
				{'pen_color': [0, 0, 0]},
				{'pen_width': 0},
				{'pen_off': 1},
			]
		),
		launch_ros.actions.Node(  		# spawns turtle to reflect noisy sensor 2
			package='turtlesim_addon',
			executable='visualization_turtle_in_xy_turtlesim',
			name='noisy_visualization_turtle2',
			parameters=[
				{'turtle_name': 'noisy2'},
				{'spawn_location': [0., 0., 0.]},
				{'pen_color': [0, 0, 0]},
				{'pen_width': 0},
				{'pen_off': 1},
			]
		),
		launch_ros.actions.Node(  		# spawns turtle to reflect filtered measurements
			package='turtlesim_addon',
			executable='visualization_turtle_in_xy_turtlesim',
			name='kf_visualization_turtle1',
			parameters=[
				{'turtle_name': 'kf'},
				{'spawn_location': [0., 0., 0.]},
				{'pen_color': [0, 0, 255]},
				{'pen_width': 2},
				{'pen_off': 0},
			]
		),
		launch_ros.actions.Node( 		# move main turtle
			package='turtlesim_addon',
			executable='move_turtle_in_xy_turtlesim',
			name='move_turtle',
			parameters=[
				{'v': v},
				{'Q': Q_var}
			]
		),
		launch_ros.actions.Node( 		# initialize node that teleports turtle to x=0 when ir reaches the end of space
			package='turtlesim_addon',
			executable='teleport_service_in_xy_turtlesim',
			output='screen',
			name='teleport_service_turtlesim'
		),				
	])

