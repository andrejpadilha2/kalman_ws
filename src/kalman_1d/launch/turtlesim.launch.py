from launch import LaunchDescription

import launch.actions
import launch_ros.actions
from launch.actions import ExecuteProcess

def generate_launch_description():
   
    
	return LaunchDescription([
		
		# INITIALIZES TURTLESIM SIMULATION
        	launch_ros.actions.Node(
			package='turtlesim',
			#namespace='turtlesim',
			executable='turtlesim_node',
			output='screen',
			name='turtlesim'),
		ExecuteProcess(
			cmd=[[
				'ros2 service call /turtle1/teleport_absolute turtlesim/srv/TeleportAbsolute "{x: 0, y: 5.544445 , theta: 0.0}"', # hard coded y
			]],
			shell=True
		),	
		launch_ros.actions.Node(
			package='noisy_sensors',
			#namespace='turtlesim1',
			executable='noisy_pose_turtlesim',
			name='noisy_pose_turtlesim'
		),
		launch_ros.actions.Node(
			package='kalman_1d',
			#namespace='turtlesim1',
			executable='kalman_1d_turtlesim',
			name='kf_pose_turtlesim'
		),
		ExecuteProcess(
			cmd=[[
				'ros2 topic pub --rate 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"',
			]],
			shell=True
		),
		launch_ros.actions.Node(
			package='kalman_1d',
			#namespace='turtlesim',
			executable='teleport_service_turtlesim',
			output='screen',
			name='teleport_service_turtlesim'),				
	])



