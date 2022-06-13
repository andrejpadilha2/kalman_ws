from launch import LaunchDescription

import launch.actions
import launch_ros.actions

def generate_launch_description():
   
    
	return LaunchDescription([
		
		# INITIALIZES GAZEBO'S SIMULATION
        	launch_ros.actions.Node(
			package='turtlesim',
			#namespace='turtlebot3',
			executable='turtlesim_node',
			output='screen',
			name='turtlesim_kalman_filter')
						
	])



