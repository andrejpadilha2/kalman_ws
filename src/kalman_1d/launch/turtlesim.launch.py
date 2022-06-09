from launch import LaunchDescription

import launch.actions
import launch_ros.actions

def generate_launch_description():
   
    
	return LaunchDescription([
		
		# INITIALIZES GAZEBO'S SIMULATION
        	launch_ros.actions.Node( # equivalent "ros2 run" command
			package='turtlesim', # "ros2 run gazebo_ros spawn_entity.py -entity wall -file models/brick_box_3x1x3/model.sdf -x 10 -Y 1.5707"
			#namespace='turtlebot3',
			executable='turtlesim_node',
			output='screen',
			name='turtlesim_kalman_filter')
						
	])



