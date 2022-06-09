from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare, FindPackagePrefix

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

import launch.actions
import launch_ros.actions

from ament_index_python import get_package_share_directory

import os

def generate_launch_description():
    
	os.environ["TURTLEBOT3_MODEL"] = "burger" # sets os environment variable needed to empty_world.launch.py (selects the turtlebot model)
	
	wall_path = os.path.abspath("src/obstacles_3d/models/brick_box_3x1x3/model.sdf")
    
	return LaunchDescription([
		
		# INITIALIZES GAZEBO'S SIMULATION
		IncludeLaunchDescription(
			PythonLaunchDescriptionSource([
				PathJoinSubstitution([
					FindPackageShare('turtlebot3_gazebo'),
					'launch',
					'empty_world.launch.py'
				])
			])
		),
	
		# SPAWNS THE WALL IN THE SIMULATION
        	launch_ros.actions.Node( # equivalent "ros2 run" command
			package='gazebo_ros', # "ros2 run gazebo_ros spawn_entity.py -entity wall -file models/brick_box_3x1x3/model.sdf -x 10 -Y 1.5707"
			#namespace='turtlebot3',
			executable='spawn_entity.py',
			output='screen',
			name='spawn_wall',
			arguments=[
				'-entity', 'wall',
				'-file', wall_path,
				'-x', '10',
				'-Y', '1.5707']),
				
		# STARTS THE TELEPORTATION SERVICE
#		launch_ros.actions.Node(
#			package='kalman_1d', 
#			executable='teleport_service',
#			output='screen',
#			name='teleport_service',
#			arguments=[
#				'target_pose', '[0, 0, 0, 0, 0, 0, 1]']),
				
		
		# STARTS MOVING THE ROBOT
#		launch_ros.actions.Node(
#			package='kalman_1d',
#			executable='turtlebot3_move',
#			output='screen',
#			name='turtlebot3_move'),
		
	])



