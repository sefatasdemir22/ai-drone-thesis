from launch import LaunchDescription
from launch.actions import ExecuteProcess


# Basitçe Gazebo dünyasını açar. Gerekirse bridge/px4 ekleriz.


def generate_launch_description():
return LaunchDescription([
ExecuteProcess(
cmd=['gazebo', 'worlds/cave_world.world'],
output='screen'
),
])