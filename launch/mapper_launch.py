from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='online_mesh_mapper',
            namespace='online_mesh_mapper',
            executable='online_mesh_mapper',
            name='online_mesh_mapper',
            parameters=[{
                'in_topic': '', #input topic of type pointcloud_2
                'in_del_topic': '', #input topic of type pointcloud2
                'frame_id': 'map', #frame id of the input chosen input
                'odometry_msg_topic': '/Spot/odometry', #topic that outputs nav_msgs_msg_odometry
                'octomap_binary_topic' : '/octomap_binary', #input topic of type octomap
                #(octomap_binary)
                'scalar': 10,
                'render_distance_horizontal':3,
                'render_distance_vertical':3,
                'out_topic':'/navigation/mesh_map',
                'max_chunks':1<<14,#this HAS to be a power of 2
                'obj_filepath':'/home/martin/Desktop/spot_mesh_map',
                'v2_mesher': 0,
                'ros2_msg_greedy_mesher': 1,
                'wavefront_greedy_mesher': 0,
                'raycast_enable': 0
            }],
        )
    ])
