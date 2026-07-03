from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dvg_slam',
            namespace='dvg_slam',
            executable='dvg_slam_node',
            name='dvg_slam',
            parameters=[{
                'in_topic': '/a100/pointcloud', #input topic of type pointcloud_2
                'in_del_topic': '', #input topic of type pointcloud2
                'frame_id': 'base_link', #frame id of the input chosen input
                'odometry_msg_topic': '/nav/Odometry', #topic that outputs nav_msgs_msg_odometry
                'octomap_binary_topic' : '',#/octomap_binary', #input topic of type octomap
                #(octomap_binary)
                'scalar': 5,
                'render_distance_horizontal':20,
                'render_distance_vertical':20,
                'out_topic':'/navigation/mesh_map',
                'max_chunks':1<<17,#this HAS to be a power of 2
                'obj_filepath': '/home/martin/Desktop/a100_mesh_map',
                'v2_mesher': 0,
                'ros2_msg_greedy_mesher': 1,
                'wavefront_greedy_mesher': 1,
                'raycast_enable': 0,
                'icp_gate_bypass': True
            }],
        )
    ])
