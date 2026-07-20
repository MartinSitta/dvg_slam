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
                'in_topic': '/points', #input topic of type pointcloud_2
                'in_del_topic': '', #input topic of type pointcloud2
                'frame_id': 'input', #frame id of the input chosen input
                'odometry_msg_topic': '/odom', #topic that outputs nav_msgs_msg_odometry
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
            sigterm_timeout="1200",  # wait 10 min before SIGTERM
            sigkill_timeout="1200",  # wait 11 min before SIGKILL
        )
    ])
