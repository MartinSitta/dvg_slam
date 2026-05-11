Dynamic voxelgrid SLAM (dvgSLAM)
====================

The dvgSLAM is a 3D mapping utility, which generates a mesh map
in real time using a pointcloud2. The ros2 node publishes a mesh of the
type mesh\_msgs/msg/MeshGeometryStamped
(<https://github.com/naturerobots/mesh_tools/blob/main/mesh_msgs/msg/MeshGeometryStamped.msg>).
A wavefront file also gets created when terminating the node.

This mapping utility uses ICP algorithm with raycast removal of dynamic objects with a 2-bit log odd occupancy mapping with a dynamic 3D grid map.
A mesh visualizer is provided with the package.
however changes should only be made when only one thread is availibile

Parameters for usage
--------------------

Within the package directory there is a launch file with all the
parameters neccessary for the usage of the ros2 node. Those parameters
are:

1.  in\_topic: The topic name of the incoming pointcloud transformed to
    a frame with global coordinates
2.  frame\_id: The frame\_id of in\_topic
3.  odometry\_msg\_topic: The topic of type ros2
    nav\_msgs\_msg\_odometry that tells the node where the robot is
4.  scalar: The factor that determines the resolution of the map model:
    the resolution is: 1m/scalar
5.  render\_distance\_horizontal: The horizontal render distance in
    chunks. 1 chunk is 32 blocks. the render distance in meters per
    chunk is: (1m/scalar) \* 32
6.  render\_distance\_vertical: The vertical render distance in chunk.
    the math is the same as render\_distance\_horizontal
7.  out\_topic: The topic name of the topic this node publishes to
8.  max\_chunks: The maximum amount of chunks this node is allowed to
    store in RAM (ROM chunk storage is WIP). THIS HAS TO BE A POWER OF 2
9.  obj\_filepath: The filepath, where the wavefront file gets created.

Installation guide
------------------

Note that this packages target ros2 humble so I dont know if it will run
on newer ros2 versions.

-   Install the mesh navigation package from Naturerobots
    <https://github.com/naturerobots/mesh_navigation>

-   Clone this and set the parameters in the launch file

-   Build the package:

        colcon build #or colcon build --packages-select dvg_slam

Running the package
-------------------

To start the node run the command:

    ros2 launch dvg_slam mapper_launch.py

Known issues
------------

Setting the max\_chunks too high and higher causes ros2
to kill the program. I havent yet found a solution for this. so keep the hash table size at something reasonable (recommend is a 100-200 MB hashtable 2^24
To avoid this issue I recommend keeping the max\_chunks parameter at a
lower amount. Alternatively you can start 2\^32 and work your way down
until it stops crashing.
