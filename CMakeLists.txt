cmake_minimum_required(VERSION 3.8)
project(dvg_slam LANGUAGES C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)

# your ROS / PCL / Eigen dependencies here

find_package(rosidl_default_generators REQUIRED)                                             
find_package(rclcpp REQUIRED)                                                   
find_package(sensor_msgs REQUIRED)                                              
find_package(mesh_msgs REQUIRED)                                                
find_package(nav_msgs REQUIRED)                                                 
find_package(tf2 REQUIRED)                                                      
find_package(tf2_ros REQUIRED)                                                  
find_package(tf2_eigen REQUIRED)                                                  
find_package(tf2_geometry_msgs REQUIRED)                                        
find_package(geometry_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "srv/GetPath.srv"
    DEPENDENCIES geometry_msgs nav_msgs
)

add_subdirectory(lib/dvg)

add_executable(dvg_slam_node
    src/dvg_slam.cpp
)

set_target_properties(dvg_slam_node PROPERTIES
    OUTPUT_NAME dvg_slam
)

ament_target_dependencies(dvg_slam_node
    rclcpp
    sensor_msgs
    mesh_msgs
    nav_msgs
    tf2_ros
    tf2
    tf2_geometry_msgs
    geometry_msgs
    PCL
    pcl_conversions
    tf2_sensor_msgs
    tf2_eigen
)

target_link_libraries(dvg_slam_node
    dvg
)

rosidl_target_interfaces(dvg_slam_node
    ${PROJECT_NAME}
    "rosidl_typesupport_cpp"
)

install(
    TARGETS dvg_slam_node
    DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
ament_package()