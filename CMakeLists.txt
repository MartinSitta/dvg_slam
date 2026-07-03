cmake_minimum_required(VERSION 3.8)
project(dvg_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Ofast)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)                                              
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
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
    #"msg/PathResponse.msg"
    "srv/GetPath.srv"
    DEPENDENCIES nav_msgs std_msgs
    )
add_library(lib_map_model STATIC lib/HashUtils.c
    lib/murmur3.c lib/Dvg.c lib/Chunk.c
    lib/FibonacciHash.c lib/ChunkBitmap.c lib/VoxelHashMap.c lib/VoxelPriorityQueue.c
    lib/DvgVector.c lib/DynamicObjectRemoval.c)     
add_executable(dvg_slam_node src/dvg_slam_pairwise.cpp)
target_include_directories(dvg_slam_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(dvg_slam_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_link_libraries(dvg_slam_node lib_map_model)
ament_target_dependencies(dvg_slam_node rclcpp sensor_msgs mesh_msgs nav_msgs
    tf2_ros tf2 tf2_geometry_msgs geometry_msgs octomap octomap_msgs PCL
    pcl_conversions tf2_sensor_msgs tf2_eigen)
rosidl_target_interfaces(dvg_slam_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
install(TARGETS dvg_slam_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
