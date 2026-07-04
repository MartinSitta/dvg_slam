#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <pcl/impl/point_types.hpp>
#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/detail/point_cloud2__struct.hpp>
#include <string>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "rclcpp/rclcpp.hpp"
#include <climits>
#include <cassert>
#include <chrono>
#include <fstream>
#include <string>
#include <mutex>
#include <vector>
#include <thread>
#include <cmath>
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>
//#include <pcl_ros/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/filter.h>
#include "../lib/Dvg.h"
#include "../lib/VoxelHashMap.h"
#include "../lib/VoxelPriorityQueue.h"
#include "../lib/DvgVector.h"
#include "../lib/DynamicObjectRemoval.c"
#include "mesh_msgs/msg/mesh_geometry_stamped.hpp"
// ROS / messages
#include "octomap_msgs/msg/octomap.hpp"

// OctoMap core
#include <octomap/octomap.h>
#include <octomap/OcTree.h>

// OctoMap-ROS helper conversions
#include <octomap_msgs/conversions.h>   // binaryMapToMsg / fullMsgToMap helpers
//#include <octomap_msgs/Octomap.h>      // (message header if you prefer)
#include "dvg_slam/srv/get_path.hpp"
//#include "mesh_tools.h"
using std::placeholders::_1;
using namespace std;

typedef struct{
    uint32_t vertex_index1;
    uint32_t vertex_index2;
    uint32_t vertex_index3;
    int64_t optional_quad_vertex;
    uint32_t vertex_normal_index;
} OutFace_t;

typedef struct{
    float x;
    float y;
    float z;
} OutVertex_t;

typedef struct{
    int64_t x;
    int64_t y;
    int64_t z;
} DelPoint_t;

typedef struct{
    OutVertex_t v1;
    uint32_t v_index1;
    OutVertex_t v2;
    uint32_t v_index2;
    OutVertex_t v3;
    uint32_t v_index3;
    OutVertex_t v4;
    uint32_t v_index4;
    uint8_t vertex_normal_index;
    bool dead;
} InFace_t;


typedef struct{
    std::vector<uint32_t> requestor_indices;
    OutVertex_t coords;
} InVertex_t;

typedef struct{
    std::vector<OutFace_t> faces;
    std::vector<OutVertex_t> vertices;
    std::vector<OutVertex_t> vertex_normals;
} ChunkMesh_t;

typedef union{
    uint8_t buf[4];
    float valuef;
} FUCKING_WHY_DO_I_HAVE_TO_DO_THIS_GOD_IS_DEAD_AND_I_KILLED_HIM_t;


class DvgSlam : public rclcpp::Node{
  public:
    DvgSlam()
    : Node("dvg_slam"), count_(0),
      clock_(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME)){   
        this->declare_parameter<std::string>("in_topic", "");
        this->declare_parameter<std::string>("in_del_topic", "");
        this->declare_parameter<std::string>("frame_id", "");
        this->declare_parameter<std::string>("odometry_msg_topic", "");
        this->declare_parameter<std::string>("out_topic", "");
        this->declare_parameter<std::string>("obj_filepath", "");
        this->declare_parameter<std::string>("octomap_binary_topic", "");
        this->declare_parameter<int>("max_chunks", 2048);
        this->declare_parameter<int>("scalar", 0);
        this->declare_parameter<int>("render_distance_horizontal", 0);
        this->declare_parameter<int>("render_distance_vertical", 0);
        this->declare_parameter<int>("v2_mesher", 1);
        this->declare_parameter<int>("ros2_msg_greedy_mesher", 0);
        this->declare_parameter<int>("wavefront_greedy_mesher", 0);
        this->declare_parameter<int>("raycast_enable", 0);
        this->declare_parameter<bool>("icp_gate_bypass", false);
        this->render_distance_horizontal = this->get_parameter("render_distance_horizontal").as_int();
        this->render_distance_vertical = this->get_parameter("render_distance_vertical").as_int();
        this->obj_filepath = this->get_parameter("obj_filepath").as_string();
        this->chunk_amount = this->get_parameter("max_chunks").as_int();
        this->out_topic = this->get_parameter("out_topic").as_string();
        this->scalar = this->get_parameter("scalar").as_int();
        this->topic = this->get_parameter("in_topic").as_string();
        this->del_topic = this->get_parameter("in_del_topic").as_string();
        this->octomap_binary_topic = this->get_parameter("octomap_binary_topic").as_string();
        this->frame_id = this->get_parameter("frame_id").as_string();
        this->odom_topic = this->get_parameter("odometry_msg_topic").as_string();
        this->v2_mesher = this->get_parameter("v2_mesher").as_int();
        this->ros2_msg_greedy_mesher = this->get_parameter("ros2_msg_greedy_mesher").as_int();
        this->wavefront_greedy_mesher = this->get_parameter("wavefront_greedy_mesher").as_int();
        this->raycast_enable = this->get_parameter("raycast_enable").as_int();
        while(this->frame_id == "" || odom_topic == "" ||
                this->scalar == 0 || this->out_topic == "" || this->obj_filepath == ""){
            if(this->frame_id == ""){
                RCLCPP_WARN(this->get_logger(), "SET THE \"frame_id\" PARAMETER");
            }
            if(this->odom_topic == ""){
                RCLCPP_WARN(this->get_logger(), "SET THE \"odometry_msg_topic\" PARAMETER");
            }
            if(this->scalar == 0){
                RCLCPP_WARN(this->get_logger(), "SET THE \"scalar\" PARAMETER");
            }
            if(this->out_topic == "")
            {
                RCLCPP_WARN(this->get_logger(), "SET THE \"out_topic\" PARAMETER");
            }
            if(this->obj_filepath == "")
            {
                RCLCPP_WARN(this->get_logger(), "SET THE \"obj_filepath\" PARAMETER");
            }
            if(this->scalar == 0)
            {
                RCLCPP_WARN(this->get_logger(), "SET THE \"scalar\" PARAMETER");
            }
            this->odom_topic = this->get_parameter("odometry_msg_topic").as_string();
            this->topic = this->get_parameter("in_topic").as_string();
            this->frame_id = this->get_parameter("frame_id").as_string();
            this->out_topic = this->get_parameter("out_topic").as_string();
            this->scalar = this->get_parameter("scalar").as_int();
            this->obj_filepath = this->get_parameter("obj_filepath").as_string();
        }
        RCLCPP_INFO(this->get_logger(), "initializing topics\n");
        this->icp_gate_bypass = this->get_parameter("icp_gate_bypass").as_bool();
        rclcpp::sleep_for(std::chrono::seconds(5));
        publisher_ = this->create_publisher<mesh_msgs::msg::MeshGeometryStamped>(out_topic, 1);
        mesh_timer = this->create_wall_timer(
        1000ms, std::bind(&DvgSlam::timer_callback, this));
        if(topic != ""){
            subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                    topic , 1,
                    std::bind(&DvgSlam::point_cloud_in_callback, 
                    this, _1));
        }
        subscription_two = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic,
                1, std::bind(&DvgSlam::odom_callback, this, _1)); 
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        publisher_two = this->create_publisher<nav_msgs::msg::Odometry>("debug_map_pose", 1);
        pointcloud_debug_publisher =
            this->create_publisher<sensor_msgs::msg::PointCloud2>(
                "/dvg_slam/debug_point_cloud",
                rclcpp::QoS(1)
            );
        path_publisher = this->create_publisher<nav_msgs::msg::Path>("planned_path", 1);
        pose_timer = this->create_wall_timer(
        10ms, std::bind(&DvgSlam::debug_map_pose_callback, this));
        //nav_timer = this->create_wall_timer(
        //2000ms, std::bind(&DvgSlam::nav_callback, this));
        service = this->create_service<dvg_slam::srv::GetPath>(
            "get_path", std::bind(DvgSlam::nav_callback, this, std::placeholders::_1,
                    std::placeholders::_2)
        );
        current_odom_msg_pose.position.x = 0;
        current_odom_msg_pose.position.y = 0;
        current_odom_msg_pose.position.z = 0;
        current_odom_msg_pose.orientation.x = 0;
        current_odom_msg_pose.orientation.y = 0;
        current_odom_msg_pose.orientation.z = 0;
        current_odom_msg_pose.orientation.w = 1;
        prev_odom_msg_pose.position.x = 0;
        prev_odom_msg_pose.position.y = 0;
        prev_odom_msg_pose.position.z = 0;
        prev_odom_msg_pose.orientation.x = 0;
        prev_odom_msg_pose.orientation.y = 0;
        prev_odom_msg_pose.orientation.z = 0;
        prev_odom_msg_pose.orientation.w = 1;
        last_processed_octomap_msg = std::chrono::steady_clock::now();
        rclcpp::sleep_for(std::chrono::milliseconds(500));
        last_processed_pointcloud_msg = std::chrono::steady_clock::now();

        trajectory_log_file.open("/home/martin/SLAM-datasets/schlachte_dvg_estimated.csv");

        trajectory_log_file
            << "index,timestamp,x,y,z,qx,qy,qz,qw\n";

        trajectory_log_file.flush();

         RCLCPP_INFO(
        this->get_logger(),
        "ICP params: max_corr=%f m, iterations=%d, trans_eps=%e, fitness_eps=%e",
        icp_max_correspondence_m,
        icp_max_iterations,
        icp_trans_epsilon,
        icp_fitness_epsilon);


        RCLCPP_INFO(this->get_logger(), "starting the mapper\n");
        graph = dvg_init(this->chunk_amount);
        assert(graph != NULL);
        RCLCPP_INFO(this->get_logger(), "node initialized\n");
    }
    ~DvgSlam(){
        RCLCPP_WARN(this->get_logger(), "Terminating node\n");
        RCLCPP_WARN(this->get_logger(), "Publishing final mesh\n");
        if(v2_mesher){
            build_and_publish_mesh_v2(graph, publisher_);
        }
        else{
            build_and_publish_mesh(graph, publisher_);
        }
        RCLCPP_WARN(this->get_logger(), "Creating wavefront\n");
        if(v2_mesher){
            write_global_wavefront_v2(graph); 
        }
        else{
            write_global_wavefront(graph); 
        }
        RCLCPP_WARN(this->get_logger(), "Deleting the map model\n");
        RCLCPP_WARN(this->get_logger(), "Out of %ld hashtable entries, there were %ld hash collisions\n", graph->total_hash_table_insertions, graph->total_hash_collisions);
        dvg_free(&graph);
    }
  private:
        void debug_map_pose_callback(){
            nav_msgs::msg::Odometry out_msg;
            out_msg.pose.pose = global_point;
            out_msg.child_frame_id = "base_link";
            out_msg.header.frame_id = "map";
            publisher_two->publish(out_msg);
        }

    int64_t get_manhattan_dist(int64_t org_x, int64_t org_y, int64_t org_z,
             int64_t dest_x, int64_t dest_y, int64_t dest_z){
        return labs(org_x - dest_x) + labs(org_y - dest_y) + labs(org_z - dest_z);
    }
    float get_euclidean_dist(int64_t org_x, int64_t org_y, int64_t org_z,
             int64_t dest_x, int64_t dest_y, int64_t dest_z){
        float x_dist = org_x - dest_x;
        float y_dist = org_y - dest_y;
        float z_dist = org_z - dest_z;
        return std::sqrt(x_dist * x_dist + y_dist * y_dist + z_dist * z_dist);
    }
    bool out_vertex_equals(OutVertex_t v1, OutVertex_t v2){
        bool x_equal = fabs(v1.x - v2.x) < 0.005f;
        bool y_equal = fabs(v1.y - v2.y) < 0.005f;
        bool z_equal = fabs(v1.z - v2.z) < 0.005f;
        return x_equal && y_equal && z_equal;

    }
    bool out_vertex_z_equals(OutVertex_t v1, OutVertex_t v2){
        bool z_equal = fabs(v1.z - v2.z) < 0.005f;
        return z_equal;
    }
    
    bool out_vertex_x_equals(OutVertex_t v1, OutVertex_t v2){
        bool x_equal = fabs(v1.x - v2.x) < 0.005f;
        return x_equal;
    }
    bool out_vertex_2_out_of_3_equal(OutVertex_t v1, OutVertex_t v2){
        int equal_counter = 0;
        equal_counter += (fabs(v1.x - v2.x) < 0.005f) * 1;
        equal_counter += (fabs(v1.y - v2.y) < 0.005f) * 1;
        equal_counter += (fabs(v1.z - v2.z) < 0.005f) * 1;
        if(equal_counter >= 2){
            return true;
        }
        return false;
    }


    int32_t requestor_hash_table_add(int32_t index, std::vector<int32_t>* hash_table){
        uint32_t hash = 0;
        MurmurHash3_x86_32(static_cast<void*>(&index), sizeof(int32_t), 2953741627, &hash);
        hash = hash & (hash_table->size() - 1);
        uint32_t double_hash = 0;
        MurmurHash3_x86_32(static_cast<void*>(&index), sizeof(int32_t), 1623094874, &double_hash);
        
        for(uint64_t i = 0; i < 10; i++){
            uint32_t end_hash = ((uint64_t)hash + (i * (uint64_t)double_hash)) % hash_table->size();
            int32_t hash_table_val = hash_table->at(end_hash);
            if(hash_table_val == -1){
                hash_table->at(end_hash) = index;
                return index;
            }
            else if(hash_table_val == index){
                //printf("do I ever get here?\n");
                return hash_table_val;
            }
        }
        return -1;


    }
    int32_t requestor_hash_table_lookup(int32_t index, std::vector<int32_t>* hash_table){
        uint32_t hash = 0;
        MurmurHash3_x86_32(static_cast<void*>(&index), sizeof(int32_t), 2953741627, &hash);
        hash = hash & (hash_table->size() - 1);
        uint32_t double_hash = 0;
        MurmurHash3_x86_32(static_cast<void*>(&index), sizeof(int32_t), 1623094874, &double_hash);
        
        for(uint64_t i = 0; i < 10; i++){
            uint32_t end_hash = ((uint64_t)hash + (i * (uint64_t)double_hash)) % hash_table->size();
            int32_t hash_table_val = hash_table->at(end_hash);
            if(hash_table_val == -1){
                return -1;
            }
            else if(hash_table_val == index){
                //printf("do I ever get here?\n");
                return hash_table_val;
            }
        }
        return -1;
    }
    int32_t get_adjacent_face(uint32_t face_index, int32_t v1_index, int32_t v2_index,
            std::vector<InFace_t>* faces, std::vector<InVertex_t>* vertices){
        assert(v1_index < vertices->size());
        assert(v2_index < vertices->size());
        uint8_t face_normal = faces->at(face_index).vertex_normal_index;
        std::vector<int32_t> requestor_hash_table(1<<4, -1);
        for(uint32_t i = 0; i < vertices->at(v1_index).requestor_indices.size(); i++){
            requestor_hash_table_add(vertices->at(v1_index).requestor_indices.at(i), &requestor_hash_table);
        }
        for(uint32_t i = 0; i < vertices->at(v2_index).requestor_indices.size(); i++){
            int32_t index = -1;
            index = requestor_hash_table_lookup(vertices->at(v2_index).requestor_indices.at(i), &requestor_hash_table);
            if(index >= 0){
                if(index != face_index && faces->at(index).vertex_normal_index == face_normal){
                    if(faces->at(index).dead == false){
                        return index;
                    }
                }
            }
        }   
        return -1;
    }
    void face_remove_vertex(uint32_t face_index, uint32_t vertex_index,
            std::vector<InFace_t>* faces, std::vector<InVertex_t>* vertices,
            std::vector<int32_t>* vertex_hash_table){
        if(faces->at(face_index).v_index1 == vertex_index){
            faces->at(face_index).v_index1 = -1;
            vertex_hash_table_remove(faces->at(face_index).v1, face_index, vertices, vertex_hash_table);
        }
        if(faces->at(face_index).v_index2 == vertex_index){
            faces->at(face_index).v_index2 = -1;
            vertex_hash_table_remove(faces->at(face_index).v2, face_index, vertices, vertex_hash_table);
        }
        if(faces->at(face_index).v_index3 == vertex_index){
            faces->at(face_index).v_index3 = -1;
            vertex_hash_table_remove(faces->at(face_index).v3, face_index, vertices, vertex_hash_table);
        }
        if(faces->at(face_index).v_index4 == vertex_index){
            faces->at(face_index).v_index4 = -1;
            vertex_hash_table_remove(faces->at(face_index).v4, face_index, vertices, vertex_hash_table);
        }
    }

    void face_reinsert_vertex(int32_t face_index, OutVertex_t v,
            std::vector<InFace_t>* faces, std::vector<InVertex_t>* vertices,
            std::vector<int32_t>* vertex_hash_table){
        if(faces->at(face_index).v_index1 == -1){
            if(out_vertex_2_out_of_3_equal(faces->at(face_index).v1, v)){
                faces->at(face_index).v1 = v;
                faces->at(face_index).v_index1 = vertex_hash_table_request(faces->at(face_index).v1, face_index, vertices, vertex_hash_table);
                return;
            }
        }
        if(faces->at(face_index).v_index2 == -1){
            if(out_vertex_2_out_of_3_equal(faces->at(face_index).v2, v)){
                faces->at(face_index).v2 = v;
                faces->at(face_index).v_index2 = vertex_hash_table_request(faces->at(face_index).v2, face_index, vertices, vertex_hash_table);
                return;
            }
        }
        if(faces->at(face_index).v_index3 == -1){
            if(out_vertex_2_out_of_3_equal(faces->at(face_index).v3, v)){
                faces->at(face_index).v3 = v;
                faces->at(face_index).v_index3 = vertex_hash_table_request(faces->at(face_index).v3, face_index, vertices, vertex_hash_table);
                return;
            }
        }
        if(faces->at(face_index).v_index4 == -1){
            if(out_vertex_2_out_of_3_equal(faces->at(face_index).v4, v)){
                faces->at(face_index).v4 = v;
                faces->at(face_index).v_index4 = vertex_hash_table_request(faces->at(face_index).v4, face_index, vertices, vertex_hash_table);
                return;
            }
        }
    }
    void fuse_faces(int32_t face_index1, int32_t face_index2,
            int32_t shared_vertex_index1, int32_t shared_vertex_index2, 
            std::vector<InFace_t>* faces, std::vector<InVertex_t>* vertices,
            std::vector<int32_t>* vertex_hash_table){
        std::vector<OutVertex_t> vertices_to_add_on_face1;
        if(faces->at(face_index2).v_index1 != shared_vertex_index1 && faces->at(face_index2).v_index1 != shared_vertex_index2){
            vertices_to_add_on_face1.push_back(faces->at(face_index2).v1);
        }
        if(faces->at(face_index2).v_index2 != shared_vertex_index1 && faces->at(face_index2).v_index2 != shared_vertex_index2){
            vertices_to_add_on_face1.push_back(faces->at(face_index2).v2);
        }
        if(faces->at(face_index2).v_index3 != shared_vertex_index1 && faces->at(face_index2).v_index3 != shared_vertex_index2){
            vertices_to_add_on_face1.push_back(faces->at(face_index2).v3);
        }
        if(faces->at(face_index2).v_index4 != shared_vertex_index1 && faces->at(face_index2).v_index4 != shared_vertex_index2){
            vertices_to_add_on_face1.push_back(faces->at(face_index2).v4);
        }
        assert(!out_vertex_equals(vertices_to_add_on_face1.at(0), vertices->at(shared_vertex_index1).coords));
        assert(!out_vertex_equals(vertices_to_add_on_face1.at(0), vertices->at(shared_vertex_index1).coords));
        assert(!out_vertex_equals(vertices_to_add_on_face1.at(1), vertices->at(shared_vertex_index1).coords));
        assert(!out_vertex_equals(vertices_to_add_on_face1.at(1), vertices->at(shared_vertex_index1).coords));
        vertex_hash_table_remove(faces->at(face_index2).v1, face_index2, vertices, vertex_hash_table);
        vertex_hash_table_remove(faces->at(face_index2).v2, face_index2, vertices, vertex_hash_table);
        vertex_hash_table_remove(faces->at(face_index2).v3, face_index2, vertices, vertex_hash_table);
        vertex_hash_table_remove(faces->at(face_index2).v4, face_index2, vertices, vertex_hash_table);
        faces->at(face_index2).dead = true;
        
        face_remove_vertex(face_index1, shared_vertex_index1, faces, vertices, vertex_hash_table);
        face_remove_vertex(face_index1, shared_vertex_index2, faces, vertices, vertex_hash_table);
        face_reinsert_vertex(face_index1, vertices_to_add_on_face1.at(0), faces, vertices, vertex_hash_table);
        face_reinsert_vertex(face_index1, vertices_to_add_on_face1.at(1), faces, vertices, vertex_hash_table);
    }


    void fuse_faces_above_and_below(uint32_t face_index, std::vector<InFace_t>* faces,
            std::vector<InVertex_t>* vertices, std::vector<int32_t>* vertex_hash_table){
        //in the face creation in the chunk generation v1 is always an upper one
        //v3 is always a lower one. this is a bit hacky but it saves a whole lot
        //of work this is when the normal vector is ABOVE 2
        //for a normal vector at 2 or below this is reversed
        bool fusion_done = true;
        while(fusion_done){   
            fusion_done = false;
            int32_t upper_vertex_index1 = -1;
            int32_t upper_vertex_index2 = -1;
            int32_t lower_vertex_index1 = -1;
            int32_t lower_vertex_index2 = -1;
            uint8_t vertex_normal = faces->at(face_index).vertex_normal_index;
            switch(vertex_normal){
                case 1:
                    upper_vertex_index1 = faces->at(face_index).v_index4;
                    upper_vertex_index2 = faces->at(face_index).v_index3;
                    lower_vertex_index1 = faces->at(face_index).v_index1;
                    lower_vertex_index2 = faces->at(face_index).v_index2;
                    break;
                case 2:
                    upper_vertex_index1 = faces->at(face_index).v_index2;
                    upper_vertex_index2 = faces->at(face_index).v_index3;
                    lower_vertex_index1 = faces->at(face_index).v_index1;
                    lower_vertex_index2 = faces->at(face_index).v_index4;
                    break;
                case 3:
                    upper_vertex_index1 = faces->at(face_index).v_index1;
                    upper_vertex_index2 = faces->at(face_index).v_index4;
                    lower_vertex_index1 = faces->at(face_index).v_index3;
                    lower_vertex_index2 = faces->at(face_index).v_index2;
                    break;
                case 4:
                    upper_vertex_index1 = faces->at(face_index).v_index1;
                    upper_vertex_index2 = faces->at(face_index).v_index2;
                    lower_vertex_index1 = faces->at(face_index).v_index3;
                    lower_vertex_index2 = faces->at(face_index).v_index4;
                    break;
                case 5:
                    upper_vertex_index1 = faces->at(face_index).v_index1;
                    upper_vertex_index2 = faces->at(face_index).v_index2;
                    lower_vertex_index1 = faces->at(face_index).v_index4;
                    lower_vertex_index2 = faces->at(face_index).v_index3;
                    break;
                case 6:
                    upper_vertex_index1 = faces->at(face_index).v_index1;
                    upper_vertex_index2 = faces->at(face_index).v_index4;
                    lower_vertex_index1 = faces->at(face_index).v_index2;
                    lower_vertex_index2 = faces->at(face_index).v_index3;
                    break;

            }
            int32_t upper_adjacent_face_index = -1;
            int32_t lower_adjacent_face_index = -1;
            upper_adjacent_face_index = get_adjacent_face(face_index, upper_vertex_index1, upper_vertex_index2, faces, vertices);
            if(upper_adjacent_face_index != -1){
                fuse_faces(face_index, upper_adjacent_face_index, upper_vertex_index1, upper_vertex_index2, faces, vertices, vertex_hash_table);
                fusion_done = true;
            }
            lower_adjacent_face_index = get_adjacent_face(face_index, lower_vertex_index1, lower_vertex_index2, faces, vertices);
            if(lower_adjacent_face_index != -1){
                fuse_faces(face_index, lower_adjacent_face_index, lower_vertex_index1, lower_vertex_index2, faces, vertices, vertex_hash_table);
                fusion_done = true;
            }
        }
    }

    void fuse_faces_left_and_right(uint32_t face_index, std::vector<InFace_t>* faces,
            std::vector<InVertex_t>* vertices, std::vector<int32_t>* vertex_hash_table){
        bool fusion_done = true;
        while(fusion_done){   
            int32_t left_vertex_index1 = -1;
            int32_t left_vertex_index2 = -1;
            int32_t right_vertex_index1 = -1;
            int32_t right_vertex_index2 = -1;

            fusion_done = false;
            uint8_t vertex_normal = faces->at(face_index).vertex_normal_index;
            switch(vertex_normal){
                case 1:
                    left_vertex_index1 = faces->at(face_index).v_index1;
                    left_vertex_index2 = faces->at(face_index).v_index4;
                    right_vertex_index1 = faces->at(face_index).v_index3;
                    right_vertex_index2 = faces->at(face_index).v_index2;
                    break;
                case 2:
                    left_vertex_index1 = faces->at(face_index).v_index1;
                    left_vertex_index2 = faces->at(face_index).v_index2;
                    right_vertex_index1 = faces->at(face_index).v_index3;
                    right_vertex_index2 = faces->at(face_index).v_index4;
                    break;
                case 3:
                    left_vertex_index1 = faces->at(face_index).v_index4;
                    left_vertex_index2 = faces->at(face_index).v_index3;
                    right_vertex_index1 = faces->at(face_index).v_index1;
                    right_vertex_index2 = faces->at(face_index).v_index2;
                    break;
                case 4:
                    left_vertex_index1 = faces->at(face_index).v_index2;
                    left_vertex_index2 = faces->at(face_index).v_index3;
                    right_vertex_index1 = faces->at(face_index).v_index1;
                    right_vertex_index2 = faces->at(face_index).v_index4;
                    break;
                case 5:
                    left_vertex_index1 = faces->at(face_index).v_index1;
                    left_vertex_index2 = faces->at(face_index).v_index4;
                    right_vertex_index1 = faces->at(face_index).v_index3;
                    right_vertex_index2 = faces->at(face_index).v_index2;
                    break;
                case 6:
                    left_vertex_index1 = faces->at(face_index).v_index1;
                    left_vertex_index2 = faces->at(face_index).v_index2;
                    right_vertex_index1 = faces->at(face_index).v_index3;
                    right_vertex_index2 = faces->at(face_index).v_index4;
                    break;
            }
            int32_t left_adjacent_face_index = -1;
            int32_t right_adjacent_face_index = -1;
            left_adjacent_face_index = get_adjacent_face(face_index, left_vertex_index1, left_vertex_index2, faces, vertices);
            if(left_adjacent_face_index != -1){
                fuse_faces(face_index, left_adjacent_face_index, left_vertex_index1, left_vertex_index2, faces, vertices, vertex_hash_table);
                fusion_done = true;
            }
            right_adjacent_face_index = get_adjacent_face(face_index, right_vertex_index1, right_vertex_index2, faces, vertices);
            if(right_adjacent_face_index != -1){
                fuse_faces(face_index, right_adjacent_face_index, right_vertex_index1, right_vertex_index2, faces, vertices, vertex_hash_table);
                fusion_done = true;
            }
        }
    }
    void reduce_faces(std::vector<InFace_t>* faces, std::vector<InVertex_t>* vertices, std::vector<int32_t>* vertex_hash_table){
        for(uint32_t i = 0; i < faces->size(); i++){
            if(faces->at(i).dead == true){
                continue;
            }
            fuse_faces_above_and_below(i, faces, vertices, vertex_hash_table);
        }
        for(uint32_t i = 0; i < faces->size(); i++){
            if(faces->at(i).dead == true){
                continue;
            }
            fuse_faces_left_and_right(i, faces, vertices, vertex_hash_table);
        }


    }

    int32_t vertex_hash_table_request(OutVertex_t v, uint32_t face_requestor_index, 
            std::vector<InVertex_t>* vertices, std::vector<int32_t>* vertex_hash_table)
    {
        uint32_t hash = 0;
        MurmurHash3_x86_32(static_cast<void*>(&v), sizeof(OutVertex_t), 2953741627, &hash);
        hash = hash & (vertex_hash_table->size() - 1);
        uint32_t double_hash = 0;
        MurmurHash3_x86_32(static_cast<void*>(&v), sizeof(OutVertex_t), 1623094874, &double_hash);
        
        for(uint64_t i = 0; i < 10; i++){
            uint32_t end_hash = ((uint64_t)hash + (i * (uint64_t)double_hash)) % vertex_hash_table->size();
            int32_t hash_table_val = vertex_hash_table->at(end_hash);
            if(hash_table_val == -1){
                InVertex_t new_vertex;
                new_vertex.coords = v;
                new_vertex.requestor_indices.push_back(face_requestor_index);
                vertices->push_back(new_vertex);
                vertex_hash_table->at(end_hash) = vertices->size() - 1;
                return vertices->size() - 1;
            }
            else if(out_vertex_equals(vertices->at(hash_table_val).coords, v)){
                //printf("do I ever get here?\n");
                bool duplicate_found = false;
                for(uint32_t j = 0; j < vertices->at(hash_table_val).requestor_indices.size() && !duplicate_found; j++){
                    if(vertices->at(hash_table_val).requestor_indices.at(j) == face_requestor_index){
                        duplicate_found = true;
                    }
                }
                if(duplicate_found == false){
                    vertices->at(hash_table_val).requestor_indices.push_back(face_requestor_index);
                }
                return hash_table_val;
            }
        }
        return -1;
    }

    int32_t vertex_hash_table_remove(OutVertex_t v, uint32_t face_requestor_index, 
            std::vector<InVertex_t>* vertices, std::vector<int32_t>* vertex_hash_table){
        uint32_t hash = 0;
        MurmurHash3_x86_32(static_cast<void*>(&v), sizeof(OutVertex_t), 2953741627, &hash);
        hash = hash & (vertex_hash_table->size() - 1);
        uint32_t double_hash = 0;
        MurmurHash3_x86_32(static_cast<void*>(&v), sizeof(OutVertex_t), 1623094874, &double_hash);
        
        for(uint64_t i = 0; i < 10; i++){
            uint32_t end_hash = ((uint64_t)hash + (i * (uint64_t)double_hash)) % vertex_hash_table->size();
            int32_t hash_table_val = vertex_hash_table->at(end_hash);
            if(hash_table_val == -1){
                return -1;
            }
            else if(out_vertex_equals(vertices->at(hash_table_val).coords, v)){
                for(uint32_t j = 0; j < vertices->at(hash_table_val).requestor_indices.size(); j++){
                    if(vertices->at(hash_table_val).requestor_indices.at(j) == face_requestor_index){
                        //printf("do I ever get here?\n");
                        vertices->at(hash_table_val).requestor_indices.erase(vertices->at(hash_table_val).requestor_indices.begin() + j);
                    }
                }
                return hash_table_val;
            }
        }
        return -1;
    }

    void greedy_mesher_enter_vertices(std::vector<InFace_t>* faces, 
            std::vector<InVertex_t>* vertices, std::vector<int32_t>* vertex_hash_table){
        for(uint32_t i = 0; i < faces->size(); i++){
            faces->at(i).v_index1 = vertex_hash_table_request(faces->at(i).v1, i, vertices, vertex_hash_table);
            faces->at(i).v_index2 = vertex_hash_table_request(faces->at(i).v2, i, vertices, vertex_hash_table);
            faces->at(i).v_index3 = vertex_hash_table_request(faces->at(i).v3, i, vertices, vertex_hash_table);
            faces->at(i).v_index4 = vertex_hash_table_request(faces->at(i).v4, i, vertices, vertex_hash_table);
        }
    }

    void greedy_mesher_enter_vertex_normals(ChunkMesh_t* out_mesh,
            std::vector<InFace_t>* faces, std::vector<InVertex_t>* vertices){
        
        for(uint32_t i = 0; i < vertices->size(); i++){
            OutVertex_t normal;
            normal.x = 0.0f;
            normal.y = 0.0f;
            normal.z = 0.0f;
            for(uint32_t j = 0; j < vertices->at(i).requestor_indices.size(); j++){
                uint32_t face_index = vertices->at(i).requestor_indices.at(j);
                if(faces->at(face_index).vertex_normal_index == 1){
                    normal.z += 1;
                }
                if(faces->at(face_index).vertex_normal_index == 2){
                    normal.z -= 1;
                }
                if(faces->at(face_index).vertex_normal_index == 3){
                    normal.y += 1;
                }
                if(faces->at(face_index).vertex_normal_index == 4){
                    normal.y -= 1;
                }
                if(faces->at(face_index).vertex_normal_index == 5){
                    normal.z += 1;
                }
                if(faces->at(face_index).vertex_normal_index == 6){
                    normal.z -= 1;
                }
            }
            out_mesh->vertex_normals.push_back(normal);
        }
    }
    void greedy_mesher_unshare_vertices_and_write_output_mesh(ChunkMesh_t* out_mesh,
            std::vector<InFace_t>* faces, std::vector<InVertex_t>* vertices){
        printf("entry of unshare_vertices\n");
        uint32_t per_vertex_offset = 0;
        uint32_t neg_offset = 0;
        std::vector<InVertex_t> new_vertices;
        for(uint32_t i = 0; i < vertices->size(); i++){
            for(uint32_t j = 0; j < vertices->at(i).requestor_indices.size(); j++){
                uint32_t index = vertices->at(i).requestor_indices.at(j);
                InVertex_t new_vertex;
                new_vertex.coords = vertices->at(i).coords;
                new_vertex.requestor_indices.push_back(index);
                new_vertices.push_back(new_vertex);
            }   
        }
        *vertices = new_vertices;
        printf("end of unshare_vertices\n");
        greedy_mesher_reenter_vertex_indices_and_write_out_mesh(out_mesh, faces, vertices);
    }

    void greedy_mesher_reenter_vertex_indices_and_write_out_mesh(
            ChunkMesh_t* out_mesh, std::vector<InFace_t>* faces,
            std::vector<InVertex_t>* vertices){
        for(uint32_t i = 0; i < vertices->size(); i++){
            OutVertex_t v = vertices->at(i).coords;
            out_mesh->vertices.push_back(v);
            uint32_t index = vertices->at(i).requestor_indices.at(0);
            if(out_vertex_equals(faces->at(index).v1, vertices->at(i).coords)){
                faces->at(index).v_index1 = i;
            }
            else if(out_vertex_equals(faces->at(index).v2, vertices->at(i).coords)){
                faces->at(index).v_index2 = i;
            }
            else if(out_vertex_equals(faces->at(index).v3, vertices->at(i).coords)){
                faces->at(index).v_index3 = i;
            }
            else if(out_vertex_equals(faces->at(index).v4, vertices->at(i).coords)){
                faces->at(index).v_index4 = i;
            }
        }
         for(uint32_t i = 0; i < faces->size(); i++){
            if(faces->at(i).dead){
                continue;
            }
            OutFace_t out_face1;

            out_face1.vertex_index1 = faces->at(i).v_index1;
            out_face1.vertex_index2 = faces->at(i).v_index2;
            out_face1.vertex_index3 = faces->at(i).v_index3;
            out_face1.optional_quad_vertex = faces->at(i).v_index4;
            out_face1.vertex_normal_index = faces->at(i).vertex_normal_index;
            out_mesh->faces.push_back(out_face1);
        }
    }

    
    void greedy_mesher_write_output_mesh(ChunkMesh_t* out_mesh,
            std::vector<InFace_t>* faces, std::vector<InVertex_t>* vertices){
        uint32_t neg_offset = 0;
        for(uint32_t i = 0; i < vertices->size(); i++){
            if(vertices->at(i).requestor_indices.size() == 0){
                neg_offset++;
            }
            else{
                OutVertex_t v = vertices->at(i).coords;
                out_mesh->vertices.push_back(v);
                for(uint32_t j = 0; j < vertices->at(i).requestor_indices.size(); j++){
                    uint32_t index = vertices->at(i).requestor_indices.at(j);
                    if(faces->at(index).v_index1 == i){
                        faces->at(index).v_index1 -= neg_offset;
                    }
                    else if(faces->at(index).v_index2 == i){
                        faces->at(index).v_index2 -= neg_offset;
                    }
                    else if(faces->at(index).v_index3 == i){
                        faces->at(index).v_index3 -= neg_offset;
                    }
                    else if(faces->at(index).v_index4 == i){
                        faces->at(index).v_index4 -= neg_offset;
                    }
                }
            }

        }
        for(uint32_t i = 0; i < faces->size(); i++){
            if(faces->at(i).dead){
                continue;
            }
            OutFace_t out_face1;
            OutFace_t out_face2;

            out_face1.vertex_index1 = faces->at(i).v_index1;
            out_face1.vertex_index2 = faces->at(i).v_index2;
            out_face1.vertex_index3 = faces->at(i).v_index4;
            out_face1.optional_quad_vertex = -1;
            out_face1.vertex_normal_index = faces->at(i).vertex_normal_index;


            out_face2.vertex_index1 = faces->at(i).v_index2;
            out_face2.vertex_index2 = faces->at(i).v_index3;
            out_face2.vertex_index3 = faces->at(i).v_index4;
            out_face2.optional_quad_vertex = -1;
            out_face2.vertex_normal_index = faces->at(i).vertex_normal_index;
            
            out_mesh->faces.push_back(out_face1);
            out_mesh->faces.push_back(out_face2);
        }
    }
    ChunkMesh_t gen_chunk_mesh_with_greedy_mesher(Dvg_t* graph, Chunk_t* chunk, bool wavefront){   
        ChunkMesh_t output;
        std::vector<InFace_t> in_faces;
        std::vector<InVertex_t> in_vertices;
        std::vector<int32_t> vertex_hash_table(1 << 17, -1);
        uint32_t current_vertex_index = 1;
        int64_t base_x = chunk->x_offset;
        int64_t base_y = chunk->y_offset;
        int64_t base_z = chunk->z_offset;
        Chunk_t* upper_neighbor_chunk = dvg_chunk_hash_table_lookup(graph, base_x, base_y, base_z + ALT_CHUNK_LEN);
        Chunk_t* lower_neighbor_chunk = dvg_chunk_hash_table_lookup(graph, base_x, base_y, base_z - ALT_CHUNK_LEN);
        Chunk_t* left_neighbor_chunk = dvg_chunk_hash_table_lookup(graph, base_x, base_y + ALT_CHUNK_LEN, base_z);
        Chunk_t* right_neighbor_chunk = dvg_chunk_hash_table_lookup(graph, base_x, base_y - ALT_CHUNK_LEN, base_z);
        Chunk_t* foward_neighbor_chunk = dvg_chunk_hash_table_lookup(graph, base_x + ALT_CHUNK_LEN, base_y, base_z);
        Chunk_t* back_neighbor_chunk = dvg_chunk_hash_table_lookup(graph, base_x - ALT_CHUNK_LEN, base_y, base_z);
        for(int64_t moving_x = 0; moving_x < 16; moving_x++){
            for(int64_t moving_y = 0; moving_y < 16; moving_y++){
                for(int64_t moving_z = 0; moving_z < 16; moving_z++){
                    int64_t total_x = base_x + moving_x;
                    int64_t total_y = base_y + moving_y;
                    int64_t total_z = base_z + moving_z;
                    bool up_neighbor = true;
                    bool down_neighbor = true;
                    bool left_neighbor = true;
                    bool right_neighbor = true;
                    bool foward_neighbor = true;
                    bool back_neighbor = true;
                    if(chunk_lookup(chunk, total_x, total_y, total_z) < 2){
                        continue;
                    }
                    //up
                    if(moving_z == ALT_CHUNK_LEN - 1){
                        if(upper_neighbor_chunk != NULL){
                            up_neighbor = chunk_lookup(upper_neighbor_chunk, total_x, total_y, total_z + 1) >= 2;
                        }
                    }
                    else{
                        up_neighbor = chunk_lookup(chunk, total_x, total_y, total_z + 1) >= 2;
                    }
                    //down
                    if(moving_z == 0){
                        if(lower_neighbor_chunk != NULL){
                            down_neighbor = chunk_lookup(lower_neighbor_chunk, total_x, total_y, total_z - 1) >= 2;
                        }
                    }
                    else{
                        down_neighbor = chunk_lookup(chunk, total_x, total_y, total_z - 1) >= 2;
                    }
                    //left
                    if(moving_y == ALT_CHUNK_LEN - 1){
                        if(left_neighbor_chunk != NULL){
                            left_neighbor = chunk_lookup(left_neighbor_chunk, total_x, total_y + 1, total_z) >= 2;
                        }
                    }
                    else{
                        left_neighbor = chunk_lookup(chunk, total_x, total_y + 1, total_z) >= 2;
                    }
                    //right
                    if(moving_y == 0){
                        if(right_neighbor_chunk != NULL){
                            right_neighbor = chunk_lookup(right_neighbor_chunk, total_x, total_y - 1, total_z) >= 2;
                        }
                    }
                    else{
                        right_neighbor = chunk_lookup(chunk, total_x, total_y - 1, total_z) >= 2;
                    }
                    //foward
                    if(moving_x == ALT_CHUNK_LEN - 1){
                        if(foward_neighbor_chunk != NULL){
                            foward_neighbor = chunk_lookup(foward_neighbor_chunk, total_x + 1, total_y, total_z) >= 2;
                        }
                    }
                    else{
                        foward_neighbor = chunk_lookup(chunk, total_x + 1, total_y, total_z) >= 2;
                    }
                    //back
                    if(moving_x == 0){
                        if(back_neighbor_chunk != NULL){
                            back_neighbor = chunk_lookup(back_neighbor_chunk, total_x - 1, total_y, total_z) >= 2;
                        }
                    }
                    else{
                        back_neighbor = chunk_lookup(chunk, total_x - 1, total_y, total_z) >= 2;
                    }
                    OutVertex_t v1;
                    OutVertex_t v2;
                    OutVertex_t v3;
                    OutVertex_t v4;
                    InVertex_t vertex;
                    InFace_t face;
                    face.dead = false;
                    if(!up_neighbor){   
                        v1.x = (float)total_x / scalar;
                        v1.y = (float)total_y / scalar;
                        v1.z = (float)total_z / scalar;
            
                        v2.x = (float)(total_x + 1) / scalar;
                        v2.y = (float)total_y / scalar;
                        v2.z = (float)total_z / scalar;

                        v3.x = (float)(total_x + 1) / scalar;
                        v3.y = (float)(total_y + 1) / scalar;
                        v3.z = (float)total_z / scalar;

                        v4.x = (float)total_x / scalar;
                        v4.y = (float)(total_y + 1) / scalar;
                        v4.z = (float)total_z / scalar;

                        face.v1 = v1;
                        face.v2 = v4;
                        face.v3 = v3;
                        face.v4 = v2;
                        face.vertex_normal_index = 1;
                        in_faces.push_back(face);
                    }
                    if(!down_neighbor){
                        v1.x = (float)total_x / scalar;
                        v1.y = (float)total_y / scalar;
                        v1.z = (float)(total_z - 1) / scalar;
    
                        v2.x = (float)(total_x + 1) / scalar;
                        v2.y = (float)total_y / scalar;
                        v2.z = (float)(total_z - 1) / scalar;

                        v3.x = (float)(total_x + 1) / scalar;
                        v3.y = (float)(total_y + 1) / scalar;
                        v3.z = (float)(total_z - 1) / scalar;

                        v4.x = (float)total_x / scalar;
                        v4.y = (float)(total_y + 1) / scalar;
                        v4.z = (float)(total_z - 1) / scalar;
            
                        face.v1 = v1;
                        face.v2 = v2;
                        face.v3 = v3;
                        face.v4 = v4;
                        face.vertex_normal_index = 2;
                        in_faces.push_back(face);
                    }
                    if(!left_neighbor){
                        v1.x = (float)total_x / scalar;
                        v1.y = (float)(total_y + 1) / scalar;
                        v1.z = (float)total_z / scalar;
            
                        v2.x = (float)(total_x + 1) / scalar;
                        v2.y = (float)(total_y + 1) / scalar;
                        v2.z = (float)total_z / scalar;

                        v3.x = (float)(total_x + 1) / scalar;
                        v3.y = (float)(total_y + 1) / scalar;
                        v3.z = (float)(total_z - 1) / scalar;

                        v4.x = (float)total_x / scalar;
                        v4.y = (float)(total_y + 1) / scalar;
                        v4.z = (float)(total_z - 1) / scalar;
            
                        face.v1 = v1;
                        face.v2 = v4;
                        face.v3 = v3;
                        face.v4 = v2;
                        face.vertex_normal_index = 3;
                        in_faces.push_back(face);

                    }
                    if(!right_neighbor){
                        v1.x = (float)total_x / scalar;
                        v1.y = (float)total_y / scalar;
                        v1.z = (float)total_z / scalar;
            
                        v2.x = (float)(total_x + 1) / scalar;
                        v2.y = (float)total_y / scalar;
                        v2.z = (float)total_z / scalar;

                        v3.x = (float)(total_x + 1) / scalar;
                        v3.y = (float)total_y / scalar;
                        v3.z = (float)(total_z - 1) / scalar;

                        v4.x = (float)total_x / scalar;
                        v4.y = (float)total_y / scalar;
                        v4.z = (float)(total_z - 1) / scalar;
            
                        face.v1 = v1;
                        face.v2 = v2;
                        face.v3 = v3;
                        face.v4 = v4;
                        face.vertex_normal_index = 4;
                        in_faces.push_back(face);

                }
                    if(!foward_neighbor){
                        v1.x = (float)(total_x + 1) / scalar;
                        v1.y = (float)total_y / scalar;
                        v1.z = (float)total_z / scalar;
            
                        v2.x = (float)(total_x + 1) / scalar;
                        v2.y = (float)total_y / scalar;
                        v2.z = (float)(total_z - 1) / scalar;

                        v3.x = (float)(total_x + 1) / scalar;
                        v3.y = (float)(total_y + 1) / scalar;
                        v3.z = (float)(total_z - 1) / scalar;

                        v4.x = (float)(total_x + 1) / scalar;
                        v4.y = (float)(total_y + 1) / scalar;
                        v4.z = (float)total_z / scalar;
            
                        face.v1 = v1;
                        face.v2 = v4;
                        face.v3 = v3;
                        face.v4 = v2;
                        face.vertex_normal_index = 5;
                        in_faces.push_back(face);

                    }
                    if(!back_neighbor){  
                        v1.x = (float)total_x / scalar;
                        v1.y = (float)total_y / scalar;
                        v1.z = (float)total_z / scalar;
            
                        v2.x = (float)total_x / scalar;
                        v2.y = (float)total_y / scalar;
                        v2.z = (float)(total_z - 1) / scalar;
    
                        v3.x = (float)total_x / scalar;
                        v3.y = (float)(total_y + 1) / scalar;
                        v3.z = (float)(total_z - 1) / scalar;

                        v4.x = (float)total_x / scalar;
                        v4.y = (float)(total_y + 1) / scalar;
                        v4.z = (float)total_z / scalar;
            
                        face.v1 = v1;
                        face.v2 = v2;
                        face.v3 = v3;
                        face.v4 = v4;
                        face.vertex_normal_index = 6;
                        in_faces.push_back(face);
    
                    }
                }
            }
        }

        greedy_mesher_enter_vertices(&in_faces, &in_vertices, &vertex_hash_table);
        if((wavefront && wavefront_greedy_mesher) || (!wavefront && ros2_msg_greedy_mesher)){
            reduce_faces(&in_faces, &in_vertices, &vertex_hash_table);
        }
        greedy_mesher_write_output_mesh(&output, &in_faces, &in_vertices);
        /*
        if(wavefront){
            greedy_mesher_unshare_vertices_and_write_output_mesh(&output, &in_faces, &in_vertices);
        }
        else{
            greedy_mesher_write_output_mesh(&output, &in_faces, &in_vertices);
        }
        */
        greedy_mesher_enter_vertex_normals(&output, &in_faces, &in_vertices);
        return output;
    }

    void v2_mesher_get_faces_and_verts(Dvg_t* graph,
            Chunk_t* chunk,
            std::vector<InFace_t>* in_faces,
            std::vector<InVertex_t>* in_vertices, 
            std::vector<int32_t>* vertex_hash_table,
            bool wavefront){   
        uint32_t current_vertex_index = 1;
        int64_t base_x = chunk->x_offset;
        int64_t base_y = chunk->y_offset;
        int64_t base_z = chunk->z_offset;
        Chunk_t* upper_neighbor_chunk = dvg_chunk_hash_table_lookup(graph, base_x, base_y, base_z + ALT_CHUNK_LEN);
        Chunk_t* lower_neighbor_chunk = dvg_chunk_hash_table_lookup(graph, base_x, base_y, base_z - ALT_CHUNK_LEN);
        Chunk_t* left_neighbor_chunk = dvg_chunk_hash_table_lookup(graph, base_x, base_y + ALT_CHUNK_LEN, base_z);
        Chunk_t* right_neighbor_chunk = dvg_chunk_hash_table_lookup(graph, base_x, base_y - ALT_CHUNK_LEN, base_z);
        Chunk_t* foward_neighbor_chunk = dvg_chunk_hash_table_lookup(graph, base_x + ALT_CHUNK_LEN, base_y, base_z);
        Chunk_t* back_neighbor_chunk = dvg_chunk_hash_table_lookup(graph, base_x - ALT_CHUNK_LEN, base_y, base_z);
        for(int64_t moving_x = 0; moving_x < 16; moving_x++){
            for(int64_t moving_y = 0; moving_y < 16; moving_y++){
                for(int64_t moving_z = 0; moving_z < 16; moving_z++){
                    int64_t total_x = base_x + moving_x;
                    int64_t total_y = base_y + moving_y;
                    int64_t total_z = base_z + moving_z;
                    bool up_neighbor = true;
                    bool down_neighbor = true;
                    bool left_neighbor = true;
                    bool right_neighbor = true;
                    bool foward_neighbor = true;
                    bool back_neighbor = true;
                    if(chunk_lookup(chunk, total_x, total_y, total_z) < 2){
                        continue;
                    }
                    //up
                    if(moving_z == ALT_CHUNK_LEN - 1){
                        if(upper_neighbor_chunk != NULL){
                            up_neighbor = chunk_lookup(upper_neighbor_chunk, total_x, total_y, total_z + 1) >= 2;
                        }
                    }
                    else{
                        up_neighbor = chunk_lookup(chunk, total_x, total_y, total_z + 1) >= 2;
                    }
                    //down
                    if(moving_z == 0){
                        if(lower_neighbor_chunk != NULL){
                            down_neighbor = chunk_lookup(lower_neighbor_chunk, total_x, total_y, total_z - 1) >= 2;
                        }
                    }
                    else{
                        down_neighbor = chunk_lookup(chunk, total_x, total_y, total_z - 1) >= 2;
                    }
                    //left
                    if(moving_y == ALT_CHUNK_LEN - 1){
                        if(left_neighbor_chunk != NULL){
                            left_neighbor = chunk_lookup(left_neighbor_chunk, total_x, total_y + 1, total_z) >= 2;
                        }
                    }
                    else{
                        left_neighbor = chunk_lookup(chunk, total_x, total_y + 1, total_z) >= 2;
                    }
                    //right
                    if(moving_y == 0){
                        if(right_neighbor_chunk != NULL){
                            right_neighbor = chunk_lookup(right_neighbor_chunk, total_x, total_y - 1, total_z) >= 2;
                        }
                    }
                    else{
                        right_neighbor = chunk_lookup(chunk, total_x, total_y - 1, total_z) >= 2;
                    }
                    //foward
                    if(moving_x == ALT_CHUNK_LEN - 1){
                        if(foward_neighbor_chunk != NULL){
                            foward_neighbor = chunk_lookup(foward_neighbor_chunk, total_x + 1, total_y, total_z) >= 2;
                        }
                    }
                    else{
                        foward_neighbor = chunk_lookup(chunk, total_x + 1, total_y, total_z) >= 2;
                    }
                    //back
                    if(moving_x == 0){
                        if(back_neighbor_chunk != NULL){
                            back_neighbor = chunk_lookup(back_neighbor_chunk, total_x - 1, total_y, total_z) >= 2;
                        }
                    }
                    else{
                        back_neighbor = chunk_lookup(chunk, total_x - 1, total_y, total_z) >= 2;
                    }
                    OutVertex_t v1;
                    OutVertex_t v2;
                    OutVertex_t v3;
                    OutVertex_t v4;
                    InVertex_t vertex;
                    InFace_t face;
                    face.dead = false;
                    if(!up_neighbor){   
                        v1.x = (float)total_x / scalar;
                        v1.y = (float)total_y / scalar;
                        v1.z = (float)total_z / scalar;
            
                        v2.x = (float)(total_x + 1) / scalar;
                        v2.y = (float)total_y / scalar;
                        v2.z = (float)total_z / scalar;

                        v3.x = (float)(total_x + 1) / scalar;
                        v3.y = (float)(total_y + 1) / scalar;
                        v3.z = (float)total_z / scalar;

                        v4.x = (float)total_x / scalar;
                        v4.y = (float)(total_y + 1) / scalar;
                        v4.z = (float)total_z / scalar;

                        face.v1 = v1;
                        face.v2 = v4;
                        face.v3 = v3;
                        face.v4 = v2;
                        face.vertex_normal_index = 1;
                        in_faces->push_back(face);
                    }
                    if(!down_neighbor){
                        v1.x = (float)total_x / scalar;
                        v1.y = (float)total_y / scalar;
                        v1.z = (float)(total_z - 1) / scalar;
            
                        v2.x = (float)(total_x + 1) / scalar;
                        v2.y = (float)total_y / scalar;
                        v2.z = (float)(total_z - 1) / scalar;

                        v3.x = (float)(total_x + 1) / scalar;
                        v3.y = (float)(total_y + 1) / scalar;
                        v3.z = (float)(total_z - 1) / scalar;

                        v4.x = (float)total_x / scalar;
                        v4.y = (float)(total_y + 1) / scalar;
                        v4.z = (float)(total_z - 1) / scalar;
            
                        face.v1 = v1;
                        face.v2 = v2;
                        face.v3 = v3;
                        face.v4 = v4;
                        face.vertex_normal_index = 2;
                        in_faces->push_back(face);
                    }
                    if(!left_neighbor){
                        v1.x = (float)total_x / scalar;
                        v1.y = (float)(total_y + 1) / scalar;
                        v1.z = (float)total_z / scalar;
            
                        v2.x = (float)(total_x + 1) / scalar;
                        v2.y = (float)(total_y + 1) / scalar;
                        v2.z = (float)total_z / scalar;

                        v3.x = (float)(total_x + 1) / scalar;
                        v3.y = (float)(total_y + 1) / scalar;
                        v3.z = (float)(total_z - 1) / scalar;

                        v4.x = (float)total_x / scalar;
                        v4.y = (float)(total_y + 1) / scalar;
                        v4.z = (float)(total_z - 1) / scalar;
            
                        face.v1 = v1;
                        face.v2 = v4;
                        face.v3 = v3;
                        face.v4 = v2;
                        face.vertex_normal_index = 3;
                        in_faces->push_back(face);
                    }
                    if(!right_neighbor){
                        v1.x = (float)total_x / scalar;
                        v1.y = (float)total_y / scalar;
                        v1.z = (float)total_z / scalar;
            
                        v2.x = (float)(total_x + 1) / scalar;
                        v2.y = (float)total_y / scalar;
                        v2.z = (float)total_z / scalar;

                        v3.x = (float)(total_x + 1) / scalar;
                        v3.y = (float)total_y / scalar;
                        v3.z = (float)(total_z - 1) / scalar;

                        v4.x = (float)total_x / scalar;
                        v4.y = (float)total_y / scalar;
                        v4.z = (float)(total_z - 1) / scalar;
            
                        face.v1 = v1;
                        face.v2 = v2;
                        face.v3 = v3;
                        face.v4 = v4;
                        face.vertex_normal_index = 4;
                        in_faces->push_back(face);

                    }
                    if(!foward_neighbor){
                        v1.x = (float)(total_x + 1) / scalar;
                        v1.y = (float)total_y / scalar;
                        v1.z = (float)total_z / scalar;
            
                        v2.x = (float)(total_x + 1) / scalar;
                        v2.y = (float)total_y / scalar;
                        v2.z = (float)(total_z - 1) / scalar;

                        v3.x = (float)(total_x + 1) / scalar;
                        v3.y = (float)(total_y + 1) / scalar;
                        v3.z = (float)(total_z - 1) / scalar;

                        v4.x = (float)(total_x + 1) / scalar;
                        v4.y = (float)(total_y + 1) / scalar;
                        v4.z = (float)total_z / scalar;
            
                        face.v1 = v1;
                        face.v2 = v4;
                        face.v3 = v3;
                        face.v4 = v2;
                        face.vertex_normal_index = 5;
                        in_faces->push_back(face);

                    }
                    if(!back_neighbor){  
                        v1.x = (float)total_x / scalar;
                        v1.y = (float)total_y / scalar;
                        v1.z = (float)total_z / scalar;
            
                        v2.x = (float)total_x / scalar;
                        v2.y = (float)total_y / scalar;
                        v2.z = (float)(total_z - 1) / scalar;

                        v3.x = (float)total_x / scalar;
                        v3.y = (float)(total_y + 1) / scalar;
                        v3.z = (float)(total_z - 1) / scalar;

                        v4.x = (float)total_x / scalar;
                        v4.y = (float)(total_y + 1) / scalar;
                        v4.z = (float)total_z / scalar;
            
                        face.v1 = v1;
                        face.v2 = v2;
                        face.v3 = v3;
                        face.v4 = v4;
                        face.vertex_normal_index = 6;
                        in_faces->push_back(face);

                    }
                }
            }
        }
        /*
        greedy_mesher_enter_vertices(&in_faces, &in_vertices, &vertex_hash_table);
        if(!wavefront){
            reduce_faces(&in_faces, &in_vertices, &vertex_hash_table);
        }
        greedy_mesher_write_output_mesh(&output, &in_faces, &in_vertices);
        */
        /*
        if(wavefront){
            greedy_mesher_unshare_vertices_and_write_output_mesh(&output, &in_faces, &in_vertices);
        }
        else{
            greedy_mesher_write_output_mesh(&output, &in_faces, &in_vertices);
        }
        */
        //greedy_mesher_enter_vertex_normals(&output, &in_faces, &in_vertices);
    }

    
    void build_and_publish_mesh_v2(Dvg_t* graph, rclcpp::Publisher<mesh_msgs::msg::MeshGeometryStamped>::SharedPtr pub){
        std::vector<InFace_t> in_faces;
        std::vector<InVertex_t> in_vertices;
        RCLCPP_INFO(this->get_logger(), "creating final hashtable");
        std::vector<int32_t> vertex_hash_table(1 << 26, -1);
        RCLCPP_INFO(this->get_logger(), "hashtable created generating chunk geometries");
        ChunkMesh_t output;
        std::vector<OutVertex_t> vertex_normals;
        OutVertex_t up_normal = {0.0f, 0.0f, 1.0f};
        vertex_normals.push_back(up_normal);
        OutVertex_t down_normal = {0.0f, 0.0f, -1.0f};
        vertex_normals.push_back(down_normal);
        OutVertex_t left_normal = {0.0f, 1.0f, 0.0f};
        vertex_normals.push_back(left_normal);
        OutVertex_t right_normal = {0.0f, -1.0f, 0.0f};
        vertex_normals.push_back(right_normal);
        OutVertex_t foward_normal = {1.0f, 0.0f, 0.0f};
        vertex_normals.push_back(foward_normal);
        OutVertex_t back_normal = {-1.0f, 0.0f, 0.0f};
        vertex_normals.push_back(back_normal);
        for(uint32_t i = 0; i < graph->chunk_hash_table_size; i++){
            if(graph->chunk_hash_table[i] != NULL){
                v2_mesher_get_faces_and_verts(graph, graph->chunk_hash_table[i], &in_faces, &in_vertices, &vertex_hash_table, true);
            }
        }
        greedy_mesher_enter_vertices(&in_faces, &in_vertices, &vertex_hash_table);
        if(ros2_msg_greedy_mesher){
            reduce_faces(&in_faces, &in_vertices, &vertex_hash_table);
        }
        greedy_mesher_write_output_mesh(&output, &in_faces, &in_vertices);
        greedy_mesher_enter_vertex_normals(&output, &in_faces, &in_vertices);
        std::vector<ChunkMesh_t> chunk_local_meshes;
        chunk_local_meshes.push_back(output);
        publish_meshes(&chunk_local_meshes, &vertex_normals, pub);
        return;




    }
    void build_and_publish_regional_mesh_v2(Dvg_t* graph,
            uint32_t hor_chunk_radius, uint32_t vert_chunk_radius,
            rclcpp::Publisher<mesh_msgs::msg::MeshGeometryStamped>::SharedPtr pub){
        
        std::vector<InFace_t> in_faces;
        std::vector<InVertex_t> in_vertices;
        std::vector<int32_t> vertex_hash_table(1 << 22, -1);
        ChunkMesh_t output;
        std::vector<OutVertex_t> vertex_normals;
        OutVertex_t up_normal = {0.0f, 0.0f, 1.0f};
        vertex_normals.push_back(up_normal);
        OutVertex_t down_normal = {0.0f, 0.0f, -1.0f};
        vertex_normals.push_back(down_normal);
        OutVertex_t left_normal = {0.0f, 1.0f, 0.0f};
        vertex_normals.push_back(left_normal);
        OutVertex_t right_normal = {0.0f, -1.0f, 0.0f};
        vertex_normals.push_back(right_normal);
        OutVertex_t foward_normal = {1.0f, 0.0f, 0.0f};
        vertex_normals.push_back(foward_normal);
        OutVertex_t back_normal = {-1.0f, 0.0f, 0.0f};
        vertex_normals.push_back(back_normal);

        int64_t x_neg_target = (global_point.position.x * scalar) - hor_chunk_radius * ALT_CHUNK_LEN;
        int64_t x_pos_target = (global_point.position.x * scalar) + hor_chunk_radius * ALT_CHUNK_LEN;
        int64_t y_neg_target = (global_point.position.y * scalar) - hor_chunk_radius * ALT_CHUNK_LEN;
        int64_t y_pos_target = (global_point.position.y * scalar) + hor_chunk_radius * ALT_CHUNK_LEN;
        int64_t z_neg_target = (global_point.position.z * scalar) - vert_chunk_radius * ALT_CHUNK_LEN;
        int64_t z_pos_target = (global_point.position.z * scalar) + vert_chunk_radius * ALT_CHUNK_LEN;
        std::vector<Chunk_t*> chunk_ptrs;
        for(int64_t x = x_neg_target; x <= x_pos_target; x += ALT_CHUNK_LEN){
            for(int64_t y = y_neg_target; y <= y_pos_target; y += ALT_CHUNK_LEN){
                for(int64_t z = z_neg_target; z <= z_pos_target; z += ALT_CHUNK_LEN){
                    Chunk_t* chunk = dvg_chunk_hash_table_lookup(graph, x, y, z);
                    if(chunk != NULL){
                        chunk_ptrs.push_back(chunk);
                    }
                }
            }
        }
        for(uint32_t i = 0; i < chunk_ptrs.size(); i++){
            v2_mesher_get_faces_and_verts(graph, chunk_ptrs.at(i), &in_faces, &in_vertices, &vertex_hash_table, true);
        }
        greedy_mesher_enter_vertices(&in_faces, &in_vertices, &vertex_hash_table);
        if(ros2_msg_greedy_mesher){
            reduce_faces(&in_faces, &in_vertices, &vertex_hash_table);
        }
        greedy_mesher_write_output_mesh(&output, &in_faces, &in_vertices);
        greedy_mesher_enter_vertex_normals(&output, &in_faces, &in_vertices);
        std::vector<ChunkMesh_t> chunk_local_meshes;
        chunk_local_meshes.push_back(output);
        publish_meshes(&chunk_local_meshes, &vertex_normals, pub);
        return;

    }

    void build_and_publish_regional_mesh(Dvg_t* graph,
            uint32_t hor_chunk_radius, uint32_t vert_chunk_radius,
            rclcpp::Publisher<mesh_msgs::msg::MeshGeometryStamped>::SharedPtr pub){
        std::vector<ChunkMesh_t> chunk_local_meshes(graph->current_chunk_index);
        std::vector<thread*> active_threads;
        std::vector<OutVertex_t> vertex_normals;
        OutVertex_t up_normal = {0.0f, 0.0f, 1.0f};
        vertex_normals.push_back(up_normal);
        OutVertex_t down_normal = {0.0f, 0.0f, -1.0f};
        vertex_normals.push_back(down_normal);
        OutVertex_t left_normal = {0.0f, 1.0f, 0.0f};
        vertex_normals.push_back(left_normal);
        OutVertex_t right_normal = {0.0f, -1.0f, 0.0f};
        vertex_normals.push_back(right_normal);
        OutVertex_t foward_normal = {1.0f, 0.0f, 0.0f};
        vertex_normals.push_back(foward_normal);
        OutVertex_t back_normal = {-1.0f, 0.0f, 0.0f};
        vertex_normals.push_back(back_normal);

        int64_t x_neg_target = (global_point.position.x * scalar) - hor_chunk_radius * ALT_CHUNK_LEN;
        int64_t x_pos_target = (global_point.position.x * scalar) + hor_chunk_radius * ALT_CHUNK_LEN;
        int64_t y_neg_target = (global_point.position.y * scalar) - hor_chunk_radius * ALT_CHUNK_LEN;
        int64_t y_pos_target = (global_point.position.y * scalar) + hor_chunk_radius * ALT_CHUNK_LEN;
        int64_t z_neg_target = (global_point.position.z * scalar) - vert_chunk_radius * ALT_CHUNK_LEN;
        int64_t z_pos_target = (global_point.position.z * scalar) + vert_chunk_radius * ALT_CHUNK_LEN;
        std::vector<Chunk_t*> chunk_ptrs;
        for(int64_t x = x_neg_target; x <= x_pos_target; x += ALT_CHUNK_LEN){
            for(int64_t y = y_neg_target; y <= y_pos_target; y += ALT_CHUNK_LEN){
                for(int64_t z = z_neg_target; z <= z_pos_target; z += ALT_CHUNK_LEN){
                    Chunk_t* chunk = dvg_chunk_hash_table_lookup(graph, x, y, z);
                    if(chunk != NULL){
                        chunk_ptrs.push_back(chunk);
                    }
                }
            }
        }
        for(uint32_t i = 0; i < chunk_ptrs.size(); i++){
            thread* t = new thread(&DvgSlam::build_mesh_section_global, this, i, chunk_ptrs.at(i), &chunk_local_meshes, false);
            active_threads.push_back(t);
        }
        for(uint32_t i = 0; i < active_threads.size(); i++){
            active_threads[i]->join();
        }      
        publish_meshes(&chunk_local_meshes, &vertex_normals, pub);
        return;

    }
    static void build_mesh_section_global(DvgSlam* self, uint32_t vect_index,
            Chunk_t* chunk, std::vector<ChunkMesh_t>* output, bool wavefront){
        //for(uint32_t i = first_index; i <= last_index; i++){
            //output->at(i) = self->genChunkMesh(&(graph->chunks[i]));
        output->at(vect_index) = self->gen_chunk_mesh_with_greedy_mesher(self->graph, chunk, wavefront);
        //}
        return;
    }
    void build_and_publish_mesh(Dvg_t* graph, rclcpp::Publisher<mesh_msgs::msg::MeshGeometryStamped>::SharedPtr pub){
        std::vector<OutVertex_t> vertex_normals;
        OutVertex_t up_normal = {0.0f, 0.0f, 1.0f};
        vertex_normals.push_back(up_normal);
        OutVertex_t down_normal = {0.0f, 0.0f, -1.0f};
        vertex_normals.push_back(down_normal);
        OutVertex_t left_normal = {0.0f, 1.0f, 0.0f};
        vertex_normals.push_back(left_normal);
        OutVertex_t right_normal = {0.0f, -1.0f, 0.0f};
        vertex_normals.push_back(right_normal);
        OutVertex_t foward_normal = {1.0f, 0.0f, 0.0f};
        vertex_normals.push_back(foward_normal);
        OutVertex_t back_normal = {-1.0f, 0.0f, 0.0f};
        vertex_normals.push_back(back_normal);
        std::vector<ChunkMesh_t> chunk_local_meshes(graph->current_chunk_index);
        std::vector<thread*> active_threads;
        //uint32_t threadCount = std::thread::hardware_concurrency();
        //uint32_t chunksPerThread = graph->current_chunk_index / threadCount;
        //uint32_t currentIndex = 0;
        /*
        for(uint32_t i = 0; i < threadCount; i++)
        {
            thread* t = new thread(&HtMeshMapper::buildMeshSectionGlobal,this, currentIndex, currentIndex + (chunksPerThread - 1), graph, &chunk_local_meshes);
            activeThreads.push_back(t);
            currentIndex += chunksPerThread;
        }
        for(uint32_t i = 0; i < threadCount; i++)
        {
            threads[i]->join();
            delete threads[i];
            threads[i] = NULL;
        }
        */
        
        //if(currentIndex < graph->current_chunk_index)
        //{
        std::vector<Chunk_t*> chunk_ptrs;
        for(uint32_t i = 0; i < graph->chunk_hash_table_size; i++){
            if(graph->chunk_hash_table[i] != NULL){
                chunk_ptrs.push_back(graph->chunk_hash_table[i]);
            }
        }
        for(uint32_t i = 0; i < chunk_ptrs.size(); i++){
            thread* t = new thread(&DvgSlam::build_mesh_section_global, this, i, chunk_ptrs.at(i), &chunk_local_meshes, false);
            active_threads.push_back(t); 
        }
        for(uint32_t i = 0; i < active_threads.size(); i++){
            active_threads[i]->join();
        }
                       
        publish_meshes(&chunk_local_meshes, &vertex_normals, pub);
        return;
    }
    void publish_meshes(std::vector<ChunkMesh_t>* meshes,
            std::vector<OutVertex_t>* vertex_normals,
            rclcpp::Publisher<mesh_msgs::msg::MeshGeometryStamped>::SharedPtr pub){
        mesh_msgs::msg::MeshGeometryStamped msg;
        msg.header.frame_id = "map";
        RCLCPP_INFO(this->get_logger(), "publishing mesh with the tf frame %s", msg.header.frame_id.c_str());
        uint64_t vertex_offset = 0;
        for(uint64_t i = 0; i < meshes->size(); i++){
            for(uint64_t j = 0; j < meshes->at(i).vertices.size(); j++){
                geometry_msgs::msg::Point vertex;
                geometry_msgs::msg::Point vertex_normal;
                vertex.x = meshes->at(i).vertices.at(j).x;
                vertex.y = meshes->at(i).vertices.at(j).y;
                vertex.z = meshes->at(i).vertices.at(j).z;
                msg.mesh_geometry.vertices.push_back(vertex);
                vertex_normal.x = meshes->at(i).vertex_normals.at(j).x;
                vertex_normal.y = meshes->at(i).vertex_normals.at(j).y;
                vertex_normal.z = meshes->at(i).vertex_normals.at(j).z;
                msg.mesh_geometry.vertex_normals.push_back(vertex_normal);
            }
            for(uint64_t j = 0; j < meshes->at(i).faces.size(); j++)
            {
                meshes->at(i).faces.at(j).vertex_index1 += vertex_offset;
                meshes->at(i).faces.at(j).vertex_index2 += vertex_offset;
                meshes->at(i).faces.at(j).vertex_index3 += vertex_offset;
            }

            vertex_offset += meshes->at(i).vertices.size();
        }
        /*
        for(uint64_t i = 0; i < normals->size(); i++)
        {
            
        }
        */
        for(uint64_t i = 0; i < meshes->size(); i++){
            for(uint64_t j = 0; j < meshes->at(i).faces.size(); j++){
                mesh_msgs::msg::MeshTriangleIndices face;
                face.vertex_indices[0] = meshes->at(i).faces.at(j).vertex_index1;
                face.vertex_indices[1] = meshes->at(i).faces.at(j).vertex_index2;
                face.vertex_indices[2] = meshes->at(i).faces.at(j).vertex_index3;
                msg.mesh_geometry.faces.push_back(face);
            }
        }
        RCLCPP_INFO(this->get_logger(), "the global mesh contains %ld vertices", msg.mesh_geometry.vertices.size());
        pub->publish(msg);
    
    }

    void write_global_wavefront_v2(Dvg_t* graph){
        io_mutex.lock();
        ChunkMesh_t output;
        std::vector<InFace_t> in_faces;
        std::vector<InVertex_t> in_vertices;
        std::vector<int32_t> vertex_hash_table(1 << 22, -1);

        std::vector<OutVertex_t> vertex_normals;
        OutVertex_t up_normal = {0.0f, 0.0f, 1.0f};
        vertex_normals.push_back(up_normal);
        OutVertex_t down_normal = {0.0f, 0.0f, -1.0f};
        vertex_normals.push_back(down_normal);
        OutVertex_t left_normal = {0.0f, 1.0f, 0.0f};
        vertex_normals.push_back(left_normal);
        OutVertex_t right_normal = {0.0f, -1.0f, 0.0f};
        vertex_normals.push_back(right_normal);
        OutVertex_t foward_normal = {1.0f, 0.0f, 0.0f};
        vertex_normals.push_back(foward_normal);
        OutVertex_t back_normal = {-1.0f, 0.0f, 0.0f};
        vertex_normals.push_back(back_normal);
        for(uint32_t i = 0; i < graph->chunk_hash_table_size; i++){
            if(graph->chunk_hash_table[i] != NULL){
                v2_mesher_get_faces_and_verts(graph, graph->chunk_hash_table[i], &in_faces, &in_vertices, &vertex_hash_table, true);
            }
        }
        greedy_mesher_enter_vertices(&in_faces, &in_vertices, &vertex_hash_table);
        if(wavefront_greedy_mesher){
            reduce_faces(&in_faces, &in_vertices, &vertex_hash_table);
        }
        greedy_mesher_write_output_mesh(&output, &in_faces, &in_vertices);
        greedy_mesher_enter_vertex_normals(&output, &in_faces, &in_vertices);
        std::vector<ChunkMesh_t> chunk_local_meshes;
        chunk_local_meshes.push_back(output);
        write_mesh_file(&chunk_local_meshes, &vertex_normals);
        io_mutex.unlock();
    }

    void write_global_wavefront(Dvg_t* graph){
        io_mutex.lock();
        std::vector<OutVertex_t> vertex_normals;
        OutVertex_t up_normal = {0.0f, 0.0f, 1.0f};
        vertex_normals.push_back(up_normal);
        OutVertex_t down_normal = {0.0f, 0.0f, -1.0f};
        vertex_normals.push_back(down_normal);
        OutVertex_t left_normal = {0.0f, 1.0f, 0.0f};
        vertex_normals.push_back(left_normal);
        OutVertex_t right_normal = {0.0f, -1.0f, 0.0f};
        vertex_normals.push_back(right_normal);
        OutVertex_t foward_normal = {1.0f, 0.0f, 0.0f};
        vertex_normals.push_back(foward_normal);
        OutVertex_t back_normal = {-1.0f, 0.0f, 0.0f};
        vertex_normals.push_back(back_normal);
 
        std::vector<ChunkMesh_t> chunk_local_meshes;

        for(uint32_t i = 0; i < graph->chunk_hash_table_size; i++){
            if(graph->chunk_hash_table[i] != NULL){
                chunk_local_meshes.push_back(gen_chunk_mesh_with_greedy_mesher(graph, (graph->chunk_hash_table[i]),true));   
            }
        }
        write_mesh_file(&chunk_local_meshes, &vertex_normals);
        io_mutex.unlock();
    }
    void write_mesh_file(std::vector<ChunkMesh_t>* meshes, std::vector<OutVertex_t>* normals){
        RCLCPP_INFO(this->get_logger(), "exiting node: building the final mesh");
        std::string filepath = this->obj_filepath;
        fstream output_mesh(filepath + "/" + "OnlineMeshMap.obj", std::ios::out | std::ios::trunc);
        uint64_t vertex_offset = 0;
        for(uint64_t i = 0; i < meshes->size(); i++){
            for(uint64_t j = 0; j < meshes->at(i).vertices.size(); j++){
                std::string out_str = "";
                std::string x = std::to_string(meshes->at(i).vertices.at(j).x);
                std::string y = std::to_string(meshes->at(i).vertices.at(j).y);
                std::string z = std::to_string(meshes->at(i).vertices.at(j).z);
                x.erase ( x.find_last_not_of('0') + 1, std::string::npos );
                x.erase ( x.find_last_not_of('.') + 1, std::string::npos );
                y.erase ( y.find_last_not_of('0') + 1, std::string::npos );
                y.erase ( y.find_last_not_of('.') + 1, std::string::npos );
                z.erase ( z.find_last_not_of('0') + 1, std::string::npos );
                z.erase ( z.find_last_not_of('.') + 1, std::string::npos );

                out_str = "v " + x + " " + y + " " + z;
                output_mesh << out_str;
                output_mesh << "\n";
            }
            for(uint64_t j = 0; j < meshes->at(i).faces.size(); j++){
                meshes->at(i).faces.at(j).vertex_index1 += vertex_offset;
                meshes->at(i).faces.at(j).vertex_index2 += vertex_offset;
                meshes->at(i).faces.at(j).vertex_index3 += vertex_offset;
                if(meshes->at(i).faces.at(j).optional_quad_vertex != -1){
                    meshes->at(i).faces.at(j).optional_quad_vertex += vertex_offset;
                }
            }

            vertex_offset += meshes->at(i).vertices.size();
        }
        for(uint64_t i = 0; i < meshes->size(); i++){
            for(uint64_t j = 0; j < meshes->at(i).vertex_normals.size(); j++){
                std::string out_str = "";
                std::string x = std::to_string(meshes->at(i).vertex_normals.at(j).x);
                std::string y = std::to_string(meshes->at(i).vertex_normals.at(j).y);
                std::string z = std::to_string(meshes->at(i).vertex_normals.at(j).z);
                x.erase ( x.find_last_not_of('0') + 1, std::string::npos );
                x.erase ( x.find_last_not_of('.') + 1, std::string::npos );
                y.erase ( y.find_last_not_of('0') + 1, std::string::npos );
                y.erase ( y.find_last_not_of('.') + 1, std::string::npos );
                z.erase ( z.find_last_not_of('0') + 1, std::string::npos );
                z.erase ( z.find_last_not_of('.') + 1, std::string::npos );
                out_str = "vn "+ x + " " + y + " " + z;
                output_mesh << out_str;
                output_mesh << "\n";
            }
        }
        /*
        for(uint64_t i = 0; i < normals->size(); i++){
            std::string out_str = "";
            std::string x = std::to_string(normals->at(i).x);
            std::string y = std::to_string(normals->at(i).y);
            std::string z = std::to_string(normals->at(i).z);
            out_str = "vn "+ x + " " + y + " " + z;
            output_mesh << out_str;
            output_mesh << "\n";
        }*/
        for(uint64_t i = 0; i < meshes->size(); i++){
            for(uint64_t j = 0; j < meshes->at(i).faces.size(); j++){
                std::string out_str = "";
                std::string normal_index = to_string(meshes->at(i).faces.at(j).vertex_normal_index);
                std::string out_vertex_index1 = to_string(meshes->at(i).faces.at(j).vertex_index1 + 1);
                std::string out_vertex_index2 = to_string(meshes->at(i).faces.at(j).vertex_index2 + 1);
                std::string out_vertex_index3 = to_string(meshes->at(i).faces.at(j).vertex_index3 + 1);
                std::string out_vertex_index4 = "";
                if(meshes->at(i).faces.at(j).optional_quad_vertex != -1){
                    out_vertex_index4 = to_string(meshes->at(i).faces.at(j).optional_quad_vertex + 1);
                }
                std::string index1 = out_vertex_index1 + "//" + out_vertex_index1;
                std::string index2 = out_vertex_index2 + "//" + out_vertex_index2;
                std::string index3 = out_vertex_index3 + "//" + out_vertex_index3;
                std::string index4 = out_vertex_index4 + "//" + out_vertex_index4;
                if(out_vertex_index4 == ""){
                    out_str = "f " + index3 + " " + index2 + " " + index1;
                }
                else{
                    out_str = "f " + index4 + " " + index3 + " " + index2 + " " + index1;
                }
                output_mesh << out_str;
                output_mesh << "\n";

            }
        }
    }
    void timer_callback(){
        return;
        io_mutex.lock();
        RCLCPP_WARN(this->get_logger(), "generating mesh");
        if(graph->current_chunk_index == 0)
        {
            io_mutex.unlock();
            return;
        }
        const auto start = std::chrono::steady_clock::now();
        //buildAndPublishMesh(graph, publisher_);
        if(v2_mesher){
            build_and_publish_regional_mesh_v2(graph, render_distance_horizontal, render_distance_vertical, publisher_);
        }
        else{ 
            build_and_publish_regional_mesh(graph, render_distance_horizontal, render_distance_vertical, publisher_);
        }
        const auto end = std::chrono::steady_clock::now();
        auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        RCLCPP_INFO(this->get_logger(), "generating the mesh took %ld ms", diff.count());
        io_mutex.unlock();
    }
    
    void point_cloud_in_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::unique_lock<std::mutex> lock(io_mutex);

        const auto pose_start = std::chrono::steady_clock::now();

        auto time_since_last_processed_pointcloud =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                pose_start - last_processed_pointcloud_msg);

        if (time_since_last_processed_pointcloud < std::chrono::milliseconds{50}) {
            return;
        }

        auto deduplicate_scaled_cloud =
            [&](const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud)
                -> pcl::PointCloud<pcl::PointXYZ>::Ptr
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr dedup_cloud(
                new pcl::PointCloud<pcl::PointXYZ>);

            VoxelHashMap_t* hashmap = voxel_hash_map_init(1 << 17, 30, 0.5f);

            if (!hashmap) {
                RCLCPP_ERROR(this->get_logger(), "failed to allocate voxel hash map for de-duplication");
                return dedup_cloud;
            }

            for (const auto& p : input_cloud->points) {
                // Important: input_cloud is already transformed and scaled.
                // Do NOT multiply by scalar again here.
                int64_t x = static_cast<int64_t>(p.x);
                int64_t y = static_cast<int64_t>(p.y);
                int64_t z = static_cast<int64_t>(p.z);

                if (voxel_hash_map_lookup(hashmap, x, y, z)) {
                    continue;
                }

                voxel_hash_map_insert(hashmap, x, y, z);
                dedup_cloud->push_back(p);
            }

            voxel_hash_map_free(hashmap);

            return dedup_cloud;
        };

        geometry_msgs::msg::TransformStamped tf_sensor_to_base;

        try {
            tf_sensor_to_base = tf_buffer_->lookupTransform(
                "base_link",
                msg->header.frame_id,
                msg->header.stamp,
                rclcpp::Duration::from_seconds(0.1));
        }
        catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
            return;
        }

        Eigen::Affine3f sensor_to_base =
            tf2::transformToEigen(tf_sensor_to_base.transform).cast<float>();

        Eigen::Quaternionf q(
            global_point.orientation.w,
            global_point.orientation.x,
            global_point.orientation.y,
            global_point.orientation.z);

        q.normalize();

        Eigen::Vector3f t(
            global_point.position.x,
            global_point.position.y,
            global_point.position.z);

        Eigen::Affine3f base_to_pose = Eigen::Affine3f::Identity();
        base_to_pose = Eigen::Translation3f(t) * q;

        Eigen::Affine3f combined_transform = base_to_pose * sensor_to_base;

        pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud(
            new pcl::PointCloud<pcl::PointXYZ>);

        pcl::fromROSMsg(*msg, *raw_cloud);

        std::vector<int> nan_indices;
        pcl::removeNaNFromPointCloud(*raw_cloud, *raw_cloud, nan_indices);

        if (raw_cloud->empty()) {
            RCLCPP_WARN(this->get_logger(), "pointcloud empty after NaN removal!");
            return;
        }

        // Full-density scan for ICP.
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(
            new pcl::PointCloud<pcl::PointXYZ>);

        pcl::transformPointCloud(*raw_cloud, *transformed_cloud, combined_transform);

        // Convert transformed cloud from meters into scaled map / voxel units.
        for (auto& p : transformed_cloud->points) {
            p.x = p.x * static_cast<float>(scalar);
            p.y = p.y * static_cast<float>(scalar);
            p.z = p.z * static_cast<float>(scalar);
        }

        if (transformed_cloud->empty()) {
            RCLCPP_WARN(this->get_logger(), "transformed pointcloud empty!");
            return;
        }

        // First scan:
        // - seed previous_full_scan with the full-density transformed scan
        // - seed the DVG map with a deduplicated version
        if (!previous_full_scan || previous_full_scan->empty() || first_call_pc_in) {
            first_call_pc_in = false;

            previous_full_scan = pcl::PointCloud<pcl::PointXYZ>::Ptr(
                new pcl::PointCloud<pcl::PointXYZ>(*transformed_cloud));

            pcl::PointCloud<pcl::PointXYZ>::Ptr map_update_cloud =
                deduplicate_scaled_cloud(transformed_cloud);

            if (map_update_cloud->empty()) {
                RCLCPP_WARN(this->get_logger(), "pointcloud empty after post-transform de-duplication!");
                last_processed_pointcloud_msg = std::chrono::steady_clock::now();
                return;
            }

            for (const auto& p : map_update_cloud->points) {
                int64_t x_point = static_cast<int64_t>(p.x);
                int64_t y_point = static_cast<int64_t>(p.y);
                int64_t z_point = static_cast<int64_t>(p.z);

                dvg_insert(graph, x_point, y_point, z_point);
            }

            last_processed_pointcloud_msg = std::chrono::steady_clock::now();
            write_estimated_trajectory_row(msg);
            return;
        }

        pcl::PointCloud<pcl::PointXYZ> corrected_cloud;
        Eigen::Matrix4f corrective_transform = Eigen::Matrix4f::Identity();
        float fitness = 1000.0f;

        const auto icp_start = std::chrono::steady_clock::now();

        bool converged = run_icp(
            *transformed_cloud,
            *previous_full_scan,
            &corrected_cloud,
            &corrective_transform,
            &fitness);

        const auto icp_end = std::chrono::steady_clock::now();

        auto icp_diff =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                icp_end - icp_start);

        RCLCPP_INFO(
            this->get_logger(),
            "scan-to-scan ICP took %ld ms",
            static_cast<long>(icp_diff.count()));

        if (!converged) {
            RCLCPP_ERROR(this->get_logger(), "pose correction failed!");

            // Keep scan history moving forward.
            // Otherwise the next scan would match against an increasingly stale scan.
            previous_full_scan = pcl::PointCloud<pcl::PointXYZ>::Ptr(
                new pcl::PointCloud<pcl::PointXYZ>(*transformed_cloud));

            last_processed_pointcloud_msg = std::chrono::steady_clock::now();
            write_estimated_trajectory_row(msg);
            return;
        }

        float correction_magnitude =
            corrective_transform.block<3, 1>(0, 3).norm();

        Eigen::Matrix3f R =
            corrective_transform.block<3, 3>(0, 0);

        Eigen::AngleAxisf angle_axis(R);
        float rotation_angle = std::abs(angle_axis.angle());

        if ((fitness > dynamic_map_entry_cap ||
            correction_magnitude > 1.0f * static_cast<float>(scalar) ||
            rotation_angle > 0.35f) && !icp_gate_bypass)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "ICP correction rejected: fitness=%f, translation=%f, rotation=%f",
                fitness,
                correction_magnitude,
                rotation_angle);

            dynamic_map_entry_cap = dynamic_map_entry_cap * 1.05f;

            // Same reason as above: do not keep matching against a stale scan.
            previous_full_scan = pcl::PointCloud<pcl::PointXYZ>::Ptr(
                new pcl::PointCloud<pcl::PointXYZ>(*transformed_cloud));

            last_processed_pointcloud_msg = std::chrono::steady_clock::now();
            write_estimated_trajectory_row(msg);
            return;
        }

        dynamic_map_entry_cap = dynamic_map_entry_cap * 0.7f;

        // Apply ICP correction to the full-density transformed scan.
        // corrective_transform is in scaled map units.
        pcl::transformPointCloud(
            *transformed_cloud,
            *transformed_cloud,
            corrective_transform);

        // Cache corrected full-density scan as the next scan-to-scan ICP target.
        previous_full_scan = pcl::PointCloud<pcl::PointXYZ>::Ptr(
            new pcl::PointCloud<pcl::PointXYZ>(*transformed_cloud));

        // Build correction transform for robot pose.
        Eigen::Affine3f T_correction = Eigen::Affine3f::Identity();
        T_correction.matrix() = corrective_transform;

        // Convert correction translation from scaled map units back to meters.
        T_correction.translation() /= static_cast<float>(scalar);

        // Current robot pose in global/map coordinates.
        Eigen::Affine3f T_global = pose_to_eigen(global_point);

        // Apply ICP correction.
        // This matches the cloud correction direction:
        // corrected_cloud = corrective_transform * transformed_cloud
        T_global = T_correction * T_global;

        // Write corrected pose back into global_point.
        eigen_to_pose(T_global, global_point);

        int64_t robot_point_x =
            static_cast<int64_t>(global_point.position.x * static_cast<float>(scalar));
        int64_t robot_point_y =
            static_cast<int64_t>(global_point.position.y * static_cast<float>(scalar));
        int64_t robot_point_z =
            static_cast<int64_t>(global_point.position.z * static_cast<float>(scalar));

        const auto pose_end = std::chrono::steady_clock::now();

        auto pose_diff =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                pose_end - pose_start);

        RCLCPP_INFO(
            this->get_logger(),
            "pointcloud pose pipeline took %ld ms",
            static_cast<long>(pose_diff.count()));

        last_processed_pointcloud_msg = pose_end;

        // Only now de-duplicate for DVG map update.
        // ICP already used the full-density scan.
        pcl::PointCloud<pcl::PointXYZ>::Ptr map_update_cloud =
            deduplicate_scaled_cloud(transformed_cloud);

        if (map_update_cloud->empty()) {
            RCLCPP_WARN(this->get_logger(), "pointcloud empty after post-ICP de-duplication!");
            return;
        }

        const auto pc_start = std::chrono::steady_clock::now();

        for (const auto& p : map_update_cloud->points) {
            int64_t x_point = static_cast<int64_t>(p.x);
            int64_t y_point = static_cast<int64_t>(p.y);
            int64_t z_point = static_cast<int64_t>(p.z);

            dynamic_object_removal(
                graph,
                robot_point_x,
                robot_point_y,
                robot_point_z,
                x_point,
                y_point,
                z_point,
                true,
                5,
                scalar);
        }

        for (const auto& p : map_update_cloud->points) {
            int64_t x_point = static_cast<int64_t>(p.x);
            int64_t y_point = static_cast<int64_t>(p.y);
            int64_t z_point = static_cast<int64_t>(p.z);

            dvg_insert(graph, x_point, y_point, z_point);
        }

        const auto pc_end = std::chrono::steady_clock::now();

        auto pc_diff =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                pc_end - pc_start);

        RCLCPP_INFO(
            this->get_logger(),
            "entering the pointcloud took %ld ms",
            static_cast<long>(pc_diff.count()));
            write_estimated_trajectory_row(msg);
    }

    Eigen::Affine3f pose_to_eigen(const geometry_msgs::msg::Pose& pose)
    {
        Eigen::Quaternionf q(
            pose.orientation.w,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z
        );
        q.normalize();

        Eigen::Vector3f t(
            pose.position.x,
            pose.position.y,
            pose.position.z
        );
        
        return Eigen::Translation3f(t) * q;
    }

    void eigen_to_pose(const Eigen::Affine3f& tf, geometry_msgs::msg::Pose& pose){
        Eigen::Quaternionf q(tf.rotation());
        q.normalize();

        pose.position.x = tf.translation().x();
        pose.position.y = tf.translation().y();
        pose.position.z = tf.translation().z();

        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
        std::lock_guard<std::mutex> lock(io_mutex);

        geometry_msgs::msg::Pose incoming_pose = msg->pose.pose;

        if(!has_odom){
            prev_odom_msg_pose = incoming_pose;
            current_odom_msg_pose = incoming_pose;
            has_odom = true;
            return;
        }

        prev_odom_msg_pose = current_odom_msg_pose;
        current_odom_msg_pose = incoming_pose;

        Eigen::Affine3f T_odom_prev = pose_to_eigen(prev_odom_msg_pose);
        Eigen::Affine3f T_odom_current = pose_to_eigen(current_odom_msg_pose);

        // Relative motion from previous odom pose to current odom pose.
        Eigen::Affine3f T_delta = T_odom_prev.inverse() * T_odom_current;

        // Current corrected global/map pose.
        Eigen::Affine3f T_global = pose_to_eigen(global_point);

        // Apply odometry increment in the robot/local trajectory frame.
        T_global = T_global * T_delta;

        eigen_to_pose(T_global, global_point);
    }
    pcl::PointCloud<pcl::PointXYZ> get_local_pointcloud(geometry_msgs::msg::Pose map_pose, double radius){
        pcl::PointCloud<pcl::PointXYZ> output;
        int64_t converted_radius = radius * (float)scalar;
        int64_t hor_chunk_radius = converted_radius / ALT_CHUNK_LEN;
        int64_t vert_chunk_radius = converted_radius / ALT_CHUNK_LEN;
        int64_t x_neg_target = (map_pose.position.x * scalar) - hor_chunk_radius * ALT_CHUNK_LEN;
        int64_t x_pos_target = (map_pose.position.x * scalar) + hor_chunk_radius * ALT_CHUNK_LEN;
        int64_t y_neg_target = (map_pose.position.y * scalar) - hor_chunk_radius * ALT_CHUNK_LEN;
        int64_t y_pos_target = (map_pose.position.y * scalar) + hor_chunk_radius * ALT_CHUNK_LEN;
        int64_t z_neg_target = (map_pose.position.z * scalar) - vert_chunk_radius * ALT_CHUNK_LEN;
        int64_t z_pos_target = (map_pose.position.z * scalar) + vert_chunk_radius * ALT_CHUNK_LEN;
        std::vector<Chunk_t*> chunk_ptrs;
        for(int64_t x = x_neg_target; x <= x_pos_target; x += ALT_CHUNK_LEN){
            for(int64_t y = y_neg_target; y <= y_pos_target; y += ALT_CHUNK_LEN){
                for(int64_t z = z_neg_target; z <= z_pos_target; z += ALT_CHUNK_LEN){
                    Chunk_t* chunk = dvg_chunk_hash_table_lookup(graph, x, y, z);
                    if(chunk != NULL){
                        chunk_ptrs.push_back(chunk);
                    }
                }
            }
        }
        if(chunk_ptrs.empty()){
            return output;
        }
        for(int64_t cnt = 0; cnt < chunk_ptrs.size(); cnt++){
            add_to_pcl_cloud(&output, chunk_ptrs.at(cnt));
        }
        return output;
    }
    void add_to_pcl_cloud(pcl::PointCloud<pcl::PointXYZ>* input, Chunk_t* chunk){
        int64_t base_x = chunk->x_offset;
        int64_t base_y = chunk->y_offset;
        int64_t base_z = chunk->z_offset;
        for(int64_t moving_x = 0; moving_x < 16; moving_x++){
            for(int64_t moving_y = 0; moving_y < 16; moving_y++){
                for(int64_t moving_z = 0; moving_z < 16; moving_z++){
                    int64_t total_x = base_x + moving_x;
                    int64_t total_y = base_y + moving_y;
                    int64_t total_z = base_z + moving_z;
                    if(chunk_lookup(chunk, total_x, total_y, total_z) < 2){
                        continue;
                    }
                    else{
                        input->push_back(pcl::PointXYZ(total_x, total_y, total_z));
                    }
                }
            }
        }

    }
    // Scan-to-scan ICP: aligns the current full-resolution scan
    // (current_scan) against the previous full-resolution scan
    // (previous_scan). Parameters are tuned for frame-to-frame registration
    // rather than scan-to-map registration: consecutive scans are captured
    // milliseconds apart, so the expected inter-scan displacement is much
    // smaller than the displacement between a scan and the accumulated map,
    // and both clouds are full-resolution (denser, drawn from the same
    // sensor model), so correspondences are expected to be tighter and more
    // numerous.
    bool run_icp(pcl::PointCloud<pcl::PointXYZ>& current_scan,
            pcl::PointCloud<pcl::PointXYZ>& previous_scan,
            pcl::PointCloud<pcl::PointXYZ>* corrected_cloud_out,
            Eigen::Matrix4f* transform_out,
            float *fitness_out){
        pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>(current_scan));
        pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>(previous_scan));
        
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        // Tighter correspondence distance than scan-to-map: frame-to-frame
        // motion is small, and a tight radius avoids spurious matches
        // between full-resolution clouds that are otherwise very similar.
        icp.setMaxCorrespondenceDistance((float) scalar * icp_max_correspondence_m);
        icp.setTransformationEpsilon(icp_trans_epsilon);//stop iterating when transform delta below
        icp.setEuclideanFitnessEpsilon(icp_fitness_epsilon);//stop iterating when mean squared error below
        // Full-resolution scan-to-scan correspondences converge faster and
        // more reliably than scan-to-map with a downsampled source, so fewer
        // iterations are needed; this also keeps per-frame cost down given
        // both clouds are now full resolution.
        icp.setMaximumIterations(icp_max_iterations);
        icp.setInputSource(source);
        icp.setInputTarget(target);
        /*
        Eigen::Quaternionf q(
                global_point.orientation.w,
                global_point.orientation.x,
                global_point.orientation.y,
                global_point.orientation.z);
        Eigen::Translation3f trans(global_point.position.x * scalar, global_point.position.y * scalar, global_point.position.z * scalar);
        Eigen::Affine3f init_affine = trans * q;
        Eigen::Matrix4f initial_guess = init_affine.matrix();
*/
        pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>);
        icp.align(*aligned, Eigen::Matrix4f::Identity());
        
        if(!icp.hasConverged()){
            RCLCPP_ERROR(this->get_logger(), "ICP did not converge!");
            return false;
        }

        double fitness = icp.getFitnessScore((float)scalar * 1.0f);
        double fitness_threshold = (double) scalar * 0.2;
        *fitness_out = fitness;
        /*
        if(fitness > fitness_threshold){
            RCLCPP_ERROR(this->get_logger(), "ICP fitness score is too bad: %f", fitness);
            return false;
        }
        */
        *corrected_cloud_out = *aligned;
        *transform_out = icp.getFinalTransformation();
        
        float tx = (*transform_out)(0,3);
        float ty = (*transform_out)(1,3);
        float tz = (*transform_out)(2,3);
        float translation = std::sqrt(tx * tx + ty * ty + tz * tz);
        
        Eigen::Matrix3f rot = (*transform_out).block<3,3>(0,0);
        Eigen::AngleAxisf angle_axis(rot);
        float rotation_angle = std::abs(angle_axis.angle());
        
        // Tighter plausibility bounds than the old scan-to-map gate: a
        // single frame-to-frame step should never approach the old
        // scan-to-map correction ceiling. If it does, it's far more likely
        // to be a bad ICP alignment than a genuine one-frame displacement.
        float max_translation = 0.3f * (float) scalar;
        float max_rotation = 0.2; //radians

        if((translation > max_translation || rotation_angle > max_rotation) && !this->icp_gate_bypass){
            RCLCPP_ERROR(this->get_logger(), "ICP correction implausible, rejecting");
            return false;
        }
                
        return true;
    }
    std::mutex nav_mutex;
    static void nav_callback(DvgSlam* self,
        const std::shared_ptr<dvg_slam::srv::GetPath::Request> request,
        const std::shared_ptr<dvg_slam::srv::GetPath::Response> response){
        int64_t x = request->x * (float) self->scalar;
        int64_t y = request->y * (float) self->scalar;
        int64_t z = request->z * (float) self->scalar;
        int64_t downsample_factor = request->downsample_factor;
        std::thread t1(DvgSlam::do_astar_pathfinding, self, x, y, z, downsample_factor, response);
        //io_mutex.lock();
        //do_astar_pathfinding(this);
        //io_mutex.unlock();
        t1.detach();
    }
    static void do_astar_pathfinding(DvgSlam* self, int64_t goal_x, int64_t goal_y, int64_t goal_z, int64_t downsample_factor,
         const std::shared_ptr<dvg_slam::srv::GetPath::Response> response){
        bool mutex_state = self->nav_mutex.try_lock();
        if(!mutex_state){
            return;
        }
        bool path_find_succeeded = false;
        RCLCPP_INFO(self->get_logger(), "entering the pathfinding function");
        //int64_t downsample_factor = downsample_factor;
        if(!downsample_factor){
            downsample_factor = 1;
        }
        static const float neighbor_costs[4]{0.0f, 1.0f, 1.41421356f, 1.73205081f};
        int64_t horizontal_clearance = 0.7 * (float) self->scalar;
        int64_t vertical_clearance = 1.2 * (float) self->scalar;
        dvg_build_inflation(self->graph, horizontal_clearance, vertical_clearance);
        VoxelHashMap_t* nodes = voxel_hash_map_init(1<<17, 64, 0.5);
        VoxelPriorityQueue_t* prio_queue = voxel_priority_queue_init(1<<17);
        int64_t starting_x = self->global_point.position.x * (float) self->scalar;
        int64_t starting_y = self->global_point.position.y * (float) self->scalar;
        int64_t starting_z = self->global_point.position.z * (float) self->scalar;
        //int64_t goal_x = goal_x;
        //int64_t goal_y = goal_y;
        //int64_t goal_z = goal_z;
        starting_z += vertical_clearance / 2 + 1;
        goal_z += vertical_clearance / 2 + 1;
        if(starting_x == goal_x && starting_y == goal_y && starting_z == goal_z){
            voxel_hash_map_free(nodes);
            voxel_priority_queue_free(prio_queue);
            //response->success = false;
            return;
        }
        
        if(dvg_lookup_inflation(self->graph, starting_x, starting_y, starting_z)){
            voxel_hash_map_free(nodes);
            voxel_priority_queue_free(prio_queue);
            self->nav_mutex.unlock();
            //response->success = false;
            return;
        }
        if(dvg_lookup_inflation(self->graph, goal_x, goal_y, goal_z)){
            voxel_hash_map_free(nodes);
            voxel_priority_queue_free(prio_queue);
            self->nav_mutex.unlock();
            //response->success = false;
            return;
        }
        bool first_node = true;
        bool exit_con = false;
        RCLCPP_INFO(self->get_logger(), "starting pathfinding algorithm");
        Point_t last_point = {0,0,0};
        while(!exit_con){
            if(first_node){
                first_node = false;
                PointSlot_t* starting_slot = voxel_hash_map_insert(nodes, starting_x, starting_y, starting_z);
                starting_slot->traveled_dist = 0.0f;
                voxel_priority_queue_enqueue(prio_queue, starting_slot->key, nodes);
            }
            //RCLCPP_INFO(this->get_logger(), "get hugged");
            DequeueRetObject_t ret = voxel_priority_queue_dequeue(prio_queue, nodes);
            //RCLCPP_INFO(this->get_logger(), "idiot");
            if(!ret.valid){
                break;
            }
            Point_t current_key = ret.point;
            PointSlot_t* current_slot = voxel_hash_map_lookup(nodes, current_key.x, current_key.y, current_key.z);
            if(current_slot->visited){
                continue;
            }
            if(self->get_euclidean_dist(current_slot->key.x, current_slot->key.y, current_slot->key.z, goal_x, goal_y, goal_z) < 2.0f * (float) downsample_factor){
                RCLCPP_INFO(self->get_logger(), "goal point found!");
                path_find_succeeded = true;
                last_point.x = current_slot->key.x;
                last_point.y = current_slot->key.y;
                last_point.z = current_slot->key.z;
                exit_con = true;
                continue;
            }
            current_slot->visited = true;
            for(int64_t x = current_slot->key.x - downsample_factor; x <= current_slot->key.x + downsample_factor; x+=downsample_factor){
                for(int64_t y = current_slot->key.y - downsample_factor; y <= current_slot->key.y + downsample_factor; y+=downsample_factor){
                    for(int64_t z = current_slot->key.z - downsample_factor; z <= current_slot->key.z + downsample_factor; z+=downsample_factor){
                        if(!(x == current_slot->key.x && y == current_slot->key.y && z == current_slot->key.z) && 
                            !dvg_lookup_inflation(self->graph, x, y, z)){
                            PointSlot_t* next_point = voxel_hash_map_insert(nodes, x, y, z);
                            next_point->astar_heuristic = self->get_euclidean_dist(next_point->key.x, next_point->key.y, next_point->key.z, goal_x, goal_y, goal_z);
                            current_slot = voxel_hash_map_lookup(nodes, current_key.x, current_key.y, current_key.z);
                            int8_t dimension_diff = 0;
                            if(x != current_slot->key.x) dimension_diff++;
                            if(y != current_slot->key.y) dimension_diff++;
                            if(z != current_slot->key.z) dimension_diff++;
                            float dist_from_current = neighbor_costs[dimension_diff] * downsample_factor;
                            //RCLCPP_INFO(this->get_logger(), "pookiebear");
                            if(current_slot->traveled_dist + dist_from_current < next_point->traveled_dist){
                                next_point->traveled_dist = current_slot->traveled_dist + dist_from_current;
                                next_point->prev_key = current_slot->key;
                                next_point->has_prev = true;
                            }
                            //RCLCPP_INFO(this->get_logger(), "kitten");
                            if(!next_point->visited){
                                //RCLCPP_INFO(this->get_logger(), "UwU");
                                voxel_priority_queue_enqueue(prio_queue, next_point->key, nodes);
                                //RCLCPP_INFO(this->get_logger(), "meow");
                            }
                        }
                    }
                }
            }
        }
        std::vector<Point_t> path;
        RCLCPP_INFO(self->get_logger(), "assembling path");
        PointSlot_t* node = voxel_hash_map_lookup(nodes, last_point.x, last_point.y, last_point.z);
        while(node != NULL && node->has_prev){
            path.push_back(node->key);
            node = voxel_hash_map_lookup(nodes, node->prev_key.x, node->prev_key.y, node->prev_key.z);
        }
        std::reverse(path.begin(), path.end());
        RCLCPP_INFO(self->get_logger(), "building path message");
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = self->get_clock()->now();
        path_msg.header.frame_id = "odom";
        path_msg.poses.reserve(path.size());
        for(const auto& voxel : path){
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path_msg.header;
            pose.pose.position.x = static_cast<double>(voxel.x) / self->scalar;
            pose.pose.position.y = static_cast<double>(voxel.y) / self->scalar;
            pose.pose.position.z = static_cast<double>(voxel.z) / self->scalar;
            pose.pose.orientation.w = 1.0;
            path_msg.poses.push_back(pose);
        }
        RCLCPP_INFO(self->get_logger(), "publishing path on topic");
        self->path_publisher->publish(path_msg);
        RCLCPP_INFO(self->get_logger(), "writing service response");
        //response->path = path_msg;
        //response->success = path_find_succeeded;
        voxel_priority_queue_free(prio_queue);
        voxel_hash_map_free(nodes);
        self->nav_mutex.unlock();
    }

    void write_estimated_trajectory_row(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
        if (!trajectory_log_file.is_open()) {
            return;
        }

        trajectory_log_file
            << trajectory_scan_index << ","
            << rclcpp::Time(msg->header.stamp).seconds() << ","
            << global_point.position.x << ","
            << global_point.position.y << ","
            << global_point.position.z << ","
            << global_point.orientation.x << ","
            << global_point.orientation.y << ","
            << global_point.orientation.z << ","
            << global_point.orientation.w << "\n";

        trajectory_log_file.flush();

        trajectory_scan_index++;
    }


    bool has_odom = false;
    Dvg_t* graph;
    std::string topic;
    std::string del_topic;
    std::string frame_id = "odom";
    std::string odom_topic = "";
    std::string out_topic = "";
    std::string octomap_binary_topic = "";
    uint32_t chunk_amount;
    uint32_t scalar;
    std::string obj_filepath;
    rclcpp::TimerBase::SharedPtr mesh_timer;
    rclcpp::TimerBase::SharedPtr pose_timer;
    rclcpp::TimerBase::SharedPtr nav_timer;
    rclcpp::Publisher<mesh_msgs::msg::MeshGeometryStamped>::SharedPtr publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_two;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_debug_publisher;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_two; 
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_three;
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_binary_subscription;
    size_t count_;
    std::mutex io_mutex;
    geometry_msgs::msg::Pose global_point;
    geometry_msgs::msg::Pose current_odom_msg_pose;
    geometry_msgs::msg::Pose prev_odom_msg_pose;
    std::shared_ptr<rclcpp::Clock> clock_;
   std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    uint32_t render_distance_horizontal = 1;
    uint32_t render_distance_vertical = 1;
    int v2_mesher = 0;
    int ros2_msg_greedy_mesher = 0;
    int wavefront_greedy_mesher = 0;
    int raycast_enable = 0;
    int64_t prev_x = 0;
    int64_t prev_y = 0;
    int64_t prev_z = 0;
    float dynamic_map_entry_cap = 0.1;
    std::chrono::steady_clock::time_point last_processed_octomap_msg;
    std::chrono::steady_clock::time_point last_processed_pointcloud_msg;
    bool first_call_pc_in = true;
    rclcpp::Service<dvg_slam::srv::GetPath>::SharedPtr service;
    bool icp_gate_bypass;
    // Holds the previous full-resolution transformed scan for scan-to-scan ICP.
    pcl::PointCloud<pcl::PointXYZ>::Ptr previous_full_scan;
    std::ofstream trajectory_log_file;
    size_t trajectory_scan_index = 1;  // Schlachte uses scan001..scan019
    float icp_sample_ratio = 0.1f; //sample ratio as percentage of pointcloud size
    float icp_max_correspondence_m = 0.1f; //max correspondence range in meters
    int icp_max_iterations = 5;
    float icp_trans_epsilon = 1e-6;
    float icp_fitness_epsilon = 1e-5;
    double local_point_cloud_radius = 20.0;
};

int main(int argc, char ** argv)
{

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DvgSlam>());
    rclcpp::shutdown();
    
    return 0;


}
