#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
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
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/random_sample.h>
#include "../lib/VoxelGraph.h"
#include "mesh_msgs/msg/mesh_geometry_stamped.hpp"
// ROS / messages
#include "octomap_msgs/msg/octomap.hpp"

// OctoMap core
#include <octomap/octomap.h>
#include <octomap/OcTree.h>

// OctoMap-ROS helper conversions
#include <octomap_msgs/conversions.h>   // binaryMapToMsg / fullMsgToMap helpers
//#include <octomap_msgs/Octomap.h>      // (message header if you prefer)

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
    double x;
    double y;
    double z;
} DoubleVector_t;

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


class OnlineMeshMapper : public rclcpp::Node{
  public:
    OnlineMeshMapper()
    : Node("online_mesh_mapper"), count_(0),
      clock_(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME)),
      tf_buffer_(std::make_shared<tf2_ros::Buffer>(clock_)),
      tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)){   
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
            rclcpp::sleep_for(std::chrono::seconds(5));
        }
        RCLCPP_INFO(this->get_logger(), "initializing topics\n");
        publisher_ = this->create_publisher<mesh_msgs::msg::MeshGeometryStamped>(out_topic, 1);
        timer_ = this->create_wall_timer(
        1000ms, std::bind(&OnlineMeshMapper::timer_callback, this));
        if(topic != ""){
            subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                    topic , 1,
                    std::bind(&OnlineMeshMapper::point_cloud_in_callback, 
                    this, _1));
        }
        subscription_two = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic,
                1, std::bind(&OnlineMeshMapper::odom_callback, this, _1)); 
        if(del_topic != ""){
            subscription_three = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                    del_topic , 1,
                    std::bind(&OnlineMeshMapper::del_point_cloud_in_callback, 
                    this, _1));
        }
        
        if(octomap_binary_topic != "")
        {
            octomap_binary_subscription = this->create_subscription<octomap_msgs::msg::Octomap>(
                    octomap_binary_topic, 1,
                    std::bind(&OnlineMeshMapper::octomap_bin_callback,
                    this, _1));
        }
        publisher_two = this->create_publisher<nav_msgs::msg::Odometry>("debug_map_pose", 1);
        timer_two = this->create_wall_timer(
        10ms, std::bind(&OnlineMeshMapper::debug_map_pose_callback, this));
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
        RCLCPP_INFO(this->get_logger(), "starting the mapper\n");
        graph = voxel_graph_init(this->chunk_amount);
        RCLCPP_INFO(this->get_logger(), "node initialized\n");
    }
    ~OnlineMeshMapper(){
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
        voxel_graph_free(&graph);
    }
  private:
        void debug_map_pose_callback(){
            nav_msgs::msg::Odometry out_msg;
            out_msg.pose.pose = global_point;
            out_msg.child_frame_id = "base_link";
            out_msg.header.frame_id = "odom";
            publisher_two->publish(out_msg);
        }
        void raycast_inverse_delete(int64_t org_x, int64_t org_y,
            int64_t org_z, int64_t dest_x, int64_t dest_y, int64_t dest_z,
            std::vector<DelPoint_t>* points){
        DoubleVector_t diff_vect;
        diff_vect.x = dest_x - org_x;
        diff_vect.y = dest_y - org_y;
        diff_vect.z = dest_z - org_z;
        DoubleVector_t inverse_diff_vect;
        //RCLCPP_WARN(this->get_logger(), "org_vect is %ld %ld %ld \n", org_x, org_y, org_z);
        //RCLCPP_WARN(this->get_logger(), "dest_vect is %ld %ld %ld \n", dest_x, dest_y, dest_z);
        //RCLCPP_WARN(this->get_logger(), "diff_vect is %f %f %f \n", diff_vect.x, diff_vect.y, diff_vect.z);
        DoubleVector_t normal = double_vect_normalize(diff_vect);
        DoubleVector_t inverse_normal;
        inverse_normal.x = -normal.x;
        inverse_normal.y = -normal.y;
        inverse_normal.z = -normal.z;
        int64_t goal_x = dest_x;
        int64_t goal_y = dest_y;
        int64_t goal_z = dest_z;
        uint32_t counter = 1;
        bool exit_con = false;
        bool airgap_mode = false;
        uint16_t airgap_size = 0;
        uint16_t max_airgap = 0.3 * (float)scalar;
        while(!exit_con){
            if(voxel_graph_lookup(graph, goal_x, goal_y, goal_y)){
                if(!airgap_mode){
                    airgap_mode = true;
                    airgap_size = 1;
                }
                else{
                    airgap_size = 0;
                }
            }
            else if(airgap_mode){
                airgap_size++;
            }
            if(airgap_size >= max_airgap){
                exit_con = true;
            }
            goal_x = std::lround(dest_x + (inverse_normal.x * counter));
            goal_y = std::lround(dest_y + (inverse_normal.y * counter));
            goal_z = std::lround(dest_z + (inverse_normal.z * counter));
            counter++;
            if(goal_x == org_x && goal_y == org_y && goal_z == org_z)
            {
                exit_con == true;
                DelPoint_t point;
                point.x = dest_x;
                point.y = dest_y;
                point.z = dest_z;
                points->push_back(point);
                return;
            }
        }
        //RCLCPP_WARN(this->get_logger(), "normal_vect is %f %f %f \n", normal.x, normal.y, normal.z);
        diff_vect.x = goal_x - org_x;
        diff_vect.y = goal_y - org_y;
        diff_vect.z = goal_z - org_z;
        normal = double_vect_normalize(diff_vect);
        int64_t travel_x = org_x;
        int64_t travel_y = org_y;
        int64_t travel_z = org_z;
        counter = 1;
        while(get_manhattan_dist(travel_x, travel_y, travel_z, goal_x, goal_y, goal_z) > 1){
            if(voxel_graph_lookup(graph, travel_x, travel_y, travel_z)){
                if(travel_x != goal_x && travel_y != goal_y && travel_z != goal_z){
                    return;
                }
                /*
                voxel_graph_delete(graph, travel_x + 1, travel_y, travel_z);
                voxel_graph_delete(graph, travel_x - 1, travel_y, travel_z);
                voxel_graph_delete(graph, travel_x, travel_y + 1, travel_z);
                voxel_graph_delete(graph, travel_x, travel_y - 1, travel_z);
                voxel_graph_delete(graph, travel_x, travel_y, travel_z + 1);
                voxel_graph_delete(graph, travel_x, travel_y, travel_z - 1);

                voxel_graph_delete(graph, travel_x + 1, travel_y + 1, travel_z - 1);
                voxel_graph_delete(graph, travel_x + 1, travel_y + 1, travel_z);
                voxel_graph_delete(graph, travel_x + 1, travel_y + 1, travel_z);

                voxel_graph_delete(graph, travel_x + 1, travel_y - 1, travel_z - 1);
                voxel_graph_delete(graph, travel_x + 1, travel_y - 1, travel_z);
                voxel_graph_delete(graph, travel_x + 1, travel_y - 1, travel_z);

                voxel_graph_delete(graph, travel_x - 1, travel_y + 1, travel_z - 1);
                voxel_graph_delete(graph, travel_x - 1, travel_y + 1, travel_z);
                voxel_graph_delete(graph, travel_x - 1, travel_y + 1, travel_z);
                
                voxel_graph_delete(graph, travel_x - 1, travel_y - 1, travel_z - 1);
                voxel_graph_delete(graph, travel_x - 1, travel_y - 1, travel_z);
                voxel_graph_delete(graph, travel_x - 1, travel_y - 1, travel_z);
               */
            }
            //RCLCPP_WARN(this->get_logger(), "old_ray_vect is %ld %ld %ld \n", travel_x, travel_y, travel_z);
            counter += 1;
            travel_x = std::lround(org_x + (normal.x * counter));
            travel_y = std::lround(org_y + (normal.y * counter));
            travel_z = std::lround(org_z + (normal.z * counter));
            //RCLCPP_WARN(this->get_logger(), "new_ray_vect is %ld %ld %ld \n", travel_x, travel_y, travel_z);
        }
        //raycast_delete(goal_x, goal_y, goal_z, dest_x, dest_y, dest_z);
        DelPoint_t point;
        point.x = dest_x;
        point.y = dest_y;
        point.z = dest_z;
        points->push_back(point);
    }

    void raycast_delete(int64_t org_x, int64_t org_y, int64_t org_z,
                    int64_t dest_x, int64_t dest_y, int64_t dest_z, bool splash_delete){
        if(org_x == dest_x && org_y == dest_y && org_z == dest_z) return;

        DoubleVector_t diff_vect{(double)(dest_x-org_x), (double)(dest_y-org_y), (double)(dest_z-org_z)};
        double len2 = diff_vect.x*diff_vect.x + diff_vect.y*diff_vect.y + diff_vect.z*diff_vect.z;
        if(len2 < 1.0) return;

        DoubleVector_t normal = double_vect_normalize(diff_vect);
        int64_t travel_x = org_x, travel_y = org_y, travel_z = org_z;
        uint32_t counter = 1;
        uint32_t threshold = splash_delete ? 3 : 2;

        uint32_t max_iter = (uint32_t)std::ceil(std::sqrt(len2)) + 16;

        while(counter < max_iter &&
            get_manhattan_dist(travel_x, travel_y, travel_z, dest_x, dest_y, dest_z) > threshold){

        bool point_deleted = voxel_graph_delete(graph, travel_x, travel_y, travel_z);
        if(point_deleted && splash_delete){
        }
        counter++;
        travel_x = std::lround(org_x + normal.x * counter);
        travel_y = std::lround(org_y + normal.y * counter);
        travel_z = std::lround(org_z + normal.z * counter);
        }
    }

    int64_t get_manhattan_dist(int64_t org_x, int64_t org_y, int64_t org_z,
             int64_t dest_x, int64_t dest_y, int64_t dest_z)
    {
        return labs(org_x - dest_x) + labs(org_y - dest_y) + labs(org_z - dest_z) ;
    }
    DoubleVector_t double_vect_normalize(DoubleVector_t in){       
        DoubleVector_t out;
        double vector_len = sqrt(in.x * in.x + in.y * in.y + in.z * in.z);
        out.x = in.x / vector_len;
        out.y = in.y / vector_len;
        out.z = in.z / vector_len;
        return out;
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
    ChunkMesh_t gen_chunk_mesh_with_greedy_mesher(VoxelGraph_t* graph, AltChunk_t* chunk, bool wavefront){   
        ChunkMesh_t output;
        std::vector<InFace_t> in_faces;
        std::vector<InVertex_t> in_vertices;
        std::vector<int32_t> vertex_hash_table(1 << 17, -1);
        uint32_t current_vertex_index = 1;
        int64_t base_x = chunk->x_offset;
        int64_t base_y = chunk->y_offset;
        int64_t base_z = chunk->z_offset;
        AltChunk_t* upper_neighbor_chunk = voxel_graph_chunk_hash_table_lookup(graph, base_x, base_y, base_z + ALT_CHUNK_LEN);
        AltChunk_t* lower_neighbor_chunk = voxel_graph_chunk_hash_table_lookup(graph, base_x, base_y, base_z - ALT_CHUNK_LEN);
        AltChunk_t* left_neighbor_chunk = voxel_graph_chunk_hash_table_lookup(graph, base_x, base_y + ALT_CHUNK_LEN, base_z);
        AltChunk_t* right_neighbor_chunk = voxel_graph_chunk_hash_table_lookup(graph, base_x, base_y - ALT_CHUNK_LEN, base_z);
        AltChunk_t* foward_neighbor_chunk = voxel_graph_chunk_hash_table_lookup(graph, base_x + ALT_CHUNK_LEN, base_y, base_z);
        AltChunk_t* back_neighbor_chunk = voxel_graph_chunk_hash_table_lookup(graph, base_x - ALT_CHUNK_LEN, base_y, base_z);
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
                    if(!alt_chunk_lookup(chunk, total_x, total_y, total_z)){
                        continue;
                    }
                    //up
                    if(moving_z == ALT_CHUNK_LEN - 1){
                        if(upper_neighbor_chunk != NULL){
                            up_neighbor = alt_chunk_lookup(upper_neighbor_chunk, total_x, total_y, total_z + 1);
                        }
                    }
                    else{
                        up_neighbor = alt_chunk_lookup(chunk, total_x, total_y, total_z + 1);
                    }
                    //down
                    if(moving_z == 0){
                        if(lower_neighbor_chunk != NULL){
                            down_neighbor = alt_chunk_lookup(lower_neighbor_chunk, total_x, total_y, total_z - 1);
                        }
                    }
                    else{
                        down_neighbor = alt_chunk_lookup(chunk, total_x, total_y, total_z - 1);
                    }
                    //left
                    if(moving_y == ALT_CHUNK_LEN - 1){
                        if(left_neighbor_chunk != NULL){
                            left_neighbor = alt_chunk_lookup(left_neighbor_chunk, total_x, total_y + 1, total_z);
                        }
                    }
                    else{
                        left_neighbor = alt_chunk_lookup(chunk, total_x, total_y + 1, total_z);
                    }
                    //right
                    if(moving_y == 0){
                        if(right_neighbor_chunk != NULL){
                            right_neighbor = alt_chunk_lookup(right_neighbor_chunk, total_x, total_y - 1, total_z);
                        }
                    }
                    else{
                        right_neighbor = alt_chunk_lookup(chunk, total_x, total_y - 1, total_z);
                    }
                    //foward
                    if(moving_x == ALT_CHUNK_LEN - 1){
                        if(foward_neighbor_chunk != NULL){
                            foward_neighbor = alt_chunk_lookup(foward_neighbor_chunk, total_x + 1, total_y, total_z);
                        }
                    }
                    else{
                        foward_neighbor = alt_chunk_lookup(chunk, total_x + 1, total_y, total_z);
                    }
                    //back
                    if(moving_x == 0){
                        if(back_neighbor_chunk != NULL){
                            back_neighbor = alt_chunk_lookup(back_neighbor_chunk, total_x - 1, total_y, total_z);
                        }
                    }
                    else{
                        back_neighbor = alt_chunk_lookup(chunk, total_x - 1, total_y, total_z);
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

    void v2_mesher_get_faces_and_verts(VoxelGraph_t* graph,
            AltChunk_t* chunk,
            std::vector<InFace_t>* in_faces,
            std::vector<InVertex_t>* in_vertices, 
            std::vector<int32_t>* vertex_hash_table,
            bool wavefront){   
        uint32_t current_vertex_index = 1;
        int64_t base_x = chunk->x_offset;
        int64_t base_y = chunk->y_offset;
        int64_t base_z = chunk->z_offset;
        AltChunk_t* upper_neighbor_chunk = voxel_graph_chunk_hash_table_lookup(graph, base_x, base_y, base_z + ALT_CHUNK_LEN);
        AltChunk_t* lower_neighbor_chunk = voxel_graph_chunk_hash_table_lookup(graph, base_x, base_y, base_z - ALT_CHUNK_LEN);
        AltChunk_t* left_neighbor_chunk = voxel_graph_chunk_hash_table_lookup(graph, base_x, base_y + ALT_CHUNK_LEN, base_z);
        AltChunk_t* right_neighbor_chunk = voxel_graph_chunk_hash_table_lookup(graph, base_x, base_y - ALT_CHUNK_LEN, base_z);
        AltChunk_t* foward_neighbor_chunk = voxel_graph_chunk_hash_table_lookup(graph, base_x + ALT_CHUNK_LEN, base_y, base_z);
        AltChunk_t* back_neighbor_chunk = voxel_graph_chunk_hash_table_lookup(graph, base_x - ALT_CHUNK_LEN, base_y, base_z);
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
                    if(!alt_chunk_lookup(chunk, total_x, total_y, total_z)){
                        continue;
                    }
                    //up
                    if(moving_z == ALT_CHUNK_LEN - 1){
                        if(upper_neighbor_chunk != NULL){
                            up_neighbor = alt_chunk_lookup(upper_neighbor_chunk, total_x, total_y, total_z + 1);
                        }
                    }
                    else{
                        up_neighbor = alt_chunk_lookup(chunk, total_x, total_y, total_z + 1);
                    }
                    //down
                    if(moving_z == 0){
                        if(lower_neighbor_chunk != NULL){
                            down_neighbor = alt_chunk_lookup(lower_neighbor_chunk, total_x, total_y, total_z - 1);
                        }
                    }
                    else{
                        down_neighbor = alt_chunk_lookup(chunk, total_x, total_y, total_z - 1);
                    }
                    //left
                    if(moving_y == ALT_CHUNK_LEN - 1){
                        if(left_neighbor_chunk != NULL){
                            left_neighbor = alt_chunk_lookup(left_neighbor_chunk, total_x, total_y + 1, total_z);
                        }
                    }
                    else{
                        left_neighbor = alt_chunk_lookup(chunk, total_x, total_y + 1, total_z);
                    }
                    //right
                    if(moving_y == 0){
                        if(right_neighbor_chunk != NULL){
                            right_neighbor = alt_chunk_lookup(right_neighbor_chunk, total_x, total_y - 1, total_z);
                        }
                    }
                    else{
                        right_neighbor = alt_chunk_lookup(chunk, total_x, total_y - 1, total_z);
                    }
                    //foward
                    if(moving_x == ALT_CHUNK_LEN - 1){
                        if(foward_neighbor_chunk != NULL){
                            foward_neighbor = alt_chunk_lookup(foward_neighbor_chunk, total_x + 1, total_y, total_z);
                        }
                    }
                    else{
                        foward_neighbor = alt_chunk_lookup(chunk, total_x + 1, total_y, total_z);
                    }
                    //back
                    if(moving_x == 0){
                        if(back_neighbor_chunk != NULL){
                            back_neighbor = alt_chunk_lookup(back_neighbor_chunk, total_x - 1, total_y, total_z);
                        }
                    }
                    else{
                        back_neighbor = alt_chunk_lookup(chunk, total_x - 1, total_y, total_z);
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

    ChunkMesh_t gen_chunk_mesh(Chunk_t* chunk){   
        ChunkMesh_t output;
        uint32_t current_vertex_index = 0;
        for(uint32_t i = 0; i < chunk->current_node_index; i++){
            OutVertex_t v1;
            OutVertex_t v2;
            OutVertex_t v3;
            OutVertex_t v4;

            OutFace_t face;
            Vertex_t anchor_vertex = chunk->nodes[i].coord_and_mesh_info;
            if(!vertex_get_up_bit(&anchor_vertex)){   
                v1.x = (float)((uint32_t)vertex_pick_x_coord(anchor_vertex.vertex_coords) + chunk->x_offset) / scalar;
                v1.y = (float)((uint32_t)vertex_pick_y_coord(anchor_vertex.vertex_coords) + chunk->y_offset) / scalar;
                v1.z = (float)((uint32_t)vertex_pick_z_coord(anchor_vertex.vertex_coords) + chunk->z_offset) / scalar;
                
                v2.x = (float)((uint32_t)vertex_pick_x_coord(anchor_vertex.vertex_coords) + chunk->x_offset + 1) / scalar;
                v2.y = (float)((uint32_t)vertex_pick_y_coord(anchor_vertex.vertex_coords) + chunk->y_offset) / scalar;
                v2.z = (float)((uint32_t)vertex_pick_z_coord(anchor_vertex.vertex_coords) + chunk->z_offset) / scalar;

                v3.x = (float)((uint32_t)vertex_pick_x_coord(anchor_vertex.vertex_coords) + chunk->x_offset + 1) / scalar;
                v3.y = (float)((uint32_t)vertex_pick_y_coord(anchor_vertex.vertex_coords) + chunk->y_offset + 1) / scalar;
                v3.z = (float)((uint32_t)vertex_pick_z_coord(anchor_vertex.vertex_coords) + chunk->z_offset) / scalar;

                v4.x = (float)((uint32_t)vertex_pick_x_coord(anchor_vertex.vertex_coords) + chunk->x_offset) / scalar;
                v4.y = (float)((uint32_t)vertex_pick_y_coord(anchor_vertex.vertex_coords) + chunk->y_offset + 1) / scalar;
                v4.z = (float)((uint32_t)vertex_pick_z_coord(anchor_vertex.vertex_coords) + chunk->z_offset) / scalar;

                output.vertices.push_back(v1);
                output.vertices.push_back(v4);
                output.vertices.push_back(v3);
                output.vertices.push_back(v2);
                face.vertex_index1 = current_vertex_index;
                face.vertex_index2 = current_vertex_index + 1;
                face.vertex_index3 = current_vertex_index + 3;
                face.vertex_normal_index = 1;
                output.faces.push_back(face);

                face.vertex_index1 = current_vertex_index + 1;
                face.vertex_index2 = current_vertex_index + 2;
                face.vertex_index3 = current_vertex_index + 3;
                face.vertex_normal_index = 1;
                output.faces.push_back(face);

                current_vertex_index += 4;

            }
            if(!vertex_get_down_bit(&anchor_vertex)){
                v1.x = (float)((uint32_t)vertex_pick_x_coord(anchor_vertex.vertex_coords) + chunk->x_offset) / scalar;
                v1.y = (float)((uint32_t)vertex_pick_y_coord(anchor_vertex.vertex_coords) + chunk->y_offset) / scalar;
                v1.z = (float)((uint32_t)vertex_pick_z_coord(anchor_vertex.vertex_coords) + chunk->z_offset - 1) / scalar;
                
                v2.x = (float)((uint32_t)vertex_pick_x_coord(anchor_vertex.vertex_coords) + chunk->x_offset + 1) / scalar;
                v2.y = (float)((uint32_t)vertex_pick_y_coord(anchor_vertex.vertex_coords) + chunk->y_offset) / scalar;
                v2.z = (float)((uint32_t)vertex_pick_z_coord(anchor_vertex.vertex_coords) + chunk->z_offset - 1) / scalar;

                v3.x = (float)((uint32_t)vertex_pick_x_coord(anchor_vertex.vertex_coords) + chunk->x_offset + 1) / scalar;
                v3.y = (float)((uint32_t)vertex_pick_y_coord(anchor_vertex.vertex_coords) + chunk->y_offset + 1) / scalar;
                v3.z = (float)((uint32_t)vertex_pick_z_coord(anchor_vertex.vertex_coords) + chunk->z_offset - 1) / scalar;

                v4.x = (float)((uint32_t)vertex_pick_x_coord(anchor_vertex.vertex_coords) + chunk->x_offset) / scalar;
                v4.y = (float)((uint32_t)vertex_pick_y_coord(anchor_vertex.vertex_coords) + chunk->y_offset + 1) / scalar;
                v4.z = (float)((uint32_t)vertex_pick_z_coord(anchor_vertex.vertex_coords) + chunk->z_offset - 1) / scalar;
                
                output.vertices.push_back(v1);
                output.vertices.push_back(v2);
                output.vertices.push_back(v3);
                output.vertices.push_back(v4);

                face.vertex_index1 = current_vertex_index;
                face.vertex_index2 = current_vertex_index + 1;
                face.vertex_index3 = current_vertex_index + 3;
                face.vertex_normal_index = 2;
                output.faces.push_back(face);

                face.vertex_index1 = current_vertex_index + 1;
                face.vertex_index2 = current_vertex_index + 2;
                face.vertex_index3 = current_vertex_index + 3;
                face.vertex_normal_index = 2;
                output.faces.push_back(face);

                current_vertex_index += 4;

            }
            if(!vertex_get_left_bit(&anchor_vertex)){
                v1.x = (float)((uint32_t)vertex_pick_x_coord(anchor_vertex.vertex_coords) + chunk->x_offset) / scalar;
                v1.y = (float)((uint32_t)vertex_pick_y_coord(anchor_vertex.vertex_coords) + chunk->y_offset + 1) / scalar;
                v1.z = (float)((uint32_t)vertex_pick_z_coord(anchor_vertex.vertex_coords) + chunk->z_offset) / scalar;
                
                v2.x = (float)((uint32_t)vertex_pick_x_coord(anchor_vertex.vertex_coords) + chunk->x_offset + 1) / scalar;
                v2.y = (float)((uint32_t)vertex_pick_y_coord(anchor_vertex.vertex_coords) + chunk->y_offset + 1) / scalar;
                v2.z = (float)((uint32_t)vertex_pick_z_coord(anchor_vertex.vertex_coords) + chunk->z_offset) / scalar;

                v3.x = (float)((uint32_t)vertex_pick_x_coord(anchor_vertex.vertex_coords) + chunk->x_offset + 1) / scalar;
                v3.y = (float)((uint32_t)vertex_pick_y_coord(anchor_vertex.vertex_coords) + chunk->y_offset + 1) / scalar;
                v3.z = (float)((uint32_t)vertex_pick_z_coord(anchor_vertex.vertex_coords) + chunk->z_offset - 1) / scalar;

                v4.x = (float)((uint32_t)vertex_pick_x_coord(anchor_vertex.vertex_coords) + chunk->x_offset) / scalar;
                v4.y = (float)((uint32_t)vertex_pick_y_coord(anchor_vertex.vertex_coords) + chunk->y_offset + 1) / scalar;
                v4.z = (float)((uint32_t)vertex_pick_z_coord(anchor_vertex.vertex_coords) + chunk->z_offset - 1) / scalar;
                
                output.vertices.push_back(v1);
                output.vertices.push_back(v4);
                output.vertices.push_back(v3);
                output.vertices.push_back(v2);

                face.vertex_index1 = current_vertex_index;
                face.vertex_index2 = current_vertex_index + 1;
                face.vertex_index3 = current_vertex_index + 3;
                face.vertex_normal_index = 3;
                output.faces.push_back(face);

                face.vertex_index1 = current_vertex_index + 1;
                face.vertex_index2 = current_vertex_index + 2;
                face.vertex_index3 = current_vertex_index + 3;
                face.vertex_normal_index = 3;
                output.faces.push_back(face);


                current_vertex_index += 4;

            }
            if(!vertex_get_right_bit(&anchor_vertex)){
                v1.x = (float)((uint32_t)vertex_pick_x_coord(anchor_vertex.vertex_coords) + chunk->x_offset) / scalar;
                v1.y = (float)((uint32_t)vertex_pick_y_coord(anchor_vertex.vertex_coords) + chunk->y_offset) / scalar;
                v1.z = (float)((uint32_t)vertex_pick_z_coord(anchor_vertex.vertex_coords) + chunk->z_offset) / scalar;
                
                v2.x = (float)((uint32_t)vertex_pick_x_coord(anchor_vertex.vertex_coords) + chunk->x_offset + 1) / scalar;
                v2.y = (float)((uint32_t)vertex_pick_y_coord(anchor_vertex.vertex_coords) + chunk->y_offset) / scalar;
                v2.z = (float)((uint32_t)vertex_pick_z_coord(anchor_vertex.vertex_coords) + chunk->z_offset) / scalar;

                v3.x = (float)((uint32_t)vertex_pick_x_coord(anchor_vertex.vertex_coords) + chunk->x_offset + 1) / scalar;
                v3.y = (float)((uint32_t)vertex_pick_y_coord(anchor_vertex.vertex_coords) + chunk->y_offset) / scalar;
                v3.z = (float)((uint32_t)vertex_pick_z_coord(anchor_vertex.vertex_coords) + chunk->z_offset - 1) / scalar;

                v4.x = (float)((uint32_t)vertex_pick_x_coord(anchor_vertex.vertex_coords) + chunk->x_offset) / scalar;
                v4.y = (float)((uint32_t)vertex_pick_y_coord(anchor_vertex.vertex_coords) + chunk->y_offset) / scalar;
                v4.z = (float)((uint32_t)vertex_pick_z_coord(anchor_vertex.vertex_coords) + chunk->z_offset - 1) / scalar;
                
                output.vertices.push_back(v1);
                output.vertices.push_back(v2);
                output.vertices.push_back(v3);
                output.vertices.push_back(v4);

                face.vertex_index1 = current_vertex_index;
                face.vertex_index2 = current_vertex_index + 1;
                face.vertex_index3 = current_vertex_index + 3;
                face.vertex_normal_index = 4;
                output.faces.push_back(face);

                face.vertex_index1 = current_vertex_index + 1;
                face.vertex_index2 = current_vertex_index + 2;
                face.vertex_index3 = current_vertex_index + 3;
                face.vertex_normal_index = 4;
                output.faces.push_back(face);


                current_vertex_index += 4;

            }
            if(!vertex_get_foward_bit(&anchor_vertex)){
                v1.x = (float)((uint32_t)vertex_pick_x_coord(anchor_vertex.vertex_coords) + chunk->x_offset + 1) / scalar;
                v1.y = (float)((uint32_t)vertex_pick_y_coord(anchor_vertex.vertex_coords) + chunk->y_offset) / scalar;
                v1.z = (float)((uint32_t)vertex_pick_z_coord(anchor_vertex.vertex_coords) + chunk->z_offset) / scalar;
                
                v2.x = (float)((uint32_t)vertex_pick_x_coord(anchor_vertex.vertex_coords) + chunk->x_offset + 1) / scalar;
                v2.y = (float)((uint32_t)vertex_pick_y_coord(anchor_vertex.vertex_coords) + chunk->y_offset) / scalar;
                v2.z = (float)((uint32_t)vertex_pick_z_coord(anchor_vertex.vertex_coords) + chunk->z_offset - 1) / scalar;

                v3.x = (float)((uint32_t)vertex_pick_x_coord(anchor_vertex.vertex_coords) + chunk->x_offset + 1) / scalar;
                v3.y = (float)((uint32_t)vertex_pick_y_coord(anchor_vertex.vertex_coords) + chunk->y_offset + 1) / scalar;
                v3.z = (float)((uint32_t)vertex_pick_z_coord(anchor_vertex.vertex_coords) + chunk->z_offset - 1) / scalar;

                v4.x = (float)((uint32_t)vertex_pick_x_coord(anchor_vertex.vertex_coords) + chunk->x_offset + 1) / scalar;
                v4.y = (float)((uint32_t)vertex_pick_y_coord(anchor_vertex.vertex_coords) + chunk->y_offset + 1) / scalar;
                v4.z = (float)((uint32_t)vertex_pick_z_coord(anchor_vertex.vertex_coords) + chunk->z_offset) / scalar;
                
                output.vertices.push_back(v1);
                output.vertices.push_back(v4);
                output.vertices.push_back(v3);
                output.vertices.push_back(v2);

                face.vertex_index1 = current_vertex_index;
                face.vertex_index2 = current_vertex_index + 1;
                face.vertex_index3 = current_vertex_index + 3;
                face.vertex_normal_index = 5;
                output.faces.push_back(face);

                face.vertex_index1 = current_vertex_index + 1;
                face.vertex_index2 = current_vertex_index + 2;
                face.vertex_index3 = current_vertex_index + 3;
                face.vertex_normal_index = 5;
                output.faces.push_back(face);


                current_vertex_index += 4;

            }
            if(!vertex_get_back_bit(&anchor_vertex)){  
                v1.x = (float)((uint32_t)vertex_pick_x_coord(anchor_vertex.vertex_coords) + chunk->x_offset) / scalar;
                v1.y = (float)((uint32_t)vertex_pick_y_coord(anchor_vertex.vertex_coords) + chunk->y_offset) / scalar;
                v1.z = (float)((uint32_t)vertex_pick_z_coord(anchor_vertex.vertex_coords) + chunk->z_offset) / scalar;
                
                v2.x = (float)((uint32_t)vertex_pick_x_coord(anchor_vertex.vertex_coords) + chunk->x_offset) / scalar;
                v2.y = (float)((uint32_t)vertex_pick_y_coord(anchor_vertex.vertex_coords) + chunk->y_offset) / scalar;
                v2.z = (float)((uint32_t)vertex_pick_z_coord(anchor_vertex.vertex_coords) + chunk->z_offset - 1) / scalar;

                v3.x = (float)((uint32_t)vertex_pick_x_coord(anchor_vertex.vertex_coords) + chunk->x_offset) / scalar;
                v3.y = (float)((uint32_t)vertex_pick_y_coord(anchor_vertex.vertex_coords) + chunk->y_offset + 1) / scalar;
                v3.z = (float)((uint32_t)vertex_pick_z_coord(anchor_vertex.vertex_coords) + chunk->z_offset - 1) / scalar;

                v4.x = (float)((uint32_t)vertex_pick_x_coord(anchor_vertex.vertex_coords) + chunk->x_offset) / scalar;
                v4.y = (float)((uint32_t)vertex_pick_y_coord(anchor_vertex.vertex_coords) + chunk->y_offset + 1) / scalar;
                v4.z = (float)((uint32_t)vertex_pick_z_coord(anchor_vertex.vertex_coords) + chunk->z_offset) / scalar;
                
                output.vertices.push_back(v1);
                output.vertices.push_back(v2);
                output.vertices.push_back(v3);
                output.vertices.push_back(v4);

                face.vertex_index1 = current_vertex_index;
                face.vertex_index2 = current_vertex_index + 1;
                face.vertex_index3 = current_vertex_index + 3;
                face.vertex_normal_index = 6;
                output.faces.push_back(face);

                face.vertex_index1 = current_vertex_index + 1;
                face.vertex_index2 = current_vertex_index + 2;
                face.vertex_index3 = current_vertex_index + 3;
                face.vertex_normal_index = 6;
                output.faces.push_back(face);


                current_vertex_index += 4;

            }
        }
        //RCLCPP_INFO(this->get_logger(), "chunk construction done %ld faces are in the mesh", output.faces.size());
        return output;

    }
    void build_and_publish_mesh_v2(VoxelGraph_t* graph, rclcpp::Publisher<mesh_msgs::msg::MeshGeometryStamped>::SharedPtr pub){
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
    void build_and_publish_regional_mesh_v2(VoxelGraph_t* graph,
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
        std::vector<AltChunk_t*> chunk_ptrs;
        for(int64_t x = x_neg_target; x <= x_pos_target; x += ALT_CHUNK_LEN){
            for(int64_t y = y_neg_target; y <= y_pos_target; y += ALT_CHUNK_LEN){
                for(int64_t z = z_neg_target; z <= z_pos_target; z += ALT_CHUNK_LEN){
                    AltChunk_t* chunk = voxel_graph_chunk_hash_table_lookup(graph, x, y, z);
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

    void build_and_publish_regional_mesh(VoxelGraph_t* graph,
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
        std::vector<AltChunk_t*> chunk_ptrs;
        for(int64_t x = x_neg_target; x <= x_pos_target; x += ALT_CHUNK_LEN){
            for(int64_t y = y_neg_target; y <= y_pos_target; y += ALT_CHUNK_LEN){
                for(int64_t z = z_neg_target; z <= z_pos_target; z += ALT_CHUNK_LEN){
                    AltChunk_t* chunk = voxel_graph_chunk_hash_table_lookup(graph, x, y, z);
                    if(chunk != NULL){
                        chunk_ptrs.push_back(chunk);
                    }
                }
            }
        }
        for(uint32_t i = 0; i < chunk_ptrs.size(); i++){
            thread* t = new thread(&OnlineMeshMapper::build_mesh_section_global, this, i, chunk_ptrs.at(i), &chunk_local_meshes, false);
            active_threads.push_back(t);
        }
        for(uint32_t i = 0; i < active_threads.size(); i++){
            active_threads[i]->join();
        }      
        publish_meshes(&chunk_local_meshes, &vertex_normals, pub);
        return;

    }
    static void build_mesh_section_global(OnlineMeshMapper* self, uint32_t vect_index,
            AltChunk_t* chunk, std::vector<ChunkMesh_t>* output, bool wavefront){
        //for(uint32_t i = first_index; i <= last_index; i++){
            //output->at(i) = self->genChunkMesh(&(graph->chunks[i]));
        output->at(vect_index) = self->gen_chunk_mesh_with_greedy_mesher(self->graph, chunk, wavefront);
        //}
        return;
    }
    void build_and_publish_mesh(VoxelGraph_t* graph, rclcpp::Publisher<mesh_msgs::msg::MeshGeometryStamped>::SharedPtr pub){
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
        std::vector<AltChunk_t*> chunk_ptrs;
        for(uint32_t i = 0; i < graph->chunk_hash_table_size; i++){
            if(graph->chunk_hash_table[i] != NULL){
                chunk_ptrs.push_back(graph->chunk_hash_table[i]);
            }
        }
        for(uint32_t i = 0; i < chunk_ptrs.size(); i++){
            thread* t = new thread(&OnlineMeshMapper::build_mesh_section_global, this, i, chunk_ptrs.at(i), &chunk_local_meshes, false);
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
        msg.header.frame_id = this->frame_id;
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

    void write_global_wavefront_v2(VoxelGraph_t* graph){
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

    void write_global_wavefront(VoxelGraph_t* graph){
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
    VoxelGraph_t* graph;
    std::string topic;
    std::string del_topic;
    std::string frame_id = "odom";
    std::string odom_topic = "";
    std::string out_topic = "";
    std::string octomap_binary_topic = "";
    uint32_t chunk_amount;
    uint32_t scalar;
    std::string obj_filepath;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer_two;
    rclcpp::Publisher<mesh_msgs::msg::MeshGeometryStamped>::SharedPtr publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_two;
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
    std::chrono::steady_clock::time_point last_processed_octomap_msg;
    std::chrono::steady_clock::time_point last_processed_pointcloud_msg;
    void del_point_cloud_in_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {        
        //this code assumes little endian and x, y, z formatting

        io_mutex.lock();
        std::vector<DelPoint_t> points_to_be_deleted;
        int64_t point_count = msg->height * msg->width;
        size_t size_of_each_point = msg->point_step;
        const auto start = std::chrono::steady_clock::now();
        uint64_t added_points = 0;
        for(int64_t cnt = 0; cnt < point_count; cnt++){
            uint32_t starting_index = cnt * size_of_each_point;
            FUCKING_WHY_DO_I_HAVE_TO_DO_THIS_GOD_IS_DEAD_AND_I_KILLED_HIM_t binary_blob;
            //std::memcpy(&x, ptrToStartOfPoint, sizeof(float));
            //std::memcpy(&y, ptrToStartOfPoint + sizeof(float), sizeof(float));
            //std::memcpy(&z, ptrToStartOfPoint + sizeof(float) * 2, sizeof(float));
            float input_values[3] = {0,0,0};
            for(uint8_t data_cnt = 0; data_cnt < 3; data_cnt++){
                binary_blob.buf[0] = msg->data[starting_index + (data_cnt * 4)];
                binary_blob.buf[1] = msg->data[starting_index + 1 + (data_cnt * 4)];
                binary_blob.buf[2] = msg->data[starting_index + 2 + (data_cnt * 4)];
                binary_blob.buf[3] = msg->data[starting_index + 3 + (data_cnt * 4)];
                if(binary_blob.valuef > 9999999999999 || binary_blob.valuef < -9999999999999){
                    continue;
                }   
                if(data_cnt == 2){
                    input_values[data_cnt] = (int64_t) ((binary_blob.valuef * ((float)scalar)));
                }
                else{
                    input_values[data_cnt] = std::roundf((binary_blob.valuef * ((float)scalar)));
                }
            }
            starting_index += size_of_each_point;
            if(input_values[0] == prev_x && input_values[1] == prev_y && input_values[2] == prev_z){
                continue;
            }
            if(input_values[0] < -100000){
                RCLCPP_ERROR(this->get_logger(), "something got corrupted input value was %f", input_values[0]);
                continue;
            }
            prev_x = input_values[0];
            prev_y = input_values[1];
            prev_z = input_values[2];
            //debug_var = voxel_graph_delete(graph, (int64_t) input_values[0], (int64_t) input_values[1], (int64_t) input_values[2]);
            
            if(input_values[2] > global_point.position.z - 0.5 * (float)scalar){
                raycast_inverse_delete(global_point.position.x, global_point.position.y, global_point.position.z, input_values[0], input_values[1], input_values[2], &points_to_be_deleted);
                //debug_var = voxel_graph_delete(graph, (int64_t) input_values[0], (int64_t) input_values[1], (int64_t) input_values[2]);
            }/*
            else{ //if(input_values[2] > global_point.z - 0.5 * (float)scalar){
                debug_var = voxel_graph_delete(graph, (int64_t) input_values[0], (int64_t) input_values[1], (int64_t) input_values[2]);
            }
            */
        }
        for(uint32_t i = 0; i < points_to_be_deleted.size(); i++){
            voxel_graph_delete(graph, points_to_be_deleted.at(i).x, points_to_be_deleted.at(i).y, points_to_be_deleted.at(i).z);
            //raycast_delete(global_point.x, global_point.y, global_point.z, points_to_be_deleted.at(i).x, points_to_be_deleted.at(i).y, points_to_be_deleted.at(i).z, true);
        }
        const auto end = std::chrono::steady_clock::now();
        auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        RCLCPP_INFO(this->get_logger(), "deleting %ld graph points took %ld ms", points_to_be_deleted.size() ,diff.count());
        io_mutex.unlock();
    }



    bool first_call_pc_in = true;
    void point_cloud_in_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {        
        //this code assumes little endian and x, y, z formatting

        io_mutex.lock();
        const auto start = std::chrono::steady_clock::now();
        auto time_since_last_processed_pointcloud = std::chrono::duration_cast<std::chrono::milliseconds>(start - last_processed_pointcloud_msg);
        if(time_since_last_processed_pointcloud < std::chrono::milliseconds{500}){
            io_mutex.unlock();
            return;
        }
        last_processed_pointcloud_msg = start;
        sensor_msgs::msg::PointCloud2 transformed_cloud;
        try{
            auto tf = tf_buffer_->lookupTransform(
                    "base_link",
                    msg->header.frame_id,
                    msg->header.stamp,
                    rclcpp::Duration::from_seconds(0.1));
            tf2::doTransform(*msg, transformed_cloud, tf);
        }
        catch(const tf2::TransformException& ex){
            RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
            io_mutex.unlock();
            return;
        }
        pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(transformed_cloud, *raw_cloud);
        pcl::RandomSample<pcl::PointXYZ> rs;
        rs.setInputCloud(raw_cloud);
        rs.setSample(raw_cloud->size() * ((float)scalar * 0.02f));
        rs.filter(*downsampled_cloud);
        Eigen::Quaternionf q(
                global_point.orientation.w,
                global_point.orientation.x,
                global_point.orientation.y,
                global_point.orientation.z);

        Eigen::Vector3f t(
                global_point.position.x,
                global_point.position.y,
                global_point.position.z);
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        transform.translate(t);
        transform.rotate(q);
        pcl::PointCloud<pcl::PointXYZ> input_cloud;
        pcl::transformPointCloud(*downsampled_cloud, input_cloud, transform);
        for(auto &p : input_cloud.points){
            p.x = (int64_t) (p.x * (float)scalar);
            p.y = (int64_t) (p.y * (float)scalar);
            p.z = (int64_t) (p.z * (float)scalar); 
        }
        //TODO: I HAVE TO TRANSFORM THIS POINTCLOUD FROM THE FRAME ID OF THE MSG
        //TO GLOBAL POSE THEN I HAVE TO DO ICP
        //THIS SHOULD LOOK LIKE: TRANSFORM FROM MSG FRAME TO ODOM AND THEN APPLY
        //THE INTERNAL TRANSFORM FROM ODOM TO GLOBAL POSE
        double radius = 20.0;
        pcl::PointCloud<pcl::PointXYZ> map_cloud = get_local_pointcloud(global_point, radius);
        if(map_cloud.empty() || first_call_pc_in){
            first_call_pc_in = false;
            for(const auto &p : input_cloud.points){
                int64_t x_point = p.x;
                int64_t y_point = p.y;
                int64_t z_point = p.z;
                voxel_graph_insert(graph, x_point, y_point, z_point);
            }
            io_mutex.unlock();
            return;
        }
            /*
            for (const auto &p : input_cloud.points) { 
                voxel_graph_insert(graph, p.x, p.y, p.z);
            }
            const auto end = std::chrono::steady_clock::now();
            auto diff = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
            RCLCPP_INFO(this->get_logger(), "entering the pointcloud took %ld ns", diff.count());
            */
        pcl::PointCloud<pcl::PointXYZ> corrected_cloud;
        Eigen::Matrix4f corrective_transform;
        bool converged = run_icp(input_cloud, map_cloud, &corrected_cloud, &corrective_transform);
        if(!converged){
            RCLCPP_ERROR(this->get_logger(), "pose correction failed!");
            /*
            int64_t robot_point_x = global_point.position.x * (float) scalar;
            int64_t robot_point_y = global_point.position.y * (float) scalar;
            int64_t robot_point_z = global_point.position.z * (float) scalar;
            for(const auto &p : input_cloud.points){
                int64_t x_point = p.x;
                int64_t y_point = p.y;
                int64_t z_point = p.z;
                if(z_point > robot_point_z){
                    raycast_delete(robot_point_x, robot_point_y, robot_point_z, x_point, y_point, z_point, false);
                }
            }
            for(const auto &p : input_cloud.points){
                int64_t x_point = p.x;
                int64_t y_point = p.y;
                int64_t z_point = p.z;
                voxel_graph_insert(graph, x_point, y_point, z_point);
            }
            */
            io_mutex.unlock();
            return;
        }
        global_point.position.x += corrective_transform(0,3) / (float)scalar;
        global_point.position.y += corrective_transform(1,3) / (float)scalar;
        global_point.position.z += corrective_transform(2,3) / (float)scalar;
        Eigen::Matrix3f rotation = corrective_transform.block<3,3>(0,0);
        Eigen::Quaternionf q_correction(rotation);
        Eigen::Quaternionf q_current(
                global_point.orientation.w,
                global_point.orientation.x,
                global_point.orientation.y,
                global_point.orientation.z);
        Eigen::Quaternionf q_corrected = (q_correction * q_current).normalized();
        global_point.orientation.x = q_corrected.x();
        global_point.orientation.y = q_corrected.y();
        global_point.orientation.z = q_corrected.z();
        global_point.orientation.w = q_corrected.w();
        int64_t robot_point_x = global_point.position.x * (float) scalar;
        int64_t robot_point_y = global_point.position.y * (float) scalar;
        int64_t robot_point_z = global_point.position.z * (float) scalar;
        int64_t random_sample_interval = 1000;
        int64_t current_sample = 0;
        
        for(const auto &p : corrected_cloud.points){
            int64_t x_point = p.x;
            int64_t y_point = p.y;
            int64_t z_point = p.z;
            current_sample++;
            /*
            if(z_point > global_point.position.z * (float) scalar){
                raycast_delete(robot_point_x, robot_point_y, robot_point_z, x_point, y_point, z_point, false);
            }
            */
            if(current_sample >= scalar){
                //RCLCPP_INFO(this->get_logger(), "raycast debug!");
                if(z_point > global_point.position.z * (float) scalar){
                    raycast_delete(robot_point_x, robot_point_y, robot_point_z, x_point, y_point, z_point, false);
                }

                //
                //RCLCPP_INFO(this->get_logger(), "raycast done!");
                current_sample = 0;
            }
            
        }
        for(const auto &p : corrected_cloud.points){
            int64_t x_point = p.x;
            int64_t y_point = p.y;
            int64_t z_point = p.z;
            voxel_graph_insert(graph, x_point, y_point, z_point);
        }

        const auto end = std::chrono::steady_clock::now();
        auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

        RCLCPP_INFO(this->get_logger(), "entering the pointcloud took %d ms", diff.count());

        io_mutex.unlock();

        /*
        int64_t point_count = msg->height * msg->width;
        size_t size_of_each_point = msg->point_step;
        const auto start = std::chrono::steady_clock::now();
        uint64_t added_points = 0;
        for(int64_t cnt = 0; cnt < point_count; cnt++){
            uint32_t starting_index = cnt * size_of_each_point;
            FUCKING_WHY_DO_I_HAVE_TO_DO_THIS_GOD_IS_DEAD_AND_I_KILLED_HIM_t binary_blob;
            //std::memcpy(&x, ptrToStartOfPoint, sizeof(float));
            //std::memcpy(&y, ptrToStartOfPoint + sizeof(float), sizeof(float));
            //std::memcpy(&z, ptrToStartOfPoint + sizeof(float) * 2, sizeof(float));
            float input_values[3] = {0,0,0};
            for(uint8_t data_cnt = 0; data_cnt < 3; data_cnt++){
                binary_blob.buf[0] = msg->data[starting_index + (data_cnt * 4)];
                binary_blob.buf[1] = msg->data[starting_index + 1 + (data_cnt * 4)];
                binary_blob.buf[2] = msg->data[starting_index + 2 + (data_cnt * 4)];
                binary_blob.buf[3] = msg->data[starting_index + 3 + (data_cnt * 4)];
                if(binary_blob.valuef > 9999999999999 || binary_blob.valuef < -9999999999999){
                    continue;
                }   
                if(data_cnt == 2){
                    input_values[data_cnt] = (int64_t) ((binary_blob.valuef * ((float)scalar)));
                }
                else{
                    input_values[data_cnt] = std::roundf((binary_blob.valuef * ((float)scalar)));
                }
            }
            starting_index += size_of_each_point;
            if(input_values[0] == prev_x && input_values[1] == prev_y && input_values[2] == prev_z){
                continue;
            }
            if(input_values[0] < -100000){
                RCLCPP_ERROR(this->get_logger(), "something got corrupted input value was %f", input_values[0]);
                continue;
            }
            bool debug_var = false;
            debug_var = voxel_graph_insert(graph, (int64_t) input_values[0], (int64_t) input_values[1], (int64_t) input_values[2]);
            if(input_values[2] > global_point.z - 0.5 * (float)scalar && raycast_enable){
                raycast_delete(global_point.x, global_point.y, global_point.z, input_values[0], input_values[1], input_values[2], false);
            }
            if(debug_var){
                added_points++;
            }
        }
        const auto end = std::chrono::steady_clock::now();
        auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        RCLCPP_INFO(this->get_logger(), "entering %ld graph points took %ld ms", added_points ,diff.count());
        io_mutex.unlock();
        */
    }
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg){                                                                           
        io_mutex.lock();
        prev_odom_msg_pose = current_odom_msg_pose;
        current_odom_msg_pose.position = msg->pose.pose.position;
        current_odom_msg_pose.orientation = msg->pose.pose.orientation;
        global_point.position.x += current_odom_msg_pose.position.x - prev_odom_msg_pose.position.x;
        global_point.position.y += current_odom_msg_pose.position.y - prev_odom_msg_pose.position.y;
        global_point.position.z += current_odom_msg_pose.position.z - prev_odom_msg_pose.position.z;
        
        Eigen::Quaternionf q_prev(
                prev_odom_msg_pose.orientation.w,
                prev_odom_msg_pose.orientation.x,
                prev_odom_msg_pose.orientation.y,
                prev_odom_msg_pose.orientation.z);
        Eigen::Quaternionf q_current(
                current_odom_msg_pose.orientation.w,
                current_odom_msg_pose.orientation.x,
                current_odom_msg_pose.orientation.y,
                current_odom_msg_pose.orientation.z);
        Eigen::Quaternionf q_delta = q_current * q_prev.inverse();
        q_delta.normalize();
        Eigen::Quaternionf q_global(
                global_point.orientation.w,
                global_point.orientation.x,
                global_point.orientation.y,
                global_point.orientation.z);
        q_global = q_delta * q_global;
        q_global.normalize();
        global_point.orientation.x = q_global.x();
        global_point.orientation.y = q_global.y();
        global_point.orientation.z = q_global.z();
        global_point.orientation.w = q_global.w();

        io_mutex.unlock();
        /*
        try{
            transform_stamped = tf_buffer_->lookupTransform(frame_id, "odom", tf2::TimePointZero);

            prev_odom_msg_pose.x = current_odom_msg_pose.x;
            prev_odom_msg_pose.y = current_odom_msg_pose.y;
            prev_odom_msg_pose.z = current_odom_msg_pose.z;
            current_odom_msg_pose.x = msg->pose.x;
            current_odom_msg_pose.y = msg->pose.y;
            current_odom_msg_pose.z = msg->pose.z;
            global_point.x = current_odom_msg_pose.x - prev_odom_msg_pose.x;
            global_point.y = current_odom_msg_pose.y - prev_odom_msg_pose.y;
            global_point.z = current_odom_msg_pose.z - prev_odom_msg_pose.z;
            //global_point.x = transform_stamped.transform.translation.x * scalar;
            //global_point.y = transform_stamped.transform.translation.y * scalar;
            //global_point.z = transform_stamped.transform.translation.z * scalar;
            RCLCPP_INFO(this->get_logger(), "Transformed Point: [%f, %f, %f]",
                     globalPoint.x, globalPoint.y, globalPoint.z);
        } 
        */
        /*
        catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Transform error: %s", ex.what());
        }
        */
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
        std::vector<AltChunk_t*> chunk_ptrs;
        for(int64_t x = x_neg_target; x <= x_pos_target; x += ALT_CHUNK_LEN){
            for(int64_t y = y_neg_target; y <= y_pos_target; y += ALT_CHUNK_LEN){
                for(int64_t z = z_neg_target; z <= z_pos_target; z += ALT_CHUNK_LEN){
                    AltChunk_t* chunk = voxel_graph_chunk_hash_table_lookup(graph, x, y, z);
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
    void add_to_pcl_cloud(pcl::PointCloud<pcl::PointXYZ>* input, AltChunk_t* chunk){
        int64_t base_x = chunk->x_offset;
        int64_t base_y = chunk->y_offset;
        int64_t base_z = chunk->z_offset;
        for(int64_t moving_x = 0; moving_x < 16; moving_x++){
            for(int64_t moving_y = 0; moving_y < 16; moving_y++){
                for(int64_t moving_z = 0; moving_z < 16; moving_z++){
                    int64_t total_x = base_x + moving_x;
                    int64_t total_y = base_y + moving_y;
                    int64_t total_z = base_z + moving_z;
                    if(!alt_chunk_lookup(chunk, total_x, total_y, total_z)){
                        continue;
                    }
                    else{
                        input->push_back(pcl::PointXYZ(total_x, total_y, total_z));
                    }
                }
            }
        }

    }
    bool run_icp(pcl::PointCloud<pcl::PointXYZ>& input_cloud,
            pcl::PointCloud<pcl::PointXYZ>& map_cloud,
            pcl::PointCloud<pcl::PointXYZ>* corrected_cloud_out,
            Eigen::Matrix4f* transform_out){
        pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>(input_cloud));
        pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>(map_cloud));
        
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setMaxCorrespondenceDistance((float) scalar * 1.0f);
        icp.setTransformationEpsilon(1e-7);//stop iterating when transform delta below
        icp.setEuclideanFitnessEpsilon(1e-6);//stop iterating when mean squared error below
        icp.setMaximumIterations(30);
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
        double fitness_threshold = (double) scalar * 0.15;
        if(fitness > fitness_threshold){
            RCLCPP_ERROR(this->get_logger(), "ICP fitness score is too bad: %f", fitness);
            return false;
        }
        *corrected_cloud_out = *aligned;
        *transform_out = icp.getFinalTransformation();
        
        float tx = (*transform_out)(0,3);
        float ty = (*transform_out)(1,3);
        float tz = (*transform_out)(2,3);
        float translation = std::sqrt(tx * tx + ty * ty + tz * tz);
        
        Eigen::Matrix3f rot = (*transform_out).block<3,3>(0,0);
        Eigen::AngleAxisf angle_axis(rot);
        float rotation_angle = std::abs(angle_axis.angle());
        
        float max_translation = 0.1f * (float) scalar;
        float max_rotation = 0.3; //radians

        if(translation > max_translation || rotation_angle > max_rotation){
            RCLCPP_ERROR(this->get_logger(), "ICP correction implausible, rejecting");
            return false;
        }
        
        return true;
    }
    void octomap_bin_callback(const octomap_msgs::msg::Octomap::SharedPtr msg){
        io_mutex.lock();
        const auto start = std::chrono::steady_clock::now();
        auto time_since_last_processed_octomap = std::chrono::duration_cast<std::chrono::milliseconds>(start - last_processed_octomap_msg);
        if(time_since_last_processed_octomap < std::chrono::milliseconds{1000}){
            io_mutex.unlock();
            return;
        }
        last_processed_octomap_msg = start;
        octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(octomap_msgs::binaryMsgToMap(*msg));
        if (!octree) {
            RCLCPP_WARN(this->get_logger(), "Failed to convert Octomap message to tree");
            io_mutex.unlock();
            return;
        }
        double radius = 20.0;
        pcl::PointCloud<pcl::PointXYZ> raw_cloud;
        pcl::PointCloud<pcl::PointXYZ> raw_del_cloud;
        pcl::PointCloud<pcl::PointXYZ> input_cloud;
        pcl::PointCloud<pcl::PointXYZ> input_del_cloud;
        pcl::PointCloud<pcl::PointXYZ> del_cloud;
        pcl::PointCloud<pcl::PointXYZ> map_cloud = get_local_pointcloud(global_point, radius);
        octree->updateInnerOccupancy();
    
        Eigen::Affine3f x_to_base = Eigen::Affine3f::Identity();
        try{
            auto tf = tf_buffer_->lookupTransform(
                    msg->header.frame_id, "base_link",
                    msg->header.stamp,
                    rclcpp::Duration::from_seconds(0.1));
            Eigen::Vector3f to_base_translation(
                    tf.transform.translation.x,
                    tf.transform.translation.y,
                    tf.transform.translation.z);
            Eigen::Quaternionf to_base_rotation(
                    tf.transform.rotation.w,
                    tf.transform.rotation.x,
                    tf.transform.rotation.y,
                    tf.transform.rotation.z);
            x_to_base.translate(to_base_translation);
            x_to_base.rotate(to_base_rotation);
        }catch(const tf2::TransformException& ex){ 
            RCLCPP_ERROR(this->get_logger(), "Tf lookup failed %s", ex.what());
            delete octree;
            io_mutex.unlock();
            return;
        }
        Eigen::Quaternionf q_global(
                global_point.orientation.w,
                global_point.orientation.x,
                global_point.orientation.y,
                global_point.orientation.z);
        Eigen::Vector3f t_global(
                global_point.position.x,
                global_point.position.y,
                global_point.position.z);
        Eigen::Affine3f base_to_global = Eigen::Affine3f::Identity();
        base_to_global.translate(t_global);
        base_to_global.rotate(q_global);
        Eigen::Affine3f delta_transform = base_to_global * x_to_base.inverse();
        for(auto it = octree->begin_leafs(); it != octree->end_leafs(); ++it){
            if(octree->isNodeOccupied(*it)){
                float x = it.getX();
                float y = it.getY();
                float z = it.getZ();
                raw_cloud.push_back(pcl::PointXYZ(x, y, z));
            }
            else{
                float x = it.getX();
                float y = it.getY();
                float z = it.getZ();
                raw_del_cloud.push_back(pcl::PointXYZ(x, y, z));
            }
        }
        delete octree;
        pcl::transformPointCloud(raw_cloud, input_cloud, delta_transform);
        pcl::transformPointCloud(raw_del_cloud, input_del_cloud, delta_transform);
        int64_t robot_point_x = global_point.position.x * (float) scalar;
        int64_t robot_point_y = global_point.position.y * (float) scalar;
        int64_t robot_point_z = global_point.position.z * (float) scalar;
        for(const auto &p : input_cloud.points){
            int64_t x_point = p.x * (float) scalar;
            int64_t y_point = p.y * (float) scalar;
            int64_t z_point = p.z * (float) scalar;
            raycast_delete(robot_point_x, robot_point_y, robot_point_z, x_point, y_point, z_point, false);
        }
        for(auto &p : input_cloud.points){
            int64_t x_point = p.x * (float) scalar;
            int64_t y_point = p.y * (float) scalar;
            int64_t z_point = p.z * (float) scalar;
            p.x = x_point;
            p.y = y_point;
            p.z = z_point;
            voxel_graph_insert(graph, x_point, y_point, z_point);
            /*
            float x_dist = x_point - global_point.position.x * (float) scalar;
            float y_dist = y_point - global_point.position.y * (float) scalar;
            if(x_dist * x_dist + y_dist * y_dist > (1.0f * (float)scalar) * (1.0f * (float) scalar)){
                voxel_graph_insert(graph, x_point, y_point, z_point);
            }
            */
        }
        /*
        for(auto &p : input_del_cloud.points){
            int64_t x_point = p.x * (float) scalar;
            int64_t y_point = p.y * (float) scalar;
            int64_t z_point = p.z * (float) scalar;
            p.x = x_point;
            p.y = y_point;
            p.z = z_point;
            voxel_graph_delete(graph, x_point, y_point, z_point);
        }
        */
        /*
        if(input_cloud.empty()){
            RCLCPP_ERROR(this->get_logger(), "input octomap empty!");
            io_mutex.unlock();
            return;
        }
        for(const auto &p : input_cloud.points){
            int64_t x_point = p.x;
            int64_t y_point = p.y;
            int64_t z_point = p.z;
            //voxel_graph_insert(graph, x_point, y_point, z_point);
        }
        for(const auto &p : del_cloud.points){
            int64_t x_point = p.x;
            int64_t y_point = p.y;
            int64_t z_point = p.z;
            //voxel_graph_delete(graph, x_point, y_point, z_point);
        } 
        
        if(map_cloud.empty()){
            for (const auto &p : input_cloud.points) { 
                voxel_graph_insert(graph, p.x, p.y, p.z);
            }
            const auto end = std::chrono::steady_clock::now();
            auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
            RCLCPP_INFO(this->get_logger(), "entering the octomap took %ld ms", diff.count());
            io_mutex.unlock();
            return;
        }
        
        //REWRITE. JUST CONVERT TO POINTCLOUD AND INSERT THOSE POINTS FOR NOW
        //WITH A INSERTION POINTCLOUD AND A DELETION POINTCLOUD
        //THE POSE CORRECTION WILL BE HANDLED WITH A DEDICATED ICP FUNCTION ON A
        //RAW POINTCOULD
        
        pcl::PointCloud<pcl::PointXYZ> corrected_cloud;
        Eigen::Matrix4f corrective_transform;
        bool converged = run_icp(input_cloud, map_cloud, &corrected_cloud, &corrective_transform);
        if(!converged){
            RCLCPP_ERROR(this->get_logger(), "Error in insertion of the octomap! Localization required!");
            io_mutex.unlock();
            return;
        }
        for(const auto &p : corrected_cloud.points){
            int64_t x_point = p.x;
            int64_t y_point = p.y;
            int64_t z_point = p.z;
            voxel_graph_insert(graph, x_point, y_point, z_point);
        }
        
        for(const auto &p : input_del_cloud.points){
            int64_t x_point = p.x;
            int64_t y_point = p.y;
            int64_t z_point = p.z;
            voxel_graph_delete(graph, x_point, y_point, z_point);
        }
        
        global_point.position.x += corrective_transform(0,3) / (float)scalar;
        global_point.position.y += corrective_transform(1,3) / (float)scalar;
        global_point.position.z += corrective_transform(2,3) / (float)scalar;
        Eigen::Matrix3f rotation = corrective_transform.block<3,3>(0,0);
        Eigen::Quaternionf q_correction(rotation);
        Eigen::Quaternionf q_current(
                global_point.orientation.w,
                global_point.orientation.x,
                global_point.orientation.y,
                global_point.orientation.z);
        Eigen::Quaternionf q_corrected = (q_correction * q_current).normalized();
        global_point.orientation.x = q_corrected.x();
        global_point.orientation.y = q_corrected.y();
        global_point.orientation.z = q_corrected.z();
        global_point.orientation.w = q_corrected.w();
        
        */
        const auto end = std::chrono::steady_clock::now();
        auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        RCLCPP_INFO(this->get_logger(), "entering the octomap took %ld ms", diff.count());
        io_mutex.unlock();
    }
    
};

int main(int argc, char ** argv)
{

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OnlineMeshMapper>());
    rclcpp::shutdown();
    
    return 0;


}
