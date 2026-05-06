#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "cluster_msg/msg/point_cloud_cluster.hpp"
#include "state_msg/msg/state.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <chrono>
#include <Eigen/Dense>
#include <unordered_set>
#include <sstream>

Eigen::Vector4f readGoal(std::string robot_goal){
    std::vector<float> goal;

    std::stringstream ss(robot_goal);

    std::string num;
    while(std::getline(ss,num,',')){
        goal.push_back(std::stof(num));
    }

    if(goal.size() != 2){
        std::cerr << "Error: the size of robot goal is not 2!";
        std::exit(EXIT_FAILURE);
    }

    Eigen::Vector4f goal_in_w = Eigen::Vector4f::Zero();
        
    goal_in_w(0) = goal[0];
    goal_in_w(1) = goal[1]; 
    goal_in_w(2) = 0.0;
    goal_in_w(3) = 1.0;

    return goal_in_w;
}

class StateProcessorNode : public rclcpp::Node {
public:
    StateProcessorNode(const std::string& robot_name, const std::string& robot_goal) : Node("state_processor_node_" + robot_name) {
        odometry_subscriber_ = std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>(
            this, "/"+robot_name+"/sensors/position/ground_truth_odometry");
        lidar_subscriber_ = std::make_shared<message_filters::Subscriber<cluster_msg::msg::PointCloudCluster>>(
            this, "/"+robot_name+"/sensors/lidars/lidar_wamv_sensor/clusters");

        // Synchronize Odometry and LiDAR messages based on timestamps
        time_sync_ = std::make_shared<message_filters::TimeSynchronizer<nav_msgs::msg::Odometry, cluster_msg::msg::PointCloudCluster>>(*odometry_subscriber_, *lidar_subscriber_, 10);
        time_sync_->registerCallback(&StateProcessorNode::synchronizeCallback, this);

        // Publisher for synchronized timestamps
        // timestamp_publisher_ = this->create_publisher<std_msgs::msg::String>("/"+robot_name+"/sync_timestamp", 10);

        // State publisher
        state_publisher_ = this->create_publisher<state_msg::msg::State>("/"+robot_name+"/robot_state", 10);

        // if(robot_name == "wamv1") print_latency = true;
        // else print_latency = false;
        print_latency = false;

        last_odometry_msg = nullptr;
        last_lidar_msg = nullptr;

        // Initialize clock
        lastMessageTime_ = std::chrono::steady_clock::now();

        // Initialize thresholds
        max_robot_speed = 6.0;
        max_num_points_change = 0.3;
        min_timestamp_interval = 0.1;

        robot_radius = 3.0;

        goal_in_world_frame = readGoal(robot_goal);
    }

private:
    void synchronizeCallback(const nav_msgs::msg::Odometry::SharedPtr odometry_msg, const cluster_msg::msg::PointCloudCluster::SharedPtr lidar_msg) {
        // Your synchronization logic goes here
        // Access odometry_msg and lidar_msg data for processing

        auto now = std::chrono::steady_clock::now();
        
        geometry_msgs::msg::Point velocity_zero;
        velocity_zero.x = 0.0;
        velocity_zero.y = 0.0;
        velocity_zero.z = 0.0;
        std::vector<geometry_msgs::msg::Point> cluster_velocities(lidar_msg->cluster_centroids.size(),velocity_zero);

        if(last_odometry_msg != nullptr){
            // Timestamp latency (ms)
            float last_timestamp = last_lidar_msg->header.stamp.sec + last_lidar_msg->header.stamp.nanosec * 1e-9;
            float curr_timestamp = lidar_msg->header.stamp.sec + lidar_msg->header.stamp.nanosec * 1e-9;
            float diff_timestamp = curr_timestamp - last_timestamp;

            if(diff_timestamp < min_timestamp_interval) return;

            // Clock time latency (ms)
            auto clock_diff = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastMessageTime_).count();

            if(print_latency){
                RCLCPP_INFO(this->get_logger(), "Velocity x: %f, y: %f, z: %f", odometry_msg->twist.twist.linear.x,
                            odometry_msg->twist.twist.linear.y, odometry_msg->twist.twist.linear.z);
            }

            computeClusterVelocities(odometry_msg,lidar_msg,cluster_velocities,diff_timestamp);
        }

        // Create state message and publish
        state_msg::msg::State state;
        state.header = lidar_msg->header;
        
        geometry_msgs::msg::Point goal;
        Eigen::Vector4f goal_vector = computeGoalInRobotFrame(odometry_msg);
        goal.x = goal_vector(0);
        goal.y = goal_vector(1);
        goal.z = goal_vector(2);
        state.goal = goal;

        state.self_pose = odometry_msg->pose.pose;

        state.self_velocity = odometry_msg->twist.twist;

        state.self_radius = robot_radius;
        
        state.object_positions = lidar_msg->cluster_centroids;
        state.object_radii = lidar_msg->cluster_radii;
        state.object_velocities = cluster_velocities;

        last_odometry_msg = odometry_msg;
        last_lidar_msg = lidar_msg;
        lastMessageTime_ = now;

        state_publisher_->publish(state);

        // Publish synchronized timestamps
        // auto sync_timestamps = std_msgs::msg::String();
        // sync_timestamps.data = "IMU timestamp: " + std::to_string(imu_msg->header.stamp.sec) + "." + std::to_string(imu_msg->header.stamp.nanosec) +
        //                        ", LiDAR timestamp: " + std::to_string(lidar_msg->header.stamp.sec) + "." + std::to_string(lidar_msg->header.stamp.nanosec);
        // timestamp_publisher_->publish(sync_timestamps);
    }

    void computeClusterVelocities(const nav_msgs::msg::Odometry::SharedPtr odometry_msg,
                                  const cluster_msg::msg::PointCloudCluster::SharedPtr lidar_msg,
                                  std::vector<geometry_msgs::msg::Point>& cluster_velocities, float diff_time){
        
        if(last_lidar_msg->cluster_centroids.size() == 0 || lidar_msg->cluster_centroids.size() == 0) return;

        // Compute pose transformation between last and current frame
        Eigen::Matrix4f T_last = computePoseTransformation(last_odometry_msg);


        Eigen::Matrix4f T_curr = computePoseTransformation(odometry_msg);

        Eigen::Matrix3f R_curr = T_curr.block<3,3>(0,0);

        Eigen::Vector3f t_curr = T_curr.block<3,1>(0,3);
        
        Eigen::Matrix4f T_curr_inv = Eigen::Matrix4f::Identity();
        T_curr_inv.block<3,3>(0,0) = R_curr.transpose();
        T_curr_inv.block<3,1>(0,3) = -R_curr.transpose()*t_curr;


        Eigen::Matrix4f T_curr_to_last = T_curr_inv * T_last;

        
        // Project clusters in the last frame to the current frame
        for(int i=0; i<last_lidar_msg->cluster_centroids.size(); i++){
            Eigen::Vector4f centroid(last_lidar_msg->cluster_centroids[i].x,
                                     last_lidar_msg->cluster_centroids[i].y,
                                     last_lidar_msg->cluster_centroids[i].z,1.0);
            
            centroid = T_curr_to_last * centroid;

            last_lidar_msg->cluster_centroids[i].x = centroid(0);
            last_lidar_msg->cluster_centroids[i].y = centroid(1);
            last_lidar_msg->cluster_centroids[i].z = centroid(2);
        }

        std::unordered_set<int> paired_clusters;
        for(int i=0; i<lidar_msg->cluster_centroids.size(); i++){
            
            int min_cluster_id = 0;
            float min_dist = computeDistance(lidar_msg->cluster_centroids[i],last_lidar_msg->cluster_centroids[0]);
            
            for(int j=1; j<last_lidar_msg->cluster_centroids.size(); j++){
                float dist = computeDistance(lidar_msg->cluster_centroids[i],last_lidar_msg->cluster_centroids[j]);
                if(dist < min_dist){
                    min_dist = dist;
                    min_cluster_id = j;
                }
            }

            if(paired_clusters.find(min_cluster_id) != paired_clusters.end()) continue;

            // Distance is too large
            if(min_dist > max_robot_speed * diff_time) continue;

            // The difference in number of points is too large
            if(!compareNumPoints(lidar_msg->cluster_point_counts[i],last_lidar_msg->cluster_point_counts[min_cluster_id])) continue;

            paired_clusters.insert(min_cluster_id);

            cluster_velocities[i].x = (lidar_msg->cluster_centroids[i].x - last_lidar_msg->cluster_centroids[min_cluster_id].x) / diff_time;
            cluster_velocities[i].y = (lidar_msg->cluster_centroids[i].y - last_lidar_msg->cluster_centroids[min_cluster_id].y) / diff_time;
            cluster_velocities[i].z = (lidar_msg->cluster_centroids[i].z - last_lidar_msg->cluster_centroids[min_cluster_id].z) / diff_time;
        }
    }

    Eigen::Matrix4f computePoseTransformation(const nav_msgs::msg::Odometry::SharedPtr odometry_msg){
        
        Eigen::Matrix3f R = Eigen::Quaternionf(odometry_msg->pose.pose.orientation.w,
                                               odometry_msg->pose.pose.orientation.x,
                                               odometry_msg->pose.pose.orientation.y,
                                               odometry_msg->pose.pose.orientation.z).toRotationMatrix();

        Eigen::Vector3f t = Eigen::Vector3f(odometry_msg->pose.pose.position.x,
                                            odometry_msg->pose.pose.position.y,
                                            odometry_msg->pose.pose.position.z);

        Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
        T.block<3,3>(0,0) = R;
        T.block<3,1>(0,3) = t;

        return T;
    } 

    Eigen::Vector4f computeGoalInRobotFrame(const nav_msgs::msg::Odometry::SharedPtr odometry_msg){

        Eigen::Matrix4f T_curr = computePoseTransformation(odometry_msg);

        Eigen::Matrix3f R_curr = T_curr.block<3,3>(0,0);

        Eigen::Vector3f t_curr = T_curr.block<3,1>(0,3);
        
        Eigen::Matrix4f T_curr_inv = Eigen::Matrix4f::Identity();
        T_curr_inv.block<3,3>(0,0) = R_curr.transpose();
        T_curr_inv.block<3,1>(0,3) = -R_curr.transpose()*t_curr;

        Eigen::Vector4f goal_in_robot_frame = T_curr_inv * goal_in_world_frame;

        return goal_in_robot_frame;
    }

    float computeDistance(geometry_msgs::msg::Point& p1, geometry_msgs::msg::Point& p2){
        // Compute distance in x-y plane
        return sqrt((p1.x - p2.x)*(p1.x - p2.x)+(p1.y - p2.y)*(p1.y - p2.y));
    }

    bool compareNumPoints(uint32_t num1, uint32_t num2){
        uint32_t num_max = std::max(num1,num2);
        uint32_t num_min = std::min(num1,num2);
        float num_diff = num_max - num_min;
        float ratio_diff = num_diff / num_max;
        return ratio_diff <= max_num_points_change;
    }

    std::shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> odometry_subscriber_;
    std::shared_ptr<message_filters::Subscriber<cluster_msg::msg::PointCloudCluster>> lidar_subscriber_;
    std::shared_ptr<message_filters::TimeSynchronizer<nav_msgs::msg::Odometry, cluster_msg::msg::PointCloudCluster>> time_sync_;
    // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr timestamp_publisher_;
    rclcpp::Publisher<state_msg::msg::State>::SharedPtr state_publisher_;
    
    bool print_latency;
    nav_msgs::msg::Odometry::SharedPtr last_odometry_msg;
    cluster_msg::msg::PointCloudCluster::SharedPtr last_lidar_msg;
    std::chrono::time_point<std::chrono::steady_clock> lastMessageTime_; // clock time of last message

    float max_robot_speed;
    float max_num_points_change; // Threshold of change in number of points of the same cluster between consecutive frames (ratio)
    float min_timestamp_interval; 

    float robot_radius;  

    Eigen::Vector4f goal_in_world_frame; // Robot goal position in world frame
};

int main(int argc, char** argv) {
    if(argc < 3){
        fprintf(stderr,"Usage: %s robot_name robot_goal\n",argv[0]);
        return 1;
    }

    rclcpp::init(argc, argv);
    auto node = std::make_shared<StateProcessorNode>(std::string(argv[1]),std::string(argv[2]));
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
