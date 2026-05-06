#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/bool.hpp"
#include <unordered_map>
#include <sstream>
#include <vector>

std::vector<std::string> split(const std::string& s, char delimiter) {
    std::vector<std::string> tokens;
    std::stringstream ss(s);
    std::string token;
    while (std::getline(ss, token, delimiter)) {
        tokens.push_back(token);
    }
    return tokens;
}

class CollisionDetectorNode : public rclcpp::Node{
public:
    CollisionDetectorNode(const std::string& robot_names, const std::string& buoy_poses) : Node("collision_detector_node") {
        std::istringstream iss(robot_names);
        std::string robot_name;
        while (iss >> robot_name) {
            // Subscribe to odometry information of each robot
            odometry_subscribers_[robot_name] = this->create_subscription<nav_msgs::msg::Odometry>(
                "/"+robot_name+"/sensors/position/ground_truth_odometry", 10, 
                std::bind(&CollisionDetectorNode::odometryCallback, this, std::placeholders::_1));
        }

        readBuoyPoses(buoy_poses);

        robot_r = 2.8; // m
        publish_freqency = 20; // Hz
        int t_int = std::ceil(1000 / publish_freqency); // ms
        collision_detection_publish_timer_ = this->create_wall_timer(std::chrono::milliseconds(t_int), 
                                             std::bind(&CollisionDetectorNode::checkForCollisions, this));

        collision_detection_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/collision_detection", 10);
    }
private:
    void readBuoyPoses(const std::string& buoy_poses_str){
        buoy_poses.clear();
        std::vector<std::string> buoy_poses_str_v = split(buoy_poses_str, ';');
        for(auto buoy_pos : buoy_poses_str_v){
            std::vector<std::string> pos_str = split(buoy_pos, ',');
            std::vector<double> pos;
            for(auto p_str:pos_str){
                pos.push_back(std::stod(p_str));
            }
            buoy_poses.push_back(pos);
        }
    }
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::string robot_name = msg->header.frame_id.substr(0, msg->header.frame_id.find('/'));
        odometry_data_[robot_name] = *msg;
    }
    void checkForCollisions() {
        bool collision_detected = false;
        for (auto it1 = odometry_data_.begin(); it1 != odometry_data_.end(); it1++) {
            // Check collisions with other robots
            for (auto it2 = std::next(it1); it2 != odometry_data_.end(); it2++) {
                double distance = calculateDistance(it1->second.pose.pose.position, it2->second.pose.pose.position);
                if (distance < 2 * robot_r) {
                    RCLCPP_INFO(this->get_logger(), "Collision detected between %s and %s", it1->first.c_str(), it2->first.c_str());
                    collision_detected = true;
                }
            }
            // Check collisions with buoys
            for(auto pos:buoy_poses){
                double distance = calculateDistance(it1->second.pose.pose.position, pos);
                if (distance < (robot_r + pos[2])) {
                    RCLCPP_INFO(this->get_logger(), "Collision detected between %s and a buoy", it1->first.c_str());
                    collision_detected = true;
                }
            }
        }
        auto msg = std_msgs::msg::Bool();
        msg.data = collision_detected;
        collision_detection_publisher_->publish(msg);
    }
    double calculateDistance(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2) {
        return std::sqrt((p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y));
    }
    double calculateDistance(const geometry_msgs::msg::Point& p1, const std::vector<double>& p2){
        return std::sqrt((p1.x - p2[0])*(p1.x - p2[0]) + (p1.y - p2[1])*(p1.y - p2[1]));
    }

    std::unordered_map<std::string, rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> odometry_subscribers_;
    std::unordered_map<std::string, nav_msgs::msg::Odometry> odometry_data_;
    rclcpp::TimerBase::SharedPtr collision_detection_publish_timer_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr collision_detection_publisher_;

    float robot_r;
    float publish_freqency;

    std::vector<std::vector<double>> buoy_poses;
};

int main(int argc, char** argv) {
    if (argc < 2) {
        fprintf(stderr,"Usage: %s robot_names buoy_poses\n",argv[0]);
        return 1;
    }

    rclcpp::init(argc, argv);
    auto node = std::make_shared<CollisionDetectorNode>(std::string(argv[1]),std::string(argv[2]));
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}