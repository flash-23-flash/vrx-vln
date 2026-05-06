#include "rclcpp/rclcpp.hpp"
#include "state_msg/msg/state.hpp"
#include "std_msgs/msg/empty.hpp"
#include "robot_info_msg/msg/robot_info.hpp"
#include <torch/script.h>
#include <torch/torch.h>
#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include <algorithm>
#include <sstream>
#include <limits>
#include <random>

const double PI = 3.14159265358979323846;

class ActionPlannerNode : public rclcpp::Node {
public:
    ActionPlannerNode(const std::string& robot_name) : Node("action_planner_node_" + robot_name) {
        name = robot_name;
        
        // Subscribe to the robot state topic
        state_subscriber_ = this->create_subscription<state_msg::msg::State>(
            "/"+robot_name+"/robot_state", 10, std::bind(&ActionPlannerNode::actionPlanningCallback, this, std::placeholders::_1));

        // Subscribe to the unpause signal topic
        unpause_signal_subscriber_ = this->create_subscription<std_msgs::msg::Empty>(
            "/"+robot_name+"/unpause_signal", 10, std::bind(&ActionPlannerNode::unpauseCallback, this, std::placeholders::_1));

        // Publish action commands at a given frequency
        publish_freqency = 20; // Hz
        int t_int = std::ceil(1000 / publish_freqency); // ms
        action_publish_timer_ = this->create_wall_timer(std::chrono::milliseconds(t_int), std::bind(&ActionPlannerNode::publishAction, this));
        robot_info_publish_timer_ = this->create_wall_timer(std::chrono::milliseconds(t_int), std::bind(&ActionPlannerNode::publishRobotInfo, this));

        // Create publishers
        left_pos_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/"+robot_name+"/thrusters/left/pos", 10);
        left_thrust_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/"+robot_name+"/thrusters/left/thrust", 10);
        right_pos_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/"+robot_name+"/thrusters/right/pos", 10);
        right_thrust_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/"+robot_name+"/thrusters/right/thrust", 10);

        robot_info_publisher_ = this->create_publisher<robot_info_msg::msg::RobotInfo>("/"+robot_name+"/robot_info", 10);

        left_pos.data = PI / 2;
        left_thrust.data = 50.0;
        right_pos.data = PI / 2;
        right_thrust.data = 50.0;

        reach_goal = false;
        travel_time = 0.0;
        last_state_msg = nullptr;

        action_time_interval = 1.0; // Second(s)

        min_pos = -1.0 * PI / 2;
        max_pos = PI / 2;
        min_thrust = 0.0;
        max_thrust = 300.0;

        goal_dis = 2.0;

        pos_change = {0.0,-PI/3, PI/3};
        thrust_change = {0.0,-100.0,100.0};

        print_info = false;

        unpaused = false;
    }

protected:
    virtual void actionPlanningCallback(const state_msg::msg::State::SharedPtr msg) {
        if(!unpaused) 
            // Start signal has not been received yet
            return;
        
        // Plan action based on the robot state
        auto now = std::chrono::steady_clock::now();
        auto clock_diff = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastActionTime_).count();
        if(clock_diff >= (action_time_interval*1000)){
            left_pos.data *= -1.0;
            right_pos.data *= -1.0;
            lastActionTime_ = now;
        }
    }
    void unpauseCallback(const std_msgs::msg::Empty::SharedPtr msg) {
        if(!unpaused){
            startTime_ = std::chrono::steady_clock::now();
        }
        unpaused = true;
    }
    void publishAction() {
        left_pos_publisher_->publish(left_pos);
        left_thrust_publisher_->publish(left_thrust);
        right_pos_publisher_->publish(right_pos);
        right_thrust_publisher_->publish(right_thrust);
    }
    void publishRobotInfo() {
        if(!unpaused) return;
        robot_info_msg::msg::RobotInfo msg;
        msg.robot_name.data = name;
        msg.reach_goal.data = reach_goal;
        msg.travel_time.data = travel_time;
        msg.header = last_state_msg->header;
        msg.pose = last_state_msg->self_pose;
        msg.velocity = last_state_msg->self_velocity;
        robot_info_publisher_->publish(msg);
    }
    void checkReachGoal(const state_msg::msg::State::SharedPtr msg) {
        if(!reach_goal){
            auto now = std::chrono::steady_clock::now();
            auto clock_diff = std::chrono::duration_cast<std::chrono::milliseconds>(now - startTime_).count();
            travel_time = static_cast<float>(clock_diff) / 1000.0;
            if(std::sqrt(msg->goal.x * msg->goal.x + msg->goal.y * msg->goal.y) <= goal_dis){
                reach_goal = true;
            }
        }
    }

    std::string name;

    rclcpp::Subscription<state_msg::msg::State>::SharedPtr state_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr unpause_signal_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_pos_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_thrust_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_pos_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_thrust_publisher_;
    rclcpp::Publisher<robot_info_msg::msg::RobotInfo>::SharedPtr robot_info_publisher_;
    rclcpp::TimerBase::SharedPtr action_publish_timer_;
    rclcpp::TimerBase::SharedPtr robot_info_publish_timer_;
    
    std_msgs::msg::Float64 left_pos;
    std_msgs::msg::Float64 left_thrust;
    std_msgs::msg::Float64 right_pos;
    std_msgs::msg::Float64 right_thrust;

    bool reach_goal;
    float travel_time;
    state_msg::msg::State::SharedPtr last_state_msg;

    float publish_freqency;
    float action_time_interval;

    float min_pos;
    float max_pos;
    float min_thrust;
    float max_thrust;

    float goal_dis;

    std::vector<float> pos_change; // Thruster Pose variation per second
    std::vector<float> thrust_change; // Thruster Thrust variation per second

    std::chrono::time_point<std::chrono::steady_clock> startTime_;
    std::chrono::time_point<std::chrono::steady_clock> lastActionTime_;

    bool print_info;

    bool unpaused; 
};

// RL agents
class RL_agent: public ActionPlannerNode
{
public:
    RL_agent(const std::string& robot_name, const std::string& model_path, const std::string& agent_type) : ActionPlannerNode(robot_name), agent_type(agent_type) {
        // std::cout << "\n\n\ncreate IQN model\n\n\n";
        
        model = torch::jit::load(model_path);
        
        max_obj_num = 5;

        min_thrust = -500.0;
        max_thrust = 1000.0;

        dt = 1.0/publish_freqency;

        left_pos.data = 0.0;
        left_thrust.data = 0.0;
        right_pos.data = 0.0;
        right_thrust.data = 0.0;
        action_time_interval = 0.25;
        action_scale = 0.5;

        if(agent_type != "AC-IQN" && agent_type != "IQN" && agent_type != "DDPG" && agent_type != "DQN" && agent_type != "SAC" && agent_type != "Rainbow"){
            std::cerr << "Error: Agent not implemented!";
            std::exit(EXIT_FAILURE);
        }
        
        action_mean = 0.0;
        action_amplitude = 1000.0;

        current_action_c = std::vector<float>(2,0);

        left_thrust_change = {0.0,-500.0,-1000.0,500.0,1000.0};
        right_thrust_change = {0.0,-500.0,-1000.0,500.0,1000.0};

        actions.clear();
        for(auto l:left_thrust_change){
            for(auto r:right_thrust_change){
                actions.push_back({l,r});
            }
        }

        current_action_d = -1;

        atoms = 51;
        Vmin = -1.0;
        Vmax = 1.0;
        support = torch::linspace(Vmin, Vmax, atoms, torch::dtype(torch::kFloat64));
    }
private:
    void actionPlanningCallback(const state_msg::msg::State::SharedPtr msg) override {
        last_state_msg = msg;

        if(!unpaused) 
            // Start signal has not been received yet
            return;
        
        auto now = std::chrono::steady_clock::now();
        auto clock_diff = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastActionTime_).count();
        
        // std::cout << "call action planner" << std::endl;
        if(clock_diff >= (action_time_interval*1000)){
            // Update current action
            // std::cout << "update action" << std::endl;
            lastActionTime_ = now;
            checkReachGoal(msg);
            computeAction(msg);
        
            // Apply action
            float l_thrust = left_thrust.data;
            float r_thrust = right_thrust.data;
            
            if(agent_type == "AC-IQN" || agent_type == "DDPG" || agent_type == "SAC"){
                l_thrust += current_action_c[0] * action_scale;
                r_thrust += current_action_c[1] * action_scale;
            }
            else if(agent_type == "IQN" || agent_type == "DQN" || agent_type == "Rainbow"){
                l_thrust += actions[current_action_d][0] * action_scale;
                r_thrust += actions[current_action_d][1] * action_scale;
            }
            else{
                std::cerr << "Error: Agent not implemented!";
                std::exit(EXIT_FAILURE);
            }

            left_thrust.data = std::clamp(l_thrust,min_thrust,max_thrust);
            right_thrust.data = std::clamp(r_thrust,min_thrust,max_thrust); 
        }
    }
    void computeAction(const state_msg::msg::State::SharedPtr msg){
        // The coordinate used in the RL model has the opposite y and z direction
        // Thus need to reverse y coordinate and angular velocity value 
        
        // Self state
        double l_thrust = left_thrust.data;
        double r_thrust = right_thrust.data;
        torch::Tensor self_state = torch::tensor({{msg->goal.x,
                                                  -1.0*msg->goal.y,
                                                  msg->self_velocity.linear.x,
                                                  -1.0*msg->self_velocity.linear.y,
                                                  -1.0*msg->self_velocity.angular.z,
                                                  l_thrust,
                                                  r_thrust}},torch::kFloat64);

        // Object states
        std::vector<std::vector<double>> object_states_v;
        std::vector<double> object_state_masks_v;
        for(int i=0; i<msg->object_positions.size(); i++){
            object_states_v.push_back({msg->object_positions[i].x,
                                       -1.0*msg->object_positions[i].y,
                                       msg->object_velocities[i].x,
                                       -1.0*msg->object_velocities[i].y,
                                       msg->object_radii[i]});
            object_state_masks_v.push_back(1.0);
        }
        std::sort(object_states_v.begin(),object_states_v.end(),compareByDistance);
        
        object_states_v.resize(max_obj_num,{0.0,0.0,0.0,0.0,0.0});
        object_state_masks_v.resize(max_obj_num,0.0);

        torch::Tensor object_states = torch::zeros({1,max_obj_num,5},torch::kFloat64);
        torch::Tensor object_state_masks = torch::zeros({1,max_obj_num},torch::kFloat64);

        for(int i=0; i<max_obj_num; i++){
            for(int j=0; j<object_states_v[0].size(); j++) object_states[0][i][j] = object_states_v[i][j];
            object_state_masks[0][i] = object_state_masks_v[i];
        }

        // Forward to RL agent
        std::tuple<torch::Tensor, torch::Tensor, torch::Tensor> tensorTuple(self_state, object_states, object_state_masks);
        if(agent_type == "AC-IQN" || agent_type == "DDPG"){
            torch::Tensor action_tensor = model.forward({tensorTuple}).toTensor(); 
            current_action_c[0] = action_tensor[0][0].cpu().item<float>() * action_amplitude + action_mean;
            current_action_c[1] = action_tensor[0][1].cpu().item<float>() * action_amplitude + action_mean;
        }
        else if(agent_type == "IQN"){
            torch::Tensor taus = torch::rand({1,32},torch::kFloat64);
            torch::Tensor quantiles = model.forward({tensorTuple,taus}).toTensor();
            current_action_d = torch::argmax(quantiles.mean(1), 1).item<int>();
        }
        else if(agent_type == "DQN"){
            torch::Tensor action_values_tensor = model.forward({tensorTuple}).toTensor();
            current_action_d = torch::argmax(action_values_tensor[0]).item<int>();
        }
        else if(agent_type == "SAC"){
            auto action_tensors = model.forward({tensorTuple}).toTuple();
            torch::Tensor mu = action_tensors->elements()[0].toTensor();
            torch::Tensor log_std = action_tensors->elements()[1].toTensor();
            torch::Tensor sigma = torch::exp(log_std);
            torch::Tensor z = mu + sigma * torch::randn_like(mu);
            torch::Tensor actions = (2.0/M_PI) * torch::atan(z);
            current_action_c[0] = actions[0][0].cpu().item<float>() * action_amplitude + action_mean;
            current_action_c[1] = actions[0][1].cpu().item<float>() * action_amplitude + action_mean;
        }
        else if(agent_type == "Rainbow"){
            torch::Tensor action_value_probs_tensor = model.forward({tensorTuple}).toTensor();
            current_action_d = (action_value_probs_tensor * support).sum(2).argmax(1).item<int>();
        }
        else{
            std::cerr << "Error: Agent not implemented!";
            std::exit(EXIT_FAILURE);
        }
    }

    static bool compareByDistance(const std::vector<double>& v1, const std::vector<double>& v2) {
        double distance1 = std::sqrt(v1[0]*v1[0] + v1[1]*v1[1]);
        double distance2 = std::sqrt(v2[0]*v2[0] + v2[1]*v2[1]);
        return distance1 < distance2;
    }

    torch::jit::script::Module model;
    int max_obj_num;
    float dt;
    float action_scale;

    std::string agent_type;

    // action of continuous control
    float action_mean;
    float action_amplitude;
    std::vector<float> current_action_c; 

    // action of discretized control
    std::vector<float> left_thrust_change;
    std::vector<float> right_thrust_change;
    std::vector<std::vector<float>> actions;
    int current_action_d;

    // parameters of the Rainbow agent
    int atoms;
    double Vmin;
    double Vmax;
    torch::Tensor support; 
};

// Artificial Potential Field
class APF_agent : public ActionPlannerNode
{
public:
    APF_agent(const std::string& robot_name) : ActionPlannerNode(robot_name) {
        k_att = 50.0;
        k_rep = 500.0;
        k_v = 1.0;
        m = 400.0;
        Izz = 450.0;
        width = 2.5;
        length = 5.0;
        d0 = 15.0;
        n = 2;

        goal_dis = 4.0;

        min_thrust = -500.0;
        max_thrust = 1000.0;

        left_pos.data = 0.0;
        left_thrust.data = 0.0;
        right_pos.data = 0.0;
        right_thrust.data = 0.0;

        action_time_interval = 0.25;
        action_scale = 0.5;

        F_total = Eigen::Vector2f::Zero();

        xDotU = 20;
        yDotV = 0;
        yDotR = 0;
        nDotR = -980;
        nDotV = 0;
        xU = -100;
        xUU = -150;
        yV = -100;
        yVV = -150;
        yR = 0;
        yRV = 0;
        yVR = 0;
        yRR = 0;
        nR = -980;
        nRR = -950;
        nV = 0;
        nVV = 0;
        nRV = 0;
        nVR = 0;

        M_RB << m, 0.0, 0.0,
                0.0, m, 0.0,
                0.0, 0.0, Izz;

        M_A << xDotU, 0.0, 0.0,
               0.0, yDotV, yDotR,
               0.0, nDotV, nDotR;
        M_A *= -1.0;

        D << xU, 0.0, 0.0,
             0.0, yV, yR,
             0.0, nV, nR;
        D *= -1.0;

        C_RB = Eigen::Matrix3f::Zero();
        C_A = Eigen::Matrix3f::Zero();
        D_n = Eigen::Matrix3f::Zero();
        N = Eigen::Matrix3f::Zero();
        A = Eigen::Matrix3f::Zero();

        V = Eigen::Vector3f::Zero();
        V_r = Eigen::Vector3f::Zero();
        b_V = Eigen::Vector3f::Zero();

        
        left_thrust_change = {0.0,-500.0,-1000.0,500.0,1000.0};
        right_thrust_change = {0.0,-500.0,-1000.0,500.0,1000.0};

        actions.clear();
        for(auto l:left_thrust_change){
            for(auto r:right_thrust_change){
                actions.push_back({l,r});
            }
        }

        current_action = -1;

        head_on_zone_x_dim = 17.0;
        head_on_zone_y_dim = 9.0;
        left_crossing_zone_x_dim << -9.0,12.0;
        left_crossing_zone_y_dim_front << -17.0,-7.0;

        virtual_obj_num = 4;
        virtual_obj_dis = 5.0;
    }
private:
    void actionPlanningCallback(const state_msg::msg::State::SharedPtr msg) override {
        last_state_msg = msg;

        if(!unpaused) 
            // Start signal has not been received yet
            return;
        
        auto now = std::chrono::steady_clock::now();
        auto clock_diff = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastActionTime_).count();
        
        if(clock_diff >= (action_time_interval*1000)){
            lastActionTime_ = now;
            checkReachGoal(msg);

            computeForce(msg);
            computeAction(msg);

            // Apply action
            float l_thrust = left_thrust.data;
            float r_thrust = right_thrust.data;
            
            l_thrust += actions[current_action][0] * action_scale;
            r_thrust += actions[current_action][1] * action_scale;

            left_thrust.data = std::clamp(l_thrust,min_thrust,max_thrust);
            right_thrust.data = std::clamp(r_thrust,min_thrust,max_thrust); 
        }

        // std::vector<int> action = computeAction(msg);


        // // Apply thruster pos and thrust variation
        // left_pos.data += pos_change[action[0]] * action_time_interval;
        // right_pos.data += pos_change[action[0]] * action_time_interval;
        
        // float lp_value = left_pos.data;
        // float rp_value = right_pos.data;
        
        // left_pos.data = std::clamp(lp_value,min_pos,max_pos);
        // right_pos.data = std::clamp(rp_value,min_pos,max_pos);

        // left_thrust.data += thrust_change[action[1]] * action_time_interval;
        // right_thrust.data += thrust_change[action[1]] * action_time_interval;
        
        // float lp_thrust = left_thrust.data;
        // float rp_thrust = right_thrust.data;
        
        // left_thrust.data = std::clamp(lp_thrust,min_thrust,max_thrust);
        // right_thrust.data = std::clamp(rp_thrust,min_thrust,max_thrust);
    }

    bool check_in_left_crossing_zone(Eigen::Vector2f& pos, Eigen::Vector2f& vel){
        bool x_in_range = ((pos(0) >= left_crossing_zone_x_dim(0)) && (pos(0) <= left_crossing_zone_x_dim(1)));
        bool y_in_range = ((pos(1) >= left_crossing_zone_y_dim_front(0)) && (pos(1) <= 0.0));

        float x_diff = pos(0) - left_crossing_zone_x_dim(1);
        float y_diff = pos(1) - left_crossing_zone_y_dim_front(1);
        float grad = left_crossing_zone_y_dim_front(1) / left_crossing_zone_x_dim(1);
        bool in_trangle_area = (y_diff > grad * x_diff);

        bool pos_in_left_crossing_zone = (x_in_range && y_in_range && (!in_trangle_area));

        float vel_angle = std::atan2(vel(1),vel(0));

        bool angle_in_left_crossing_zone = ((vel_angle > M_PI/4) && (vel_angle <= 3*M_PI/4));

        if(pos_in_left_crossing_zone && angle_in_left_crossing_zone){
            return true;
        }
        return false;
    }
    bool check_in_head_on_zone(Eigen::Vector2f& pos, Eigen::Vector2f& vel){
        bool x_in_range = ((pos(0) >= 0.0) && (pos(0) <= head_on_zone_x_dim));
        bool y_in_range = ((pos(1) >= -0.5 * head_on_zone_y_dim) && (pos(1) <= 0.5 * head_on_zone_y_dim));

        bool pos_in_head_on_zone = (x_in_range && y_in_range);
        
        float vel_angle = std::atan2(vel(1),vel(0));

        bool angle_in_head_on_zone = (std::abs(vel_angle) > 3*M_PI/4);

        if(pos_in_head_on_zone && angle_in_head_on_zone){
            return true;
        }
        return false;
    }
    void project_ego_to_vehicle_frame(const Eigen::Vector2f& ego_vel, const Eigen::Vector2f& obj_pos, const Eigen::Vector2f& obj_vel,
                                      Eigen::Vector2f& ego_pos_proj, Eigen::Vector2f& ego_vel_proj){
        float obj_vel_angle = std::atan2(obj_vel(1),obj_vel(0));

        Eigen::Matrix2f R;
        R << std::cos(obj_vel_angle), -1.0*std::sin(obj_vel_angle),
             std::sin(obj_vel_angle), std::cos(obj_vel_angle);
        Eigen::Vector2f t(obj_pos);

        ego_pos_proj = -1.0 * R.transpose() * t;

        ego_vel_proj = R.transpose() * ego_vel;  
    }
    void create_left_crossing_virtual_objects(const Eigen::Vector2f& obj_pos, const Eigen::Vector2f& obj_vel,
                                    std::vector<Eigen::Vector2f>& virtual_objs){
        Eigen::Vector2f virtual_obj_dir = obj_vel/obj_vel.norm();
        
        virtual_objs.push_back(obj_pos);
        for(int i=0; i<virtual_obj_num; i++){
            virtual_objs.push_back(obj_pos + (i+1) * virtual_obj_dis * virtual_obj_dir);
        } 
    }
    void create_head_on_virtual_objects(const Eigen::Vector2f& obj_pos, const Eigen::Vector2f& obj_vel,
                                        std::vector<Eigen::Vector2f>& virtual_objs){
        // Rotate the obj velocity direction by -90 degree
        Eigen::Matrix2f Rot;
        Rot << 0.0, 1.0, 
               -1.0, 0.0;
        Eigen::Vector2f virtual_obj_dir = Rot * obj_vel;
        virtual_obj_dir /= virtual_obj_dir.norm();

        virtual_objs.push_back(obj_pos);
        for(int i=0; i<virtual_obj_num; i++){
            virtual_objs.push_back(obj_pos + (i+1) * virtual_obj_dis * virtual_obj_dir);
        } 
    }
    void check_COLREGs(const Eigen::Vector2f& ego_vel, const Eigen::Vector2f& obj_pos, const Eigen::Vector2f& obj_vel,
                       std::vector<Eigen::Vector2f>& virtual_objs){
        if(obj_vel.norm() < 0.5){ 
            // Considered as static
            return;
        }

        if(ego_vel.norm() < 0.5){
            // Ego vechile move too slow
            return;
        }

        Eigen::Vector2f ego_pos_proj;
        Eigen::Vector2f ego_vel_proj;
        project_ego_to_vehicle_frame(ego_vel,obj_pos,obj_vel,ego_pos_proj,ego_vel_proj);

        bool in_left_crossing_zone = check_in_left_crossing_zone(ego_pos_proj,ego_vel_proj);
        bool in_head_on_zone = check_in_head_on_zone(ego_pos_proj,ego_vel_proj);

        if(in_left_crossing_zone) create_left_crossing_virtual_objects(obj_pos,obj_vel,virtual_objs);
        else if(in_head_on_zone) create_head_on_virtual_objects(obj_pos,obj_vel,virtual_objs);
        else virtual_objs.push_back(obj_pos);
    }
    void computeForce(const state_msg::msg::State::SharedPtr msg){
        // Compute Attractive Force
        Eigen::Vector2f F_att = Eigen::Vector2f::Zero();
        F_att(0) = k_att * msg->goal.x;
        F_att(1) = k_att * msg->goal.y;

        // Compute Repulsive Forces
        Eigen::Vector2f F_rep = Eigen::Vector2f::Zero();
        Eigen::Vector2f v_self = Eigen::Vector2f::Zero();
        v_self(0) = msg->self_velocity.linear.x;
        v_self(1) = msg->self_velocity.linear.y;
        for(int i=0; i<msg->object_positions.size(); i++){
            Eigen::Vector2f p_obj = Eigen::Vector2f::Zero();
            p_obj(0) = msg->object_positions[i].x;
            p_obj(1) = msg->object_positions[i].y;
            
            Eigen::Vector2f e_ao = p_obj / p_obj.norm();
            
            Eigen::Vector2f v_obj = Eigen::Vector2f::Zero();
            v_obj(0) = msg->object_velocities[i].x;
            v_obj(1) = msg->object_velocities[i].y;
            
            Eigen::Vector2f v_diff = v_self - v_obj;
            float v_ao = v_diff.dot(e_ao);

            if(v_ao > 0.0){
                // Ego robot is moving towards the object
                
                std::vector<Eigen::Vector2f> virtual_objs;
                check_COLREGs(v_self,p_obj,v_obj,virtual_objs);

                for(auto obj:virtual_objs){
                    F_rep += positionForce(msg,i,obj);
                    // F_rep += velocityForce(v_ao,msg,i);
                }           
            }
        }

        Eigen::Vector2f F_total_in_r = F_att + F_rep;

        // Transform force to the world frame
        Eigen::Matrix4f robot_pose = computePoseTransformation(msg);
        Eigen::Vector4f F_total_in_r_3D = Eigen::Vector4f(F_total_in_r(0),F_total_in_r(1),0.0,0.0);
        Eigen::Vector4f F_total_in_w_3D = robot_pose * F_total_in_r_3D;
        F_total(0) = F_total_in_w_3D(0);
        F_total(1) = F_total_in_w_3D(1);
    }

    Eigen::Matrix4f computePoseTransformation(const state_msg::msg::State::SharedPtr msg){
        
        Eigen::Matrix3f R = Eigen::Quaternionf(msg->self_pose.orientation.w,
                                               msg->self_pose.orientation.x,
                                               msg->self_pose.orientation.y,
                                               msg->self_pose.orientation.z).toRotationMatrix();

        Eigen::Vector3f t = Eigen::Vector3f(msg->self_pose.position.x,
                                            msg->self_pose.position.y,
                                            msg->self_pose.position.z);

        Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
        T.block<3,3>(0,0) = R;
        T.block<3,1>(0,3) = t;

        return T;
    }

    void computeAction(const state_msg::msg::State::SharedPtr msg){
        // Transform force to the robot frame
        Eigen::Matrix4f robot_pose = computePoseTransformation(msg);
        Eigen::Matrix3f R = robot_pose.block<3,3>(0,0);
        Eigen::Vector3f t = robot_pose.block<3,1>(0,3);
        Eigen::Matrix4f robot_pose_inv = Eigen::Matrix4f::Identity();
        robot_pose_inv.block<3,3>(0,0) = R.transpose();
        robot_pose_inv.block<3,1>(0,3) = -R.transpose()*t;

        Eigen::Vector4f F_total_in_w_3D = Eigen::Vector4f(F_total(0),F_total(1),0.0,0.0);
        Eigen::Vector4f F_total_in_r_3D = robot_pose_inv * F_total_in_w_3D;
        Eigen::Vector2f F_total_in_r = Eigen::Vector2f(F_total_in_r_3D(0),F_total_in_r_3D(1));

        // Choose action that has the closest velocity prediction with force
        update_model_parameters(msg);

        float lp = left_pos.data;
        float rp = right_pos.data;
        float lt = left_thrust.data;
        float rt = right_thrust.data;
        
        Eigen::Matrix3f A_pseudo_inv = (A.transpose()*A).inverse()*A.transpose();

        float max_score = -std::numeric_limits<float>::infinity();
        int max_action = -1;
        for(int i=0; i<actions.size(); i++){ 
            float lt_next = lt + actions[i][0] * action_scale;
            float rt_next = rt + actions[i][1] * action_scale;

            // Compute propulsion forces and moments
            float F_x_left = lt_next * std::cos(lp);
            float F_y_left = lt_next * std::sin(lp);
            float M_x_left = F_x_left * width/2;
            float M_y_left = -F_y_left * length/2;

            float F_x_right = rt_next * std::cos(rp);
            float F_y_right = rt_next * std::sin(rp);
            float M_x_right = -F_x_right * width/2;
            float M_y_right = -F_y_right * length/2;

            float F_x = F_x_left + F_x_right;
            float F_y = F_y_left + F_y_right;
            float M_n = M_x_left + M_y_left + M_x_right + M_y_right;
            Eigen::Vector3f tau_p(F_x,F_y,M_n);

            Eigen::Vector3f acc = A_pseudo_inv * (b_V + tau_p);

            Eigen::Vector3f V_r_next = V_r + acc * action_time_interval;
            
            // Get back to VRX coordinates
            Eigen::Vector2f V_r_next_2d(V_r_next(0),-1.0*V_r_next(1));
            float heading_next = -1.0 * V_r_next(2) * action_time_interval;

            // Compute the projection of the new velocity in the force direction
            float proj = F_total_in_r.dot(V_r_next_2d)/F_total_in_r.norm();

            // Compute the difference in angle between the next heading and force direction
            float F_angle = std::atan2(F_total_in_r(1),F_total_in_r(0));
            float angle_diff = std::abs(warpToPi(heading_next-F_angle));

            // Compute the score of action
            float score = 0.15*proj-angle_diff;

            // if(name == "wamv1"){
            //     RCLCPP_INFO(this->get_logger(), "size: %d, l: %f, r: %f, score: %f",actions.size(),actions[i][0],actions[i][0],score);
            // }

            if(score > max_score){
                max_score = score;
                max_action = i;
            }
        }
        // if(name == "wamv1"){
        //     RCLCPP_INFO(this->get_logger(), "\n\n\n");
        // }

        current_action = max_action;

        // // Select thruster pos variation
        // float F_angle = 0.0;
        // if(F_total_in_r.norm() > 1e-03){
        //     F_angle = atan2(F_total_in_r(1),F_total_in_r(0));
        // } 

        // float diff_angle = warpToPi(F_angle);

        // int pos_change_idx = 0;
        // // Robot heading change is the opposite of thruster pos change
        // float heading = -1.0 * pos_change[0] * action_time_interval; 
        // float min_diff_heading = std::abs(heading - diff_angle);
        // for(int i=1; i<pos_change.size(); i++){
        //     heading = -1.0 * pos_change[i] * action_time_interval;
        //     float curr_diff_heading = std::abs(heading - diff_angle);
        //     if(curr_diff_heading < min_diff_heading){
        //         min_diff_heading = curr_diff_heading;
        //         pos_change_idx = i;
        //     }
        // }


        // // Select thruster thrust variation
        // Eigen::Vector2f a_total = F_total_in_r;
        // Eigen::Vector2f V_dir = Eigen::Vector2f::Zero();
        // V_dir(0) = 1.0;
        
        // Eigen::Vector2f v_self = Eigen::Vector2f::Zero();
        // v_self(0) = msg->self_velocity.linear.x;
        // v_self(1) = msg->self_velocity.linear.y;
        // if(v_self.norm() > 1e-03){
        //     V_dir = v_self / v_self.norm();
        // }
        // float a_proj = a_total.dot(V_dir);

        // int thrust_change_idx = 0;
        // float min_diff_thrust = std::abs(thrust_change[0] * action_time_interval - a_proj);
        // for(int i=1; i<thrust_change.size(); i++){
        //     float curr_diff_thrust = std::abs(thrust_change[i] * action_time_interval - a_proj);
        //     if(curr_diff_thrust < min_diff_thrust){
        //         min_diff_thrust = curr_diff_thrust;
        //         thrust_change_idx = i;
        //     }
        // }

        // return {pos_change_idx,thrust_change_idx};

        // RCLCPP_INFO(this->get_logger(), "F_att x: %f, y: %f",F_att(0),F_att(1));
        // RCLCPP_INFO(this->get_logger(), "F_rep x: %f, y: %f",F_rep(0),F_rep(1));
        // RCLCPP_INFO(this->get_logger(), "F_total x: %f, y: %f\n",F_total(0),F_total(1));
        // RCLCPP_INFO(this->get_logger(), "pos change: %f, thrust change: %f",pos_change[pos_change_idx],thrust_change[thrust_change_idx]);
        // RCLCPP_INFO(this->get_logger(), "a_proj: %f",a_proj);
    }

    void update_model_parameters(const state_msg::msg::State::SharedPtr msg){
        float u = msg->self_velocity.linear.x;
        float v = -1.0*msg->self_velocity.linear.y;
        float r = -1.0*msg->self_velocity.angular.z;
        float u_r = u;
        float v_r = v;

        V(0) = u;
        V(1) = v;
        V(2) = r;

        V_r(0) = u_r;
        V_r(1) = v_r;
        V_r(2) = r;

        C_RB(0,1) = -1.0 * m * r;
        C_RB(1,0) = m * r;

        C_A(0,2) = yDotV * v_r + yDotR * r;
        C_A(1,2) = -1.0 * xDotU * u_r;
        C_A(2,0) = -1.0 * yDotV * v_r - yDotR * r;
        C_A(2,1) = xDotU * u_r;

        D_n(0,0) = -1.0 * xUU * std::abs(u_r);
        D_n(1,1) = -1.0 * (yVV * std::abs(v_r) + yRV * std::abs(r));
        D_n(1,2) = -1.0 * (yVR * std::abs(v_r) + yRR * std::abs(r));
        D_n(2,1) = -1.0 * (nVV * std::abs(v_r) + nRV * std::abs(r));
        D_n(2,2) = -1.0 * (nVR * std::abs(v_r) + nRR * std::abs(r));

        N = C_A + D + D_n;

        A = M_RB + M_A;

        b_V = -1.0 * C_RB * V - N * V_r;
    }
    
    Eigen::Vector2f positionForce(const state_msg::msg::State::SharedPtr msg, int idx, const Eigen::Vector2f& pos){
        float d_obs = pos.norm() - msg->self_radius - msg->object_radii[idx];
        
        Eigen::Vector2f goal = Eigen::Vector2f::Zero();
        goal(0) = msg->goal.x;
        goal(1) = msg->goal.y;
        float d_goal = goal.norm();

        // RCLCPP_INFO(this->get_logger(), "d_obs: %f, d_goal: %f",d_obs, d_goal);

        // Repulsive force component to move away from the obstacle 
        float mag_1 = k_rep * ((1.0/d_obs)-(1.0/d0)) * pow(d_goal,n) / pow(d_obs,2);
        Eigen::Vector2f dir_1 = -1.0 * pos / pos.norm();
        Eigen::Vector2f F_rep_1 = mag_1 * dir_1;

        // Repulsive force component to move towards the goal
        float mag_2 = (n / 2.0) * k_rep * pow((1.0/d_obs)-(1.0/d0),2.0) * pow(d_goal,n-1);
        Eigen::Vector2f dir_2 = -1.0 * goal / d_goal;
        Eigen::Vector2f F_rep_2 = mag_2 * dir_2;

        // RCLCPP_INFO(this->get_logger(), "mag_1: %f, mag_2: %f",mag_1, mag_2);

        return (F_rep_1 + F_rep_2);
    }

    Eigen::Vector2f velocityForce(float v_ao, const state_msg::msg::State::SharedPtr msg, int idx){
        Eigen::Vector2f pos = Eigen::Vector2f::Zero();
        pos(0) = msg->object_positions[idx].x;
        pos(1) = msg->object_positions[idx].y;
        
        float d_obs = pos.norm() - msg->self_radius - msg->object_radii[idx];

        float mag = -1.0 * k_v * v_ao / d_obs;
        Eigen::Vector2f dir = pos / pos.norm();
        Eigen::Vector2f F_rep = mag * dir;

        // RCLCPP_INFO(this->get_logger(), "v_ao: %f, mag: %f",v_ao, mag);

        return F_rep;
    }

    float warpToPi(float angle){
        while(angle < -1.0 * PI) angle += 2 * PI;
        while(angle >= PI) angle -= 2 * PI;
        return angle;
    }

    float k_att; // Attractive force constant
    float k_rep; // Repulsive force constant
    float k_v; // Velocity force constant
    float m; // Robot weight (kg)
    float Izz; // momonet of inertia
    float width; // Robot width (m)
    float length; // Robot length (m)
    float d0; // Obstacle distance threshold (m)
    float n; // Power constant of repulsive force

    Eigen::Vector2f F_total; // APF force in world frame

    // hydrodynamic derivatives
    float xDotU;
    float yDotV;
    float yDotR;
    float nDotR;
    float nDotV;
    float xU;
    float xUU;
    float yV;
    float yVV;
    float yR;
    float yRV;
    float yVR;
    float yRR;
    float nR;
    float nRR;
    float nV;
    float nVV;
    float nRV;
    float nVR;

    // Matrices in the ship maneuvering model
    Eigen::Matrix3f M_RB;
    Eigen::Matrix3f M_A;
    Eigen::Matrix3f D;
    Eigen::Matrix3f C_RB;
    Eigen::Matrix3f C_A;
    Eigen::Matrix3f D_n;
    Eigen::Matrix3f N;
    Eigen::Matrix3f A;

    // Velocitiy related vectors
    Eigen::Vector3f V;
    Eigen::Vector3f V_r;
    Eigen::Vector3f b_V;

    float action_scale;
    std::vector<float> left_thrust_change;
    std::vector<float> right_thrust_change;
    std::vector<std::vector<float>> actions;
    int current_action; 

    // COLREGs checking zone
    float head_on_zone_x_dim;
    float head_on_zone_y_dim;
    Eigen::Vector2f left_crossing_zone_x_dim;
    Eigen::Vector2f left_crossing_zone_y_dim_front;

    int virtual_obj_num;
    int virtual_obj_dis;
};

class MPC_agent : public ActionPlannerNode
{
public:
    MPC_agent(const std::string& robot_name) : ActionPlannerNode(robot_name) {
        m = 400.0;
        Izz = 450.0;
        width = 2.5;
        length = 5.0;

        T = 5;
        sim_step = 0.2;

        goal_dis = 4.0;

        min_thrust = -500.0;
        max_thrust = 1000.0;

        left_pos.data = 0.0;
        left_thrust.data = 0.0;
        right_pos.data = 0.0;
        right_thrust.data = 0.0;

        action_time_interval = 0.25;
        action_scale = 0.5;

        xDotU = 20;
        yDotV = 0;
        yDotR = 0;
        nDotR = -980;
        nDotV = 0;
        xU = -100;
        xUU = -150;
        yV = -100;
        yVV = -150;
        yR = 0;
        yRV = 0;
        yVR = 0;
        yRR = 0;
        nR = -980;
        nRR = -950;
        nV = 0;
        nVV = 0;
        nRV = 0;
        nVR = 0;

        M_RB << m, 0.0, 0.0,
                0.0, m, 0.0,
                0.0, 0.0, Izz;

        M_A << xDotU, 0.0, 0.0,
               0.0, yDotV, yDotR,
               0.0, nDotV, nDotR;
        M_A *= -1.0;

        D << xU, 0.0, 0.0,
             0.0, yV, yR,
             0.0, nV, nR;
        D *= -1.0;

        C_RB = Eigen::Matrix3f::Zero();
        C_A = Eigen::Matrix3f::Zero();
        D_n = Eigen::Matrix3f::Zero();
        N = Eigen::Matrix3f::Zero();
        A = Eigen::Matrix3f::Zero();

        V = Eigen::Vector3f::Zero();
        V_r = Eigen::Vector3f::Zero();
        b_V = Eigen::Vector3f::Zero();

        
        left_thrust_change = {0.0,-500.0,-1000.0,500.0,1000.0};
        right_thrust_change = {0.0,-500.0,-1000.0,500.0,1000.0};

        actions.clear();
        for(auto l:left_thrust_change){
            for(auto r:right_thrust_change){
                actions.push_back({l,r});
            }
        }

        current_action = -1;

        head_on_zone_x_dim = 17.0;
        head_on_zone_y_dim = 9.0;
        left_crossing_zone_x_dim << -9.0,12.0;
        left_crossing_zone_y_dim_front << -17.0,-7.0;

        virtual_obj_num = 4;
        virtual_obj_dis = 5.0;

        safe_dis = 10.0;
        collision_penalty = 50;
        COLREGs_penalty = 10;
        transition_penalty = 2;
    }
private:
    void actionPlanningCallback(const state_msg::msg::State::SharedPtr msg) override {
        last_state_msg = msg;

        if(!unpaused) 
            // Start signal has not been received yet
            return;
        
        auto now = std::chrono::steady_clock::now();
        auto clock_diff = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastActionTime_).count();
        
        if(clock_diff >= (action_time_interval*1000)){
            lastActionTime_ = now;
            checkReachGoal(msg);

            obstaclesForwardSimuation(msg);
            computeAction(msg);

            // Apply action
            float l_thrust = left_thrust.data;
            float r_thrust = right_thrust.data;
            
            l_thrust += actions[current_action][0] * action_scale;
            r_thrust += actions[current_action][1] * action_scale;

            left_thrust.data = std::clamp(l_thrust,min_thrust,max_thrust);
            right_thrust.data = std::clamp(r_thrust,min_thrust,max_thrust); 
        }
    }
    bool check_in_left_crossing_zone(Eigen::Vector2f& pos, Eigen::Vector2f& vel){
        bool x_in_range = ((pos(0) >= left_crossing_zone_x_dim(0)) && (pos(0) <= left_crossing_zone_x_dim(1)));
        bool y_in_range = ((pos(1) >= left_crossing_zone_y_dim_front(0)) && (pos(1) <= 0.0));

        float x_diff = pos(0) - left_crossing_zone_x_dim(1);
        float y_diff = pos(1) - left_crossing_zone_y_dim_front(1);
        float grad = left_crossing_zone_y_dim_front(1) / left_crossing_zone_x_dim(1);
        bool in_trangle_area = (y_diff > grad * x_diff);

        bool pos_in_left_crossing_zone = (x_in_range && y_in_range && (!in_trangle_area));

        float vel_angle = std::atan2(vel(1),vel(0));

        bool angle_in_left_crossing_zone = ((vel_angle > M_PI/4) && (vel_angle <= 3*M_PI/4));

        if(pos_in_left_crossing_zone && angle_in_left_crossing_zone){
            return true;
        }
        return false;
    }
    bool check_in_head_on_zone(Eigen::Vector2f& pos, Eigen::Vector2f& vel){
        bool x_in_range = ((pos(0) >= 0.0) && (pos(0) <= head_on_zone_x_dim));
        bool y_in_range = ((pos(1) >= -0.5 * head_on_zone_y_dim) && (pos(1) <= 0.5 * head_on_zone_y_dim));

        bool pos_in_head_on_zone = (x_in_range && y_in_range);
        
        float vel_angle = std::atan2(vel(1),vel(0));

        bool angle_in_head_on_zone = (std::abs(vel_angle) > 3*M_PI/4);

        if(pos_in_head_on_zone && angle_in_head_on_zone){
            return true;
        }
        return false;
    }
    void project_ego_to_vehicle_frame(const Eigen::Vector2f& ego_vel, const Eigen::Vector2f& obj_pos, const Eigen::Vector2f& obj_vel,
                                      Eigen::Vector2f& ego_pos_proj, Eigen::Vector2f& ego_vel_proj){
        float obj_vel_angle = std::atan2(obj_vel(1),obj_vel(0));

        Eigen::Matrix2f R;
        R << std::cos(obj_vel_angle), -1.0*std::sin(obj_vel_angle),
             std::sin(obj_vel_angle), std::cos(obj_vel_angle);
        Eigen::Vector2f t(obj_pos);

        ego_pos_proj = -1.0 * R.transpose() * t;

        ego_vel_proj = R.transpose() * ego_vel;  
    }
    void create_left_crossing_virtual_objects(const Eigen::Vector2f& obj_pos, const Eigen::Vector2f& obj_vel,
                                              std::vector<Eigen::Vector2f>& virtual_objs){
        Eigen::Vector2f virtual_obj_dir = obj_vel/obj_vel.norm();
        
        for(int i=0; i<virtual_obj_num; i++){
            virtual_objs.push_back(obj_pos + (i+1) * virtual_obj_dis * virtual_obj_dir);
        } 
    }
    void create_head_on_virtual_objects(const Eigen::Vector2f& obj_pos, const Eigen::Vector2f& obj_vel,
                                        std::vector<Eigen::Vector2f>& virtual_objs){
        // Rotate the obj velocity direction by -90 degree
        Eigen::Matrix2f Rot;
        Rot << 0.0, 1.0, 
               -1.0, 0.0;
        Eigen::Vector2f virtual_obj_dir = Rot * obj_vel;
        virtual_obj_dir /= virtual_obj_dir.norm();

        for(int i=0; i<virtual_obj_num; i++){ 
            virtual_objs.push_back(obj_pos + (i+1) * virtual_obj_dis * virtual_obj_dir);
        } 
    }
    void check_COLREGs(const Eigen::Vector2f& ego_vel, const Eigen::Vector2f& obj_pos, const Eigen::Vector2f& obj_vel,
                       std::vector<Eigen::Vector2f>& virtual_objs){
        if(obj_vel.norm() < 0.5){ 
            // Considered as static
            return;
        }

        if(ego_vel.norm() < 0.5){
            // Ego vechile move too slow
            return;
        }

        Eigen::Vector2f ego_pos_proj;
        Eigen::Vector2f ego_vel_proj;
        project_ego_to_vehicle_frame(ego_vel,obj_pos,obj_vel,ego_pos_proj,ego_vel_proj);

        bool in_left_crossing_zone = check_in_left_crossing_zone(ego_pos_proj,ego_vel_proj);
        bool in_head_on_zone = check_in_head_on_zone(ego_pos_proj,ego_vel_proj);

        if(in_left_crossing_zone) create_left_crossing_virtual_objects(obj_pos,obj_vel,virtual_objs);
        else if(in_head_on_zone) create_head_on_virtual_objects(obj_pos,obj_vel,virtual_objs);
        else virtual_objs.push_back(obj_pos);
    }
    void obstaclesForwardSimuation(const state_msg::msg::State::SharedPtr msg){
        sim_COLREGs_obs.clear();
        sim_obs.clear();
        
        Eigen::Vector2f v_self = Eigen::Vector2f::Zero();
        v_self(0) = msg->self_velocity.linear.x;
        v_self(1) = msg->self_velocity.linear.y;
        
        for(int i=0; i<msg->object_positions.size(); i++){
            Eigen::Vector2f pos = Eigen::Vector2f::Zero();
            pos(0) = msg->object_positions[i].x;
            pos(1) = msg->object_positions[i].y;

            Eigen::Vector2f vel = Eigen::Vector2f::Zero();
            vel(0) = msg->object_velocities[i].x;
            vel(1) = msg->object_velocities[i].y;
            
            // Check COLREGs
            std::vector<Eigen::Vector2f> virtual_obs;
            check_COLREGs(v_self,pos,vel,virtual_obs);
            sim_COLREGs_obs.push_back(virtual_obs);

            // Forward simulation
            Eigen::Vector4f sim_obj;
            sim_obj << pos(0) + T * sim_step * vel(0),
                       pos(1) + T * sim_step * vel(1),
                       vel(0), vel(1);
            sim_obs.push_back(sim_obj);
            sim_obs_r.push_back(msg->object_radii[i]);
        }
    }
    void computeAction(const state_msg::msg::State::SharedPtr msg){
        
        update_model_parameters(msg);

        float lp = left_pos.data;
        float rp = right_pos.data;
        float lt = left_thrust.data;
        float rt = right_thrust.data;

        Eigen::Vector2f goal(msg->goal.x,msg->goal.y);

        Eigen::Vector2f v_self = Eigen::Vector2f::Zero();
        v_self(0) = msg->self_velocity.linear.x;
        v_self(1) = msg->self_velocity.linear.y;

        float r_self = msg->self_radius;

        Eigen::Matrix3f A_pseudo_inv = (A.transpose()*A).inverse()*A.transpose();
        
        float min_cost = std::numeric_limits<float>::infinity();
        int min_action = -1;
        for(int i=0; i<actions.size(); i++){
            float lt_next = lt + actions[i][0] * action_scale;
            float rt_next = rt + actions[i][1] * action_scale;

            // Compute propulsion forces and moments
            float F_x_left = lt_next * std::cos(lp);
            float F_y_left = lt_next * std::sin(lp);
            float M_x_left = F_x_left * width/2;
            float M_y_left = -F_y_left * length/2;

            float F_x_right = rt_next * std::cos(rp);
            float F_y_right = rt_next * std::sin(rp);
            float M_x_right = -F_x_right * width/2;
            float M_y_right = -F_y_right * length/2;

            float F_x = F_x_left + F_x_right;
            float F_y = F_y_left + F_y_right;
            float M_n = M_x_left + M_y_left + M_x_right + M_y_right;
            Eigen::Vector3f tau_p(F_x,F_y,M_n);

            Eigen::Vector3f acc = A_pseudo_inv * (b_V + tau_p);

            Eigen::Vector3f V_r_next = V_r + acc * action_time_interval;

            // Get back to VRX coordinates
            Eigen::Vector2f V_r_next_2d(V_r_next(0),-1.0*V_r_next(1));
            float heading_next = -1.0 * V_r_next(2) * action_time_interval;
            
            Eigen::Vector2f linear_acc(acc(0),-1.0*acc(1));


            float cost = 0.0;

            // Compute the reward for moving towards the goal
            float proj = goal.dot(V_r_next_2d)/goal.norm();
            float goal_angle = std::atan2(goal(1),goal(0));
            float angle_diff = std::abs(warpToPi(heading_next-goal_angle));

            cost += 20 * (angle_diff - 0.15 * proj);


            // Approximate future state through integration
            Eigen::Vector2f ego_p(0.0,0.0);
            Eigen::Vector2f ego_v(v_self);
            for(int i=0; i<T; i++){
                ego_p += ego_v * sim_step;
                ego_v += linear_acc * sim_step;
            }

            Eigen::Vector4f ego(ego_p(0),ego_p(1),ego_v(0),ego_v(1));

            // Compute obstacle avoidance costs 
            cost += computeCost(ego,r_self);

            // Compute maneuvering cost
            cost += 0.0005 * (std::abs(lt_next) + std::abs(rt_next)) * T * sim_step;

            if(cost < min_cost){
                min_cost = cost;
                min_action = i;
            }
        }

        current_action = min_action;
    }
    void update_model_parameters(const state_msg::msg::State::SharedPtr msg){
        float u = msg->self_velocity.linear.x;
        float v = -1.0*msg->self_velocity.linear.y;
        float r = -1.0*msg->self_velocity.angular.z;
        float u_r = u;
        float v_r = v;

        V(0) = u;
        V(1) = v;
        V(2) = r;

        V_r(0) = u_r;
        V_r(1) = v_r;
        V_r(2) = r;

        C_RB(0,1) = -1.0 * m * r;
        C_RB(1,0) = m * r;

        C_A(0,2) = yDotV * v_r + yDotR * r;
        C_A(1,2) = -1.0 * xDotU * u_r;
        C_A(2,0) = -1.0 * yDotV * v_r - yDotR * r;
        C_A(2,1) = xDotU * u_r;

        D_n(0,0) = -1.0 * xUU * std::abs(u_r);
        D_n(1,1) = -1.0 * (yVV * std::abs(v_r) + yRV * std::abs(r));
        D_n(1,2) = -1.0 * (yVR * std::abs(v_r) + yRR * std::abs(r));
        D_n(2,1) = -1.0 * (nVV * std::abs(v_r) + nRV * std::abs(r));
        D_n(2,2) = -1.0 * (nVR * std::abs(v_r) + nRR * std::abs(r));

        N = C_A + D + D_n;

        A = M_RB + M_A;

        b_V = -1.0 * C_RB * V - N * V_r;
    }
    float collisionCost(const Eigen::Vector4f& ego, const Eigen::Vector4f obs, float dis){
        // Collsion Risk Factor
        float R = 1/(T*sim_step) * pow(safe_dis/std::max(dis,9.0f),4);
        
        // Collision Cost
        float C = safe_dis - dis;
        if(dis < 0.0) C += collision_penalty;

        return R*C;
    }
    float computeCost(const Eigen::Vector4f& ego, float r_self){
        float cost = 0.0;
        for(int i=0; i<sim_obs.size(); i++){
            // Penalize getting close to and collision with obstacles 
            float dis = distance(ego(0),ego(1),sim_obs[i](0),sim_obs[i](1)) - r_self - sim_obs_r[i];
            if(dis < safe_dis) cost += collisionCost(ego,sim_obs[i],dis);
            // if(dis < safe_dis) cost += (safe_dis - dis);
            // if(dis < 0.0) cost += collision_penalty;

            // Penalize COLREGs violation
            for(auto virtual_obs:sim_COLREGs_obs[i]){
                float vir_dis = distance(ego(0),ego(1),virtual_obs(0),virtual_obs(1)) - r_self - sim_obs_r[i];
                if(vir_dis < 0.0) cost += (COLREGs_penalty + transition_penalty);
            }
        }
        return cost;
    }
    float distance(float x1, float y1, float x2, float y2){
        return std::sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
    }
    float warpToPi(float angle){
        while(angle < -1.0 * PI) angle += 2 * PI;
        while(angle >= PI) angle -= 2 * PI;
        return angle;
    }

    float m; // Robot weight (kg)
    float Izz; // momonet of inertia
    float width; // Robot width (m)
    float length; // Robot length (m)

    int T; // Planning horizon
    float sim_step; // time step in forward simulation (s)

    // hydrodynamic derivatives
    float xDotU;
    float yDotV;
    float yDotR;
    float nDotR;
    float nDotV;
    float xU;
    float xUU;
    float yV;
    float yVV;
    float yR;
    float yRV;
    float yVR;
    float yRR;
    float nR;
    float nRR;
    float nV;
    float nVV;
    float nRV;
    float nVR;

    // Matrices in the ship maneuvering model
    Eigen::Matrix3f M_RB;
    Eigen::Matrix3f M_A;
    Eigen::Matrix3f D;
    Eigen::Matrix3f C_RB;
    Eigen::Matrix3f C_A;
    Eigen::Matrix3f D_n;
    Eigen::Matrix3f N;
    Eigen::Matrix3f A;

    // Velocitiy related vectors
    Eigen::Vector3f V;
    Eigen::Vector3f V_r;
    Eigen::Vector3f b_V;

    float action_scale;
    std::vector<float> left_thrust_change;
    std::vector<float> right_thrust_change;
    std::vector<std::vector<float>> actions;
    int current_action; 

    // COLREGs checking zone
    float head_on_zone_x_dim;
    float head_on_zone_y_dim;
    Eigen::Vector2f left_crossing_zone_x_dim;
    Eigen::Vector2f left_crossing_zone_y_dim_front;

    int virtual_obj_num;
    int virtual_obj_dis;

    std::vector<Eigen::Vector4f> sim_obs; // forward simulation obstacles
    std::vector<float> sim_obs_r; // obstacle radii
    std::vector<std::vector<Eigen::Vector2f>> sim_COLREGs_obs; // virtual obstacles for COLREGs

    float safe_dis; 
    float collision_penalty; // cost of collision
    float COLREGs_penalty; // cost of violating COLREGs
    float transition_penalty; // cost of transition
};

int main(int argc, char** argv) {
    if(argc < 5){
        fprintf(stderr,"Usage: %s method robot_name [model_path] [agent_type]\n",argv[0]);
        return 1;
    }
    
    std::string method = std::string(argv[1]);

    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node;
    if(method == "RL"){
        node = std::make_shared<RL_agent>(std::string(argv[2]),std::string(argv[3]),std::string(argv[4]));
    }
    else if(method == "APF"){
        node = std::make_shared<APF_agent>(std::string(argv[2]));
    }
    else if(method == "MPC"){
        node = std::make_shared<MPC_agent>(std::string(argv[2]));
    }
    // else if(method == "VO"){
    //     node = std::make_shared<VO_agent>(std::string(argv[2]));
    // }
    else{
        fprintf(stderr,"Agent not implemented!\n");
        return 1;
    }
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
