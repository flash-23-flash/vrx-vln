#include "nanoflann.hpp"
#include "KDTreeVectorOfVectorsAdaptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "cluster_msg/msg/point_cloud_cluster.hpp"
#include <vector>
#include <cmath>
#include <iostream>
#include <cstdlib>
#include <queue>
#include <array>
#include <unordered_set>
#include <unordered_map>
#include <Eigen/Dense>
#include <map>

const double PI = 3.14159265358979323846;

typedef std::vector<Eigen::Vector3f> cloud;
typedef KDTreeVectorOfVectorsAdaptor<cloud, float>  kd_tree;

class LidarProcessorNode : public rclcpp::Node {
public:
    LidarProcessorNode(const std::string& robot_name) : Node("lidar_processor_node_" + robot_name) {
        // Subscribe to the LiDAR PointCloud2 topic
        lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/"+robot_name+"/sensors/lidars/lidar_wamv_sensor/points", 10, std::bind(&LidarProcessorNode::lidarCallback, this, std::placeholders::_1));

        // Publish the filtered PointCloud2 and clusters information
        processed_lidar_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/"+robot_name+"/sensors/lidars/lidar_wamv_sensor/processed_points", 10);

        clusters_publisher_ = this->create_publisher<cluster_msg::msg::PointCloudCluster>(
            "/"+robot_name+"/sensors/lidars/lidar_wamv_sensor/clusters", 10);

        // Compute horizontal and vertical angle interval between beams
        h_angle_interval = (max_horizontal_angle-min_horizontal_angle)/num_samples;
        v_angle_interval = (max_vertical_angle-min_vertical_angle)/num_beams;

        // Compute vertical margin
        min_vertical_angle_margin = min_vertical_angle * v_margin_factor;
        max_vertical_angle_margin = max_vertical_angle * v_margin_factor;
        num_rows = num_beams * v_margin_factor;
        num_cols = num_samples;
    }

private:
    float intensity_interval = 20.0;

    // LiDAR translation from robot center
    float x_trans = 0.43;
    float y_trans = 0.00;
    float z_trans = 1.93;

    // LiDAR beam parameters
    float min_horizontal_angle = -PI;
    float max_horizontal_angle = PI;
    float min_vertical_angle = -0.186;
    float max_vertical_angle = 0.54;
    float max_range = 20.0;

    int num_beams = 32;
    int num_samples = 2187;

    float h_angle_interval;
    float v_angle_interval;

    // Vertical margin factor
    int v_margin_factor = 2;

    float min_vertical_angle_margin;
    float max_vertical_angle_margin;

    // Size of point map
    int num_rows;
    int num_cols;

    // LiDAR point parameters
    uint32_t x_offset = 0;
    uint32_t y_offset = 4;
    uint32_t z_offset = 8;
    uint32_t intensity_offset = 16;

    // Minimum angle threshold for the same group
    float theta = 60 * PI / 180;

    // Distance threshold for searching neighboring centroids
    float d_centroids = 1.0;

    // Minimum angle threshold for merging groups
    float theta_2 = 10 * PI / 180;

    // Minimum number of points in group
    uint32_t min_points_in_group = 20;

    struct PointData
    {
        PointData(uint32_t h, float r): head(h), range(r), group(0), merged(false), num_points_in_group(0){};
        uint32_t head; // the begin position in the original uint8 int data array
        float range;
        uint32_t group;
        bool merged; // indicator of whether the group is checked or merged to another group 
        uint32_t num_points_in_group;
    };

    int four_neighbors[4][2] = {{-1,0},{1,0},{0,-1},{0,1}};

    void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // Process the received PointCloud2
        sensor_msgs::msg::PointCloud2 processed_cloud = *msg; // Create a copy

        // Initialize 2D projection point map
        PointData*** point_map = createPointMap(); 

        // Access point cloud data and create a new point cloud without points above intensity threshold
        float intensity_threshold = 50.0;

        // float min_v_angle = PI;
        // float max_v_angle = -PI;
        // std::vector<float> min_point = {0.0,0.0,0.0};
        // std::vector<float> max_point = {0.0,0.0,0.0};

        uint32_t num_points = processed_cloud.width * processed_cloud.height;
        uint32_t new_num_points = 0;
        for (uint32_t i = 0; i < num_points; ++i) {
            float x,y,z,intensity;
            memcpy(&x, &processed_cloud.data[i * processed_cloud.point_step + x_offset], sizeof(float));
            memcpy(&y, &processed_cloud.data[i * processed_cloud.point_step + y_offset], sizeof(float));
            memcpy(&z, &processed_cloud.data[i * processed_cloud.point_step + z_offset], sizeof(float));
            memcpy(&intensity, &processed_cloud.data[i * processed_cloud.point_step + intensity_offset], sizeof(float));

            // Coordinates in LiDAR frame 
            float x_lidar = x - x_trans;
            float y_lidar = y - y_trans;
            float z_lidar = z - z_trans;

            // Flase LiDAR reflection from water
            if (intensity > intensity_threshold) continue;

            // False LiDAR reflection out of range
            float range = computeDistance(x_lidar,y_lidar,z_lidar);
            if (range > max_range) continue;

            // Remove points not match any beams or too close to existing points 
            if (!projectToPointMap(x_lidar,y_lidar,z_lidar,range,new_num_points,point_map)) continue;

            // Copy the point to the new point cloud
            memcpy(&processed_cloud.data[new_num_points * processed_cloud.point_step],
                    &processed_cloud.data[i * processed_cloud.point_step],
                    processed_cloud.point_step);

            ++new_num_points;
        }

        // std::cout << min_v_angle << " " << min_point[0] << " " << min_point[1] << " " << min_point[2] << std::endl;
        // std::cout << max_v_angle << " " << max_point[0] << " " << max_point[1] << " " << max_point[2] << std::endl;
        // std::exit(EXIT_FAILURE);

        // Points to clusters
        std::vector<std::vector<int>> clusters;
        cloud centroids;
        pointToCluster(processed_cloud, point_map, clusters, centroids);

        // Clusters information to publish
        std::vector<uint32_t> new_cluster_point_counts;
        std::vector<geometry_msgs::msg::Point> new_cluster_centroids;
        std::vector<float> new_cluster_radii;

        if(clusters.size()>1){
            // Merge clusters
            mergeClusters(processed_cloud, point_map, clusters, centroids, 
                          new_cluster_point_counts, new_cluster_centroids, new_cluster_radii);
        }
        else if(clusters.size()==1){
            uint32_t new_cluster_point_count = clusters.back().size();

            geometry_msgs::msg::Point centroid;
            centroid.x = centroids.back()(0);
            centroid.y = centroids.back()(1);
            centroid.z = centroids.back()(2);

            float radius = 0.0;
            for(auto point:clusters.back()){
                int r = point / num_cols;
                int c = point % num_cols;

                point_map[r][c]->num_points_in_group = new_cluster_point_count;

                // Update the radius of cluster
                uint32_t head = point_map[r][c]->head;

                float x,y;
                memcpy(&x, &processed_cloud.data[head * processed_cloud.point_step + x_offset], sizeof(float));
                memcpy(&y, &processed_cloud.data[head * processed_cloud.point_step + y_offset], sizeof(float));

                float curr_r = computeDistance(centroid.x - x, centroid.y - y, 0.0);
                radius = std::max(radius,curr_r);
            }
            
            if(new_cluster_point_count >= min_points_in_group){
                new_cluster_point_counts.push_back(new_cluster_point_count);
                new_cluster_centroids.push_back(centroid);
                new_cluster_radii.push_back(radius);
            }
        }

        // Remove clusters with too few number of points
        sensor_msgs::msg::PointCloud2 processed_cloud_2 = *msg; // Create a second copy
        uint32_t processed_num_points = 0;
        for(int i=0; i<num_rows; i++){
            for(int j=0; j<num_cols; j++){
                if(point_map[i][j] == nullptr) continue;
                if(point_map[i][j]->num_points_in_group < min_points_in_group) continue;

                uint32_t head = point_map[i][j]->head;
                memcpy(&processed_cloud_2.data[processed_num_points * processed_cloud_2.point_step],
                    &processed_cloud.data[head * processed_cloud.point_step],
                    processed_cloud.point_step);
                
                processed_num_points++;
            }
        }

        // Resize the new point cloud data and publish
        processed_cloud_2.data.resize(processed_num_points * processed_cloud_2.point_step);
        processed_cloud_2.width = processed_num_points;
        processed_cloud_2.height = 1;

        // Create clusters message and publish
        cluster_msg::msg::PointCloudCluster point_cloud_clusters;
        point_cloud_clusters.header = processed_cloud_2.header;
        point_cloud_clusters.point_cloud = processed_cloud_2;
        point_cloud_clusters.cluster_point_counts = new_cluster_point_counts;
        point_cloud_clusters.cluster_centroids = new_cluster_centroids;
        point_cloud_clusters.cluster_radii = new_cluster_radii;

        clusters_publisher_->publish(point_cloud_clusters);

        // Publish the modified PointCloud2
        processed_lidar_publisher_->publish(processed_cloud_2);

        clearPointMap(point_map);
    }

    bool projectToPointMap(float x_lidar, float y_lidar, float z_lidar, 
                           float range, uint32_t index, PointData*** point_map){
        // Horizontal angle
        float h_angle = atan2(y_lidar,x_lidar);

        // Vertical angle
        float h_distance = sqrt(x_lidar*x_lidar+y_lidar*y_lidar);
        float v_angle = atan2(z_lidar,h_distance);

        // Project to the nearest beam
        int row = std::round((v_angle - min_vertical_angle_margin)/v_angle_interval);
        int col = std::round((h_angle - min_horizontal_angle)/h_angle_interval);

        // Remove the point if no match any beams
        if(row < 0 || row >= num_rows || col < 0 || col >= num_cols) return false;

        // Remove the point if there is an existing point that matches the beam
        if(point_map[row][col] != nullptr) return false;

        // Add point to map
        point_map[row][col] = new PointData(index,range);

        return true;
    }

    void pointToCluster(sensor_msgs::msg::PointCloud2& point_cloud, PointData*** point_map,
                        std::vector<std::vector<int>>& clusters, cloud& centroids){
        std::vector<int> cluster;
        for(int i=0; i<num_rows; i++){
            for(int j=0; j<num_cols; j++){
                // No point reflection or the point belongs to an existing group 
                if(point_map[i][j] == nullptr || point_map[i][j]->group > 0) continue;
                checkNeighbor(i,j,point_cloud,point_map,clusters,centroids);
            }
        }
        // std::cout << "clusters size: " << clusters.size() << std::endl;
    }

    void checkNeighbor(int r, int c, sensor_msgs::msg::PointCloud2& point_cloud,
                       PointData*** point_map, std::vector<std::vector<int>>& clusters,
                       cloud& centroids){
        std::queue<std::array<int,2>> q;
        q.push({r,c});
        
        std::unordered_set<int> explored;
        explored.insert(r*num_cols+c);

        clusters.push_back(std::vector<int>());
        uint32_t label = clusters.size();

        Eigen::Vector3f centroid = Eigen::Vector3f::Zero();

        while(!q.empty()){
            auto curr = q.front();
            q.pop();
            point_map[curr[0]][curr[1]]->group = label;

            // Add point to cluster
            clusters[label-1].push_back(curr[0]*num_cols+curr[1]);
            
            // Update centroid
            uint32_t index = point_map[curr[0]][curr[1]]->head;
            float x,y,z;
            memcpy(&x, &point_cloud.data[index * point_cloud.point_step + x_offset], sizeof(float));
            memcpy(&y, &point_cloud.data[index * point_cloud.point_step + y_offset], sizeof(float));
            memcpy(&z, &point_cloud.data[index * point_cloud.point_step + z_offset], sizeof(float));

            int num_points = clusters[label-1].size();
            centroid(0) = (centroid(0) * (num_points-1) + x)/num_points;
            centroid(1) = (centroid(1) * (num_points-1) + y)/num_points;
            centroid(2) = (centroid(2) * (num_points-1) + z)/num_points;

            // // set the intensity according to group id for visualization
            // float intensity = label * intensity_interval;
            // uint32_t index = point_map[curr[0]][curr[1]]->head;
            // memcpy(&point_cloud.data[index * point_cloud.point_step + intensity_offset], &intensity, sizeof(float));

            float curr_range = point_map[curr[0]][curr[1]]->range;

            for(int k=0; k<4; k++){
                int r_n = curr[0] + four_neighbors[k][0];
                int c_n = curr[1] + four_neighbors[k][1];
                
                // Point out of range
                if(r_n < 0 || r_n >= num_rows || c_n < 0 || c_n >= num_cols) continue;
                
                // Point has been explored
                if(explored.find(r_n*num_cols+c_n) != explored.end()) continue;

                explored.insert(r_n*num_cols+c_n);

                // No point reflection or the point belongs to an existing group
                if(point_map[r_n][c_n] == nullptr || point_map[r_n][c_n]->group > 0) continue;

                // std::cout << "parent: " << curr[0] << " " << curr[1] << " " << point_map[curr[0]][curr[1]]->group << std::endl;
                // std::cout << "current: " << r_n << " " << c_n << " " << point_map[r_n][c_n]->group << std::endl;

                // Check angle
                float neighbor_range = point_map[r_n][c_n]->range;
                float d1 = std::max(curr_range,neighbor_range);
                float d2 = std::min(curr_range,neighbor_range);    
                float alpha = (four_neighbors[k][0] == 0)? h_angle_interval:v_angle_interval;

                float beta = atan2(d2*sin(alpha),d1-d2*cos(alpha));
                if(beta > theta){
                    // This neighbor belongs to the current group
                    q.push({r_n,c_n});
                }
            }
        }
        centroids.push_back(centroid);
    }

    void mergeClusters(sensor_msgs::msg::PointCloud2& point_cloud, PointData*** point_map,
                       std::vector<std::vector<int>>& clusters, cloud& centroids, 
                       std::vector<uint32_t>& new_cluster_point_counts,
                       std::vector<geometry_msgs::msg::Point>& new_cluster_centroids,
                       std::vector<float>& new_cluster_radii){ 
        kd_tree centroid_tree(3,centroids,10);
        centroid_tree.index->buildIndex();
        int count = 0;
        int processed_num = 0;
        for(int i=0; i<clusters.size(); i++){
            // If the cluster has been merged to another group, continue
            int r = clusters[i][0] / num_cols;
            int c = clusters[i][0] % num_cols;
            if(point_map[r][c]->merged) continue; 
            if(checkCluster(i,point_cloud,point_map,clusters,centroids,centroid_tree,
                            new_cluster_point_counts,new_cluster_centroids,
                            new_cluster_radii) < min_points_in_group) processed_num++;
            count++; 
        }
        // std::cout << clusters.size() << " " << (count - processed_num) << std::endl;
    }

    uint32_t checkCluster(int idx, sensor_msgs::msg::PointCloud2& point_cloud,
                      PointData*** point_map, std::vector<std::vector<int>>& clusters,
                      cloud& centroids, kd_tree& tree, 
                      std::vector<uint32_t>& new_cluster_point_counts,
                      std::vector<geometry_msgs::msg::Point>& new_cluster_centroids,
                      std::vector<float>& new_cluster_radii){
        
        uint32_t new_cluster_point_count = 0;

        geometry_msgs::msg::Point centroid;
        centroid.x = centroid.y = centroid.z = 0.0;
        
        std::queue<int> q;
        q.push(idx);
        
        int label = idx + 1;

        std::unordered_set<int> explored;
        explored.insert(idx);

        while(!q.empty()){
            int curr = q.front();
            q.pop();

            // Revise group id and intensity for points in the cluster
            for(auto id:clusters[curr]){
                int r = id / num_cols;
                int c = id % num_cols;

                point_map[r][c]->group = label;
                point_map[r][c]->merged = true;
                
                float intensity = label * intensity_interval;
                uint32_t head = point_map[r][c]->head;
                memcpy(&point_cloud.data[head * point_cloud.point_step + intensity_offset], &intensity, sizeof(float));
            }

            // Update information of the new cluster
            int added_point_counts = clusters[curr].size();

            centroid.x = (new_cluster_point_count * centroid.x + added_point_counts * centroids[curr](0))
                         / (new_cluster_point_count + added_point_counts);
            centroid.y = (new_cluster_point_count * centroid.y + added_point_counts * centroids[curr](1))
                         / (new_cluster_point_count + added_point_counts);
            centroid.z = (new_cluster_point_count * centroid.z + added_point_counts * centroids[curr](2))
                         / (new_cluster_point_count + added_point_counts);
            
            new_cluster_point_count += added_point_counts;
            

            // find neighboring clusters with kd tree;
            const float search_range = d_centroids*d_centroids;
            std::vector<std::pair<size_t,float>>  search_results;

            nanoflann::SearchParams params;

            const size_t nNeighbors = tree.index->radiusSearch(&(centroids)[curr](0), search_range, search_results, params);

            // std::cout << "num clusters: " << clusters.size() << " num centroids: " << centroids.size() 
            //           << " num neighbors: " << nNeighbors << std::endl; 

            for(size_t i=0; i<nNeighbors; i++){
                int neighbor = search_results[i].first;
                // float dist = search_results[i].second;
                // std::cout << "neighbor " << neighbor << " distance: " << dist << std::endl;

                // Neighbor has been explored
                if(explored.find(neighbor) != explored.end()) continue; 

                explored.insert(neighbor);

                q.push(neighbor);
            }
            // computeNeighborDistances(curr,centroids);
        }

        float radius = 0.0; // Radius of the cluster
        for(auto cluster:explored){
            for(auto point:clusters[cluster]){
                int r = point / num_cols;
                int c = point % num_cols;

                point_map[r][c]->num_points_in_group = new_cluster_point_count;

                // Update the radius of cluster
                uint32_t head = point_map[r][c]->head;

                float x,y;
                memcpy(&x, &point_cloud.data[head * point_cloud.point_step + x_offset], sizeof(float));
                memcpy(&y, &point_cloud.data[head * point_cloud.point_step + y_offset], sizeof(float));

                float curr_r = computeDistance(centroid.x - x, centroid.y - y, 0.0);
                radius = std::max(radius,curr_r);
            }
        }
        
        // Include the cluster if it has enough number of points
        if(new_cluster_point_count >= min_points_in_group){
            new_cluster_point_counts.push_back(new_cluster_point_count);
            new_cluster_centroids.push_back(centroid);
            new_cluster_radii.push_back(radius);
        }

        return new_cluster_point_count;
    }

    // for debugging KD Tree
    void computeNeighborDistances(int idx, cloud& centroids){
        std::map<float,int> dist;
        for(int i=0; i<centroids.size();i++){
            Eigen::Vector3f diff = centroids[idx] - centroids[i];
            dist[diff.norm()]=i;
        }
        // std::sort(dist.begin(),dist.end());
        std::cout << "Compute all distances" << std::endl;
        for(auto pair:dist){
            std::cout << "neighbor " << pair.second << " distance: " << pair.first << std::endl;
        }
    }

    bool acceptMerge(int i, int j, cloud& centroids){

        // Compute angle between them
        float dotProduct = centroids[i].dot(centroids[j]);

        float range_i = centroids[i].norm();
        float range_j = centroids[j].norm();

        float cos_angle = dotProduct / (range_i * range_j);
        float alpha = acos(cos_angle);

        // Check angle
        float d1 = std::max(range_i,range_j);
        float d2 = std::min(range_i,range_j);

        float beta = atan2(d2*sin(alpha),d1-d2*cos(alpha));
        
        return (beta > theta_2);
        // return true;
    }

    float computeDistance(float x, float y, float z){
        return sqrt(x*x+y*y+z*z);
    }

    PointData*** createPointMap(){
        PointData*** point_map = new PointData**[num_rows];
        for(int i=0; i<num_rows; i++){
            point_map[i] = new PointData*[num_cols];
            for(int j=0; j<num_cols; j++){
                point_map[i][j] = nullptr;
            }
        }
        return point_map;
    }

    void clearPointMap(PointData*** point_map){
        for(int i=0; i<num_rows; i++){
            for(int j=0; j<num_cols; j++){
                if(point_map[i][j] != nullptr){
                    delete point_map[i][j];
                }
            }
            delete [] point_map[i];
        }
        delete [] point_map;
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr processed_lidar_publisher_;
    rclcpp::Publisher<cluster_msg::msg::PointCloudCluster>::SharedPtr clusters_publisher_;
};

int main(int argc, char** argv) {
    if(argc < 2){
        fprintf(stderr,"Usage: %s robot_name\n",argv[0]);
        return 1;
    }
    
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarProcessorNode>(std::string(argv[1]));
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
