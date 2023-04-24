#include <vector>
#include <queue>
#include <ros/ros.h>
#include <rpg_mpc/PointArray.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <yaml-cpp/yaml.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

class DummyObstacleDetector {
private:
    float detection_radius_;
    int last_pub_time_;  

    std::vector<float> xyz_{0.0, 0.0, 0.0};
    // {x, y, id, radius}
    std::vector<std::vector<float>> gt_obstacles_;
    std::vector<std::vector<float>> active_obstacles_;

    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Subscriber sub;
    ros::Publisher marker_pub;

public:

    DummyObstacleDetector() {
        pub = nh.advertise<rpg_mpc::PointArray>("obstacles", 1);
        sub = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1, &DummyObstacleDetector::gazeboStateCallback, this);
        marker_pub = nh.advertise<visualization_msgs::MarkerArray>("obstacles/visualization_marker", 1);
        std::string config_path;
        nh.getParam("/dummy_obstacle_detector/obs_config", config_path);
        nh.getParam("/dummy_obstacle_detector/detection_radius", detection_radius_);
        loadConfig(config_path);  
        last_pub_time_ = ros::Time::now().nsec;
    }
    ~ DummyObstacleDetector() {};

    void gazeboStateCallback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
        xyz_[0] = msg->pose.back().position.x;
        xyz_[1] = msg->pose.back().position.y;
        xyz_[2] = msg->pose.back().position.z;
        detectObstacles();
    }

    void loadConfig(std::string config_path) {
        // Parse the YAML file
        YAML::Node data = YAML::LoadFile(config_path);

        int num_obs = data["num_obstacles"].as<int>();
        ROS_INFO("%d obstacles loaded", num_obs);
        for (int i=0; i<num_obs; i++) {
            gt_obstacles_.push_back({data["obstacles"][i]['x'].as<float>(), data["obstacles"][i]['y'].as<float>(), float(i), data["obstacles"][i]['r'].as<float>()});
        }
    }

    void detectObstacles() {
        active_obstacles_.clear();
        using OD = std::pair<float, std::vector<float>>;
        std::priority_queue<OD, std::vector<OD>, std::greater<OD>> pq;
        for (auto& o : gt_obstacles_) {
            float dist = pow((o[0]-xyz_[0]),2) + pow((o[1]-xyz_[1]),2);
            if (dist <= pow(detection_radius_,2)) {
                pq.push(std::make_pair(dist, o));
            }
        }
        while (pq.size() > 0 && active_obstacles_.size() < 3) {
            std::pair<float, std::vector<float>> p = pq.top();
            active_obstacles_.push_back(p.second);
            pq.pop();
        }
        publishObstacles();
    }

    void publishObstacles() {
        rpg_mpc::PointArray msg;
        // ROS_INFO("PUBLISHING %d", int(active_obstacles_.size()));
        for (unsigned int i=0; i<active_obstacles_.size(); i++) {
            geometry_msgs::Point p;
            p.x = active_obstacles_[i][0];
            p.y = active_obstacles_[i][1];
            msg.points.push_back(p);
            msg.r.push_back(active_obstacles_[i][3]);
            // ROS_INFO("(%f, %f)", active_obstacles_[i][0], active_obstacles_[i][1]);
        }
        pub.publish(msg);
        if ((ros::Time::now().nsec - last_pub_time_) > 5e8) {
            publishObstacleMarkers();
            last_pub_time_ = ros::Time::now().nsec;
        }
    }

    void publishObstacleMarkers() {
        // publish as marker for rviz
        visualization_msgs::MarkerArray obs;
        int32_t shape = visualization_msgs::Marker::CYLINDER;
        for (unsigned int i=0; i<gt_obstacles_.size(); i++) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "world";
            marker.header.stamp = ros::Time::now();
            marker.ns = "obstacles";
            marker.id = i;
            marker.type = shape;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = gt_obstacles_[i][0];
            marker.pose.position.y = gt_obstacles_[i][1];
            marker.pose.position.z = 1.5;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            // Set the scale of the marker -- 1x1x1 here means 1m on a side
            marker.scale.x = gt_obstacles_[i][3]*2;
            marker.scale.y = gt_obstacles_[i][3]*2;
            marker.scale.z = 3.0;

            // Set the color -- be sure to set alpha to something non-zero!
            float color = 0.2;
            for (auto& o: active_obstacles_) {
                if (i == int(o[2])) {
                    color = 0.8;
                    break;
                }
            }
            marker.color.r = color;
            marker.color.g = color;
            marker.color.b = color;
            marker.color.a = 1.0;

            marker.lifetime = ros::Duration();

            // Publish the marker
            obs.markers.push_back(marker);
        }
        marker_pub.publish(obs);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "obstacle_detector");
    DummyObstacleDetector dod;
    ros::spin();

    return 0;
}