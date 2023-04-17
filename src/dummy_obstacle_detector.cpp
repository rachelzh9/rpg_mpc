#include <vector>
#include <queue>
#include <ros/ros.h>
#include <rpg_mpc/PointArray.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <yaml-cpp/yaml.h>

class DummyObstacleDetector {
private:
    float detection_radius_;

    std::vector<float> xyz_{0.0, 0.0, 0.0};
    std::vector<std::vector<float>> gt_obstacles_;

    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Subscriber sub;

public:

    DummyObstacleDetector() {
        pub = nh.advertise<rpg_mpc::PointArray>("obstacles", 1);
        sub = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1, &DummyObstacleDetector::gazeboStateCallback, this);
        std::string config_path;
        nh.getParam("/dummy_obstacle_detector/obs_config", config_path);
        nh.getParam("/dummy_obstacle_detector/detection_radius", detection_radius_);
        loadConfig(config_path);
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
            gt_obstacles_.push_back({data["obstacles"][i]['x'].as<float>(), data["obstacles"][i]['y'].as<float>()});
        }
    }

    void detectObstacles() {
        std::vector<std::vector<float>> obs;
        using OD = std::pair<float, std::vector<float>>;
        std::priority_queue<OD, std::vector<OD>, std::greater<OD>> pq;
        for (auto& o : gt_obstacles_) {
            float dist = pow((o[0]-xyz_[0]),2) + pow((o[1]-xyz_[1]),2);
            if (dist <= pow(detection_radius_,2)) {
                pq.push(std::make_pair(dist, o));
            }
        }
        while (pq.size() > 0 && obs.size() < 3) {
            std::pair<float, std::vector<float>> p = pq.top();
            obs.push_back(p.second);
            pq.pop();
        }
        publishObstacles(obs);
    }

    void publishObstacles(std::vector<std::vector<float>>& obs) {
        rpg_mpc::PointArray msg;
        // ROS_INFO("PUBLISHING %d", int(obs.size()));
        for (unsigned int i=0; i<obs.size(); i++) {
            geometry_msgs::Point p;
            p.x = obs[i][0];
            p.y = obs[i][1];
            msg.points.push_back(p);
            // ROS_INFO("(%f, %f)", obs[i][0], obs[i][1]);
        }

        pub.publish(msg);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "obstacle_detector");
    DummyObstacleDetector dod;
    ros::spin();

    return 0;
}