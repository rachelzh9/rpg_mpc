#include <vector>
#include <ros/ros.h>
#include <rpg_mpc/PointArray.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>

class DummyObstacleDetector {
private:
    float detection_radius_ = 3.0;

    std::vector<float> xyz_{0.0, 0.0, 0.0};
    std::vector<std::vector<float>> gt_obstacles_{{2.0, 2.0}, {5.0, 0.0}, {0.0, 3.0}, {4.0, 4.0}, {2.0, 8.0}, {9.0, 6.0}};

    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Subscriber sub;

public:

    DummyObstacleDetector() {
        pub = nh.advertise<rpg_mpc::PointArray>("obstacles", 1);
        sub = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1, &DummyObstacleDetector::gazeboStateCallback, this);
    }
    ~DummyObstacleDetector() {};

    void gazeboStateCallback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
        xyz_[0] = msg->pose.back().position.x;
        xyz_[1] = msg->pose.back().position.y;
        xyz_[2] = msg->pose.back().position.z;
        detectObstacles();
    }

    void detectObstacles() {
        std::vector<std::vector<float>> obs;
        for (auto& o : gt_obstacles_) {
            if (pow((o[0]-xyz_[0]),2) + pow((o[1]-xyz_[1]),2) <= pow(detection_radius_,2)) {
                obs.push_back(o);
            }
            if (obs.size() >= 3) break;
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