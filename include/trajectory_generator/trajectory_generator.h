#ifndef TRAJECTORY_GENERATOR_H_
#define TRAJECTORY_GENERATOR_H_

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/MarkerArray.h>

#include "trajectory_generator/PathArray.h"

class TrajectoryGenerator {
 public:
    TrajectoryGenerator();

    struct State {
        double x;
        double y;
        tf2::Quaternion quaternion;
        double velocity;
        double yawrate;
    };

    void process();
    void calc_trajectories(trajectory_generator::PathArray* paths_ptr);
    State get_init_state();
    void simulate_trajectory(double velocity, double yawrate, const State& init_state, nav_msgs::Path* path_ptr);
    void motion(const double velocity, const double yawrate, State* state);
    void visualize_trajectories(const trajectory_generator::PathArray &paths);
    void scan_callback(const sensor_msgs::LaserScanConstPtr& input_scan);
    void pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& input_pose);
    void odom_callback(const nav_msgs::OdometryConstPtr& input_odom);

 private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Publisher trajectories_pub_;
    ros::Publisher visualize_trajectories_pub_;
    ros::Subscriber scan_sub_;
    ros::Subscriber pose_sub_;
    ros::Subscriber odom_sub_;
    std::string ROOMBA;
    std::string ROBOT_FRAME;
    bool scan_updated_;
    bool pose_updated_;
    bool odom_updated_;
    double HZ;
    double MAX_VELOCITY;
    double MIN_VELOCITY;
    double MAX_YAWRATE;
    double MAX_ACCELERATION;
    double MAX_D_YAWRATE;
    double VELOCITY_RESOLUTION;
    double YAWRATE_RESOLUTION;
    double PREDICT_TIME;
    double TIME_DEFFERENCE;
    sensor_msgs::LaserScan scan_;
    geometry_msgs::PoseWithCovarianceStamped pose_;
    nav_msgs::Odometry odom_;
};

#endif  // TRAJECTORY_GENERATOR_H_
