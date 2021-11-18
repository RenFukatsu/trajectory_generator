#include "trajectory_generator/trajectory_generator.h"

TrajectoryGenerator::TrajectoryGenerator() : private_nh_("~"), scan_updated_(false), pose_updated_(false) {
    ROS_ASSERT(private_nh_.hasParam("ROOMBA"));
    private_nh_.getParam("ROOMBA", ROOMBA);
    private_nh_.param("ROBOT_FRAME", ROBOT_FRAME, std::string("base_link"));
    private_nh_.param("HZ", HZ, 10.0);
    private_nh_.param("MAX_VELOCITY", MAX_VELOCITY, 0.5);
    private_nh_.param("MIN_VELOCITY", MIN_VELOCITY, -0.5);
    private_nh_.param("MAX_YAWRATE", MAX_YAWRATE, 1.5);
    private_nh_.param("MAX_ACCELERATION", MAX_ACCELERATION, 1.8);
    private_nh_.param("MAX_D_YAWRATE", MAX_D_YAWRATE, 1.6);
    private_nh_.param("VELOCITY_RESOLUTION", VELOCITY_RESOLUTION, 0.01);
    private_nh_.param("YAWRATE_RESOLUTION", YAWRATE_RESOLUTION, 0.05);
    private_nh_.param("PREDICT_TIME", PREDICT_TIME, 5.0);
    TIME_DEFFERENCE = 1.0 / HZ;

    visualize_trajectories_pub_ = private_nh_.advertise<visualization_msgs::MarkerArray>("visualize_trajectories", 1);
    scan_sub_ = nh_.subscribe("scan", 1, &TrajectoryGenerator::scan_callback, this);
    pose_sub_ = nh_.subscribe("pose", 1, &TrajectoryGenerator::pose_callback, this);
    odom_sub_ = nh_.subscribe("odom", 1, &TrajectoryGenerator::odom_callback, this);
}

void TrajectoryGenerator::scan_callback(const sensor_msgs::LaserScanConstPtr& input_scan) {
    scan_ = *input_scan;
    scan_updated_ = true;
}

void TrajectoryGenerator::pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& input_pose) {
    pose_ = *input_pose;
    pose_updated_ = true;
}

void TrajectoryGenerator::odom_callback(const nav_msgs::OdometryConstPtr& input_odom) {
    odom_ = *input_odom;
    odom_updated_ = true;
}

void TrajectoryGenerator::calc_trajectories(trajectory_generator::PathArray* paths_ptr) {
    double min_velocity = std::max(odom_.twist.twist.linear.x - MAX_ACCELERATION * TIME_DEFFERENCE, MIN_VELOCITY);
    double max_velocity = std::min(odom_.twist.twist.linear.x + MAX_ACCELERATION * TIME_DEFFERENCE, MAX_VELOCITY);
    double min_yawrate = std::max(odom_.twist.twist.angular.z - MAX_D_YAWRATE * TIME_DEFFERENCE, -MAX_YAWRATE);
    double max_yawrate = std::min(odom_.twist.twist.angular.z + MAX_D_YAWRATE * TIME_DEFFERENCE, MAX_YAWRATE);

    State init_state = get_init_state();
    for (double velocity = min_velocity; velocity <= max_velocity; velocity += VELOCITY_RESOLUTION) {
        for (double yawrate = min_yawrate; yawrate <= max_yawrate; yawrate += YAWRATE_RESOLUTION) {
            nav_msgs::Path path;
            path.header = pose_.header;
            simulate_trajectory(velocity, yawrate, init_state, &path);
            paths_ptr->pathes.push_back(path);
        }
    }
}

TrajectoryGenerator::State TrajectoryGenerator::get_init_state() {
    State init_state;
    init_state.x = pose_.pose.pose.position.x;
    init_state.y = pose_.pose.pose.position.y;
    tf2::convert(pose_.pose.pose.orientation, init_state.quaternion);
    init_state.velocity = odom_.twist.twist.linear.x;
    init_state.yawrate = odom_.twist.twist.angular.z;
    return init_state;
}

void TrajectoryGenerator::simulate_trajectory(double velocity, double yawrate, const State& init_state,
                                              nav_msgs::Path* path_ptr) {
    State state = init_state;
    for (double time = 0; time <= PREDICT_TIME; time += TIME_DEFFERENCE) {
        motion(velocity, yawrate, &state);
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = pose_.header;
        pose_stamped.pose.position.x = state.x;
        pose_stamped.pose.position.y = state.y;
        pose_stamped.pose.position.z = 0;
        tf2::convert(state.quaternion, pose_stamped.pose.orientation);
        path_ptr->poses.push_back(pose_stamped);
    }
}

void TrajectoryGenerator::motion(const double velocity, const double yawrate, State* state) {
    double yaw = tf2::getYaw(state->quaternion);
    yaw += yawrate * TIME_DEFFERENCE;
    state->x += velocity * TIME_DEFFERENCE * std::cos(yaw);
    state->y += velocity * TIME_DEFFERENCE * std::sin(yaw);
    tf2::Quaternion quat;
    quat.setEuler(0.0, 0.0, yawrate * TIME_DEFFERENCE);
    state->quaternion *= quat;
    state->velocity = velocity;
    state->yawrate = yawrate;
}

void TrajectoryGenerator::visualize_trajectories(const trajectory_generator::PathArray& paths) {
    static const int max_trajectories_size = 1000;
    ROS_ASSERT(paths.pathes.size() < max_trajectories_size);
    visualization_msgs::MarkerArray trajectories;
    for (int i = 0; i < max_trajectories_size; i++) {
        if (i < paths.pathes.size()) {
            visualization_msgs::Marker trajectory;
            trajectory.header.frame_id = pose_.header.frame_id;
            trajectory.header.stamp = ros::Time::now();
            trajectory.color.r = 1;
            trajectory.color.g = 0;
            trajectory.color.b = 0;
            trajectory.color.a = 0.8;
            trajectory.ns = visualize_trajectories_pub_.getTopic();
            trajectory.type = visualization_msgs::Marker::LINE_STRIP;
            trajectory.action = visualization_msgs::Marker::ADD;
            trajectory.lifetime = ros::Duration();
            trajectory.id = i;
            trajectory.scale.x = 0.02;
            geometry_msgs::Pose pose;
            pose.orientation.w = 1;
            trajectory.pose = pose;
            geometry_msgs::Point p;
            for (const auto& pose : paths.pathes[i].poses) {
                p.x = pose.pose.position.x;
                p.y = pose.pose.position.y;
                trajectory.points.push_back(p);
            }
            trajectories.markers.push_back(trajectory);
        } else {
            visualization_msgs::Marker trajectory;
            trajectory.header.frame_id = ROBOT_FRAME;
            trajectory.header.stamp = ros::Time::now();
            trajectory.ns = visualize_trajectories_pub_.getTopic();
            trajectory.type = visualization_msgs::Marker::LINE_STRIP;
            trajectory.action = visualization_msgs::Marker::DELETE;
            trajectory.lifetime = ros::Duration();
            trajectory.id = i;
            trajectories.markers.push_back(trajectory);
        }
    }
    visualize_trajectories_pub_.publish(trajectories);
}

void TrajectoryGenerator::process() {
    ros::Rate loop_rate(HZ);

    while (ros::ok()) {
        double start_time = ros::Time::now().toSec();
        if (scan_updated_ && pose_updated_ && odom_updated_) {
            trajectory_generator::PathArray paths;
            paths.header = pose_.header;
            paths.my_number = ROOMBA.back() - '0';
            calc_trajectories(&paths);
            visualize_trajectories(paths);
        } else {
            if (!scan_updated_) {
                ROS_WARN_THROTTLE(1.0, "Scan has not been updated");
            }
            if (!pose_updated_) {
                ROS_WARN_THROTTLE(1.0, "Pose has not been updated");
            }
            if (!odom_updated_) {
                ROS_WARN_THROTTLE(1.0, "Odom has not been updated");
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
        ROS_DEBUG_STREAM("loop time : " << ros::Time::now().toSec() - start_time << "[sec]");
    }
}
