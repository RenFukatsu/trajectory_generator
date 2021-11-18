#include "trajectory_generator/trajectory_generator.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_generator");
    TrajectoryGenerator trajectory_generator;
    trajectory_generator.process();
    return 0;
}
