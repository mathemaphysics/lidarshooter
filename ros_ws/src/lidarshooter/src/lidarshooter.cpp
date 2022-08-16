#include <ros/ros.h>

#include "MeshProjector.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pandar");
    lidarshooter::MeshProjector meshProjector;
    ros::spin();

    return 0;
}
