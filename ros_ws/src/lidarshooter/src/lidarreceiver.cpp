#include <ros/ros.h>
#include <somenode/StringWithHeader.h>
#include <iostream>

void callbackFunction(const somenode::StringWithHeaderConstPtr& msg)
{
    std::cout << msg->data << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "somereader");

    ros::NodeHandle nodeHandle;

    ros::Subscriber subscriber = nodeHandle.subscribe("somenode_topic", 1, &callbackFunction);

    ros::spin();
}
