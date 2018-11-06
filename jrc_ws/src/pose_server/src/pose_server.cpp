//
// Created by sirius on 18-11-3.
//

#include "ros/ros.h"
#include "jrc_srvs/pose.h"

bool pose(jrc_srvs::pose::Request  &req,
          jrc_srvs::pose::Response &res)
{
    res.product = req.a * req.b;
    ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
    ROS_INFO("sending back response: [%ld]", (long int)res.product);
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_server_node");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("pose_server", pose);
    ROS_INFO("Ready to add two ints.");
    ros::spin();

    return 0;
}