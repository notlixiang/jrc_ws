#include <iostream>
#include "ros/ros.h"
#include "jrc_srvs/pose.h"


using namespace std;

int main(int argc, char** argv) {
	cout << "Hello JRC" << endl;
    ros::init(argc, argv, "jrc_mian_node");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<jrc_srvs::pose>("pose_server");
    jrc_srvs::pose srv;
    ros::Rate loop_rate(200);
    int i = 1;
    int j = 1;
    while (true) {
        srv.request.a = i;
        srv.request.b = j;

        if (client.call(srv))
        {
            ROS_INFO("Sum: %ld", (long int)srv.response.product);
        }
        else
        {
            ROS_ERROR("Failed to call service add_two_ints");
        }
        loop_rate.sleep();
    }

    return 0;

}