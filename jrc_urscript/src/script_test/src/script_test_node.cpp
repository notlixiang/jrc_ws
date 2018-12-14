#include <iostream>
#include "ros/ros.h"
#include "../../../devel/include/ur_msgs/Single_JointPosition.h"
#include "../../../devel/include/ur_msgs/Single_JointPositionRequest.h"
#include "../../../devel/include/ur_msgs/Single_JointPositionResponse.h"
#include "../../../devel/include/ur_msgs/Multi_JointPosition.h"
#include "../../../devel/include/ur_msgs/Multi_JointPositionRequest.h"
#include "../../../devel/include/ur_msgs/Multi_JointPositionResponse.h"

using namespace std;

int main(int argc,char **argv)
{
    ros::init(argc, argv, "script_test_node");

    ros::NodeHandle n;
    ros::ServiceClient single_joint_client = n.serviceClient<ur_msgs::Single_JointPosition>("ur_driver/single_joint_position");
    ros::ServiceClient multi_joint_client = n.serviceClient<ur_msgs::Multi_JointPosition>("ur_driver/multi_joint_position");
    ros::ServiceClient movel_joint_client = n.serviceClient<ur_msgs::Single_JointPosition>("ur_driver/movel_joint_position");

    ur_msgs::Single_JointPosition single_joint_srv;
    ur_msgs::Multi_JointPosition multi_joint_srv;
    ur_msgs::Single_JointPosition movel_joint_srv;

    single_joint_srv.request.positions.resize(6);
    //single_joint_srv.request.time = 1;
    single_joint_srv.request.positions[0] = -0.437;
    single_joint_srv.request.positions[1] = -1;
    single_joint_srv.request.positions[2] = -1.62;
    single_joint_srv.request.positions[3] = 1.3625;
    single_joint_srv.request.positions[4] = -1.1244;
    single_joint_srv.request.positions[5] = -0.6764;

    if (single_joint_client.call(single_joint_srv))
    {
      ROS_INFO("call success");
      cout<<"execute result: "<<(bool)single_joint_srv.response.result<<endl;
    }
    else
    {
      ROS_ERROR("Failed to call service");
      return -1;
    }

//    multi_joint_srv.request.points.resize(3);
//    multi_joint_srv.request.points[0].positions.resize(6);
//    //multi_joint_srv.request.points[0].time = 1;
//    multi_joint_srv.request.points[0].positions[0] = 3.2195;
//    multi_joint_srv.request.points[0].positions[1] = -1.554;
//    multi_joint_srv.request.points[0].positions[2] = -1.2587;
//    multi_joint_srv.request.points[0].positions[3] = -1.528;
//    multi_joint_srv.request.points[0].positions[4] = 1.75461;
//    multi_joint_srv.request.points[0].positions[5] = -0.74286;

//    multi_joint_srv.request.points[1].positions.resize(6);
//    //multi_joint_srv.request.points[1].time = 1;
//    multi_joint_srv.request.points[1].positions[0] = 3.92940;
//    multi_joint_srv.request.points[1].positions[1] = -1.613;
//    multi_joint_srv.request.points[1].positions[2] = -1.210;
//    multi_joint_srv.request.points[1].positions[3] = -1.489;
//    multi_joint_srv.request.points[1].positions[4] = 1.47640;
//    multi_joint_srv.request.points[1].positions[5] = -0.0861;

//    multi_joint_srv.request.points[2].positions.resize(6);
//    //multi_joint_srv.request.points[2].time = 1;
//    multi_joint_srv.request.points[2].positions[0] = 4.7663;
//    multi_joint_srv.request.points[2].positions[1] = -0.9798;
//    multi_joint_srv.request.points[2].positions[2] = -1.5433;
//    multi_joint_srv.request.points[2].positions[3] = -1.987;
//    multi_joint_srv.request.points[2].positions[4] = 1.2120;
//    multi_joint_srv.request.points[2].positions[5] = 0.7301;

//    if (multi_joint_client.call(multi_joint_srv))
//    {
//      ROS_INFO("call success");
//      cout<<"execute result: "<<(bool)multi_joint_srv.response.result<<endl;
//    }
//    else
//    {
//      ROS_ERROR("Failed to call service");
//      return -1;
//    }


//    movel_joint_srv.request.positions.resize(6);
//    movel_joint_srv.request.time = 1;
//    movel_joint_srv.request.positions[0] = 3.2195;
//    movel_joint_srv.request.positions[1] = -1.554;
//    movel_joint_srv.request.positions[2] = -1.2587;
//    movel_joint_srv.request.positions[3] = -1.528;
//    movel_joint_srv.request.positions[4] = 1.75461;
//    movel_joint_srv.request.positions[5] = -0.74286;

//    if (movel_joint_client.call(movel_joint_srv))
//    {
//      ROS_INFO("call success");
//      cout<<"execute result: "<<(bool)movel_joint_srv.response.result<<endl;
//    }
//    else
//    {
//      ROS_ERROR("Failed to call service");
//      return -1;
//    }

	return 0;
}
