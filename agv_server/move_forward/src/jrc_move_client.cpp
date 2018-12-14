#include <ros/ros.h>
#include <jrc_srvs/smooth.h>
#include <jrc_srvs/smooth_multi.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Pose.h>
using namespace std;
geometry_msgs::Pose point[4];

void  get_point_parameter()
{
//起始点
point[0].position.x = -0.964;
point[0].position.y = -0.069;
point[0].position.z = 0.0;
point[0].orientation.x = 0.0;
point[0].orientation.y = 0.0;
point[0].orientation.z = -0.691;
point[0].orientation.w = 0.723;

//第一个抓取点
point[1].position.x = 3.037;
point[1].position.y = 0.172;
point[1].position.z = 0.0;
point[1].orientation.x = 0.0;
point[1].orientation.y = 0.0;
point[1].orientation.z = -0.002;
point[1].orientation.w = 0.9999;

//第二个抓取点
point[2].position.x = -0.964;
point[2].position.y = -0.069;
point[2].position.z = 0.0;
point[2].orientation.x = 0.0;
point[2].orientation.y = 0.0;
point[2].orientation.z = -0.002;
point[2].orientation.w = 0.9999;

//返回过渡点
point[3].position.x = 3.037;  //change here
point[3].position.y = 0.172;
point[3].position.z = 0.0;
point[3].orientation.x = 0.0;
point[3].orientation.y = 0.0;
point[3].orientation.z = -0.002;
point[3].orientation.w = 0.9999;
}



void get_srv(jrc_srvs::smooth& srv, geometry_msgs::Pose& point)
{
    geometry_msgs::Quaternion quat1;
    quat1.x = point.orientation.x;
    quat1.y = point.orientation.y;
    quat1.z = point.orientation.z;
    quat1.w = point.orientation.w;

    srv.request.x = point.position.x;
    srv.request.y = point.position.y;
    srv.request.theta = tf::getYaw(quat1);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "Jrc_move_client");
    ros::NodeHandle node;
    ros::ServiceClient client = node.serviceClient<jrc_srvs::smooth>("agv_move");
    jrc_srvs::smooth srv;
    get_point_parameter();
    get_srv(srv, point[2]);
   //ROS_INFO("SUCCESS");
    sleep(4);
    if(client.call(srv))
    {
        ROS_INFO("success");
      ROS_INFO("the response is %ld", (long int)srv.response.mark);
    }
    else
    {
        ROS_ERROR("Failed");
    }

    get_srv(srv, point[1]);

    sleep(10);
    if(client.call(srv))
    {
        ROS_INFO("success");
       ROS_INFO("the response is %ld", (long int)srv.response.mark);
    }
    else
    {
        ROS_ERROR("Failed");
    }

//返回过渡
 get_srv(srv, point[2]);

    sleep(10);
    if(client.call(srv))
    {
        ROS_INFO("success");
        ROS_INFO("the response is %ld", (long int)srv.response.mark);
    }
    else
    {
        ROS_ERROR("Failed");
    }


    return 0;
}
