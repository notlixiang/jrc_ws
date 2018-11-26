#include <ros/ros.h>
#include <jrc_srvs/smooth.h>
#include <jrc_srvs/smooth_multi.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Pose.h>
#include <map>

using namespace std;
geometry_msgs::Pose point[4];

void  get_point_parameter()
{
//起始点
point[0].position.x = 2.195;
point[0].position.y = -0.438;
point[0].position.z = 0.0;
point[0].orientation.x = 0.0;
point[0].orientation.y = 0.0;
point[0].orientation.z = 0.662;
point[0].orientation.w = 0.749;

//第一个抓取点
point[1].position.x = 2.045;
point[1].position.y = 0.16;
point[1].position.z = 0.0;
point[1].orientation.x = 0.0;
point[1].orientation.y = 0.0;
point[1].orientation.z = -0.031;
point[1].orientation.w = 0.999;

//第二个抓取点
point[2].position.x = -3.049;
point[2].position.y = 0.0;
point[2].position.z = 0.0;
point[2].orientation.x = 0.0;
point[2].orientation.y = 0.0;
point[2].orientation.z = 0.752;
point[2].orientation.w = 0.659;

//返回过渡点
point[3].position.x = 1.6;  //change here
point[3].position.y = -0.438;
point[3].position.z = 0.0;
point[3].orientation.x = 0.0;
point[3].orientation.y = 0.0;
point[3].orientation.z = 0.662;
point[3].orientation.w = 0.749;
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

void get_srv_multi(jrc_srvs::smooth_multi& srv_multi, std::vector<geometry_msgs::Pose> & points)
{
  // int size;
 //  size=points.size();
   std::vector<geometry_msgs::Quaternion> quats;
   std::vector<geometry_msgs::Pose>::iterator it;
   for (it = points.begin(); it != points.end(); it++)
   {   geometry_msgs::Quaternion quat;
      // cout << *it << endl;
       quat.x = (*it).orientation.x;
       quat.y = (*it).orientation.y;
       quat.z = (*it).orientation.z;
       quat.w = (*it).orientation.w;
       quats.push_back(quat);
       srv_multi.request.x_multi.push_back((*it).position.x);
       srv_multi.request.y_multi.push_back((*it).position.y);
       srv_multi.request.theta_multi.push_back(tf::getYaw(quat));
   }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "velocity_smooth_client");
    ros::NodeHandle node;
    ros::ServiceClient client = node.serviceClient<jrc_srvs::smooth>("/Jrc_move"); 
    jrc_srvs::smooth srv;
    get_point_parameter();
    get_srv(srv, point[1]);
    //**********************************multiple poins exist***********************************//
    //when multiple points exist in one single path
//    ros::ServiceClient client_multi=node.serviceClient<velocity_smooth::smooth_srv_multi>("/Jrc_move_multi");
//    velocity_smooth::smooth_srv_multi srv_multi;
//    std::vector<geometry_msgs::Pose>  points;
//    points.push_back(point[1]);//start point
//    points.push_back(point[2]);//path point 1
//    points.push_back(point[3]);//path point 2
//    points.push_back(point[4]);//ending point
    //...........................
    //................
    //........
//    get_srv_multi(srv_multi, points);
    // ROS_INFO("SUCCESS");
      sleep(5);
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
