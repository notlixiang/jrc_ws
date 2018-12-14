#include <ros/ros.h>
#include <jrc_srvs/smooth.h>
#include <jrc_srvs/smooth_multi.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Pose.h>
#include <map>

using namespace std;
geometry_msgs::Pose point[5];

void  get_point_parameter()
{
//起始点
point[0].position.x = -1.181;
point[0].position.y = -0.060;
point[0].position.z = 0.0;
point[0].orientation.x = 0.0;
point[0].orientation.y = 0.0;
point[0].orientation.z = -0.025;
point[0].orientation.w = 0.9999;

//第一个抓取点
point[1].position.x = 0.003;
point[1].position.y =-0.004;
point[1].position.z = 0.0;
point[1].orientation.x = 0.0;
point[1].orientation.y = 0.0;
point[1].orientation.z = -0.025;
point[1].orientation.w = 0.9999;

//第二个抓取点
point[2].position.x = 0.003;
point[2].position.y = -0.004;
point[2].position.z = 0.0;
point[2].orientation.x = 0.0;
point[2].orientation.y = 0.0;
point[2].orientation.z = 0.9999;
point[2].orientation.w = 0.004;

//the third point
point[3].position.x = 3.037;  
point[3].position.y = 0.172;
point[3].position.z = 0.0;
point[3].orientation.x = 0.0;
point[3].orientation.y = 0.0;
point[3].orientation.z = 0.9999;
point[3].orientation.w = 0.004;

//the third point
point[4].position.x = 3.037;  
point[4].position.y = 0.172;
point[4].position.z = 0.0;
point[4].orientation.x = 0.0;
point[4].orientation.y = 0.0;
point[4].orientation.z = -0.002;
point[4].orientation.w = 0.9999;
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
    ros::ServiceClient client = node.serviceClient<jrc_srvs::smooth>("agv_move"); 
    jrc_srvs::smooth srv;
    get_point_parameter();
    get_srv(srv, point[1]);
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

    get_srv(srv, point[2]);
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

    get_srv(srv, point[3]);
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

     get_srv(srv, point[4]);
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

 get_srv(srv, point[0]);
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
