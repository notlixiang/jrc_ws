#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include "../../../devel/include/msg_package/grasp.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_main_node");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<msg_package::grasp>("ur_grasp");
    msg_package::grasp srv;

        srv.request.mode = 3;
        srv.request.agv_position = 1;
        if (client.call(srv))
        {
            if(!srv.response.result.data)
            {
                ROS_ERROR("Failed to move to place");
                return 0;
            }
        }
        else
        {
          ROS_ERROR("Failed to call service");
          return 0;
        }


    geometry_msgs::Pose pose;

    pose.position.x = -0.5;
    pose.position.y = -1.2;
    pose.position.z = -0.24;
    pose.orientation.z = 1;

    srv.request.pose = pose;
    while(srv.request.pose.position.x<=-0.2)
    {
        //observe
        srv.request.mode = 5;
        srv.request.agv_position = 1;
        if (client.call(srv))
        {
            if(!srv.response.result.data)
            {
                ROS_ERROR("Failed to move to observe");
                return 0;
            }
        }
        else
        {
          ROS_ERROR("Failed to call service");
          return 0;
        }

        //grasp
        srv.request.mode = 1;
        srv.request.agv_position = 1;
        srv.request.id = 1;
        if (client.call(srv))
        {
            if(!srv.response.result.data)
            {
                ROS_ERROR("Failed to grasp");
                return 0;
            }
        }
        else
        {
          ROS_ERROR("Failed to call service");
          return 0;
        }

        //place
        srv.request.mode = 2;
        srv.request.agv_position = 1;
        if (client.call(srv))
        {
            if(!srv.response.result.data)
            {
                ROS_ERROR("Failed to place");
                return 0;
            }
        }
        else
        {
          ROS_ERROR("Failed to call service");
          return 0;
        }

        srv.request.mode = 3;
        srv.request.agv_position = 1;
        if (client.call(srv))
        {
            if(!srv.response.result.data)
            {
                ROS_ERROR("Failed to move to place");
                return 0;
            }
        }
        else
        {
          ROS_ERROR("Failed to call service");
          return 0;
        }

        srv.request.pose.position.x += 0.1;
    }





    pose.position.x = -0.18;
    pose.position.y = -1.2;
    pose.position.z = -0.24;
    pose.orientation.z = 1;

    srv.request.pose = pose;
    while(srv.request.pose.position.x<=0.2)
    {
        //observe
        srv.request.mode = 7;
        srv.request.agv_position = 1;
        if (client.call(srv))
        {
            if(!srv.response.result.data)
            {
                ROS_ERROR("Failed to move to observe");
                return 0;
            }
        }
        else
        {
          ROS_ERROR("Failed to call service");
          return 0;
        }

        //grasp
        srv.request.mode = 1;
        srv.request.agv_position = 1;
        srv.request.id = 3;
        if (client.call(srv))
        {
            if(!srv.response.result.data)
            {
                ROS_ERROR("Failed to grasp");
                return 0;
            }
        }
        else
        {
          ROS_ERROR("Failed to call service");
          return 0;
        }

        //place
        srv.request.mode = 2;
        srv.request.agv_position = 1;
        if (client.call(srv))
        {
            if(!srv.response.result.data)
            {
                ROS_ERROR("Failed to place");
                return 0;
            }
        }
        else
        {
          ROS_ERROR("Failed to call service");
          return 0;
        }

        srv.request.mode = 3;
        srv.request.agv_position = 1;
        if (client.call(srv))
        {
            if(!srv.response.result.data)
            {
                ROS_ERROR("Failed to move to place");
                return 0;
            }
        }
        else
        {
          ROS_ERROR("Failed to call service");
          return 0;
        }

        srv.request.pose.position.x += 0.1;
    }




    pose.position.x = 0.23;
    pose.position.y = -1.2;
    pose.position.z = -0.24;
    pose.orientation.z = 1;

    srv.request.pose = pose;
    while(srv.request.pose.position.x<=0.5)
    {
        //observe
        srv.request.mode = 9;
        srv.request.agv_position = 1;
        if (client.call(srv))
        {
            if(!srv.response.result.data)
            {
                ROS_ERROR("Failed to move to observe");
                return 0;
            }
        }
        else
        {
          ROS_ERROR("Failed to call service");
          return 0;
        }

        //grasp
        srv.request.mode = 1;
        srv.request.agv_position = 1;
        srv.request.id = 3;
        if (client.call(srv))
        {
            if(!srv.response.result.data)
            {
                ROS_ERROR("Failed to grasp");
                return 0;
            }
        }
        else
        {
          ROS_ERROR("Failed to call service");
          return 0;
        }

        //place
        srv.request.mode = 2;
        srv.request.agv_position = 1;
        if (client.call(srv))
        {
            if(!srv.response.result.data)
            {
                ROS_ERROR("Failed to place");
                return 0;
            }
        }
        else
        {
          ROS_ERROR("Failed to call service");
          return 0;
        }

        srv.request.mode = 3;
        srv.request.agv_position = 1;
        if (client.call(srv))
        {
            if(!srv.response.result.data)
            {
                ROS_ERROR("Failed to move to place");
                return 0;
            }
        }
        else
        {
          ROS_ERROR("Failed to call service");
          return 0;
        }

        srv.request.pose.position.x += 0.1;
    }



    pose.position.x = -0.50;
    pose.position.y = -1.3;
    pose.position.z = 0.36;
    pose.orientation.z = 1;

    srv.request.pose = pose;
    while(srv.request.pose.position.x<=-0.2)
    {
        //observe
        srv.request.mode = 4;
        srv.request.agv_position = 1;
        if (client.call(srv))
        {
            if(!srv.response.result.data)
            {
                ROS_ERROR("Failed to move to observe");
                return 0;
            }
        }
        else
        {
          ROS_ERROR("Failed to call service");
          return 0;
        }

        //grasp
        srv.request.mode = 1;
        srv.request.agv_position = 1;
        srv.request.id = 3;
        if (client.call(srv))
        {
            if(!srv.response.result.data)
            {
                ROS_ERROR("Failed to grasp");
                return 0;
            }
        }
        else
        {
          ROS_ERROR("Failed to call service");
          return 0;
        }

        //place
        srv.request.mode = 2;
        srv.request.agv_position = 1;
        if (client.call(srv))
        {
            if(!srv.response.result.data)
            {
                ROS_ERROR("Failed to place");
                return 0;
            }
        }
        else
        {
          ROS_ERROR("Failed to call service");
          return 0;
        }

        srv.request.mode = 3;
        srv.request.agv_position = 1;
        if (client.call(srv))
        {
            if(!srv.response.result.data)
            {
                ROS_ERROR("Failed to move to observe");
                return 0;
            }
        }
        else
        {
          ROS_ERROR("Failed to call service");
          return 0;
        }

        srv.request.pose.position.x += 0.1;
    }



    pose.position.x = -0.18;
    pose.position.y = -1.2;
    pose.position.z = 0.36;
    pose.orientation.z = 1;

    srv.request.pose = pose;
    while(srv.request.pose.position.x<=0.2)
    {
        //observe
        srv.request.mode = 6;
        srv.request.agv_position = 1;
        if (client.call(srv))
        {
            if(!srv.response.result.data)
            {
                ROS_ERROR("Failed to move to observe");
                return 0;
            }
        }
        else
        {
          ROS_ERROR("Failed to call service");
          return 0;
        }

        //grasp
        srv.request.mode = 1;
        srv.request.agv_position = 1;
        srv.request.id = 3;
        if (client.call(srv))
        {
            if(!srv.response.result.data)
            {
                ROS_ERROR("Failed to grasp");
                return 0;
            }
        }
        else
        {
          ROS_ERROR("Failed to call service");
          return 0;
        }

        //place
        srv.request.mode = 2;
        srv.request.agv_position = 1;
        if (client.call(srv))
        {
            if(!srv.response.result.data)
            {
                ROS_ERROR("Failed to place");
                return 0;
            }
        }
        else
        {
          ROS_ERROR("Failed to call service");
          return 0;
        }

        srv.request.mode = 3;
        srv.request.agv_position = 1;
        if (client.call(srv))
        {
            if(!srv.response.result.data)
            {
                ROS_ERROR("Failed to move to place");
                return 0;
            }
        }
        else
        {
          ROS_ERROR("Failed to call service");
          return 0;
        }

        srv.request.pose.position.x += 0.1;
    }



    pose.position.x = 0.22;
    pose.position.y = -1.3;
    pose.position.z = 0.36;
    pose.orientation.z = 1;

    srv.request.pose = pose;
    while(srv.request.pose.position.x<=0.5)
    {
        //observe
        srv.request.mode = 8;
        srv.request.agv_position = 1;
        if (client.call(srv))
        {
            if(!srv.response.result.data)
            {
                ROS_ERROR("Failed to move to observe");
                return 0;
            }
        }
        else
        {
          ROS_ERROR("Failed to call service");
          return 0;
        }

        //grasp
        srv.request.mode = 1;
        srv.request.agv_position = 1;
        srv.request.id = 3;
        if (client.call(srv))
        {
            if(!srv.response.result.data)
            {
                ROS_ERROR("Failed to grasp");
                return 0;
            }
        }
        else
        {
          ROS_ERROR("Failed to call service");
          return 0;
        }

        //place
        srv.request.mode = 2;
        srv.request.agv_position = 1;
        if (client.call(srv))
        {
            if(!srv.response.result.data)
            {
                ROS_ERROR("Failed to place");
                return 0;
            }
        }
        else
        {
          ROS_ERROR("Failed to call service");
          return 0;
        }

        srv.request.mode = 3;
        srv.request.agv_position = 1;
        if (client.call(srv))
        {
            if(!srv.response.result.data)
            {
                ROS_ERROR("Failed to move to place");
                return 0;
            }
        }
        else
        {
          ROS_ERROR("Failed to call service");
          return 0;
        }

        srv.request.pose.position.x += 0.1;
    }










/*
    pose.position.x = -0.4;
    pose.position.y = -1.1;
    pose.position.z = 0.2;
    pose.orientation.z = 1;

    srv.request.pose = pose;
    while(srv.request.pose.position.x<=-0.2)
    {
        //observe
        srv.request.mode = 10;
        srv.request.agv_position = 2;
        if (client.call(srv))
        {
            if(!srv.response.result.data)
            {
                ROS_ERROR("Failed to move to observe");
                return 0;
            }
        }
        else
        {
          ROS_ERROR("Failed to call service");
          return 0;
        }

        //grasp
        srv.request.mode = 1;
        srv.request.agv_position = 2;
        srv.request.id = 1;
        if (client.call(srv))
        {
            if(!srv.response.result.data)
            {
                ROS_ERROR("Failed to grasp");
                return 0;
            }
        }
        else
        {
          ROS_ERROR("Failed to call service");
          return 0;
        }

        //place
        srv.request.mode = 2;
        srv.request.agv_position = 2;
        if (client.call(srv))
        {
            if(!srv.response.result.data)
            {
                ROS_ERROR("Failed to place");
                return 0;
            }
        }
        else
        {
          ROS_ERROR("Failed to call service");
          return 0;
        }

        srv.request.pose.position.x += 0.1;
		std::cout<<"position.x: "<<srv.request.pose.position.x<<std::endl;
    }





    pose.position.x = -0.18;
    pose.position.y = -1.1;
    pose.position.z = 0.2;
    pose.orientation.z = 1;

    srv.request.pose = pose;
    while(srv.request.pose.position.x<=0.2)
    {
        //observe
        srv.request.mode = 11;
        srv.request.agv_position = 2;
        if (client.call(srv))
        {
            if(!srv.response.result.data)
            {
                ROS_ERROR("Failed to move to observe");
                return 0;
            }
        }
        else
        {
          ROS_ERROR("Failed to call service");
          return 0;
        }

        //grasp
        srv.request.mode = 1;
        srv.request.agv_position = 2;
        srv.request.id = 3;
        if (client.call(srv))
        {
            if(!srv.response.result.data)
            {
                ROS_ERROR("Failed to grasp");
                return 0;
            }
        }
        else
        {
          ROS_ERROR("Failed to call service");
          return 0;
        }

        //place
        srv.request.mode = 2;
        srv.request.agv_position = 2;
        if (client.call(srv))
        {
            if(!srv.response.result.data)
            {
                ROS_ERROR("Failed to place");
                return 0;
            }
        }
        else
        {
          ROS_ERROR("Failed to call service");
          return 0;
        }

        srv.request.pose.position.x += 0.1;
std::cout<<"position.x: "<<srv.request.pose.position.x<<std::endl;
    }




    pose.position.x = 0.23;
    pose.position.y = -1.1;
    pose.position.z = 0.2;
    pose.orientation.z = 1;

    srv.request.pose = pose;
    while(srv.request.pose.position.x<=0.4)
    {
        //observe
        srv.request.mode = 12;
        srv.request.agv_position = 2;
        if (client.call(srv))
        {
            if(!srv.response.result.data)
            {
                ROS_ERROR("Failed to move to observe");
                return 0;
            }
        }
        else
        {
          ROS_ERROR("Failed to call service");
          return 0;
        }

        //grasp
        srv.request.mode = 1;
        srv.request.agv_position = 2;
        srv.request.id = 3;
        if (client.call(srv))
        {
            if(!srv.response.result.data)
            {
                ROS_ERROR("Failed to grasp");
                return 0;
            }
        }
        else
        {
          ROS_ERROR("Failed to call service");
          return 0;
        }

        //place
        srv.request.mode = 2;
        srv.request.agv_position = 2;
        if (client.call(srv))
        {
            if(!srv.response.result.data)
            {
                ROS_ERROR("Failed to place");
                return 0;
            }
        }
        else
        {
          ROS_ERROR("Failed to call service");
          return 0;
        }

        srv.request.pose.position.x += 0.1;
    }

*/




	return 0;
}
