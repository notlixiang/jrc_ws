#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

double max_num[6] = {-999,-999,-999,-999,-999,-999};
double min_num[6] = {999,999,999,999,999,999};

void Callback(const sensor_msgs::JointState msg)
{
    for(int i=0;i<6;i++)
    {
        if(msg.position[i]>max_num[i])
        {
            max_num[i] = msg.position[i];
        }

        if(msg.position[i]<min_num[i])
        {
            min_num[i] = msg.position[i];
        }
    }

	std::cout<<"max_num: "<<std::endl;
    for(int i=0;i<6;i++)
    {
        std::cout<<max_num[i]<<"     ";
    }
    std::cout<<std::endl;

	std::cout<<"min_num: "<<std::endl;
    for(int i=0;i<6;i++)
    {
        std::cout<<min_num[i]<<"     ";
    }
    std::cout<<std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("joint_states", 1000, Callback);

    ros::spin();

	return 0;
}
