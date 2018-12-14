#include <iostream>
#include "ros/ros.h"
#include <tf/transform_listener.h>
#include "jrc_srvs/rgbd_image.h"
#include "jrc_srvs/bbox_msgs.h"
#include "jrc_srvs/obj_6d.h"
#include "jrc_srvs/call_grasp.h"
#include "jrc_srvs/call_grasp_state.h"
#include "jrc_srvs/call_twist.h"
#include "jrc_srvs/grasp.h"
#include "jrc_srvs/smooth.h"


using namespace std;
int object_list[10] = {1,9,3,11,5,13,7,15,4,10};
int object_status[10]  = {0,0,0,0,0,0,0,0,0,0}; //0 is un-grasped; 
                                                //1 is grasped; 
                                                //2 is not pick in this around; 
                                                //3 is need detect again;
                                                //4 is need grasp again
//agv postion point
geometry_msgs::Pose point[5];

class object_info
{
public:
    std::vector<int32_t> info;
};

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
point[1].position.y =0.104;
point[1].position.z = 0.0;
point[1].orientation.x = 0.0;
point[1].orientation.y = 0.0;
point[1].orientation.z = -0.025;
point[1].orientation.w = 0.9999;

//第二个抓取点
point[2].position.x = 0.003;
point[2].position.y = 0.104;
point[2].position.z = 0.0;
point[2].orientation.x = 0.0;
point[2].orientation.y = 0.0;
point[2].orientation.z = 0.9999;
point[2].orientation.w = 0.004;

//the third point
point[3].position.x = 3.037;  
point[3].position.y = -0.158;
point[3].position.z = 0.0;
point[3].orientation.x = 0.0;
point[3].orientation.y = 0.0;
point[3].orientation.z = 0.9999;
point[3].orientation.w = 0.004;

//the forth point
point[4].position.x = 3.037;  
point[4].position.y = -0.1582;
point[4].position.z = 0.0;
point[4].orientation.x = 0.0;
point[4].orientation.y = 0.0;
point[4].orientation.z = -0.002;
point[4].orientation.w = 0.9999;
}

int object_detection(ros::ServiceClient detect_client, 
                     int &bbox_label, int &bbox_xmin, int &bbox_ymin, int &bbox_xmax, int &bbox_ymax)
{
    bbox_label = -1;
    bbox_xmin  = 0;
    bbox_ymin  = 0;
    bbox_xmax  = 0;
    bbox_ymax  = 0;
    jrc_srvs::bbox_msgs detect_srv;
    object_info objects[16];
    detect_srv.request.start = 1;
    if(detect_client.call(detect_srv))
    {
        cout<<"Get detection result!"<<endl;
        objects[0].info  = detect_srv.response.object1;
        objects[1].info  = detect_srv.response.object2;
        objects[2].info  = detect_srv.response.object3;
        objects[3].info  = detect_srv.response.object4;
        objects[4].info  = detect_srv.response.object5;
        objects[5].info  = detect_srv.response.object6;
        objects[6].info  = detect_srv.response.object7;
        objects[7].info  = detect_srv.response.object8;
        objects[8].info  = detect_srv.response.object9;
        objects[9].info  = detect_srv.response.object10;
        objects[10].info = detect_srv.response.object11;
        objects[11].info = detect_srv.response.object12;
        objects[12].info = detect_srv.response.object13;
        objects[13].info = detect_srv.response.object14;
        objects[14].info = detect_srv.response.object15;
        objects[15].info = detect_srv.response.object16;
        for(int i=0;i<16;i++)
        {
            if(objects[i].info.size()>0)
            {
                for(int j=0;j<10;j++)
                {
                    if((object_list[j] == objects[i].info[0])&&((object_status[j] == 0)||(object_status[j] == 3)||(object_status[j] == 4)))
                    {
                        // object_status[j] = 2;
                        bbox_label = objects[i].info[0];
                        bbox_xmin  = objects[i].info[1];
                        bbox_ymin  = objects[i].info[2];
                        bbox_xmax  = objects[i].info[3];
                        bbox_ymax  = objects[i].info[4];
                        cout<<"Grasp object ID:"<<bbox_label<<endl;
                        i = 17;
                        break;
                    }
                }
            }
        }
        cout<<"Object label:"<<bbox_label<<" xmin:"<<bbox_xmin<<" ymin:"<<bbox_ymin
            <<" xmax:"<<bbox_xmax<<" ymax:"<<bbox_ymax<<endl;
        cout<<"Finish detection service."<<endl;
        return 1;
    }
    else
    {
        cout<<"Error in object detection."<<endl;
        return 0;
    }
}

int pose_estimation(ros::ServiceClient pose_client,
                    int bbox_label, int bbox_xmin, int bbox_ymin, int bbox_xmax, int bbox_ymax,
                    int &object_label, geometry_msgs::Pose &object_pose)
{
    jrc_srvs::obj_6d pose_srv;
    pose_srv.request.start = 1;
    pose_srv.request.label = bbox_label;
    pose_srv.request.xmin  = bbox_xmin;
    pose_srv.request.ymin  = bbox_ymin;
    pose_srv.request.xmax  = bbox_xmax;
    pose_srv.request.ymax  = bbox_ymax;
    if(pose_client.call(pose_srv))
    {
        object_label = pose_srv.response.label;
        object_pose = pose_srv.response.obj_pose;
        cout<<"Object_label:"<<object_label<<endl<<" object pose:"<< object_pose <<endl;
        cout<<"Finish pose service."<<endl;
        return 1;
    }
    else
    {
        cout<<"Error in object pose estimation."<<endl;
        return 0;
    }
}

int pose_center_normal_estimation(ros::ServiceClient pose_center_normal_client,
                                  int bbox_label, int bbox_xmin, int bbox_ymin, int bbox_xmax, int bbox_ymax,
                                  int &object_label, geometry_msgs::Pose &object_pose)
{
    jrc_srvs::obj_6d pose_srv;
    pose_srv.request.start = 1;
    pose_srv.request.label = bbox_label;
    pose_srv.request.xmin  = bbox_xmin;
    pose_srv.request.ymin  = bbox_ymin;
    pose_srv.request.xmax  = bbox_xmax;
    pose_srv.request.ymax  = bbox_ymax;
    if(pose_center_normal_client.call(pose_srv))
    {
        object_label = pose_srv.response.label;
        object_pose = pose_srv.response.obj_pose;
        cout<<"Object_label:"<<object_label<<endl<<" object pose:"<< object_pose <<endl;
        cout<<"Finish pose service."<<endl;
        return 1;
    }
    else
    {
        cout<<"Error in object pose estimation."<<endl;
        return 0;
    }
}

int ur_10_service_grasp(ros::ServiceClient ur10_client_,
                    int mode_,int agv_,int id_,geometry_msgs::Pose pose_,int &fail_status_)
{
    jrc_srvs::grasp ur10_srv_;
    ur10_srv_.request.mode = mode_;
    ur10_srv_.request.agv_position = agv_;
    ur10_srv_.request.id = id_;
    ur10_srv_.request.pose = pose_;
    if(ur10_client_.call(ur10_srv_))
    {
        if((bool)ur10_srv_.response.result.data)
        {
            fail_status_ = 0;
            cout<<"Finish ur10 service."<<endl;
            return 1;
        }
        else
        {                                                  //2 object pose is invaild
            fail_status_ = ur10_srv_.response.fail_status; //1 object is not at observe place
            ROS_ERROR("ur10 action failed!");
            return 0;
        }
    }
    else
    {
        cout<<"Error in calling service."<<endl;
        return 0;
    }
}

int ur_10_service_move(ros::ServiceClient ur10_client_,
                    int mode_,int agv_)
{
    jrc_srvs::grasp ur10_srv_;
    ur10_srv_.request.mode = mode_;
    ur10_srv_.request.agv_position = agv_;
    if(ur10_client_.call(ur10_srv_))
    {
        if((bool)ur10_srv_.response.result.data)
        {
            cout<<"Finish ur10 service."<<endl;
            return 1;
        }
        else
        {                                              
            ROS_ERROR("ur10 action failed!");
            return 0;
        }
    }
    else
    {
        cout<<"Error in calling service."<<endl;
        return 0;
    }
}


int grasp_service(ros::ServiceClient grasp_client, bool grasp_signal)
{
    jrc_srvs::call_grasp grasp_srv;
    grasp_srv.request.grasp = grasp_signal;
    if (grasp_client.call(grasp_srv))
    {
        cout<<"Finish grasp service."<<endl;
        return 1;
    }
    else
    {
        cout<<"Grasp Servrice failed."<<endl;
        return 0;
    }
}

int grasp_state_service(ros::ServiceClient grasp_state_client, bool &grasp_state)
{
    jrc_srvs::call_grasp_state grasp_state_srv;
    // grasp_state_srv.request.grasp = grasp_state_signal;
    if (grasp_state_client.call(grasp_state_srv))
    {
        if(grasp_state_srv.response.grasped == true)
        {
            cout<<"Grasp success."<<endl;
            grasp_state = 1;
            return 1;
        }
        else
        {
            cout<<"Grasp failed."<<endl;
            grasp_state = 0;
            return 1;
        }
    }
    else
    {
        cout<<"Grasp state service failed."<<endl;
        return 0;
    }
}

int twist_service(ros::ServiceClient twist_client, int object_label, int angle)
{
    jrc_srvs::call_twist twist_srv;
    if((object_label == 2) || (object_label == 5) || (object_label == 6) || (object_label == 7) || (object_label == 13) || (object_label == 14))
        angle = 70;
    else if((object_label == 1) || (object_label == 3) || (object_label == 4) || (object_label == 8) || (object_label == 9) ||
             (object_label == 10) || (object_label == 11) || (object_label == 12) || (object_label == 15) || (object_label == 16))
        angle = 180;
    twist_srv.request.angle = angle;
    if (twist_client.call(twist_srv))
    {
        sleep(1);
        cout<<"Finish twist service."<<endl;
        return 1;
    }
    else
    {
        cout<<"Twist Servrice failed."<<endl;
        return 0;
    }

}

int agv_service(ros::ServiceClient agv_client, geometry_msgs::Pose& point)
{
    jrc_srvs::smooth agv_srv;
    geometry_msgs::Quaternion quat1;
    quat1.x = point.orientation.x;
    quat1.y = point.orientation.y;
    quat1.z = point.orientation.z;
    quat1.w = point.orientation.w;

    agv_srv.request.x = point.position.x;
    agv_srv.request.y = point.position.y;
    agv_srv.request.theta = tf::getYaw(quat1);

    if(agv_client.call(agv_srv))
    {
        ROS_INFO("success");
        ROS_INFO("the response is %ld", (long int)agv_srv.response.mark);
        cout<<"Finish agv service."<<endl;
        return 1;
    }
    else
    {
        cout<<"AGV Servrice failed."<<endl;
        return 0;
    }
}

int update_object_status(int object_label, int state)
{
    for(int i=0;i<10;i++)
    {
        if(object_label == object_list[i])
        {
            object_status[i] = state;
        }
    }
    return 1;
}

int get_object_status(int object_label)
{
    int state = 0;
    for(int i=0;i<10;i++)
    {
        if(object_label == object_list[i])
        {
            state = object_status[i];
        }
    }
    return state;
}


int main(int argc, char** argv) 
{
    cout << "Hello JRC" << endl;
    ros::init(argc, argv, "jrc_main_node");
    ros::NodeHandle nh;
    ros::ServiceClient detect_client = nh.serviceClient<jrc_srvs::bbox_msgs>("bbox");
    ros::ServiceClient pose_client = nh.serviceClient<jrc_srvs::obj_6d>("object_pose");
    ros::ServiceClient pose_center_normal_client = nh.serviceClient<jrc_srvs::obj_6d>("object_pose_central_normal");
    ros::ServiceClient ur10_client = nh.serviceClient<jrc_srvs::grasp>("ur_grasp");
    ros::ServiceClient grasp_client = nh.serviceClient<jrc_srvs::call_grasp>("call_grasp");    
    ros::ServiceClient grasp_state_client = nh.serviceClient<jrc_srvs::call_grasp_state>("call_grasp_state");
    ros::ServiceClient twist_client = nh.serviceClient<jrc_srvs::call_twist>("call_twist");
    ros::ServiceClient agv_client = nh.serviceClient<jrc_srvs::smooth>("agv_move");
    get_point_parameter();
    //agv_service(agv_client,point[1]);
    // jrc_srvs::bbox_msgs detect_srv;
    // jrc_srvs::obj_6d pose_srv;
    // jrc_srvs::grasp ur10_srv;
    // jrc_srvs::call_grasp grasp_srv;
    // jrc_srvs::call_twist twist_srv;

    int object_label;
    int twist_angle = 0;
    geometry_msgs::Pose object_pose;
    int bbox_label = -1;
    int bbox_xmin  = 0;
    int bbox_ymin  = 0;
    int bbox_xmax  = 0;
    int bbox_ymax  = 0;
    int fail_status = 0;
    int ss = 1; //service state
    bool grasp_state_bool = 1; 

    ROS_INFO("Start JRC MAIN!");
    ur_10_service_move(ur10_client,3,1);
    twist_service(twist_client,-1,70);
    while(1)
    {   
        cout<<"before object_status:";
        for(int i=0;i<10;i++)
        {
            cout<<object_status[i]<<",";
        }
        cout<<endl;
        ur_10_service_move(ur10_client,4,1);
        twist_service(twist_client,-1,70);
        object_detection(detect_client,bbox_label,bbox_xmin,bbox_ymin,bbox_xmax,bbox_ymax);
        if(bbox_label == -1)
        {
            cout<<"No target object in observe area."<<endl;
            break;
        }
        else
        {
            object_label = bbox_label;
        }
        if(get_object_status(object_label) == 0)
        {
            ss = pose_estimation(pose_client,bbox_label,bbox_xmin,bbox_ymin,bbox_xmax,bbox_ymax,object_label,object_pose);
            if(ss == 0)
            {
                update_object_status(object_label,3);
                ss = 1;
                continue;
            }
        }
        else
        {
            ss = pose_center_normal_estimation(pose_center_normal_client,bbox_label,bbox_xmin,bbox_ymin,bbox_xmax,bbox_ymax,object_label,object_pose);
            if(ss == 0)
            {
                ss = 1;
                continue;
            }
        }
        ur_10_service_grasp(ur10_client,14,1,object_label,object_pose,fail_status);
        if(fail_status == 1)// object is not in observe area
        {
            cout<<"target object is not in observe area."<<endl;
            update_object_status(object_label,2);
            continue;
        }
        else if(fail_status == 2)// object is invalid
        {
            cout<<"object is invalid."<<endl;
            if(get_object_status(object_label) == 3)
            {
                update_object_status(object_label,2);
                continue;
            }
            else
            {
                update_object_status(object_label,3);
                continue;
            }
        }

        twist_service(twist_client,bbox_label,0);
        grasp_service(grasp_client,1);
        ss = ur_10_service_grasp(ur10_client,1,1,object_label,object_pose,fail_status);
        if(ss == 0)
        {
            ss = 1;
            continue;
        }
        sleep(0.5);
        ur_10_service_move(ur10_client,2,1);
        grasp_state_service(grasp_state_client, grasp_state_bool);
        if(grasp_state_bool == 0)
        {
            ROS_ERROR("NO OBJECT IN END!");
            grasp_state_bool = 1;
            if(get_object_status(object_label) == 4)
            {
                update_object_status(object_label,2);
                continue;
            }
            else
            {
                update_object_status(object_label,4);
                continue;
            }
        }
        ur_10_service_move(ur10_client,3,1);
        twist_service(twist_client,-1,70);
        grasp_service(grasp_client,0);
        sleep(1);

        cout<<"object_status:";
        for(int i=0;i<10;i++)
        {   
            if(object_list[i] == object_label)
                object_status[i] = 1;
            cout<<object_status[i]<<",";
        }
        cout<<endl;

        object_label = 0;
        bbox_label = -1;
        bbox_xmin  = 0;
        bbox_ymin  = 0;
        bbox_xmax  = 0;
        bbox_ymax  = 0;
    }
    cout<<"object_status:";
    for(int i=0;i<10;i++)
    {
        cout<<object_status[i]<<",";
        if(object_status[i] == 2)
            object_status[i] = 0;
    }
    cout<<endl;

    while(1)
    {   
        cout<<"before object_status:";
        for(int i=0;i<10;i++)
        {
            cout<<object_status[i]<<",";
        }
        cout<<endl;
        ur_10_service_move(ur10_client,6,1);
        twist_service(twist_client,-1,70);
        object_detection(detect_client,bbox_label,bbox_xmin,bbox_ymin,bbox_xmax,bbox_ymax);
        if(bbox_label == -1)
        {
            cout<<"No target object in observe area."<<endl;
            break;
        }
        else
        {
            object_label = bbox_label;
        }
        if(get_object_status(object_label) == 0)
        {
            ss = pose_estimation(pose_client,bbox_label,bbox_xmin,bbox_ymin,bbox_xmax,bbox_ymax,object_label,object_pose);
            if(ss == 0)
            {
                update_object_status(object_label,3);
                ss = 1;
                continue;
            }
        }
        else
        {
            ss = pose_center_normal_estimation(pose_center_normal_client,bbox_label,bbox_xmin,bbox_ymin,bbox_xmax,bbox_ymax,object_label,object_pose);
            if(ss == 0)
            {
                ss = 1;
                continue;
            }
        }
        ur_10_service_grasp(ur10_client,14,1,object_label,object_pose,fail_status);
        if(fail_status == 1)// object is not in observe area
        {
            cout<<"target object is not in observe area."<<endl;
            update_object_status(object_label,2);
            continue;
        }
        else if(fail_status == 2)// object is invalid
        {
            cout<<"object is invalid."<<endl;
            if(get_object_status(object_label) == 3)
            {
                update_object_status(object_label,2);
                continue;
            }
            else
            {
                update_object_status(object_label,3);
                continue;
            }
        }

        twist_service(twist_client,bbox_label,0);
        grasp_service(grasp_client,1);
        ss = ur_10_service_grasp(ur10_client,1,1,object_label,object_pose,fail_status);
        if(ss == 0)
        {
            ss = 1;
            continue;
        }
        sleep(0.5);
        ur_10_service_move(ur10_client,2,1);
        grasp_state_service(grasp_state_client, grasp_state_bool);
        if(grasp_state_bool == 0)
        {
            ROS_ERROR("NO OBJECT IN END!");
            grasp_state_bool = 1;
            if(get_object_status(object_label) == 4)
            {
                update_object_status(object_label,2);
                continue;
            }
            else
            {
                update_object_status(object_label,4);
                continue;
            }
        }
        ur_10_service_move(ur10_client,3,1);
        twist_service(twist_client,-1,70);
        grasp_service(grasp_client,0);
        sleep(1);

        cout<<"object_status:";
        for(int i=0;i<10;i++)
        {   
            if(object_list[i] == object_label)
                object_status[i] = 1;
            cout<<object_status[i]<<",";
        }
        cout<<endl;

        object_label = 0;
        bbox_label = -1;
        bbox_xmin  = 0;
        bbox_ymin  = 0;
        bbox_xmax  = 0;
        bbox_ymax  = 0;
    }
    cout<<"object_status:";
    for(int i=0;i<10;i++)
    {
        cout<<object_status[i]<<",";
        if(object_status[i] == 2)
            object_status[i] = 0;
    }
    cout<<endl;

        while(1)
    {   
        cout<<"before object_status:";
        for(int i=0;i<10;i++)
        {
            cout<<object_status[i]<<",";
        }
        cout<<endl;
        ur_10_service_move(ur10_client,8,1);
        twist_service(twist_client,-1,70);
        object_detection(detect_client,bbox_label,bbox_xmin,bbox_ymin,bbox_xmax,bbox_ymax);
        if(bbox_label == -1)
        {
            cout<<"No target object in observe area."<<endl;
            break;
        }
        else
        {
            object_label = bbox_label;
        }
        if(get_object_status(object_label) == 0)
        {
            ss = pose_estimation(pose_client,bbox_label,bbox_xmin,bbox_ymin,bbox_xmax,bbox_ymax,object_label,object_pose);
            if(ss == 0)
            {
                update_object_status(object_label,3);
                ss = 1;
                continue;
            }
        }
        else
        {
            ss = pose_center_normal_estimation(pose_center_normal_client,bbox_label,bbox_xmin,bbox_ymin,bbox_xmax,bbox_ymax,object_label,object_pose);
            if(ss == 0)
            {
                ss = 1;
                continue;
            }
        }
        ur_10_service_grasp(ur10_client,14,1,object_label,object_pose,fail_status);
        if(fail_status == 1)// object is not in observe area
        {
            cout<<"target object is not in observe area."<<endl;
            update_object_status(object_label,2);
            continue;
        }
        else if(fail_status == 2)// object is invalid
        {
            cout<<"object is invalid."<<endl;
            if(get_object_status(object_label) == 3)
            {
                update_object_status(object_label,2);
                continue;
            }
            else
            {
                update_object_status(object_label,3);
                continue;
            }
        }

        twist_service(twist_client,bbox_label,0);
        grasp_service(grasp_client,1);
        ss = ur_10_service_grasp(ur10_client,1,1,object_label,object_pose,fail_status);
        if(ss == 0)
        {
            ss = 1;
            continue;
        }
        sleep(0.5);
        ur_10_service_move(ur10_client,2,1);
        grasp_state_service(grasp_state_client, grasp_state_bool);
        if(grasp_state_bool == 0)
        {
            ROS_ERROR("NO OBJECT IN END!");
            grasp_state_bool = 1;
            if(get_object_status(object_label) == 4)
            {
                update_object_status(object_label,2);
                continue;
            }
            else
            {
                update_object_status(object_label,4);
                continue;
            }
        }
        ur_10_service_move(ur10_client,3,1);
        twist_service(twist_client,-1,70);
        grasp_service(grasp_client,0);
        sleep(1);

        cout<<"object_status:";
        for(int i=0;i<10;i++)
        {   
            if(object_list[i] == object_label)
                object_status[i] = 1;
            cout<<object_status[i]<<",";
        }
        cout<<endl;

        object_label = 0;
        bbox_label = -1;
        bbox_xmin  = 0;
        bbox_ymin  = 0;
        bbox_xmax  = 0;
        bbox_ymax  = 0;
    }
    cout<<"object_status:";
    for(int i=0;i<10;i++)
    {
        cout<<object_status[i]<<",";
        if(object_status[i] == 2)
            object_status[i] = 0;
    }
    cout<<endl;

        while(1)
    {   
        cout<<"before object_status:";
        for(int i=0;i<10;i++)
        {
            cout<<object_status[i]<<",";
        }
        cout<<endl;
        ur_10_service_move(ur10_client,9,1);
        twist_service(twist_client,-1,70);
        object_detection(detect_client,bbox_label,bbox_xmin,bbox_ymin,bbox_xmax,bbox_ymax);
        if(bbox_label == -1)
        {
            cout<<"No target object in observe area."<<endl;
            break;
        }
        else
        {
            object_label = bbox_label;
        }
        if(get_object_status(object_label) == 0)
        {
            ss = pose_estimation(pose_client,bbox_label,bbox_xmin,bbox_ymin,bbox_xmax,bbox_ymax,object_label,object_pose);
            if(ss == 0)
            {
                update_object_status(object_label,3);
                ss = 1;
                continue;
            }
        }
        else
        {
            ss = pose_center_normal_estimation(pose_center_normal_client,bbox_label,bbox_xmin,bbox_ymin,bbox_xmax,bbox_ymax,object_label,object_pose);
            if(ss == 0)
            {
                ss = 1;
                continue;
            }
        }
        ur_10_service_grasp(ur10_client,14,1,object_label,object_pose,fail_status);
        if(fail_status == 1)// object is not in observe area
        {
            cout<<"target object is not in observe area."<<endl;
            update_object_status(object_label,2);
            continue;
        }
        else if(fail_status == 2)// object is invalid
        {
            cout<<"object is invalid."<<endl;
            if(get_object_status(object_label) == 3)
            {
                update_object_status(object_label,2);
                continue;
            }
            else
            {
                update_object_status(object_label,3);
                continue;
            }
        }

        twist_service(twist_client,bbox_label,0);
        grasp_service(grasp_client,1);
        ss = ur_10_service_grasp(ur10_client,1,1,object_label,object_pose,fail_status);
        if(ss == 0)
        {
            ss = 1;
            continue;
        }
        sleep(0.5);
        ur_10_service_move(ur10_client,2,1);
        grasp_state_service(grasp_state_client, grasp_state_bool);
        if(grasp_state_bool == 0)
        {
            ROS_ERROR("NO OBJECT IN END!");
            grasp_state_bool = 1;
            if(get_object_status(object_label) == 4)
            {
                update_object_status(object_label,2);
                continue;
            }
            else
            {
                update_object_status(object_label,4);
                continue;
            }
        }
        ur_10_service_move(ur10_client,3,1);
        twist_service(twist_client,-1,70);
        grasp_service(grasp_client,0);
        sleep(1);

        cout<<"object_status:";
        for(int i=0;i<10;i++)
        {   
            if(object_list[i] == object_label)
                object_status[i] = 1;
            cout<<object_status[i]<<",";
        }
        cout<<endl;

        object_label = 0;
        bbox_label = -1;
        bbox_xmin  = 0;
        bbox_ymin  = 0;
        bbox_xmax  = 0;
        bbox_ymax  = 0;
    }
    cout<<"object_status:";
    for(int i=0;i<10;i++)
    {
        cout<<object_status[i]<<",";
        if(object_status[i] == 2)
            object_status[i] = 0;
    }
    cout<<endl;

        while(1)
    {   
        cout<<"before object_status:";
        for(int i=0;i<10;i++)
        {
            cout<<object_status[i]<<",";
        }
        cout<<endl;
        ur_10_service_move(ur10_client,7,1);
        twist_service(twist_client,-1,70);
        object_detection(detect_client,bbox_label,bbox_xmin,bbox_ymin,bbox_xmax,bbox_ymax);
        if(bbox_label == -1)
        {
            cout<<"No target object in observe area."<<endl;
            break;
        }
        else
        {
            object_label = bbox_label;
        }
        if(get_object_status(object_label) == 0)
        {
            ss = pose_estimation(pose_client,bbox_label,bbox_xmin,bbox_ymin,bbox_xmax,bbox_ymax,object_label,object_pose);
            if(ss == 0)
            {
                update_object_status(object_label,3);
                ss = 1;
                continue;
            }
        }
        else
        {
            ss = pose_center_normal_estimation(pose_center_normal_client,bbox_label,bbox_xmin,bbox_ymin,bbox_xmax,bbox_ymax,object_label,object_pose);
            if(ss == 0)
            {
                ss = 1;
                continue;
            }
        }
        ur_10_service_grasp(ur10_client,14,1,object_label,object_pose,fail_status);
        if(fail_status == 1)// object is not in observe area
        {
            cout<<"target object is not in observe area."<<endl;
            update_object_status(object_label,2);
            continue;
        }
        else if(fail_status == 2)// object is invalid
        {
            cout<<"object is invalid."<<endl;
            if(get_object_status(object_label) == 3)
            {
                update_object_status(object_label,2);
                continue;
            }
            else
            {
                update_object_status(object_label,3);
                continue;
            }
        }

        twist_service(twist_client,bbox_label,0);
        grasp_service(grasp_client,1);
        ss = ur_10_service_grasp(ur10_client,1,1,object_label,object_pose,fail_status);
        if(ss == 0)
        {
            ss = 1;
            continue;
        }
        sleep(0.5);
        ur_10_service_move(ur10_client,2,1);
        grasp_state_service(grasp_state_client, grasp_state_bool);
        if(grasp_state_bool == 0)
        {
            ROS_ERROR("NO OBJECT IN END!");
            grasp_state_bool = 1;
            if(get_object_status(object_label) == 4)
            {
                update_object_status(object_label,2);
                continue;
            }
            else
            {
                update_object_status(object_label,4);
                continue;
            }
        }
        ur_10_service_move(ur10_client,3,1);
        twist_service(twist_client,-1,70);
        grasp_service(grasp_client,0);
        sleep(1);

        cout<<"object_status:";
        for(int i=0;i<10;i++)
        {   
            if(object_list[i] == object_label)
                object_status[i] = 1;
            cout<<object_status[i]<<",";
        }
        cout<<endl;

        object_label = 0;
        bbox_label = -1;
        bbox_xmin  = 0;
        bbox_ymin  = 0;
        bbox_xmax  = 0;
        bbox_ymax  = 0;
    }
    cout<<"object_status:";
    for(int i=0;i<10;i++)
    {
        cout<<object_status[i]<<",";
        if(object_status[i] == 2)
            object_status[i] = 0;
    }
    cout<<endl;

        while(1)
    {   
        cout<<"before object_status:";
        for(int i=0;i<10;i++)
        {
            cout<<object_status[i]<<",";
        }
        cout<<endl;
        ur_10_service_move(ur10_client,5,1);
        twist_service(twist_client,-1,70);
        object_detection(detect_client,bbox_label,bbox_xmin,bbox_ymin,bbox_xmax,bbox_ymax);
        if(bbox_label == -1)
        {
            cout<<"No target object in observe area."<<endl;
            break;
        }
        else
        {
            object_label = bbox_label;
        }
        if(get_object_status(object_label) == 0)
        {
            ss = pose_estimation(pose_client,bbox_label,bbox_xmin,bbox_ymin,bbox_xmax,bbox_ymax,object_label,object_pose);
            if(ss == 0)
            {
                update_object_status(object_label,3);
                ss = 1;
                continue;
            }
        }
        else
        {
            ss = pose_center_normal_estimation(pose_center_normal_client,bbox_label,bbox_xmin,bbox_ymin,bbox_xmax,bbox_ymax,object_label,object_pose);
            if(ss == 0)
            {
                ss = 1;
                continue;
            }
        }
        ur_10_service_grasp(ur10_client,14,1,object_label,object_pose,fail_status);
        if(fail_status == 1)// object is not in observe area
        {
            cout<<"target object is not in observe area."<<endl;
            update_object_status(object_label,2);
            continue;
        }
        else if(fail_status == 2)// object is invalid
        {
            cout<<"object is invalid."<<endl;
            if(get_object_status(object_label) == 3)
            {
                update_object_status(object_label,2);
                continue;
            }
            else
            {
                update_object_status(object_label,3);
                continue;
            }
        }

        twist_service(twist_client,bbox_label,0);
        grasp_service(grasp_client,1);
        ss = ur_10_service_grasp(ur10_client,1,1,object_label,object_pose,fail_status);
        if(ss == 0)
        {
            ss = 1;
            continue;
        }
        sleep(0.5);
        ur_10_service_move(ur10_client,2,1);
        grasp_state_service(grasp_state_client, grasp_state_bool);
        if(grasp_state_bool == 0)
        {
            ROS_ERROR("NO OBJECT IN END!");
            grasp_state_bool = 1;
            if(get_object_status(object_label) == 4)
            {
                update_object_status(object_label,2);
                continue;
            }
            else
            {
                update_object_status(object_label,4);
                continue;
            }
        }
        ur_10_service_move(ur10_client,3,1);
        twist_service(twist_client,-1,70);
        grasp_service(grasp_client,0);
        sleep(1);

        cout<<"object_status:";
        for(int i=0;i<10;i++)
        {   
            if(object_list[i] == object_label)
                object_status[i] = 1;
            cout<<object_status[i]<<",";
        }
        cout<<endl;

        object_label = 0;
        bbox_label = -1;
        bbox_xmin  = 0;
        bbox_ymin  = 0;
        bbox_xmax  = 0;
        bbox_ymax  = 0;
    }
    cout<<"object_status:";
    for(int i=0;i<10;i++)
    {
        cout<<object_status[i]<<",";
        if(object_status[i] == 2)
            object_status[i] = 0;
    }
    cout<<endl;

    ur_10_service_move(ur10_client,3,1);
    twist_service(twist_client,-1,50);

    return 0;
}