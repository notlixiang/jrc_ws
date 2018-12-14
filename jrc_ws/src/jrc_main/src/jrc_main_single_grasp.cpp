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
int object_list[10] = {7,16,10,11,15,8,2,3,4,6};
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

//第一个抓取点 cbb
// point[1].position.x = 0.003;
// point[1].position.y =0.104;
// point[1].position.z = 0.0;
// point[1].orientation.x = 0.0;
// point[1].orientation.y = 0.0;
// point[1].orientation.z = -0.025;
// point[1].orientation.w = 0.9999;


//第一个抓取点 hxq
point[1].position.x = -2.844;
point[1].position.y = -0.532;
point[1].position.z = 0.0;
point[1].orientation.x = 0.0;
point[1].orientation.y = 0.0;
point[1].orientation.z = -0.071;
point[1].orientation.w = 0.997;

// //第二个抓取点 cbb
// point[2].position.x = 0.003;
// point[2].position.y = 0.104;
// point[2].position.z = 0.0;
// point[2].orientation.x = 0.0;
// point[2].orientation.y = 0.0;
// point[2].orientation.z = 0.9999;
// point[2].orientation.w = 0.004;

//第二个抓取点 hxq
point[2].position.x = -2.894;
point[2].position.y = -0.196;
point[2].position.z = 0.0;
point[2].orientation.x = 0.0;
point[2].orientation.y = 0.0;
point[2].orientation.z = 0.999;
point[2].orientation.w = 0.041;

//第3个抓取点 hxq
point[3].position.x = -0.054;
point[3].position.y = -0.863;
point[3].position.z = 0.0;
point[3].orientation.x = 0.0;
point[3].orientation.y = 0.0;
point[3].orientation.z = -0.071;
point[3].orientation.w = 0.997;

//the forth point hxq
point[4].position.x = -0.366;  
point[4].position.y = 0.141;
point[4].position.z = 0.0;
point[4].orientation.x = 0.0;
point[4].orientation.y = 0.0;
point[4].orientation.z = 0.999;
point[4].orientation.w = 0.041;



//the 5 point hxq
point[5].position.x = 0.572;  
point[5].position.y = -0.017;
point[5].position.z = 0.0;
point[5].orientation.x = 0.0;
point[5].orientation.y = 0.0;
point[5].orientation.z = 0.999;
point[5].orientation.w = 0.033;

// //the third point cbb
// point[3].position.x = 3.037;  
// point[3].position.y = -0.158;
// point[3].position.z = 0.0;
// point[3].orientation.x = 0.0;
// point[3].orientation.y = 0.0;
// point[3].orientation.z = 0.9999;
// point[3].orientation.w = 0.004; 

// //the forth point cbb
// point[4].position.x = 3.037;  
// point[4].position.y = -0.1582;
// point[4].position.z = 0.0;
// point[4].orientation.x = 0.0;
// point[4].orientation.y = 0.0;
// point[4].orientation.z = -0.002;
// point[4].orientation.w = 0.9999;
}

class grid_grasp
{
private:
    ros::NodeHandle nh;
    ros::ServiceClient detect_client;
    ros::ServiceClient pose_client;
    ros::ServiceClient pose_center_normal_client;
    ros::ServiceClient ur10_client;
    ros::ServiceClient grasp_client;    
    ros::ServiceClient grasp_state_client;
    ros::ServiceClient twist_client;
    ros::ServiceClient agv_client;
    int object_label;
    int twist_angle;
    geometry_msgs::Pose object_pose;
    int bbox_label;
    int bbox_xmin;
    int bbox_ymin;
    int bbox_xmax;
    int bbox_ymax;
    int fail_status;
    int ss;
    bool grasp_state_bool;

public:
    grid_grasp()
    {
        detect_client = nh.serviceClient<jrc_srvs::bbox_msgs>("bbox");
        pose_client = nh.serviceClient<jrc_srvs::obj_6d>("object_pose");
        pose_center_normal_client = nh.serviceClient<jrc_srvs::obj_6d>("object_pose_central_normal");
        ur10_client = nh.serviceClient<jrc_srvs::grasp>("ur_grasp");
        grasp_client = nh.serviceClient<jrc_srvs::call_grasp>("call_grasp");    
        grasp_state_client = nh.serviceClient<jrc_srvs::call_grasp_state>("call_grasp_state");
        twist_client = nh.serviceClient<jrc_srvs::call_twist>("call_twist");
        agv_client = nh.serviceClient<jrc_srvs::smooth>("agv_move");
        object_label = -1;
        twist_angle = 0;
        bbox_label = -1;
        bbox_xmin  = 0;
        bbox_ymin  = 0;
        bbox_xmax  = 0;
        bbox_ymax  = 0;
        fail_status = 0;
        ss = 1;
        grasp_state_bool = 1; 

    }
    ~grid_grasp()
    {}
    int agv_service(geometry_msgs::Pose& point)
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

    int twist_service(int object_label, int angle)
    {
        jrc_srvs::call_twist twist_srv;
        if((object_label == 2) || (object_label == 5) || (object_label == 6) || (object_label == 7) || (object_label == 13) || (object_label == 14))
            angle = 90;
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

    int grasp_state_service(bool &grasp_state)
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

    int grasp_service(bool grasp_signal)
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

    int ur_10_service_move(
                    int mode_,int agv_)
    {
        jrc_srvs::grasp ur10_srv_;
        ur10_srv_.request.mode = mode_;
        ur10_srv_.request.agv_position = agv_;
        if(ur10_client.call(ur10_srv_))
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

    int ur_10_service_grasp(
                    int mode_,int agv_,int id_,geometry_msgs::Pose pose_,int &fail_status_)
    {
        jrc_srvs::grasp ur10_srv_;
        ur10_srv_.request.mode = mode_;
        ur10_srv_.request.agv_position = agv_;
        ur10_srv_.request.id = id_;
        ur10_srv_.request.pose = pose_;
        if(ur10_client.call(ur10_srv_))
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

    int pose_estimation(
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

    int pose_center_normal_estimation(
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
            cout<<"Finish pose center normal service."<<endl;
            return 1;
        }
        else
        {
            cout<<"Error in object pose center normal estimation."<<endl;
            return 0;
        }
    }

    int object_detection(
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

    bool run(int grasp_mode,int grasp_agv_position)
    {
        while(1)
        {   
            cout<<"before object_status:";
            for(int i=0;i<10;i++)
            {
                cout<<object_status[i]<<",";
            }
            cout<<endl;
            ur_10_service_move(grasp_mode,grasp_agv_position);
            twist_service(-1,90);
            object_detection(bbox_label,bbox_xmin,bbox_ymin,bbox_xmax,bbox_ymax);
            if(bbox_label == -1)
            {
                cout<<"No target object in observe area."<<endl;
                break;
            }
            else
            {
                object_label = bbox_label;
            }
            // cout<<"middle object status:";
            // for(int i=0;i<10;i++)
            // {
            //     cout<<object_status[i]<<",";
            // }
            // cout<<"test:"<<get_object_status(object_label)<<endl;
            if(get_object_status(object_label) == 0)
            {
                ss = pose_estimation(bbox_label,bbox_xmin,bbox_ymin,bbox_xmax,bbox_ymax,object_label,object_pose);
                if(ss == 0)
                {
                    update_object_status(object_label,3);
                    ss = 1;
                    continue;
                }
            }
            else
            {
                ss = pose_center_normal_estimation(bbox_label,bbox_xmin,bbox_ymin,bbox_xmax,bbox_ymax,object_label,object_pose);
                if(ss == 0)
                {
                    ss = 1;
                    continue;
                }
            }
            ur_10_service_grasp(14,grasp_agv_position,object_label,object_pose,fail_status);
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

            twist_service(bbox_label,0);
            grasp_service(1);
            ss = ur_10_service_grasp(1,grasp_agv_position,object_label,object_pose,fail_status);
            if(ss == 0)
            {
                ss = 1;
                continue;
            }
            sleep(1);
            ur_10_service_move(2,grasp_agv_position);
            grasp_state_service(grasp_state_bool);
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
            ur_10_service_move(3,grasp_agv_position);
            twist_service(-1,90);
            grasp_service(0);
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

        return 1;
    }

};

int main(int argc, char** argv) 
{
    cout << "Hello JRC" << endl;
    ros::init(argc, argv, "jrc_main_node");
    grid_grasp grasp;
    get_point_parameter();
    ROS_INFO("Start JRC MAIN!");
    grasp.ur_10_service_move(3,1);
    grasp.twist_service(-1,90);
    // grasp.agv_service(point[1]);
    // sleep(2);
    // grasp.run(10,2);
    // grasp.run(11,2);
    // grasp.run(12,2);
    // grasp.ur_10_service_move(3,1);
    // grasp.twist_service(-1,90);

    // grasp.agv_service(point[2]);

    // grasp.run(4,1);
    // grasp.run(6,1);
    // grasp.run(8,1);
    // grasp.run(9,1);
    // grasp.run(7,1);
    // grasp.run(5,1);

    grasp.run(10,2);
    grasp.run(12,2);

    grasp.ur_10_service_move(3,1);
    grasp.twist_service(-1,90);

    // grasp.agv_service(point[3]);

    // grasp.run(4,1);
    // grasp.run(6,1);
    // grasp.run(8,1);
    // grasp.run(9,1);
    // grasp.run(7,1);
    // grasp.run(5,1);

    // grasp.ur_10_service_move(3,1);
    // grasp.twist_service(-1,90);

    // grasp.agv_service(point[5]);
    // grasp.agv_service(point[4]);
    // grasp.twist_service(-1,0);
    // sleep(3);
    // grasp.ur_10_service_move(13,1);

    return 0;
}