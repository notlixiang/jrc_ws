#include <iostream>
#include <string>

#include <ros/ros.h>
#include <ros/package.h>
#include <msg_package/grasp.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <boost/timer.hpp>
#include <trac_ik/trac_ik.hpp>

#include "../../../devel/include/ur_msgs/Single_JointPosition.h"
#include "../../../devel/include/ur_msgs/Single_JointPositionRequest.h"
#include "../../../devel/include/ur_msgs/Single_JointPositionResponse.h"

#include "../../../devel/include/ur_msgs/Multi_JointPosition.h"
#include "../../../devel/include/ur_msgs/Multi_JointPositionRequest.h"
#include "../../../devel/include/ur_msgs/Multi_JointPositionResponse.h"

std::vector<double> place={-0.3970096747027796, -1.2241390387164515, -1.829761807118551, 1.4546838998794556, -1.1228402296649378, -0.34300262132753545};
std::vector<double> place1={-0.7838166395770472, -1.466745678578512, -1.6080964247332972, 1.6563516855239868, -1.1451790968524378, -0.7699616591082972};
bool place_key = true;

std::vector<double> push_board_left_up2={-0.4722994009601038, -1.6343877951251429, -1.7045272032367151, 1.811212182044983, -1.590679947529928, -2.009348217641012};
std::vector<double> push_board_left_down2={-0.4728863875018518, -1.7716253439532679, -1.951355282460348, 2.195178985595703, -1.5900686422931116, -2.0126078764544886};

std::vector<double> push_board_left_up={-0.5830748716937464, -1.700639549885885, -1.5402382055865687, 1.7232493162155151, -1.5924528280841272, -2.1184971968280237};
std::vector<double> push_board_left_down={-0.5841172377215784, -1.8427050749408167, -1.8591859976397913, 2.184173583984375, -1.5919020811664026, -2.1226919333087366};

std::vector<double> push_board_middle_up={-0.7294791380511683, -1.8523781935321253, -1.3904078642474573, 1.7160145044326782, -1.6047800222979944, -2.261364761983053};
std::vector<double> push_board_middle_down={-0.7306774298297327, -1.9759672323810022, -1.6785920302020472, 2.1276259422302246, -1.604145352040426, -2.265414539967672};

std::vector<double> push_board_right_up={-0.8517087141620081, -2.035632912312643, -1.1670358816729944, 1.6771103143692017, -1.595532242451803, -2.396341864262716};
std::vector<double> push_board_right_down={-0.852846924458639, -2.126411739979879, -1.4321849981891077, 2.0328502655029297, -1.5951603094684046, -2.400259319935934};

std::vector<double> push_board_right_up2={-0.8847129980670374, -2.129815403615133, -0.9688156286822718, 1.5562101602554321, -1.6022279898272913, -2.4222312609301966};
std::vector<double> push_board_right_down2={-0.8864148298846644, -2.204508129750387, -1.3019993940936487, 1.963771939277649, -1.6016891638385218, -2.426687542592184};

std::vector<double> observe_up_right={-0.7508109251605433, -1.8570292631732386, -1.000451389943258, 1.7932463884353638, -2.1206777731524866, -2.2503774801837366};
std::vector<double> observe_down_right={-0.7083023230182093, -2.1216653029071253, -1.715290371571676, 2.767934799194336, -2.0456698576556605, -2.2114599386798304};
std::vector<double> observe_up_left={1.0734087228775024, -1.5148752371417444, -1.4152510801898401, 1.9858320951461792, -1.08287221590151, -0.806427303944723};
std::vector<double> observe_down_left={1.1234005689620972, -2.060036007558004, -1.8429935614215296, 2.7270407676696777, -1.0266035238849085, -0.6130736509906214};
std::vector<double> observe_up_middle={0.12226343899965286, -1.3514974753009241, -1.4865830580340784, 2.0032453536987305, -1.5900805632220667, -1.5145471731769007};
std::vector<double> observe_down_middle={0.41147613525390625, -1.715863052998678, -2.292990748082296, 3.085113048553467, -1.3876169363604944, -1.27545673051943};

std::vector<double> pre_down_right = {-0.47122,-2.6468,-1.91,4.538,-2.0767,-1.67};
std::vector<double> pre_up_right = {-0.66,-1.2241,-2.326,3.568,-2.2556,-1.646};
std::vector<double> pre_down_left = {0.925,-2.638,-1.988,4.617,-0.681,-1.652};
std::vector<double> pre_up_left = {0.7982,-1.759,-1.906,3.63,-0.847,-1.541};
std::vector<double> pre_down_middle = {0.3650,-2.564,-2.351,4.7527,-1.247,-1.5026};
std::vector<double> pre_up_middle = {0.6812,-0.9714,-2.445,3.4338,-0.915,-1.5814};

std::vector<double> observe_desk_left={0.09529593586921692, -1.58759051958193, -1.431633774434225, 1.9038177728652954, -1.7077191511737269, -1.5053265730487269};
std::vector<double> observe_desk_right={-0.20147353807558233, -1.7846668402301233, -1.251810375844137, 1.7992347478866577, -1.68620473543276, -1.8027623335467737};
std::vector<double> observe_desk_middle={0.20919, -1.6696, -1.22, 1.8531, -1.4869, -1.4689};

std::vector<double> pre_desk_middle = {0.235, -1.4785, -2.09185, 3.6122, -1.3637, -1.5748};
std::vector<double> pre_desk_right = {-0.39277917543520147, -1.8877981344806116, -2.0938661734210413, 3.940415382385254, -2.0900543371783655, -1.5432866255389612};
std::vector<double> pre_desk_left = {0.5980693697929382, -1.6522725264178675, -2.4085148016559046, 4.028441429138184, -1.0995672384845179, -1.5063698927508753};

//------------------------------------------------------------------------------------------------------------
//从放置位置到下层观察点的中间关键点位置
std::vector<double> place_to_down_right_middle = {-0.6449, -1.7768, -1.2629, 2.0178, -2.0283, -2.124};
std::vector<double> place_to_down_middle_middle = {0.1453, -1.4225, -1.6745, 2.0028, -1.5781, -1.4511};
std::vector<double> place_to_down_left_middle = {0.9279, -1.548, -1.581, 1.9197, -1.24476, -0.73046};

//从上层观察点到上层预抓取点的中间关键点
std::vector<double> up_right_observe_to_pre_up_right_middle = {-0.55852, -1.4085, -1.7378, 2.64484, -2.14713, -1.8459};
std::vector<double> up_middle_observe_to_pre_up_middle_middle = {0.4637, -1.06503, -1.9560, 2.58203, -1.2206, -1.3934};
std::vector<double> up_left_observe_to_pre_up_left_middle = {1.1978, -1.3119, -1.84608, 2.38082, -0.60021, -0.87069};

//从下层观察点到下层预抓取点的中间关键点1
std::vector<double> down_right_observe_to_pre_down_right_middle1 = {-0.5984151999102991, -2.2972949186908167, -1.6988828817950647, 3.6491546630859375, -2.2164490858661097, -1.802246395741598};
std::vector<double> down_middle_observe_to_pre_down_middle_middle1 = {0.30630218982696533, -1.9829071203814905, -2.302255932484762, 3.7776083946228027, -1.387221638356344, -1.441322151814596};
std::vector<double> down_left_observe_to_pre_down_left_middle1 = {0.9285748600959778, -2.1268909613238733, -2.042853657399313, 3.497246742248535, -0.8628271261798304, -1.0628326574908655};

//从下层观察点到下层预抓取点的中间关键点2
std::vector<double> down_right_observe_to_pre_down_right_middle2 = {-0.6116340796100062, -2.4815548102008265, -1.7852361837970179, 4.2759199142456055, -2.2616809050189417, -1.6038931051837366};
std::vector<double> down_middle_observe_to_pre_down_middle_middle2 = {0.21398882567882538, -2.3165419737445276, -2.365232292805807, 4.516878128051758, -1.4689701239215296, -1.5617268721209925};
std::vector<double> down_left_observe_to_pre_down_left_middle2 = {0.9470088481903076, -2.352328602467672, -2.137914005910055, 4.256119728088379, -0.7483127752887171, -1.4038985411273401};

//从桌子观察点到预抓取点的中间关键点
std::vector<double> desk_left_observe_to_pre_desk_left_middle = {0.592136800289154, -1.4488490263568323, -2.155006233845846, 3.253549098968506, -1.1268194357501429, -1.3589194456683558};
std::vector<double> desk_middle_observe_to_pre_desk_middle_middle = {-0.48886, -2.0710, -0.69474, 1.80261, -1.941925, -1.98175};
std::vector<double> desk_right_observe_to_pre_desk_right_middle = {-0.23228723207582647, -1.6598718802081507, -1.755249325429098, 2.8978271484375, -1.8815835157977503, -1.6853917280780237};

//从下层抓取点到放置点的中间关键点
std::vector<double> grasp_down_right_to_place_middle = {-0.31782466570009404, -1.5581372419940394, -1.7795551458941858, 2.3757987022399902, -1.7540410200702112, -1.8243139425860804};
std::vector<double> grasp_down_middle_to_place_middle = {0.41796034574508667, -1.380639378224508, -1.9404004255877894, 2.344400405883789, -1.3257296721087855, -1.2207348982440394};
std::vector<double> grasp_down_left_to_place_middle = {1.0023343563079834, -1.601124111806051, -1.7773497740374964, 2.1896824836730957, -1.0427592436419886, -0.6657102743731897};

//从上层左侧抓取点到放置点的中间关键点
std::vector<double> grasp_up_and_desk_left_to_place = {0.13934282958507538, -1.2762139479266565, -1.5652864615069788, 1.814865231513977, -1.4190567175494593, -1.395120922719137};
//从桌子左侧抓取点到放置点的中间关键点


double up_level_z_up = 0.55;
double up_level_z_down = 0.322;
double down_level_z_up = -0.05;
double down_level_z_down = -0.2743;
double x_up = 0.58;
double x_down = -0.49;
double y_up = -0.94;
double y_down = -1.45;

double desk_x_down = -0.52;
double desk_y_up = -0.94;
double desk_z_down = 0.0251;
double desk_x_up = 0.52;
double desk_y_down = -1.5;
double desk_z_up = 0.35;

double cartesian_distance = 0.05;

enum M_robot_state{s_init,s_place,
                   s_back_up_right,s_back_up_middle,s_back_up_left,
                   s_back_down_right,s_back_down_middle,s_back_down_left,
                   s_back_desk_right,s_back_desk_middle,s_back_desk_left,

                   s_grasp_up_right,s_grasp_up_middle,s_grasp_up_left,
                   s_grasp_down_right,s_grasp_down_middle,s_grasp_down_left,
                   s_grasp_desk_right,s_grasp_desk_middle,s_grasp_desk_left,

                   s_up_right_observe,s_up_left_observe,s_up_middle_observe,
                   s_down_right_observe,s_down_left_observe,s_down_middle_observe,
                   s_pre_up_right,s_pre_up_middle,s_pre_up_left,
                   s_pre_down_right,s_pre_down_middle,s_pre_down_left,

                   s_observe_desk_left,s_observe_desk_right,s_observe_desk_middle,
                   s_pre_desk_right,s_pre_desk_middle,s_pre_desk_left};
M_robot_state m_robot_state;

enum M_robot_target{t_unknown,t_place,

                    t_up_right,t_up_middle,t_up_left,t_down_right,t_down_middle,t_down_left,
                    t_pre_up_right,t_pre_up_middle,t_pre_up_left,
                    t_pre_down_right,t_pre_down_middle,t_pre_down_left,

                    t_up_grasp,t_down_grasp,

                    t_desk_right,t_desk_middle,t_desk_left,
                    t_pre_desk_right,t_pre_desk_middle,t_pre_desk_left,
                    t_desk_grasp};
M_robot_target m_robot_target;

enum M_agv_state{unknown,huojia,desk};
M_agv_state m_agv_state;

Eigen::Matrix<double,4,4> camera_to_depth;
Eigen::Matrix<double,4,4> ee_to_camera;

Eigen::Matrix<double,4,4> up_right_base_to_ee;
Eigen::Matrix<double,4,4> down_right_base_to_ee;
Eigen::Matrix<double,4,4> up_left_base_to_ee;
Eigen::Matrix<double,4,4> down_left_base_to_ee;
Eigen::Matrix<double,4,4> down_middle_base_to_ee;
Eigen::Matrix<double,4,4> up_middle_base_to_ee;

Eigen::Matrix<double,4,4> desk_right_base_to_ee;
Eigen::Matrix<double,4,4> desk_left_base_to_ee;
Eigen::Matrix<double,4,4> desk_middle_base_to_ee;

geometry_msgs::Pose current_pose;

ros::ServiceClient single_joint_client;
ros::ServiceClient multi_joint_client;
ros::ServiceClient movel_joint_client;
ros::ServiceClient multi_movel_joint_client;

bool single_joint_action(std::vector<double> q_)
{
    ur_msgs::Single_JointPosition single_joint_srv;
    single_joint_srv.request.positions.resize(6);
    single_joint_srv.request.use_time = false;
    for(int i=0;i<6;i++)
        single_joint_srv.request.positions[i] = q_[i];

    if (single_joint_client.call(single_joint_srv))
    {
      bool result = (bool)single_joint_srv.response.result;
      if(result)
      {
          std::cout<<"single joint action success!"<<std::endl;
          return true;
      }
      else
      {
          ROS_ERROR("single joint action failed!");
          return false;
      }
    }
    else
    {
      ROS_ERROR("single joint action : Failed to call service");
      return false;
    }
}

bool multi_joint_action(std::vector<std::vector<double> > q_list_)
{
    ur_msgs::Multi_JointPosition multi_joint_srv;
    multi_joint_srv.request.points.resize(q_list_.size());
    for(int i=0;i<q_list_.size();i++)
    {
        multi_joint_srv.request.points[i].positions.resize(6);
        multi_joint_srv.request.points[i].use_time = false;
        for(int j=0;j<6;j++)
        {
            multi_joint_srv.request.points[i].positions[j] = q_list_[i][j];
        }
    }

    if (multi_joint_client.call(multi_joint_srv))
    {
        bool result = (bool)multi_joint_srv.response.result;
        if(result)
        {
            std::cout<<"multi joint action success!"<<std::endl;
            return true;
        }
        else
        {
            ROS_ERROR("multi joint action failed!");
            return false;
        }
    }
    else
    {
      ROS_ERROR("multi joint action : Failed to call service");
      return false;
    }
}

bool movel_joint_action(std::vector<double> q_)
{
    ur_msgs::Single_JointPosition movel_joint_srv;
    movel_joint_srv.request.positions.resize(6);
    movel_joint_srv.request.use_time = false;
    for(int i=0;i<6;i++)
        movel_joint_srv.request.positions[i] = q_[i];

    if (movel_joint_client.call(movel_joint_srv))
    {
        bool result = (bool)movel_joint_srv.response.result;
        if(result)
        {
            std::cout<<"movel joint action success!"<<std::endl;
            return true;
        }
        else
        {
            ROS_ERROR("movel joint action failed!");
            return false;
        }
    }
    else
    {
      ROS_ERROR("movel joint action : Failed to call service");
      return false;
    }
}

bool multi_movel_joint_action(std::vector<std::vector<double> > q_list_)
{
    ur_msgs::Multi_JointPosition multi_joint_srv;
    multi_joint_srv.request.points.resize(q_list_.size());
    for(int i=0;i<q_list_.size();i++)
    {
        multi_joint_srv.request.points[i].positions.resize(6);
        multi_joint_srv.request.points[i].use_time = false;
        for(int j=0;j<6;j++)
        {
            multi_joint_srv.request.points[i].positions[j] = q_list_[i][j];
        }
    }

    if (multi_movel_joint_client.call(multi_joint_srv))
    {
        bool result = (bool)multi_joint_srv.response.result;
        if(result)
        {
            std::cout<<"multi movel joint action success!"<<std::endl;
            return true;
        }
        else
        {
            ROS_ERROR("multi movel joint action failed!");
            return false;
        }
    }
    else
    {
      ROS_ERROR("multi movel joint action : Failed to call service");
      return false;
    }
}

bool movel_joint_action_with_time(std::vector<double> q_,double time_)
{
    ur_msgs::Single_JointPosition movel_joint_srv;
    movel_joint_srv.request.positions.resize(6);
    movel_joint_srv.request.use_time = true;
    movel_joint_srv.request.time = time_;
    for(int i=0;i<6;i++)
        movel_joint_srv.request.positions[i] = q_[i];

    if (movel_joint_client.call(movel_joint_srv))
    {
        bool result = (bool)movel_joint_srv.response.result;
        if(result)
        {
            std::cout<<"movel joint action with time success!"<<std::endl;
            return true;
        }
        else
        {
            ROS_ERROR("movel joint action with time failed!");
            return false;
        }
    }
    else
    {
      ROS_ERROR("movel joint action with time : Failed to call service");
      return false;
    }
}

bool multi_movel_joint_action_with_time(std::vector<std::vector<double> > q_list_, std::vector<double> time_)
{
    ur_msgs::Multi_JointPosition multi_joint_srv;
    multi_joint_srv.request.points.resize(q_list_.size());
    for(int i=0;i<q_list_.size();i++)
    {
        multi_joint_srv.request.points[i].positions.resize(6);
        if(time_[i]>=1 && time_[i]<=3)
        {
            multi_joint_srv.request.points[i].use_time = true;
            multi_joint_srv.request.points[i].time = time_[i];
        }
        else
        {
            multi_joint_srv.request.points[i].use_time = false;
        }
        for(int j=0;j<6;j++)
        {
            multi_joint_srv.request.points[i].positions[j] = q_list_[i][j];
        }
    }

    if (multi_movel_joint_client.call(multi_joint_srv))
    {
        bool result = (bool)multi_joint_srv.response.result;
        if(result)
        {
            std::cout<<"multi movel joint action with time success!"<<std::endl;
            return true;
        }
        else
        {
            ROS_ERROR("multi movel joint action with time failed!");
            return false;
        }
    }
    else
    {
      ROS_ERROR("multi movel joint action with time: Failed to call service");
      return false;
    }
}

bool trac_ik_solver(std::vector<double> current_q,geometry_msgs::Pose pose_,std::vector<double> &result_q)
{
    KDL::Frame end_effector_pose;
    Eigen::Quaterniond q_end_effector_pose(pose_.orientation.w,pose_.orientation.x,pose_.orientation.y,pose_.orientation.z);
    Eigen::Matrix<double,3,3> r_end_effector_pose;
    r_end_effector_pose = q_end_effector_pose.toRotationMatrix();
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            end_effector_pose.M(i,j) = r_end_effector_pose(i,j);
        }
    }
    end_effector_pose.p(0) = pose_.position.x;
    end_effector_pose.p(1) = pose_.position.y;
    end_effector_pose.p(2) = pose_.position.z;

    std::string chain_start, chain_end, urdf_param;
    double timeout;

    chain_start = "base";
    chain_end = "gripper";
    urdf_param = "/robot_description";
    timeout = 0.005;
    double eps = 1e-5;

    TRAC_IK::TRAC_IK tracik_solver(chain_start, chain_end, urdf_param, timeout, eps);

    KDL::Chain chain;
    KDL::JntArray ll, ul;

    bool valid = tracik_solver.getKDLChain(chain);
    if (!valid)
    {
      ROS_ERROR("trak_ik error : There was no valid KDL chain found");
      return 0;
    }

    if(current_q.size()!=chain.getNrOfJoints())
    {
        ROS_ERROR("trak_ik error : joint number error!");
        return 0;
    }

    valid = tracik_solver.getKDLLimits(ll,ul);
    if (!valid)
    {
      ROS_ERROR("trak_ik error : There were no valid KDL joint limits found");
      return 0;
    }

    assert(chain.getNrOfJoints() == ll.data.size());
    assert(chain.getNrOfJoints() == ul.data.size());
    //ROS_INFO ("Using %d joints",chain.getNrOfJoints());

    KDL::JntArray current(chain.getNrOfJoints());
    for(int i=0;i<chain.getNrOfJoints();i++)
        current(i) = current_q[i];

    KDL::JntArray result;
    int rc;
    rc=tracik_solver.CartToJnt(current,end_effector_pose,result);
    if(rc>=0)
    {
        result_q.clear();
        for(int i=0;i<chain.getNrOfJoints();i++)
            result_q.push_back(result(i));
        return 1;
    }
    else
    {
        ROS_WARN("trac ik fail ,try again!");
        rc=tracik_solver.CartToJnt(current,end_effector_pose,result);
        if(rc>=0)
        {
            result_q.clear();
            for(int i=0;i<chain.getNrOfJoints();i++)
                result_q.push_back(result(i));
            return 1;
        }
        else
        {
            ROS_ERROR("trak_ik error : no ik solution!");
            return 0;
        }
    }
}

bool Joint_plan(std::vector<double> joint_group_positions)
{
    if(( (m_robot_state == s_init) || (m_robot_state == s_place)
        || (m_robot_state == s_back_up_middle) || (m_robot_state == s_back_up_right)
        || (m_robot_state == s_back_desk_middle) || (m_robot_state == s_back_desk_right)
        || (m_robot_state == s_up_right_observe) || (m_robot_state == s_up_middle_observe) || (m_robot_state == s_up_left_observe)
        || (m_robot_state == s_observe_desk_left) || (m_robot_state == s_observe_desk_middle) || (m_robot_state == s_observe_desk_right))
            && (m_robot_target == t_place))
    {
        bool execute_result = single_joint_action(joint_group_positions);
        if(execute_result)
        {
            ROS_INFO("JOINT plan success!");
            return true;
        }
        else
        {
            ROS_ERROR("JOINT plan failed!");
            return false;
        }
    }
    else if(( (m_robot_state == s_back_up_left) || (m_robot_state == s_back_desk_left)) && (m_robot_target == t_place))
    {
        std::vector<std::vector<double> > q_list;
        q_list.push_back(grasp_up_and_desk_left_to_place);
        q_list.push_back(joint_group_positions);

        bool execute_result = multi_joint_action(q_list);
        if(execute_result)
        {
            ROS_INFO("JOINT plan success!");
            return true;
        }
        else
        {
            ROS_ERROR("JOINT plan failed!");
            return false;
        }
    }
    else if(((m_robot_state == s_place)
             || (m_robot_state == s_back_up_middle)   || (m_robot_state == s_back_up_right) || (m_robot_state == s_back_up_left)
             || (m_robot_state == s_back_desk_middle) || (m_robot_state == s_back_desk_right) || (m_robot_state == s_back_desk_left))
            &&
            ( (m_robot_target == t_up_right) || (m_robot_target == t_up_middle) || (m_robot_target == t_up_left)
              || (m_robot_target == t_desk_left) || (m_robot_target == t_desk_middle) || (m_robot_target == t_desk_right)))
    {
        bool execute_result = single_joint_action(joint_group_positions);
        if(execute_result)
        {
            ROS_INFO("JOINT plan success!");
            return true;
        }
        else
        {
            ROS_ERROR("JOINT plan failed!");
            return false;
        }
    }
    else if(((m_robot_state == s_place) || (m_robot_state == s_back_down_right) ) && (m_robot_target == t_down_right))
    {
        std::vector<std::vector<double> > q_list;
        q_list.push_back(place_to_down_right_middle);
        q_list.push_back(joint_group_positions);

        bool execute_result = multi_joint_action(q_list);
        if(execute_result)
        {
            ROS_INFO("JOINT plan success!");
            return true;
        }
        else
        {
            ROS_ERROR("JOINT plan failed!");
            return false;
        }
    }
    else if((m_robot_state == s_down_right_observe) && (m_robot_target == t_place))
    {
        std::vector<std::vector<double> > q_list;
        q_list.push_back(place_to_down_right_middle);
        q_list.push_back(joint_group_positions);

        bool execute_result = multi_joint_action(q_list);
        if(execute_result)
        {
            ROS_INFO("JOINT plan success!");
            return true;
        }
        else
        {
            ROS_ERROR("JOINT plan failed!");
            return false;
        }
    }
    else if(((m_robot_state == s_place) || (m_robot_state == s_back_down_middle) ) && (m_robot_target == t_down_middle))
    {
        std::vector<std::vector<double> > q_list;
        q_list.push_back(place_to_down_middle_middle);
        q_list.push_back(joint_group_positions);

        bool execute_result = multi_joint_action(q_list);
        if(execute_result)
        {
            ROS_INFO("JOINT plan success!");
            return true;
        }
        else
        {
            ROS_ERROR("JOINT plan failed!");
            return false;
        }
    }
    else if((m_robot_state == s_down_middle_observe) && (m_robot_target == t_place))
    {
        std::vector<std::vector<double> > q_list;
        q_list.push_back(place_to_down_middle_middle);
        q_list.push_back(joint_group_positions);

        bool execute_result = multi_joint_action(q_list);
        if(execute_result)
        {
            ROS_INFO("JOINT plan success!");
            return true;
        }
        else
        {
            ROS_ERROR("JOINT plan failed!");
            return false;
        }
    }
    else if(((m_robot_state == s_place) || (m_robot_state == s_back_down_left) ) && (m_robot_target == t_down_left))
    {
        std::vector<std::vector<double> > q_list;
        q_list.push_back(place_to_down_left_middle);
        q_list.push_back(joint_group_positions);

        bool execute_result = multi_joint_action(q_list);
        if(execute_result)
        {
            ROS_INFO("JOINT plan success!");
            return true;
        }
        else
        {
            ROS_ERROR("JOINT plan failed!");
            return false;
        }
    }
    else if((m_robot_state == s_down_left_observe) && (m_robot_target == t_place))
    {
        std::vector<std::vector<double> > q_list;
        q_list.push_back(place_to_down_left_middle);
        q_list.push_back(joint_group_positions);

        bool execute_result = multi_joint_action(q_list);
        if(execute_result)
        {
            ROS_INFO("JOINT plan success!");
            return true;
        }
        else
        {
            ROS_ERROR("JOINT plan failed!");
            return false;
        }
    }
    else if((m_robot_state == s_up_right_observe) && (m_robot_target == t_pre_up_right))
    {
        std::vector<std::vector<double> > q_list;
        q_list.push_back(up_right_observe_to_pre_up_right_middle);
        q_list.push_back(joint_group_positions);

        bool execute_result = multi_joint_action(q_list);
        if(execute_result)
        {
            ROS_INFO("JOINT plan success!");
            return true;
        }
        else
        {
            ROS_ERROR("JOINT plan failed!");
            return false;
        }
    }
    else if( (m_robot_state == s_up_middle_observe) && (m_robot_target == t_pre_up_middle))
    {
        std::vector<std::vector<double> > q_list;
        q_list.push_back(up_middle_observe_to_pre_up_middle_middle);
        q_list.push_back(joint_group_positions);

        bool execute_result = multi_joint_action(q_list);
        if(execute_result)
        {
            ROS_INFO("JOINT plan success!");
            return true;
        }
        else
        {
            ROS_ERROR("JOINT plan failed!");
            return false;
        }
    }
    else if((m_robot_state == s_up_left_observe)  && (m_robot_target == t_pre_up_left))
    {
        std::vector<std::vector<double> > q_list;
        q_list.push_back(up_left_observe_to_pre_up_left_middle);
        q_list.push_back(joint_group_positions);

        bool execute_result = multi_joint_action(q_list);
        if(execute_result)
        {
            ROS_INFO("JOINT plan success!");
            return true;
        }
        else
        {
            ROS_ERROR("JOINT plan failed!");
            return false;
        }
    }
    else if((m_robot_state == s_down_left_observe) && (m_robot_target == t_pre_down_left))
    {
        std::vector<std::vector<double> > q_list;
        q_list.push_back(down_left_observe_to_pre_down_left_middle1);
        q_list.push_back(down_left_observe_to_pre_down_left_middle2);
        q_list.push_back(joint_group_positions);

        bool execute_result = multi_joint_action(q_list);
        if(execute_result)
        {
            ROS_INFO("JOINT plan success!");
            return true;
        }
        else
        {
            ROS_ERROR("JOINT plan failed!");
            return false;
        }
    }
    else if( (m_robot_state == s_down_middle_observe) && (m_robot_target == t_pre_down_middle))
    {
        std::vector<std::vector<double> > q_list;
        q_list.push_back(down_middle_observe_to_pre_down_middle_middle1);
        q_list.push_back(down_middle_observe_to_pre_down_middle_middle2);
        q_list.push_back(joint_group_positions);

        bool execute_result = multi_joint_action(q_list);
        if(execute_result)
        {
            ROS_INFO("JOINT plan success!");
            return true;
        }
        else
        {
            ROS_ERROR("JOINT plan failed!");
            return false;
        }
    }
    else if((m_robot_state == s_down_right_observe) && (m_robot_target == t_pre_down_right))
    {
        std::vector<std::vector<double> > q_list;
        q_list.push_back(down_right_observe_to_pre_down_right_middle1);
        q_list.push_back(down_right_observe_to_pre_down_right_middle2);
        q_list.push_back(joint_group_positions);

        bool execute_result = multi_joint_action(q_list);
        if(execute_result)
        {
            ROS_INFO("JOINT plan success!");
            return true;
        }
        else
        {
            ROS_ERROR("JOINT plan failed!");
            return false;
        }
    }
    else if((m_robot_state == s_observe_desk_left) && (m_robot_target == t_pre_desk_left))
    {
        std::vector<std::vector<double> > q_list;
        q_list.push_back(desk_left_observe_to_pre_desk_left_middle);
        q_list.push_back(joint_group_positions);

        bool execute_result = multi_joint_action(q_list);
        if(execute_result)
        {
            ROS_INFO("JOINT plan success!");
            return true;
        }
        else
        {
            ROS_ERROR("JOINT plan failed!");
            return false;
        }
    }
    else if( (m_robot_state == s_observe_desk_middle) && (m_robot_target == t_pre_desk_middle))
    {
        std::vector<std::vector<double> > q_list;
        q_list.push_back(desk_middle_observe_to_pre_desk_middle_middle);
        q_list.push_back(joint_group_positions);

        bool execute_result = multi_joint_action(q_list);
        if(execute_result)
        {
            ROS_INFO("JOINT plan success!");
            return true;
        }
        else
        {
            ROS_ERROR("JOINT plan failed!");
            return false;
        }
    }
    else if((m_robot_state == s_observe_desk_right) && (m_robot_target == t_pre_desk_right))
    {
        std::vector<std::vector<double> > q_list;
        q_list.push_back(desk_right_observe_to_pre_desk_right_middle);
        q_list.push_back(joint_group_positions);

        bool execute_result = multi_joint_action(q_list);
        if(execute_result)
        {
            ROS_INFO("JOINT plan success!");
            return true;
        }
        else
        {
            ROS_ERROR("JOINT plan failed!");
            return false;
        }
    }
    else if(((m_robot_state == s_pre_up_left) || (m_robot_state == s_pre_up_middle) || (m_robot_state == s_pre_up_right)
             || (m_robot_state == s_pre_down_left) || (m_robot_state == s_pre_down_middle) || (m_robot_state == s_pre_down_right)
             || (m_robot_state == s_pre_desk_left) || (m_robot_state == s_pre_desk_middle) || (m_robot_state == s_pre_desk_right))
            && ((m_robot_target == t_up_grasp) || (m_robot_target == t_down_grasp) || (m_robot_target == t_desk_grasp)))
    {
        bool execute_result = movel_joint_action_with_time(joint_group_positions,2.5);
        if(execute_result)
        {
            ROS_INFO("JOINT plan success!");
            return true;
        }
        else
        {
            ROS_ERROR("JOINT plan failed!");
            return false;
        }
    }
    else if((m_robot_state == s_back_down_left) && (m_robot_target == t_place))
    {
        std::vector<std::vector<double> > q_list;
        q_list.push_back(grasp_down_left_to_place_middle);
        q_list.push_back(joint_group_positions);

        bool execute_result = multi_joint_action(q_list);
        if(execute_result)
        {
            ROS_INFO("JOINT plan success!");
            return true;
        }
        else
        {
            ROS_ERROR("JOINT plan failed!");
            return false;
        }
    }
    else if((m_robot_state == s_back_down_middle) && (m_robot_target == t_place))
    {
        std::vector<std::vector<double> > q_list;
        q_list.push_back(grasp_down_middle_to_place_middle);
        q_list.push_back(joint_group_positions);

        bool execute_result = multi_joint_action(q_list);
        if(execute_result)
        {
            ROS_INFO("JOINT plan success!");
            return true;
        }
        else
        {
            ROS_ERROR("JOINT plan failed!");
            return false;
        }
    }
    else if((m_robot_state == s_back_down_right) && (m_robot_target == t_place))
    {
        std::vector<std::vector<double> > q_list;
        q_list.push_back(grasp_down_right_to_place_middle);
        q_list.push_back(joint_group_positions);

        bool execute_result = multi_joint_action(q_list);
        if(execute_result)
        {
            ROS_INFO("JOINT plan success!");
            return true;
        }
        else
        {
            ROS_ERROR("JOINT plan failed!");
            return false;
        }
    }
    else if((m_robot_state == s_up_right_observe) && (m_robot_target == t_up_middle))
    {
        bool execute_result = movel_joint_action(joint_group_positions);
        if(execute_result)
        {
            ROS_INFO("JOINT plan success!");
            return true;
        }
        else
        {
            ROS_ERROR("JOINT plan failed!");
            return false;
        }
    }
    else if((m_robot_state == s_up_middle_observe) && (m_robot_target == t_up_left))
    {
        bool execute_result = movel_joint_action(joint_group_positions);
        if(execute_result)
        {
            ROS_INFO("JOINT plan success!");
            return true;
        }
        else
        {
            ROS_ERROR("JOINT plan failed!");
            return false;
        }
    }
    else if((m_robot_state == s_up_left_observe) && (m_robot_target == t_down_left))
    {
        bool execute_result = movel_joint_action(joint_group_positions);
        if(execute_result)
        {
            ROS_INFO("JOINT plan success!");
            return true;
        }
        else
        {
            ROS_ERROR("JOINT plan failed!");
            return false;
        }
    }
    else if((m_robot_state == s_down_left_observe) && (m_robot_target == t_down_middle))
    {
        bool execute_result = movel_joint_action(joint_group_positions);
        if(execute_result)
        {
            ROS_INFO("JOINT plan success!");
            return true;
        }
        else
        {
            ROS_ERROR("JOINT plan failed!");
            return false;
        }
    }
    else if((m_robot_state == s_down_middle_observe) && (m_robot_target == t_down_right))
    {
        bool execute_result = movel_joint_action(joint_group_positions);
        if(execute_result)
        {
            ROS_INFO("JOINT plan success!");
            return true;
        }
        else
        {
            ROS_ERROR("JOINT plan failed!");
            return false;
        }
    }
    else if((m_robot_state == s_observe_desk_right) && (m_robot_target == t_desk_left))
    {
        bool execute_result = movel_joint_action(joint_group_positions);
        if(execute_result)
        {
            ROS_INFO("JOINT plan success!");
            return true;
        }
        else
        {
            ROS_ERROR("JOINT plan failed!");
            return false;
        }
    }
    else if((m_robot_state == s_observe_desk_middle) && (m_robot_target == t_desk_left))
    {
        bool execute_result = movel_joint_action(joint_group_positions);
        if(execute_result)
        {
            ROS_INFO("JOINT plan success!");
            return true;
        }
        else
        {
            ROS_ERROR("JOINT plan failed!");
            return false;
        }
    }
    else if((m_robot_state == s_up_right_observe) && (m_robot_target == t_up_right))
    {
        ROS_INFO("JOINT plan success!");
        return true;
    }
    else if((m_robot_state == s_up_middle_observe) && (m_robot_target == t_up_middle))
    {
        ROS_INFO("JOINT plan success!");
        return true;
    }
    else if((m_robot_state == s_up_left_observe) && (m_robot_target == t_up_left))
    {
        ROS_INFO("JOINT plan success!");
        return true;
    }
    else if((m_robot_state == s_down_right_observe) && (m_robot_target == t_down_right))
    {
        ROS_INFO("JOINT plan success!");
        return true;
    }
    else if((m_robot_state == s_down_left_observe) && (m_robot_target == t_down_left))
    {
        ROS_INFO("JOINT plan success!");
        return true;
    }
    else if((m_robot_state == s_down_middle_observe) && (m_robot_target == t_down_middle))
    {
        ROS_INFO("JOINT plan success!");
        return true;
    }
    else if((m_robot_state == s_observe_desk_left) && (m_robot_target == t_desk_left))
    {
        ROS_INFO("JOINT plan success!");
        return true;
    }
    else if((m_robot_state == s_observe_desk_right) && (m_robot_target == t_desk_right))
    {
        ROS_INFO("JOINT plan success!");
        return true;
    }
    else if((m_robot_state == s_observe_desk_middle) && (m_robot_target == t_desk_middle))
    {
        ROS_INFO("JOINT plan success!");
        return true;
    }
    else
    {
        ROS_ERROR("JOINT plan : invaild robot state or invaild target!");
        return false;
    }

    return 0;
}

bool is_in_range(geometry_msgs::Pose target_pose)
{
    if(m_robot_state == s_up_middle_observe)
    {
        std::vector<double> result;
        if(trac_ik_solver(pre_up_middle,target_pose,result))
        {
            return 1;
        }
        else
        {
            ROS_ERROR("c plan not in range!");
            return 0;
        }
    }
    else if(m_robot_state == s_up_right_observe)
    {
        std::vector<double> result;
        if(trac_ik_solver(pre_up_right,target_pose,result))
        {
            return 1;
        }
        else
        {
            ROS_ERROR("c plan not in range!");
            return 0;
        }
    }
    else if(m_robot_state == s_up_left_observe)
    {
        std::vector<double> result;
        if(trac_ik_solver(pre_up_left,target_pose,result))
        {
            return 1;
        }
        else
        {
            ROS_ERROR("c plan not in range!");
            return 0;
        }
    }
    else if(m_robot_state == s_down_middle_observe)
    {
        std::vector<double> result;
        if(trac_ik_solver(pre_down_middle,target_pose,result))
        {
            return 1;
        }
        else
        {
            ROS_ERROR("c plan not in range!");
            return 0;
        }
    }
    else if(m_robot_state == s_down_right_observe)
    {
        std::vector<double> result;
        if(trac_ik_solver(pre_down_right,target_pose,result))
        {
            return 1;
        }
        else
        {
            ROS_ERROR("c plan not in range!");
            return 0;
        }
    }
    else if(m_robot_state == s_down_left_observe)
    {
        std::vector<double> result;
        if(trac_ik_solver(pre_down_left,target_pose,result))
        {
            return 1;
        }
        else
        {
            ROS_ERROR("c plan not in range!");
            return 0;
        }
    }
    else if(m_robot_state == s_observe_desk_middle)
    {
        std::vector<double> result;
        if(trac_ik_solver(pre_desk_middle,target_pose,result))
        {
            return 1;
        }
        else
        {
            ROS_ERROR("c plan not in range!");
            return 0;
        }
    }
    else if(m_robot_state == s_observe_desk_right)
    {
        std::vector<double> result;
        if(trac_ik_solver(pre_desk_right,target_pose,result))
        {
            return 1;
        }
        else
        {
            ROS_ERROR("c plan not in range!");
            return 0;
        }
    }
    else if(m_robot_state == s_observe_desk_left)
    {
        std::vector<double> result;
        if(trac_ik_solver(pre_desk_left,target_pose,result))
        {
            return 1;
        }
        else
        {
            ROS_ERROR("c plan not in range!");
            return 0;
        }
    }
    else
    {
        ROS_ERROR("robot is not at observe place!");
        return 0;
    }
}

bool is_in_current_observe_place(geometry_msgs::Pose target_pose)
{
    if(m_robot_state == s_up_left_observe)
    {
	if(target_pose.position.x>=0.15)
		return true;
	else
		return false;
    }
    else if(m_robot_state == s_up_middle_observe)
    {
	if(target_pose.position.x>=-0.25 && target_pose.position.x<=0.25)
		return true;
	else
		return false;
    }
    else if(m_robot_state == s_up_right_observe)
    {
	if(target_pose.position.x<=-0.15)
		return true;
	else
		return false;
    }
    else if(m_robot_state == s_down_left_observe)
    {
	if(target_pose.position.x>=0.15)
		return true;
	else
		return false;
    }
    else if(m_robot_state == s_down_middle_observe)
    {
	if(target_pose.position.x>=-0.25 && target_pose.position.x<=0.25)
		return true;
	else
		return false;
    }
    else if(m_robot_state == s_down_right_observe)
    {
	if(target_pose.position.x<=-0.15)
		return true;
	else
		return false;
    }
    else if(m_robot_state == s_observe_desk_left)
    {
	if(target_pose.position.x>=-0.05)
		return true;
	else
		return false;
    }
    else if(m_robot_state == s_observe_desk_middle)
    {
	if(target_pose.position.x>=-0.25 && target_pose.position.x<=0.25)
		return true;
	else
		return false;
    }
    else if(m_robot_state == s_observe_desk_right)
    {
	if(target_pose.position.x<=0.05)
		return true;
	else
		return false;
    }
    else 
        return false;
}

bool pose_to_joint_plan(geometry_msgs::Pose target_pose)
{
    if(m_robot_state == s_up_middle_observe)
    {
        std::vector<double> result;
        if(trac_ik_solver(pre_up_middle,target_pose,result))
        {
            for(int i=0;i<6;i++)
                std::cout<<result[i]<<"   ";
            std::cout<<std::endl;
            m_robot_target = t_pre_up_middle;
            bool execute_result = Joint_plan(result);
            if(execute_result)
            {
                m_robot_state = s_pre_up_middle;
                m_robot_target = t_unknown;
                return 1;
            }
            else
            {
                ROS_ERROR("pose_to_joint_plan : failed!");
                m_robot_target = t_unknown;
                return 0;
            }
        }
        else
        {
            ROS_ERROR("pose_to_joint_plan : failed!");
            return 0;
        }
    }
    else if(m_robot_state == s_up_right_observe)
    {
        std::vector<double> result;
        if(trac_ik_solver(pre_up_right,target_pose,result))
        {
            for(int i=0;i<6;i++)
                std::cout<<result[i]<<"   ";
            std::cout<<std::endl;
            m_robot_target = t_pre_up_right;
            bool execute_result = Joint_plan(result);
            if(execute_result)
            {
                m_robot_state = s_pre_up_right;
                m_robot_target = t_unknown;
                return 1;
            }
            else
            {
                ROS_ERROR("pose_to_joint_plan : failed!");
                m_robot_target = t_unknown;
                return 0;
            }
        }
        else
        {
            ROS_ERROR("pose_to_joint_plan : failed!");
            return 0;
        }
    }
    else if(m_robot_state == s_up_left_observe)
    {
        std::vector<double> result;
        if(trac_ik_solver(pre_up_left,target_pose,result))
        {
            for(int i=0;i<6;i++)
                std::cout<<result[i]<<"   ";
            std::cout<<std::endl;
            m_robot_target = t_pre_up_left;
            bool execute_result = Joint_plan(result);
            if(execute_result)
            {
                m_robot_state = s_pre_up_left;
                m_robot_target = t_unknown;
                return 1;
            }
            else
            {
                ROS_ERROR("pose_to_joint_plan : failed!");
                m_robot_target = t_unknown;
                return 0;
            }
        }
        else
        {
            ROS_ERROR("pose_to_joint_plan : failed!");
            return 0;
        }
    }
    else if(m_robot_state == s_down_middle_observe)
    {
        std::vector<double> result;
        if(trac_ik_solver(pre_down_middle,target_pose,result))
        {
            for(int i=0;i<6;i++)
                std::cout<<result[i]<<"   ";
            std::cout<<std::endl;
            m_robot_target = t_pre_down_middle;
            bool execute_result = Joint_plan(result);
            if(execute_result)
            {
                m_robot_state = s_pre_down_middle;
                m_robot_target = t_unknown;
                return 1;
            }
            else
            {
                ROS_ERROR("pose_to_joint_plan : failed!");
                m_robot_target = t_unknown;
                return 0;
            }
        }
        else
        {
            ROS_ERROR("pose_to_joint_plan : failed!");
            return 0;
        }
    }
    else if(m_robot_state == s_down_right_observe)
    {
        std::vector<double> result;
        if(trac_ik_solver(pre_down_right,target_pose,result))
        {
            for(int i=0;i<6;i++)
                std::cout<<result[i]<<"   ";
            std::cout<<std::endl;
            m_robot_target = t_pre_down_right;
            bool execute_result = Joint_plan(result);
            if(execute_result)
            {
                m_robot_state = s_pre_down_right;
                m_robot_target = t_unknown;
                return 1;
            }
            else
            {
                ROS_ERROR("pose_to_joint_plan : failed!");
                m_robot_target = t_unknown;
                return 0;
            }
        }
        else
        {
            ROS_ERROR("pose_to_joint_plan : failed!");
            return 0;
        }
    }
    else if(m_robot_state == s_down_left_observe)
    {
        std::vector<double> result;
        if(trac_ik_solver(pre_down_left,target_pose,result))
        {
            for(int i=0;i<6;i++)
                std::cout<<result[i]<<"   ";
            std::cout<<std::endl;
            m_robot_target = t_pre_down_left;
            bool execute_result = Joint_plan(result);
            if(execute_result)
            {
                m_robot_state = s_pre_down_left;
                m_robot_target = t_unknown;
                return 1;
            }
            else
            {
                ROS_ERROR("pose_to_joint_plan : failed!");
                m_robot_target = t_unknown;
                return 0;
            }
        }
        else
        {
            ROS_ERROR("pose_to_joint_plan : failed!");
            return 0;
        }
    }
    else if(m_robot_state == s_observe_desk_left)
    {
        std::vector<double> result;
        if(trac_ik_solver(pre_desk_left,target_pose,result))
        {
            for(int i=0;i<6;i++)
                std::cout<<result[i]<<"   ";
            std::cout<<std::endl;
            m_robot_target = t_pre_desk_left;
            bool execute_result = Joint_plan(result);
            if(execute_result)
            {
                m_robot_state = s_pre_desk_left;
                m_robot_target = t_unknown;
                return 1;
            }
            else
            {
                ROS_ERROR("pose_to_joint_plan : failed!");
                m_robot_target = t_unknown;
                return 0;
            }
        }
        else
        {
            ROS_ERROR("pose_to_joint_plan : failed!");
            return 0;
        }
    }
    else if(m_robot_state == s_observe_desk_middle)
    {
        std::vector<double> result;
        if(trac_ik_solver(pre_desk_middle,target_pose,result))
        {
            for(int i=0;i<6;i++)
                std::cout<<result[i]<<"   ";
            std::cout<<std::endl;
            m_robot_target = t_pre_desk_middle;
            bool execute_result = Joint_plan(result);
            if(execute_result)
            {
                m_robot_state = s_pre_desk_middle;
                m_robot_target = t_unknown;
                return 1;
            }
            else
            {
                ROS_ERROR("pose_to_joint_plan : failed!");
                m_robot_target = t_unknown;
                return 0;
            }
        }
        else
        {
            ROS_ERROR("pose_to_joint_plan : failed!");
            return 0;
        }
    }
    else if(m_robot_state == s_observe_desk_right)
    {
        std::vector<double> result;
        if(trac_ik_solver(pre_desk_right,target_pose,result))
        {
            for(int i=0;i<6;i++)
                std::cout<<result[i]<<"   ";
            std::cout<<std::endl;
            m_robot_target = t_pre_desk_right;
            bool execute_result = Joint_plan(result);
            if(execute_result)
            {
                m_robot_state = s_pre_desk_right;
                m_robot_target = t_unknown;
                return 1;
            }
            else
            {
                ROS_ERROR("pose_to_joint_plan : failed!");
                m_robot_target = t_unknown;
                return 0;
            }
        }
        else
        {
            ROS_ERROR("pose_to_joint_plan : failed!");
            return 0;
        }
    }
    else
    {
        ROS_ERROR("robot is not at observe place!");
        ROS_ERROR("pose_to_joint_plan : failed!");
        return 0;
    }
}

bool C_plan(geometry_msgs::Pose target_pose)
{
    std::vector<double> result;
    bool ik_result;
    if(m_robot_state == s_pre_up_left)
        ik_result = trac_ik_solver(pre_up_left,target_pose,result);
    else if(m_robot_state == s_pre_up_middle)
        ik_result = trac_ik_solver(pre_up_middle,target_pose,result);
    else if(m_robot_state == s_pre_up_right)
        ik_result = trac_ik_solver(pre_up_right,target_pose,result);
    else if(m_robot_state == s_pre_down_right)
        ik_result = trac_ik_solver(pre_down_right,target_pose,result);
    else if(m_robot_state == s_pre_down_middle)
        ik_result = trac_ik_solver(pre_down_middle,target_pose,result);
    else if(m_robot_state == s_pre_down_left)
        ik_result = trac_ik_solver(pre_down_left,target_pose,result);
    else if(m_robot_state == s_pre_desk_left)
        ik_result = trac_ik_solver(pre_desk_left,target_pose,result);
    else if(m_robot_state == s_pre_desk_middle)
        ik_result = trac_ik_solver(pre_desk_middle,target_pose,result);
    else if(m_robot_state == s_pre_desk_right)
        ik_result = trac_ik_solver(pre_desk_right,target_pose,result);
    else
    {
        ROS_ERROR("C_plan : robot should be at pre_grasp_place!");
        return 0;
    }

    if(!ik_result)
    {
        ROS_ERROR("C_plan : failed!");
        return 0;
    }

    if((m_robot_state == s_pre_up_left) || (m_robot_state == s_pre_up_middle) || (m_robot_state == s_pre_up_right))
        m_robot_target = t_up_grasp;
    else if((m_robot_state == s_pre_down_left) || (m_robot_state == s_pre_down_middle) || (m_robot_state == s_pre_down_right))
        m_robot_target = t_down_grasp;
    else
        m_robot_target = t_desk_grasp;


    bool execute_result = Joint_plan(result);
    if(execute_result)
    {
        if(m_robot_state == s_pre_up_left)
            m_robot_state = s_grasp_up_left;
        else if(m_robot_state == s_pre_up_middle)
            m_robot_state = s_grasp_up_middle;
        else if(m_robot_state == s_pre_up_right)
            m_robot_state = s_grasp_up_right;
        else if(m_robot_state == s_pre_down_right)
            m_robot_state = s_grasp_down_right;
        else if(m_robot_state == s_pre_down_middle)
            m_robot_state = s_grasp_down_middle;
        else if(m_robot_state == s_pre_down_left)
            m_robot_state = s_grasp_down_left;
        else if(m_robot_state == s_pre_desk_left)
            m_robot_state = s_grasp_desk_left;
        else if(m_robot_state == s_pre_desk_middle)
            m_robot_state = s_grasp_desk_middle;
        else
            m_robot_state = s_grasp_desk_right;

        m_robot_target = t_unknown;
        return 1;
    }
    else
    {
        ROS_ERROR("C_plan : failed!");
        m_robot_target = t_unknown;
        return 0;
    }
}

bool C_plan2(geometry_msgs::Pose target_pose)
{
    target_pose.position.z += cartesian_distance;
    std::vector<double> result1,result2;
    bool slover_result;
    if(m_robot_state == s_pre_up_left)
        slover_result = trac_ik_solver(pre_up_left,target_pose,result1);
    else if(m_robot_state == s_pre_up_middle)
        slover_result = trac_ik_solver(pre_up_middle,target_pose,result1);
    else if(m_robot_state == s_pre_up_right)
        slover_result = trac_ik_solver(pre_up_right,target_pose,result1);
    else if(m_robot_state == s_pre_down_right)
        slover_result = trac_ik_solver(pre_down_right,target_pose,result1);
    else if(m_robot_state == s_pre_down_left)
        slover_result = trac_ik_solver(pre_down_left,target_pose,result1);
    else if(m_robot_state == s_pre_down_middle)
        slover_result = trac_ik_solver(pre_down_middle,target_pose,result1);
    else if(m_robot_state == s_pre_desk_middle)
        slover_result = trac_ik_solver(pre_desk_middle,target_pose,result1);
    else if(m_robot_state == s_pre_desk_left)
        slover_result = trac_ik_solver(pre_desk_left,target_pose,result1);
    else if(m_robot_state == s_pre_desk_right)
        slover_result = trac_ik_solver(pre_desk_right,target_pose,result1);
    else
    {
        ROS_ERROR("c_plan2 : robot should be at pre_grasp_place!");
        return false;
    }
    if(!slover_result)
    {
        ROS_ERROR("C_plan2 failed!");
        return false;
    }

    target_pose.position.z -= cartesian_distance;
    slover_result = trac_ik_solver(result1,target_pose,result2);
    if(!slover_result)
    {
        ROS_ERROR("C_plan2 failed!");
        return false;
    }

    std::vector<std::vector<double> > q_list;
    q_list.push_back(result1);
    q_list.push_back(result2);
    std::vector<double> time;
    time.push_back(-1);
    time.push_back(1);
    //bool execute_result = multi_movel_joint_action(q_list);
    //bool execute_result = multi_joint_action(q_list);
    bool execute_result = multi_movel_joint_action_with_time(q_list,time);
    if(!execute_result)
    {
        ROS_ERROR("C_plan2 failed!");
        return false;
    }
    else
    {
        if(m_robot_state == s_pre_up_left)
            m_robot_state = s_grasp_up_left;
        else if(m_robot_state == s_pre_up_middle)
            m_robot_state = s_grasp_up_middle;
        else if(m_robot_state == s_pre_up_right)
            m_robot_state = s_grasp_up_right;
        else if(m_robot_state == s_pre_down_right)
            m_robot_state = s_grasp_down_right;
        else if(m_robot_state == s_pre_down_left)
            m_robot_state = s_grasp_down_left;
        else if(m_robot_state == s_pre_down_middle)
            m_robot_state = s_grasp_down_middle;
        else if(m_robot_state == s_pre_desk_middle)
            m_robot_state = s_grasp_desk_middle;
        else if(m_robot_state == s_pre_desk_left)
            m_robot_state = s_grasp_desk_left;
        else
            m_robot_state = s_grasp_desk_right;

        return true;
    }
}

bool move_to_observe_up_right()
{
    m_robot_target = t_up_right;
    ROS_INFO("move_to_observe_up_right");
    bool execute_result;
    execute_result = Joint_plan(observe_up_right);
    if(execute_result)
    {
        ROS_INFO("move to observe_up_right succeed");
        m_robot_state = s_up_right_observe;
        m_robot_target = t_unknown;
        return true;
    }
    else
    {
        m_robot_target = t_unknown;
        ROS_ERROR("move to observe_up_right fail");
        return false;
    }
}

bool move_to_observe_down_right()
{
    m_robot_target = t_down_right;
    ROS_INFO("move_to_observe_down_right");
    bool execute_result;
    execute_result = Joint_plan(observe_down_right);
    if(execute_result)
    {
        ROS_INFO("move to observe_down_right succeed");
        m_robot_state = s_down_right_observe;
        m_robot_target = t_unknown;
        return true;
    }
    else
    {
        m_robot_target = t_unknown;
        ROS_ERROR("move to observe_down_right fail");
        return false;
    }
}

bool move_to_observe_up_left()
{
    m_robot_target = t_up_left;
    ROS_INFO("move_to_observe_up_left");
    bool execute_result;
    execute_result = Joint_plan(observe_up_left);
    if(execute_result)
    {
        ROS_INFO("move to observe_up_left succeed");
        m_robot_state = s_up_left_observe;
        m_robot_target = t_unknown;
        return true;
    }
    else
    {
        m_robot_target = t_unknown;
        ROS_ERROR("move to observe_up_left fail");
        return false;
    }
}

bool move_to_observe_down_left()
{
    m_robot_target = t_down_left;
    ROS_INFO("move_to_observe_down_left");
    bool execute_result;
    execute_result = Joint_plan(observe_down_left);
    if(execute_result)
    {
        ROS_INFO("move to observe_down_left succeed");
        m_robot_state = s_down_left_observe;
        m_robot_target = t_unknown;
        return true;
    }
    else
    {
        m_robot_target = t_unknown;
        ROS_ERROR("move to observe_down_left fail");
        return false;
    }
}

bool move_to_observe_down_middle()
{
    m_robot_target = t_down_middle;
    ROS_INFO("move_to_observe_down_middle");
    bool execute_result;
    execute_result = Joint_plan(observe_down_middle);
    if(execute_result)
    {
        ROS_INFO("move to observe_down_middle succeed");
        m_robot_state = s_down_middle_observe;
        m_robot_target = t_unknown;
        return true;
    }
    else
    {
        m_robot_target = t_unknown;
        ROS_ERROR("move to observe_down_middle fail");
        return false;
    }
}

bool move_to_observe_up_middle()
{
    m_robot_target = t_up_middle;
    ROS_INFO("move_to_observe_up_middle");
    bool execute_result;
    execute_result = Joint_plan(observe_up_middle);
    if(execute_result)
    {
        ROS_INFO("move to observe_up_middle succeed");
        m_robot_state = s_up_middle_observe;
        m_robot_target = t_unknown;
        return true;
    }
    else
    {
        m_robot_target = t_unknown;
        ROS_ERROR("move to observe_up_middle fail");
        return false;
    }
}

bool move_to_observe_desk_left()
{
    m_robot_target = t_desk_left;
    ROS_INFO("move_to_observe_desk_left");
    bool execute_result;
    execute_result = Joint_plan(observe_desk_left);
    if(execute_result)
    {
        ROS_INFO("move to observe_desk_left succeed");
        m_robot_state = s_observe_desk_left;
        m_robot_target = t_unknown;
        return true;
    }
    else
    {
        m_robot_target = t_unknown;
        ROS_ERROR("move to observe_desk_left fail");
        return false;
    }
}

bool move_to_observe_desk_right()
{
    m_robot_target = t_desk_right;
    ROS_INFO("move_to_observe_desk_right");
    bool execute_result;
    execute_result = Joint_plan(observe_desk_right);
    if(execute_result)
    {
        ROS_INFO("move to observe_desk_right succeed");
        m_robot_state = s_observe_desk_right;
        m_robot_target = t_unknown;
        return true;
    }
    else
    {
        m_robot_target = t_unknown;
        ROS_ERROR("move to observe_desk_right fail");
        return false;
    }
}

bool move_to_observe_desk_middle()
{
    m_robot_target = t_desk_middle;
    ROS_INFO("move_to_observe_desk_middle");
    bool execute_result;
    execute_result = Joint_plan(observe_desk_middle);
    if(execute_result)
    {
        ROS_INFO("move to observe_desk_middle succeed");
        m_robot_state = s_observe_desk_middle;
        m_robot_target = t_unknown;
        return true;
    }
    else
    {
        m_robot_target = t_unknown;
        ROS_ERROR("move to observe_desk_middle fail");
        return false;
    }
}

bool move_to_place()
{
    m_robot_target = t_place;
    ROS_INFO("move_to_place");
    bool execute_result;
    if(place_key)
    {
       execute_result = Joint_plan(place);
       place_key = false;
    }
    else
    {
       execute_result = Joint_plan(place1);
       place_key = true;
    }
    	
    if(execute_result)
    {
        ROS_INFO("move to place succeed");
        m_robot_state = s_place;
        m_robot_target = t_unknown;
        return true;
    }
    else
    {
        m_robot_target = t_unknown;
        ROS_ERROR("plan to place fail");
        return false;
    }
}

bool push_board()
{
    bool execute_result = single_joint_action(push_board_middle_up);
    if(!execute_result)
    {
        ROS_ERROR("move to push board middle up failed!");
        return false;
    }

    execute_result = movel_joint_action_with_time(push_board_middle_down,3);
    if(!execute_result)
    {
        ROS_ERROR("move to push board middle down failed!");
        return false;
    }

    sleep(3);

    execute_result = movel_joint_action_with_time(push_board_middle_up,2);
    if(!execute_result)
    {
        ROS_ERROR("move to push board middle up failed!");
        return false;
    }


    execute_result = single_joint_action(push_board_left_up);
    if(!execute_result)
    {
        ROS_ERROR("move to push board left up failed!");
        return false;
    }

    execute_result = movel_joint_action_with_time(push_board_left_down,3);
    if(!execute_result)
    {
        ROS_ERROR("move to push board left down failed!");
        return false;
    }

    sleep(3);

    execute_result = movel_joint_action_with_time(push_board_left_up,2);
    if(!execute_result)
    {
        ROS_ERROR("move to push board left up failed!");
        return false;
    }

    execute_result = single_joint_action(push_board_right_up);
    if(!execute_result)
    {
        ROS_ERROR("move to push board right up failed!");
        return false;
    }

    execute_result = movel_joint_action_with_time(push_board_right_down,3);
    if(!execute_result)
    {
        ROS_ERROR("move to push board right down failed!");
        return false;
    }

    sleep(3);

    execute_result = movel_joint_action_with_time(push_board_right_up,2);
    if(!execute_result)
    {
        ROS_ERROR("move to push board right up failed!");
        return false;
    }


    for(int i=0;i<2;i++)
    {
    execute_result = single_joint_action(push_board_left_up2);
    if(!execute_result)
    {
        ROS_ERROR("move to push board left up2 failed!");
        return false;
    }

    execute_result = movel_joint_action_with_time(push_board_left_down2,3);
    if(!execute_result)
    {
        ROS_ERROR("move to push board left down2 failed!");
        return false;
    }

    sleep(3);

    execute_result = movel_joint_action_with_time(push_board_left_up2,2);
    if(!execute_result)
    {
        ROS_ERROR("move to push board left up2 failed!");
        return false;
    }


    execute_result = single_joint_action(push_board_right_up2);
    if(!execute_result)
    {
        ROS_ERROR("move to push board right up2 failed!");
        return false;
    }

    execute_result = movel_joint_action_with_time(push_board_right_down2,3);
    if(!execute_result)
    {
        ROS_ERROR("move to push board right down2 failed!");
        return false;
    }

    sleep(3);

    execute_result = movel_joint_action_with_time(push_board_right_up2,2);
    if(!execute_result)
    {
        ROS_ERROR("move to push board right up failed!");
        return false;
    }

    }

    execute_result = single_joint_action(place);
    if(!execute_result)
    {
        ROS_ERROR("move to place failed!");
        return false;
    }

    return true;
}

bool back_c_plan1(geometry_msgs::Pose current_pose_m)
{
    current_pose_m.position.z += 0.1;
    std::vector<double> result1,result2;
    bool slover_result = false;
    if(m_robot_state == s_grasp_up_right)
        slover_result = trac_ik_solver(pre_up_right,current_pose_m,result1);
    else if(m_robot_state == s_grasp_up_middle)
        slover_result = trac_ik_solver(pre_up_middle,current_pose_m,result1);
    else if(m_robot_state == s_grasp_up_left)
        slover_result = trac_ik_solver(pre_up_left,current_pose_m,result1);
    else if(m_robot_state == s_grasp_desk_left)
    {
	current_pose_m.position.z += 0.1;
        slover_result = trac_ik_solver(pre_desk_left,current_pose_m,result1);
    }
    else if(m_robot_state == s_grasp_desk_middle)
    {
	current_pose_m.position.z += 0.1;
        slover_result = trac_ik_solver(pre_desk_middle,current_pose_m,result1);
    }
    else if(m_robot_state == s_grasp_desk_right)
    {
	current_pose_m.position.z += 0.1;
        slover_result = trac_ik_solver(pre_desk_right,current_pose_m,result1);
    }
    else
    {
        ROS_ERROR("robot is not at grasp place");
        return 0;
    }

    if(!slover_result)
    {
        ROS_ERROR("back_c_plan1 failed!");
        return false;
    }

    std::cout<<"result1: ";
    for(int i=0;i<6;i++)
    {
	std::cout<<result1[i]<<"    ";
    }
    std::cout<<std::endl;

    current_pose_m.position.y = (y_up+0.12);
    slover_result = trac_ik_solver(result1,current_pose_m,result2);

    if(!slover_result)
    {
        ROS_ERROR("back_c_plan1 failed!");
        return false;
    }

    std::cout<<"result2: ";
    for(int i=0;i<6;i++)
    {
	std::cout<<result2[i]<<"    ";
    }
    std::cout<<std::endl;

    std::vector<std::vector<double> > q_list;
    q_list.push_back(result1);
    q_list.push_back(result2);

    bool execute_result = multi_movel_joint_action(q_list);
    if(!execute_result)
    {
        ROS_ERROR("back_c_plan1 failed!");
        return false;
    }

    if(m_robot_state == s_grasp_up_right)
        m_robot_state = s_back_up_right;
    else if(m_robot_state == s_grasp_up_middle)
        m_robot_state = s_back_up_middle;
    else if(m_robot_state == s_grasp_up_left)
        m_robot_state = s_back_up_left;
    else if(m_robot_state == s_grasp_desk_left)
        m_robot_state = s_back_desk_left;
    else if(m_robot_state == s_grasp_desk_middle)
        m_robot_state = s_back_desk_middle;
    else
        m_robot_state = s_back_desk_right;

    return true;
}

bool back_c_plan2(geometry_msgs::Pose current_pose_m)
{
    current_pose_m.position.z += 0.1;
    std::vector<double> result1,result2,result3,result4;
    bool slover_result = false;
    if(m_robot_state == s_grasp_down_right)
        slover_result = trac_ik_solver(pre_down_right,current_pose_m,result1);
    else if(m_robot_state == s_grasp_down_middle)
        slover_result = trac_ik_solver(pre_down_middle,current_pose_m,result1);
    else if(m_robot_state == s_grasp_down_left)
        slover_result = trac_ik_solver(pre_down_left,current_pose_m,result1);
    else
    {
        ROS_ERROR("robot is not at grasp place");
        return 0;
    }

    if(!slover_result)
    {
        ROS_ERROR("back_c_plan2 failed!");
        return false;
    }

    std::cout<<"result1: ";
    for(int i=0;i<6;i++)
    {
	std::cout<<result1[i]<<"    ";
    }
    std::cout<<std::endl;

    current_pose_m.position.y = y_up;
    slover_result = trac_ik_solver(result1,current_pose_m,result2);

    if(!slover_result)
    {
        ROS_ERROR("back_c_plan2 failed!");
        return false;
    }

    std::cout<<"result2: ";
    for(int i=0;i<6;i++)
    {
	std::cout<<result2[i]<<"    ";
    }
    std::cout<<std::endl;

    current_pose_m.position.z += 0.07;
    current_pose_m.orientation.x = 0;
    current_pose_m.orientation.y = -0.27624;
    current_pose_m.orientation.z = 0.96109;
    current_pose_m.orientation.w = 0;
    slover_result = trac_ik_solver(result2,current_pose_m,result3);

    if(!slover_result)
    {
        ROS_ERROR("back_c_plan2 failed!");
        return false;
    }

    std::cout<<"result3: ";
    for(int i=0;i<6;i++)
    {
	std::cout<<result3[i]<<"    ";
    }
    std::cout<<std::endl;

    current_pose_m.position.y += 0.16;
    slover_result = trac_ik_solver(result3,current_pose_m,result4);

    if(!slover_result)
    {
        ROS_ERROR("back_c_plan2 failed!");
        return false;
    }

    std::cout<<"result4: ";
    for(int i=0;i<6;i++)
    {
	std::cout<<result4[i]<<"    ";
    }
    std::cout<<std::endl;

    std::vector<std::vector<double> > q_list;
    q_list.push_back(result1);
    q_list.push_back(result2);
    q_list.push_back(result3);
    q_list.push_back(result4);

    bool execute_result = multi_movel_joint_action(q_list);
    if(!execute_result)
    {
        ROS_ERROR("back_c_plan2 failed!");
        return false;
    }

    if(m_robot_state == s_grasp_down_right)
        m_robot_state = s_back_down_right;
    else if(m_robot_state == s_grasp_down_middle)
        m_robot_state = s_back_down_middle;
    else
        m_robot_state = s_back_down_left;

    return true;
}

bool is_obj_pose_valid(geometry_msgs::Pose pose_)
{
    if(((m_robot_state == s_up_left_observe) || (m_robot_state == s_up_right_observe) || (m_robot_state == s_up_middle_observe)) && (m_agv_state == huojia))
    {
        if(pose_.position.z>up_level_z_up || pose_.position.z<up_level_z_down)
        {
            ROS_ERROR("obj's pose Z is %f ,error!",pose_.position.z);
            return false;
        }
        if(pose_.position.x>x_up || pose_.position.x<x_down)
        {
            ROS_ERROR("obj's pose X is %f ,error!",pose_.position.x);
            return false;
        }
        if(pose_.position.y>y_up /*|| pose_.position.y<y_down*/)
        {
            ROS_ERROR("obj's pose Y is %f ,error!",pose_.position.y);
            return false;
        }

        return true;
    }
    else if(((m_robot_state == s_down_left_observe) || (m_robot_state == s_down_right_observe) || (m_robot_state == s_down_middle_observe)) && (m_agv_state == huojia))
    {
        if(pose_.position.z>down_level_z_up || pose_.position.z<down_level_z_down)
        {
            ROS_ERROR("obj's pose Z is %f ,error!",pose_.position.z);
            return false;
        }
        if(pose_.position.x>x_up || pose_.position.x<x_down)
        {
            ROS_ERROR("obj's pose X is %f ,error!",pose_.position.x);
            return false;
        }
        if(pose_.position.y>y_up /*|| pose_.position.y<y_down*/)
        {
            ROS_ERROR("obj's pose Y is %f ,error!",pose_.position.y);
            return false;
        }

        return true;
    }
    else if(((m_robot_state == s_observe_desk_left) || (m_robot_state == s_observe_desk_right) || (m_robot_state == s_observe_desk_middle)) && (m_agv_state == desk))
    {
        if(pose_.position.z>desk_z_up || pose_.position.z<desk_z_down)
        {
            ROS_ERROR("obj's pose Z is %f ,error!",pose_.position.z);
            return false;
        }
        if(pose_.position.x>desk_x_up || pose_.position.x<desk_x_down)
        {
            ROS_ERROR("obj's pose X is %f ,error!",pose_.position.x);
            return false;
        }
        if(pose_.position.y>desk_y_up /*|| pose_.position.y<desk_y_down*/)
        {
            ROS_ERROR("obj's pose Y is %f ,error!",pose_.position.y);
            return false;
        }

        return true;
    }
    else
    {
	std::cout<<m_robot_state<<std::endl;
	std::cout<<m_agv_state<<std::endl;
        ROS_ERROR("11111");
        return false;
    }
}


void adjust_object_position(geometry_msgs::Pose &grasp_pose_,int id)
{
    if(id == 1)
    {
        if(m_robot_state == s_up_left_observe || m_robot_state == s_up_right_observe || m_robot_state == s_up_middle_observe)
        {
            grasp_pose_.position.z = 0.41;
        }
        else if(m_robot_state == s_down_left_observe || m_robot_state == s_down_right_observe || m_robot_state == s_down_middle_observe)
        {
            grasp_pose_.position.z = -0.186;
        }
        else if(m_robot_state == s_observe_desk_left || m_robot_state == s_observe_desk_right || m_robot_state == s_observe_desk_middle)
        {
		grasp_pose_.position.z = 0.102;
        }
    }
    else if(id == 2)
    {
        if(m_robot_state == s_up_left_observe || m_robot_state == s_up_right_observe || m_robot_state == s_up_middle_observe)
        {
		grasp_pose_.position.z = 0.343;
        }
        else if(m_robot_state == s_down_left_observe || m_robot_state == s_down_right_observe || m_robot_state == s_down_middle_observe)
        {
		grasp_pose_.position.z = -0.253;
        }
        else if(m_robot_state == s_observe_desk_left || m_robot_state == s_observe_desk_right || m_robot_state == s_observe_desk_middle)
        {
		grasp_pose_.position.z = 0.0478;
        }
    }
    else if(id == 3)
    {
        if(m_robot_state == s_up_left_observe || m_robot_state == s_up_right_observe || m_robot_state == s_up_middle_observe)
        {
		grasp_pose_.position.z = 0.44;
        }
        else if(m_robot_state == s_down_left_observe || m_robot_state == s_down_right_observe || m_robot_state == s_down_middle_observe)
        {
		grasp_pose_.position.z = -0.17;
        }
        else if(m_robot_state == s_observe_desk_left || m_robot_state == s_observe_desk_right || m_robot_state == s_observe_desk_middle)
        {
		grasp_pose_.position.z = 0.1337;
        }
    }
    else if(id == 4)
    {
        if(m_robot_state == s_up_left_observe || m_robot_state == s_up_right_observe || m_robot_state == s_up_middle_observe)
        {
            grasp_pose_.position.z = 0.383;
        }
        else if(m_robot_state == s_down_left_observe || m_robot_state == s_down_right_observe || m_robot_state == s_down_middle_observe)
        {
            grasp_pose_.position.z = -0.219;
        }
        else if(m_robot_state == s_observe_desk_left || m_robot_state == s_observe_desk_right || m_robot_state == s_observe_desk_middle)
        {
		grasp_pose_.position.z = 0.073;
        }
    }
    else if(id == 5)
    {
        if(m_robot_state == s_up_left_observe || m_robot_state == s_up_right_observe || m_robot_state == s_up_middle_observe)
        {
            grasp_pose_.position.z = 0.3584;
        }
        else if(m_robot_state == s_down_left_observe || m_robot_state == s_down_right_observe || m_robot_state == s_down_middle_observe)
        {
            grasp_pose_.position.z = -0.246;
        }
        else if(m_robot_state == s_observe_desk_left || m_robot_state == s_observe_desk_right || m_robot_state == s_observe_desk_middle)
        {
		grasp_pose_.position.z = 0.0645;
        }
    }
    else if(id == 6)
    {
        if(m_robot_state == s_up_left_observe || m_robot_state == s_up_right_observe || m_robot_state == s_up_middle_observe)
        {
		grasp_pose_.position.z = 0.326;
        }
        else if(m_robot_state == s_down_left_observe || m_robot_state == s_down_right_observe || m_robot_state == s_down_middle_observe)
        {
		grasp_pose_.position.z = -0.274;
		//grasp_pose_.position.x -= 0.03;
        }
        else if(m_robot_state == s_observe_desk_left || m_robot_state == s_observe_desk_right || m_robot_state == s_observe_desk_middle)
        {
		grasp_pose_.position.z = 0.0252;
        }
    }
    else if(id == 7)
    {
        if(m_robot_state == s_up_left_observe || m_robot_state == s_up_right_observe || m_robot_state == s_up_middle_observe)
        {
		grasp_pose_.position.z = 0.326;
        }
        else if(m_robot_state == s_down_left_observe || m_robot_state == s_down_right_observe || m_robot_state == s_down_middle_observe)
        {
		grasp_pose_.position.z = -0.274;
        }
        else if(m_robot_state == s_observe_desk_left || m_robot_state == s_observe_desk_right || m_robot_state == s_observe_desk_middle)
        {
		grasp_pose_.position.z = 0.0252;
        }
    }
    else if(id == 8)
    {
        if(m_robot_state == s_up_left_observe || m_robot_state == s_up_right_observe || m_robot_state == s_up_middle_observe)
        {
		grasp_pose_.position.z = 0.352;
		//grasp_pose_.position.x -= 0.03;
        }
        else if(m_robot_state == s_down_left_observe || m_robot_state == s_down_right_observe || m_robot_state == s_down_middle_observe)
        {
		grasp_pose_.position.z = -0.237;
        }
        else if(m_robot_state == s_observe_desk_left || m_robot_state == s_observe_desk_right || m_robot_state == s_observe_desk_middle)
        {
		grasp_pose_.position.z = 0.054;
        }
    }
    else if(id == 9)
    {
        if(m_robot_state == s_up_left_observe || m_robot_state == s_up_right_observe || m_robot_state == s_up_middle_observe)
        {
		grasp_pose_.position.z = 0.38;
        }
        else if(m_robot_state == s_down_left_observe || m_robot_state == s_down_right_observe || m_robot_state == s_down_middle_observe)
        {
		grasp_pose_.position.z = -0.216;
        }
        else if(m_robot_state == s_observe_desk_left || m_robot_state == s_observe_desk_right || m_robot_state == s_observe_desk_middle)
        {
		grasp_pose_.position.z = 0.089;
        }
    }
    else if(id == 10)
    {
        if(m_robot_state == s_up_left_observe || m_robot_state == s_up_right_observe || m_robot_state == s_up_middle_observe)
        {
		grasp_pose_.position.z = 0.417;
        }
        else if(m_robot_state == s_down_left_observe || m_robot_state == s_down_right_observe || m_robot_state == s_down_middle_observe)
        {
		grasp_pose_.position.z = -0.185;
        }
        else if(m_robot_state == s_observe_desk_left || m_robot_state == s_observe_desk_right || m_robot_state == s_observe_desk_middle)
        {
		grasp_pose_.position.z = 0.116;
        }
    }
    else if(id == 11)
    {
        if(m_robot_state == s_up_left_observe || m_robot_state == s_up_right_observe || m_robot_state == s_up_middle_observe)
        {
		grasp_pose_.position.z = 0.345;
        }
        else if(m_robot_state == s_down_left_observe || m_robot_state == s_down_right_observe || m_robot_state == s_down_middle_observe)
        {
		grasp_pose_.position.z = -0.2545;
        }
        else if(m_robot_state == s_observe_desk_left || m_robot_state == s_observe_desk_right || m_robot_state == s_observe_desk_middle)
        {
		grasp_pose_.position.z = 0.052;
        }
    }
    else if(id == 12)
    {
        if(m_robot_state == s_up_left_observe || m_robot_state == s_up_right_observe || m_robot_state == s_up_middle_observe)
        {
		grasp_pose_.position.z = 0.38;
        }
        else if(m_robot_state == s_down_left_observe || m_robot_state == s_down_right_observe || m_robot_state == s_down_middle_observe)
        {
		grasp_pose_.position.z = -0.2184;
        }
        else if(m_robot_state == s_observe_desk_left || m_robot_state == s_observe_desk_right || m_robot_state == s_observe_desk_middle)
        {
		grasp_pose_.position.z = 0.075;
        }
    }
    else if(id == 13)
    {
        if(m_robot_state == s_up_left_observe || m_robot_state == s_up_right_observe || m_robot_state == s_up_middle_observe)
        {
		grasp_pose_.position.z = 0.345;
        }
        else if(m_robot_state == s_down_left_observe || m_robot_state == s_down_right_observe || m_robot_state == s_down_middle_observe)
        {
		grasp_pose_.position.z = -0.242;
        }
        else if(m_robot_state == s_observe_desk_left || m_robot_state == s_observe_desk_right || m_robot_state == s_observe_desk_middle)
        {
		grasp_pose_.position.z = 0.0505;
        }
    }
    else if(id == 14)
    {
        if(m_robot_state == s_up_left_observe || m_robot_state == s_up_right_observe || m_robot_state == s_up_middle_observe)
        {
		grasp_pose_.position.z = 0.345;
        }
        else if(m_robot_state == s_down_left_observe || m_robot_state == s_down_right_observe || m_robot_state == s_down_middle_observe)
        {
		grasp_pose_.position.z = -0.242;
        }
        else if(m_robot_state == s_observe_desk_left || m_robot_state == s_observe_desk_right || m_robot_state == s_observe_desk_middle)
        {
		grasp_pose_.position.z = 0.0505;
        }
    }
    else if(id == 15)
    {
        if(m_robot_state == s_up_left_observe || m_robot_state == s_up_right_observe || m_robot_state == s_up_middle_observe)
        {
		grasp_pose_.position.z = 0.357;
        }
        else if(m_robot_state == s_down_left_observe || m_robot_state == s_down_right_observe || m_robot_state == s_down_middle_observe)
        {
		grasp_pose_.position.z = -0.239;
        }
        else if(m_robot_state == s_observe_desk_left || m_robot_state == s_observe_desk_right || m_robot_state == s_observe_desk_middle)
        {
		grasp_pose_.position.z = 0.048;
        }
    }
    else if(id == 16)
    {
        if(m_robot_state == s_up_left_observe || m_robot_state == s_up_right_observe || m_robot_state == s_up_middle_observe)
        {
		grasp_pose_.position.z = 0.354;
        }
        else if(m_robot_state == s_down_left_observe || m_robot_state == s_down_right_observe || m_robot_state == s_down_middle_observe)
        {
		grasp_pose_.position.z = -0.25;
        }
        else if(m_robot_state == s_observe_desk_left || m_robot_state == s_observe_desk_right || m_robot_state == s_observe_desk_middle)
        {
		grasp_pose_.position.z = 0.05;
        }
    }
}
