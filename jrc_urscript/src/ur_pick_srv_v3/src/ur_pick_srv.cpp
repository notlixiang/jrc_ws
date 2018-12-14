#include "ur_pick_srv.h"

bool grasp(msg_package::grasp::Request  &req,
         msg_package::grasp::Response &res)
{
    if(req.agv_position == 1)
    {
        m_agv_state = huojia;
    }
    else if(req.agv_position == 2)
    {
        m_agv_state = desk;
    }
    else
    {
        m_agv_state = unknown;
        ROS_ERROR("no agv position assigned");
        res.result.data = false;
        return true;
    }
    //----------------------------------------------------------

    if(req.mode == 1)
    {
        //------------------------------------------------------
        Eigen::Matrix<double,3,3> r_matrix;
        Eigen::Quaterniond q(req.pose.orientation.w,req.pose.orientation.x,req.pose.orientation.y,req.pose.orientation.z);
        r_matrix = q.toRotationMatrix();
        Eigen::Matrix<double,4,4> depth_to_obj;
        for(int i=0;i<3;i++)
            for(int j=0;j<3;j++)
                depth_to_obj(i,j) = r_matrix(i,j);
        depth_to_obj(0,3) = req.pose.position.x;
        depth_to_obj(1,3) = req.pose.position.y;
        depth_to_obj(2,3) = req.pose.position.z;
        depth_to_obj(3,0) = 0;
        depth_to_obj(3,1) = 0;
        depth_to_obj(3,2) = 0;
        depth_to_obj(3,3) = 1;

        Eigen::Matrix<double,4,4> base_pose;
        if(m_robot_state == s_up_right_observe)
        {
            base_pose = up_right_base_to_ee*ee_to_camera*depth_to_obj;
        }
        else if(m_robot_state == s_down_right_observe)
        {
            base_pose = down_right_base_to_ee*ee_to_camera*depth_to_obj;
        }
        else if(m_robot_state == s_down_middle_observe)
        {
            base_pose = down_middle_base_to_ee*ee_to_camera*depth_to_obj;
        }
        else if(m_robot_state == s_up_middle_observe)
        {
            base_pose = up_middle_base_to_ee*ee_to_camera*depth_to_obj;
        }
        else if(m_robot_state == s_up_left_observe)
        {
            base_pose = up_left_base_to_ee*ee_to_camera*depth_to_obj;
        }
        else if(m_robot_state == s_down_left_observe)
        {
            base_pose = down_left_base_to_ee*ee_to_camera*depth_to_obj;
        }
		else if(m_robot_state == s_observe_desk_left)
		{
			base_pose = desk_left_base_to_ee*ee_to_camera*depth_to_obj;
		}
		else if(m_robot_state == s_observe_desk_right)
		{
			base_pose = desk_right_base_to_ee*ee_to_camera*depth_to_obj;
		}
        else if(m_robot_state == s_observe_desk_middle)
        {
            base_pose = desk_middle_base_to_ee*ee_to_camera*depth_to_obj;
        }
        else
        {
            ROS_ERROR("move to observe place before grasp!");
            res.result.data = false;
            return true;
        }

        for(int i=0;i<3;i++)
            for(int j=0;j<3;j++)
                r_matrix(i,j) = base_pose(i,j);

        Eigen::Quaterniond q_grasp_pose(r_matrix);
        geometry_msgs::Pose grasp_pose;
        grasp_pose.position.x = base_pose(0,3);
        grasp_pose.position.y = base_pose(1,3);
        grasp_pose.position.z = base_pose(2,3);
        grasp_pose.orientation.x = 0;
        grasp_pose.orientation.y = 0;
        grasp_pose.orientation.z = 1;
        grasp_pose.orientation.w = 0;

        if((req.id != 1) && (req.id != 2) && (req.id != 3) && (req.id != 4) && (req.id != 5)
                && (req.id != 6) && (req.id != 7) && (req.id != 8) && (req.id != 9) && (req.id != 10)
		&& (req.id != 11) && (req.id != 12) && (req.id != 13) && (req.id != 14) && (req.id != 15) && (req.id != 16))
        {
            ROS_ERROR("invaild object ID : %d", (int)req.id);
            res.result.data = false;
            return true;
        }

        adjust_object_position(grasp_pose,req.id);

        //grasp_pose = req.pose;

        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(grasp_pose.position.x, grasp_pose.position.y, grasp_pose.position.z) );
        tf::Quaternion q2(grasp_pose.orientation.x,grasp_pose.orientation.y,grasp_pose.orientation.z,grasp_pose.orientation.w);
        transform.setRotation(q2);
        for(int i=0;i<100;i++)
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base", "object_grasp"));
        //------------------------------------------------------

        std::cout<<grasp_pose.position.x<<"     "<<grasp_pose.position.y<<"    "<<grasp_pose.position.z<<std::endl;

        if((req.id == 2) || (req.id == 5) || (req.id == 6) || (req.id == 7) || (req.id == 13) || (req.id == 14))
        {
            geometry_msgs::Pose pose_copy = grasp_pose;
            grasp_pose.position.y = y_up;
            if(is_obj_pose_valid(pose_copy) && is_in_range(pose_copy))
            {
                bool execute_result;
                ROS_INFO("move to grasp pose");
                bool pre_pose_execute_result = pose_to_joint_plan(grasp_pose);
                if(!pre_pose_execute_result)
                {
                    res.result.data = false;
                    return true;
                }

                execute_result = C_plan(pose_copy);
                if(!execute_result)
                {
                    res.result.data = false;
                    return true;
                }
                else
                {
                    current_pose = pose_copy;
                    res.result.data = true;
                    return true;
                }
            }
            else
            {
                ROS_ERROR("object pose is invalid!");
                res.result.data = false;
                return true;
            }
        }
        else if((req.id == 1) || (req.id == 3) || (req.id == 4)  || (req.id == 8) || (req.id == 9) || (req.id == 10) || (req.id == 11) || (req.id == 12) || (req.id == 15) || (req.id == 16))
        {
            geometry_msgs::Pose pose_copy = grasp_pose;
            pose_copy.position.y -= 0.03;
            geometry_msgs::Pose pose_copy1 = pose_copy;
            pose_copy1.position.z += cartesian_distance;

            grasp_pose.position.z += cartesian_distance;
            grasp_pose.position.y  = y_up;
            if(is_obj_pose_valid(pose_copy) && is_in_range(pose_copy) && is_in_range(pose_copy1))
            {
                bool execute_result;
                ROS_INFO("move to grasp pose");
                bool pre_pose_execute_result = pose_to_joint_plan(grasp_pose);
                if(!pre_pose_execute_result)
                {
                    res.result.data = false;
                    return true;
                }

                execute_result = C_plan2(pose_copy);
                if(!execute_result)
                {
                    res.result.data = false;
                    return true;
                }
                else
                {
                    current_pose = pose_copy;
                    res.result.data = true;
                    return true;
                }
            }
            else
            {
                ROS_ERROR("object pose is invalid!");
                res.result.data = false;
                return true;
            }
        }
        else
        {
            ROS_ERROR("object id is invalid!");
            res.result.data = false;
            return true;
        }
    }
    else if(req.mode == 2)
    {
        if((m_robot_state == s_grasp_up_right) || (m_robot_state == s_grasp_up_left) || (m_robot_state == s_grasp_up_middle)
                || (m_robot_state == s_grasp_desk_right) || (m_robot_state == s_grasp_desk_middle) || (m_robot_state == s_grasp_desk_left))
        {
            bool execute_result;
            execute_result = back_c_plan1(current_pose);
            if(!execute_result)
            {
                ROS_ERROR("mode2 failed!");
                res.result.data = false;
                return true;
            }
            else
            {
                res.result.data = true;
                return true;
            }
        }
        else if((m_robot_state == s_grasp_down_right) || (m_robot_state == s_grasp_down_left) || (m_robot_state == s_grasp_down_middle))
        {
            bool execute_result;
            execute_result = back_c_plan2(current_pose);
            if(!execute_result)
            {
                ROS_ERROR("mode2 failed!");
                res.result.data = false;
                return true;
            }
            else
            {
                res.result.data = true;
                return true;
            }
        }
        else
        {
            ROS_ERROR("robot is not at grasp place!");
            res.result.data = false;
            return true;
        }
    }
    else if(req.mode == 3)
    {
        bool execute_result;
        execute_result = move_to_place();
        if(!execute_result)
        {
            res.result.data = false;
            return true;
        }
        else
        {
            res.result.data = true;
            return true;
        }
    }
    else if(req.mode == 4)
    {
        bool execute_result;
        execute_result = move_to_observe_up_right();
        if(!execute_result)
        {
            res.result.data = false;
            return true;
        }
        else
        {
            res.result.data = true;
            return true;
        }
    }
    else if(req.mode == 5)
    {
        bool execute_result;
        execute_result = move_to_observe_down_right();
        if(!execute_result)
        {
            res.result.data = false;
            return true;
        }
        else
        {
            res.result.data = true;
            return true;
        }
    }
    else if(req.mode == 6)
    {
        bool execute_result;
        execute_result = move_to_observe_up_middle();
        if(!execute_result)
        {
            res.result.data = false;
            return true;
        }
        else
        {
            res.result.data = true;
            return true;
        }
    }
    else if(req.mode == 7)
    {
        bool execute_result;
        execute_result = move_to_observe_down_middle();
        if(!execute_result)
        {
            res.result.data = false;
            return true;
        }
        else
        {
            res.result.data = true;
            return true;
        }
    }
    else if(req.mode == 8)
    {
        bool execute_result;
        execute_result = move_to_observe_up_left();
        if(!execute_result)
        {
            res.result.data = false;
            return true;
        }
        else
        {
            res.result.data = true;
            return true;
        }
    }
    else if(req.mode == 9)
    {
        bool execute_result;
        execute_result = move_to_observe_down_left();
        if(!execute_result)
        {
            res.result.data = false;
            return true;
        }
        else
        {
            res.result.data = true;
            return true;
        }
    }
    else if(req.mode == 10)
    {
        bool execute_result;
        execute_result = move_to_observe_desk_right();
        if(!execute_result)
        {
            res.result.data = false;
            return true;
        }
        else
        {
            res.result.data = true;
            return true;
        }
    }
    else if(req.mode == 11)
    {
        bool execute_result;
        execute_result = move_to_observe_desk_middle();
        if(!execute_result)
        {
            res.result.data = false;
            return true;
        }
        else
        {
            res.result.data = true;
            return true;
        }
    }
    else if(req.mode == 12)
    {
        bool execute_result;
        execute_result = move_to_observe_desk_left();
        if(!execute_result)
        {
            res.result.data = false;
            return true;
        }
        else
        {
            res.result.data = true;
            return true;
        }
    }
    else if(req.mode == 13)
    {
        bool execute_result;
        execute_result = push_board();
        if(!execute_result)
        {
            res.result.data = false;
            return true;
        }
        else
        {
            res.result.data = true;
            return true;
        }
    }
    else if(req.mode == 14)
    {
        //------------------------------------------------------
        Eigen::Matrix<double,3,3> r_matrix;
        Eigen::Quaterniond q(req.pose.orientation.w,req.pose.orientation.x,req.pose.orientation.y,req.pose.orientation.z);
        r_matrix = q.toRotationMatrix();
        Eigen::Matrix<double,4,4> depth_to_obj;
        for(int i=0;i<3;i++)
            for(int j=0;j<3;j++)
                depth_to_obj(i,j) = r_matrix(i,j);
        depth_to_obj(0,3) = req.pose.position.x;
        depth_to_obj(1,3) = req.pose.position.y;
        depth_to_obj(2,3) = req.pose.position.z;
        depth_to_obj(3,0) = 0;
        depth_to_obj(3,1) = 0;
        depth_to_obj(3,2) = 0;
        depth_to_obj(3,3) = 1;

        Eigen::Matrix<double,4,4> base_pose;
        if(m_robot_state == s_up_right_observe)
        {
            base_pose = up_right_base_to_ee*ee_to_camera*depth_to_obj;
        }
        else if(m_robot_state == s_down_right_observe)
        {
            base_pose = down_right_base_to_ee*ee_to_camera*depth_to_obj;
        }
        else if(m_robot_state == s_down_middle_observe)
        {
            base_pose = down_middle_base_to_ee*ee_to_camera*depth_to_obj;
        }
        else if(m_robot_state == s_up_middle_observe)
        {
            base_pose = up_middle_base_to_ee*ee_to_camera*depth_to_obj;
        }
        else if(m_robot_state == s_up_left_observe)
        {
            base_pose = up_left_base_to_ee*ee_to_camera*depth_to_obj;
        }
        else if(m_robot_state == s_down_left_observe)
        {
            base_pose = down_left_base_to_ee*ee_to_camera*depth_to_obj;
        }
        else if(m_robot_state == s_observe_desk_left)
        {
            base_pose = desk_left_base_to_ee*ee_to_camera*depth_to_obj;
        }
        else if(m_robot_state == s_observe_desk_right)
        {
            base_pose = desk_right_base_to_ee*ee_to_camera*depth_to_obj;
        }
        else if(m_robot_state == s_observe_desk_middle)
        {
            base_pose = desk_middle_base_to_ee*ee_to_camera*depth_to_obj;
        }
        else
        {
            ROS_ERROR("move to observe place before grasp!");
            res.result.data = false;
            return true;
        }

        geometry_msgs::Pose grasp_pose;
        grasp_pose.position.x = base_pose(0,3);
        grasp_pose.position.y = base_pose(1,3);
        grasp_pose.position.z = base_pose(2,3);
        grasp_pose.orientation.x = 0;
        grasp_pose.orientation.y = 0;
        grasp_pose.orientation.z = 1;
        grasp_pose.orientation.w = 0;

        if((req.id != 1) && (req.id != 2) && (req.id != 3) && (req.id != 4) && (req.id != 5)
                && (req.id != 6) && (req.id != 7) && (req.id != 8) && (req.id != 9) && (req.id != 10)
        && (req.id != 11) && (req.id != 12) && (req.id != 13) && (req.id != 14) && (req.id != 15) && (req.id != 16))
        {
            ROS_ERROR("invaild object ID : %d", (int)req.id);
            res.result.data = false;
            return true;
        }

        adjust_object_position(grasp_pose,req.id);

        //------------------------------------------------------

        if((req.id == 2) || (req.id == 5) || (req.id == 6)|| (req.id == 7) || (req.id == 13) || (req.id == 14))
        {
            geometry_msgs::Pose pose_copy = grasp_pose;
            grasp_pose.position.y = y_up;
            if(is_obj_pose_valid(pose_copy) && is_in_range(pose_copy))
            {
                bool is_in_current_observe = is_in_current_observe_place(pose_copy);
                if(is_in_current_observe)
                {
                    res.result.data = true;
                    return true;
                }
                else
                {
                    ROS_ERROR("object is not at current observe position");
                    res.result.data = false;
                    res.fail_status = 1;
                    return true;
                }
            }
            else
            {
                ROS_ERROR("object pose is invalid!");
                res.result.data = false;
                res.fail_status = 2;
                return true;
            }
        }
        else if((req.id == 1) || (req.id == 3) || (req.id == 4) || (req.id == 7) || (req.id == 8) || (req.id == 9) || (req.id == 10) || (req.id == 11) || (req.id == 12) || (req.id == 15) || (req.id == 16))
        {
            geometry_msgs::Pose pose_copy = grasp_pose;
            pose_copy.position.y -= 0.04;
            geometry_msgs::Pose pose_copy1 = pose_copy;
            pose_copy1.position.z += cartesian_distance;

            if(is_obj_pose_valid(pose_copy) && is_in_range(pose_copy) && is_in_range(pose_copy1))
            {
                bool is_in_current_observe = is_in_current_observe_place(pose_copy);
                if(is_in_current_observe)
                {
                    res.result.data = true;
                    return true;
                }
                else
                {
                    ROS_ERROR("object is not at current observe position");
                    res.result.data = false;
                    res.fail_status = 1;
                    return true;
                }
            }
            else
            {
                ROS_ERROR("object pose is invalid!");
                res.result.data = false;
                res.fail_status = 2;
                return true;
            }
        }
        else
        {
            ROS_ERROR("object id is invalid!");
            res.result.data = false;
            return true;
        }
    }
    else
    {
        ROS_ERROR("invaild mode!");
        res.result.data = false;
        return true;
    }
}
 
int main(int argc, char** argv)
{
  ros::init(argc, argv, "ur_pick_srv");
  ros::NodeHandle node_handle;

  ros::AsyncSpinner spinner(2);
  spinner.start();

  Eigen::Quaterniond q_camera_to_depth(0.5,-0.5,0.5,-0.5);
  Eigen::Matrix<double,3,3> r_matrix;
  r_matrix = q_camera_to_depth.toRotationMatrix();
  for(int i=0;i<3;i++)
      for(int j=0;j<3;j++)
          camera_to_depth(i,j) = r_matrix(i,j);
  camera_to_depth(0,3) = 0;
  camera_to_depth(1,3) = -0.02;
  camera_to_depth(2,3) = 0;
  camera_to_depth(3,0) = 0;
  camera_to_depth(3,1) = 0;
  camera_to_depth(3,2) = 0;
  camera_to_depth(3,3) = 1;

  Eigen::Quaterniond q_ee_to_camera(-0.125596,0.694712,0.137092,0.694843);
  r_matrix = q_ee_to_camera.toRotationMatrix();
  for(int i=0;i<3;i++)
      for(int j=0;j<3;j++)
          ee_to_camera(i,j) = r_matrix(i,j);
  ee_to_camera(0,3) = 0.119447;
  ee_to_camera(1,3) = 0.128357;
  ee_to_camera(2,3) = -0.0257158;
  ee_to_camera(3,0) = 0;
  ee_to_camera(3,1) = 0;
  ee_to_camera(3,2) = 0;
  ee_to_camera(3,3) = 1;


  Eigen::Quaterniond q_up_right_base_to_ee(0.27522,0.68864, -0.22602, -0.63162);
  r_matrix = q_up_right_base_to_ee.toRotationMatrix();
  for(int i=0;i<3;i++)
      for(int j=0;j<3;j++)
          up_right_base_to_ee(i,j) = r_matrix(i,j);
  up_right_base_to_ee(0,3) = -0.53434;
  up_right_base_to_ee(1,3) = -0.40292;
  up_right_base_to_ee(2,3) = 0.74993;
  up_right_base_to_ee(3,0) = 0;
  up_right_base_to_ee(3,1) = 0;
  up_right_base_to_ee(3,2) = 0;
  up_right_base_to_ee(3,3) = 1;

; ; 
; 
  Eigen::Quaterniond q_down_right_base_to_ee(0.2526,0.68607, -0.2151, -0.64749);
  r_matrix = q_down_right_base_to_ee.toRotationMatrix();
  for(int i=0;i<3;i++)
      for(int j=0;j<3;j++)
          down_right_base_to_ee(i,j) = r_matrix(i,j);
  down_right_base_to_ee(0,3) = -0.54641;
  down_right_base_to_ee(1,3) = -0.45071;
  down_right_base_to_ee(2,3) = 0.15452;
  down_right_base_to_ee(3,0) = 0;
  down_right_base_to_ee(3,1) = 0;
  down_right_base_to_ee(3,2) = 0;
  down_right_base_to_ee(3,3) = 1;


  Eigen::Quaterniond q_up_left_base_to_ee(0.30591,0.72261, -0.209, -0.58359);
  r_matrix = q_up_left_base_to_ee.toRotationMatrix();
  for(int i=0;i<3;i++)
      for(int j=0;j<3;j++)
          up_left_base_to_ee(i,j) = r_matrix(i,j);
  up_left_base_to_ee(0,3) = 0.32244;
  up_left_base_to_ee(1,3) = -0.41079;
  up_left_base_to_ee(2,3) = 0.72466;
  up_left_base_to_ee(3,0) = 0;
  up_left_base_to_ee(3,1) = 0;
  up_left_base_to_ee(3,2) = 0;
  up_left_base_to_ee(3,3) = 1;


  Eigen::Quaterniond q_down_left_base_to_ee(0.24646,0.68964, -0.21019, -0.64766);
  r_matrix = q_down_left_base_to_ee.toRotationMatrix();
  for(int i=0;i<3;i++)
      for(int j=0;j<3;j++)
          down_left_base_to_ee(i,j) = r_matrix(i,j);
  down_left_base_to_ee(0,3) = 0.4723;
  down_left_base_to_ee(1,3) = -0.46142;
  down_left_base_to_ee(2,3) = 0.15533;
  down_left_base_to_ee(3,0) = 0;
  down_left_base_to_ee(3,1) = 0;
  down_left_base_to_ee(3,2) = 0;
  down_left_base_to_ee(3,3) = 1;


  Eigen::Quaterniond q_down_middle_base_to_ee(0.24946,0.68763, -0.21301, -0.64773);
  r_matrix = q_down_middle_base_to_ee.toRotationMatrix();
  for(int i=0;i<3;i++)
      for(int j=0;j<3;j++)
          down_middle_base_to_ee(i,j) = r_matrix(i,j);
  down_middle_base_to_ee(0,3) = 0.0027836;
  down_middle_base_to_ee(1,3) = -0.45819;
  down_middle_base_to_ee(2,3) = 0.15436;
  down_middle_base_to_ee(3,0) = 0;
  down_middle_base_to_ee(3,1) = 0;
  down_middle_base_to_ee(3,2) = 0;
  down_middle_base_to_ee(3,3) = 1;

  Eigen::Quaterniond q_up_middle_base_to_ee(0.27705,0.6883, -0.22856, -0.63027);
  r_matrix = q_up_middle_base_to_ee.toRotationMatrix();
  for(int i=0;i<3;i++)
      for(int j=0;j<3;j++)
          up_middle_base_to_ee(i,j) = r_matrix(i,j);
  up_middle_base_to_ee(0,3) = -0.11348;
  up_middle_base_to_ee(1,3) = -0.40601;
  up_middle_base_to_ee(2,3) = 0.74968;
  up_middle_base_to_ee(3,0) = 0;
  up_middle_base_to_ee(3,1) = 0;
  up_middle_base_to_ee(3,2) = 0;
  up_middle_base_to_ee(3,3) = 1;

  Eigen::Quaterniond q_desk_left_base_to_ee(0.17614,0.74678, -0.13376, -0.62722);
  r_matrix = q_desk_left_base_to_ee.toRotationMatrix();
  for(int i=0;i<3;i++)
      for(int j=0;j<3;j++)
          desk_left_base_to_ee(i,j) = r_matrix(i,j);
  desk_left_base_to_ee(0,3) = -0.1017;
  desk_left_base_to_ee(1,3) = -0.52663;
  desk_left_base_to_ee(2,3) = 0.67613;
  desk_left_base_to_ee(3,0) = 0;
  desk_left_base_to_ee(3,1) = 0;
  desk_left_base_to_ee(3,2) = 0;
  desk_left_base_to_ee(3,3) = 1;

  Eigen::Quaterniond q_desk_right_base_to_ee(0.14049,0.71002, -0.10571, -0.68188);
  r_matrix = q_desk_right_base_to_ee.toRotationMatrix();
  for(int i=0;i<3;i++)
      for(int j=0;j<3;j++)
          desk_right_base_to_ee(i,j) = r_matrix(i,j);
  desk_right_base_to_ee(0,3) = -0.27425;
  desk_right_base_to_ee(1,3) = -0.57649;
  desk_right_base_to_ee(2,3) = 0.66095;
  desk_right_base_to_ee(3,0) = 0;
  desk_right_base_to_ee(3,1) = 0;
  desk_right_base_to_ee(3,2) = 0;
  desk_right_base_to_ee(3,3) = 1;


  Eigen::Quaterniond q_desk_middle_base_to_ee(0.10174,0.73089,-0.1003,-0.66738);
  r_matrix = q_desk_middle_base_to_ee.toRotationMatrix();
  for(int i=0;i<3;i++)
      for(int j=0;j<3;j++)
          desk_middle_base_to_ee(i,j) = r_matrix(i,j);
  desk_middle_base_to_ee(0,3) = -0.23952;
  desk_middle_base_to_ee(1,3) = -0.78823;
  desk_middle_base_to_ee(2,3) = 0.54054;
  desk_middle_base_to_ee(3,0) = 0;
  desk_middle_base_to_ee(3,1) = 0;
  desk_middle_base_to_ee(3,2) = 0;
  desk_middle_base_to_ee(3,3) = 1;

  single_joint_client = node_handle.serviceClient<ur_msgs::Single_JointPosition>("ur_driver/single_joint_position");
  multi_joint_client = node_handle.serviceClient<ur_msgs::Multi_JointPosition>("ur_driver/multi_joint_position");
  movel_joint_client = node_handle.serviceClient<ur_msgs::Single_JointPosition>("ur_driver/movel_joint_position");
  multi_movel_joint_client = node_handle.serviceClient<ur_msgs::Multi_JointPosition>("ur_driver/multi_movel_joint_position");
  ros::ServiceServer service = node_handle.advertiseService("ur_grasp", grasp);

  m_robot_state = s_init;
  bool init_result = move_to_place();
  if(init_result)
  {
      ROS_INFO("Ready to move!");
  }
  else
  {
      ROS_ERROR("failed to init!");
      return -1;
  }

  m_agv_state = unknown;
  m_robot_target = t_unknown;




  ros::waitForShutdown();

  return 0;
}
