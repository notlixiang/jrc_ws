#include "ur_pick_srv.h"

bool grasp(msg_package::grasp::Request  &req,
         msg_package::grasp::Response &res)
{
    if(req.agv_position == 1)
    { 
        if(m_agv_state == desk)
        {
            ROS_INFO("remove desk");
            remove_desk();
            sleep(1);
        }

        add_huojia();
        m_agv_state = huojia;
    }
    else if(req.agv_position == 2)
    {
        if(m_agv_state == huojia)
        {
            ROS_INFO("remove huojia");
            remove_huojia();
            sleep(1);
        }

        add_desk();
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
//        grasp_pose.orientation.x = q_grasp_pose.x();
//        grasp_pose.orientation.y = q_grasp_pose.y();
//        grasp_pose.orientation.z = q_grasp_pose.z();
//        grasp_pose.orientation.w = q_grasp_pose.w();
        grasp_pose.orientation.x = 0;
        grasp_pose.orientation.y = 0;
        grasp_pose.orientation.z = 1;
        grasp_pose.orientation.w = 0;

        //adjust object position
		if(req.id == 1)
		{
                    if(m_robot_state == s_up_left_observe || m_robot_state == s_up_right_observe || m_robot_state == s_up_middle_observe)
		    {
			if(grasp_pose.position.z>0)
			{
				grasp_pose.position.z = 0.364;
			}
			else
			{
				ROS_ERROR("object pose error");
            			res.result.data = false;
            			return true;
			}
		    }
	            else if(m_robot_state == s_down_left_observe || m_robot_state == s_down_right_observe || m_robot_state == s_down_middle_observe)
		    {
			if(grasp_pose.position.z<0)
			{
				grasp_pose.position.z = -0.23;
			}
			else
			{
				ROS_ERROR("object pose error");
            			res.result.data = false;
            			return true;
			}
	            }
		    else if(m_robot_state == s_observe_desk_left || m_robot_state == s_observe_desk_right || m_robot_state == s_observe_desk_middle)
		    {
			grasp_pose.position.z = grasp_pose.position.z;
		    }
		}
		else if(req.id ==2)
		{
		}
		else if(req.id ==3)
		{
		}
		else if(req.id ==4)
		{
                    if(m_robot_state == s_up_left_observe || m_robot_state == s_up_right_observe || m_robot_state == s_up_middle_observe)
		    {
			if(grasp_pose.position.z>0)
			{
				grasp_pose.position.z = 0.4;
			}
			else
			{
				ROS_ERROR("object pose error");
            			res.result.data = false;
            			return true;
			}
		    }
	            else if(m_robot_state == s_down_left_observe || m_robot_state == s_down_right_observe || m_robot_state == s_down_middle_observe)
		    {
			if(grasp_pose.position.z<0)
			{
				grasp_pose.position.z = -0.2;
			}
			else
			{
				ROS_ERROR("object pose error");
            			res.result.data = false;
            			return true;
			}
	            }
		    else if(m_robot_state == s_observe_desk_left || m_robot_state == s_observe_desk_right || m_robot_state == s_observe_desk_middle)
		    {
			grasp_pose.position.z = grasp_pose.position.z;
		    }
		}
		else if(req.id ==5)
		{
                    if(m_robot_state == s_up_left_observe || m_robot_state == s_up_right_observe || m_robot_state == s_up_middle_observe)
		    {
			if(grasp_pose.position.z>0)
			{
				grasp_pose.position.z = 0.364;
			}
			else
			{
				ROS_ERROR("object pose error");
            			res.result.data = false;
            			return true;
			}
		    }
	            else if(m_robot_state == s_down_left_observe || m_robot_state == s_down_right_observe || m_robot_state == s_down_middle_observe)
		    {
			if(grasp_pose.position.z<0)
			{
				grasp_pose.position.z = -0.23;
			}
			else
			{
				ROS_ERROR("object pose error");
            			res.result.data = false;
            			return true;
			}
	            }
		    else if(m_robot_state == s_observe_desk_left || m_robot_state == s_observe_desk_right || m_robot_state == s_observe_desk_middle)
		    {
			grasp_pose.position.z = grasp_pose.position.z;
		    }
		}
		else if(req.id ==6)
		{
		}
		else if(req.id ==7)
		{
		}
		else
		{
			ROS_ERROR("an unknown object");
            res.result.data = false;
            return true;
		}


        //grasp_pose = req.pose;

        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(grasp_pose.position.x, grasp_pose.position.y, grasp_pose.position.z) );
        tf::Quaternion q2(grasp_pose.orientation.x,grasp_pose.orientation.y,grasp_pose.orientation.z,grasp_pose.orientation.w);
        transform.setRotation(q2);
        for(int i=0;i<100;i++)
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base", "object_grasp"));
        //------------------------------------------------------

        std::string PLANNING_GROUP = "arm";
        std::cout<<grasp_pose.position.x<<"     "<<grasp_pose.position.y<<"    "<<grasp_pose.position.z<<std::endl;

        if(add_colllsion_)
            add_collision_object(grasp_pose);

        if((req.id == 1) || (req.id == 2) || (req.id == 5) )
        {
            geometry_msgs::Pose pose_copy = grasp_pose;
            grasp_pose.position.y = y_up;
            if(is_obj_pose_valid(pose_copy))
            {
                bool execute_result;
                ROS_INFO("move to grasp pose");
                execute_result = pose_to_joint_plan(PLANNING_GROUP,"base",grasp_pose);
                if(!execute_result)
                {
                    res.result.data = false;
                    if(add_colllsion_)
                        remove_collision_object();
                    return true;
                }

                std::vector<geometry_msgs::Pose> waypoints;
                waypoints.push_back(grasp_pose);
                grasp_pose.position.y = pose_copy.position.y;
                waypoints.push_back(grasp_pose);

                execute_result = C_plan(PLANNING_GROUP,"base",waypoints);
                if(!execute_result)
                {
                    res.result.data = false;
                    if(add_colllsion_)
                        remove_collision_object();
                    return true;
                }
                else
                {
                    current_pose = grasp_pose;
                    if(m_robot_state == s_up_left_observe || m_robot_state == s_up_right_observe || m_robot_state == s_up_middle_observe
                            || m_robot_state == s_down_left_observe || m_robot_state == s_down_right_observe || m_robot_state == s_down_middle_observe)
                    {
                        if(current_pose.position.z>0)
                            m_robot_state = s_grasp_up;
                        else
                            m_robot_state = s_grasp_down;
                    }
                    else
                    {
                        m_robot_state = s_grasp_desk;
                    }
                    res.result.data = true;
                    return true;
                }
            }
            else
            {
                if(add_colllsion_)
                    remove_collision_object();
                ROS_ERROR("grasp pose is invalid!");
                res.result.data = false;
                return true;
            }
        }
        else if((req.id == 3) || (req.id == 4))
        {
            geometry_msgs::Pose pose_copy = grasp_pose;
            grasp_pose.position.z += cartesian_distance;
            grasp_pose.position.y  = y_up;
            if(is_obj_pose_valid(pose_copy))
            {
                bool execute_result;
                ROS_INFO("move to grasp pose");
                execute_result = pose_to_joint_plan(PLANNING_GROUP,"base",grasp_pose);
                if(!execute_result)
                {
                    if(add_colllsion_)
                        remove_collision_object();
                    res.result.data = false;
                    return true;
                }

                std::vector<geometry_msgs::Pose> waypoints;
                waypoints.push_back(grasp_pose);
                grasp_pose.position.y = pose_copy.position.y;
                waypoints.push_back(grasp_pose);
                grasp_pose.position.z -= cartesian_distance;
                waypoints.push_back(grasp_pose);

                execute_result = C_plan(PLANNING_GROUP,"base",waypoints);
                if(!execute_result)
                {
                    if(add_colllsion_)
                        remove_collision_object();
                    res.result.data = false;
                    return true;
                }
                else
                {
                    current_pose = grasp_pose;
                    if(m_robot_state == s_up_left_observe || m_robot_state == s_up_right_observe || m_robot_state == s_up_middle_observe
                            || m_robot_state == s_down_left_observe || m_robot_state == s_down_right_observe || m_robot_state == s_down_middle_observe)
                    {
                        if(current_pose.position.z>0)
                            m_robot_state = s_grasp_up;
                        else
                            m_robot_state = s_grasp_down;
                    }
                    else
                    {
                        m_robot_state = s_grasp_desk;
                    }
                    res.result.data = true;
                    return true;
                }
            }
            else
            {
                if(add_colllsion_)
                    remove_collision_object();
                ROS_ERROR("grasp pose is invalid!");
                res.result.data = false;
                return true;
            }
        }
        else
        {
            if(add_colllsion_)
                remove_collision_object();
            ROS_ERROR("object id is invalid!");
            res.result.data = false;
            return true;
        }
    }
    else if(req.mode == 2)
    {
        if((m_robot_state == s_grasp_up) || (m_robot_state == s_grasp_down))
        {
            if(add_colllsion_)
                add_gripper_collision_object();
            bool execute_result;
            std::string PLANNING_GROUP = "arm";

            int region = getCurrentRegion(current_pose);

            std::vector<geometry_msgs::Pose> waypoints;
            if(m_robot_state == s_grasp_up)
            {
                waypoints.push_back(current_pose);
                current_pose.position.z += 0.1;
                waypoints.push_back(current_pose);
                current_pose.position.y = y_up;
                current_pose.position.y += 0.1;
                waypoints.push_back(current_pose);
            }
            else
            {
                waypoints.push_back(current_pose);
                current_pose.position.z += 0.1;
                waypoints.push_back(current_pose);
                current_pose.position.y = y_up;
                waypoints.push_back(current_pose);
                current_pose.position.z += 0.1;
                current_pose.orientation.x = 0;
                current_pose.orientation.y = -0.27624;
                current_pose.orientation.z = 0.96109;
                current_pose.orientation.w = 0;
                waypoints.push_back(current_pose);
                current_pose.position.y += 0.2;
                waypoints.push_back(current_pose);
            }

            execute_result = C_plan(PLANNING_GROUP,"base",waypoints);
            if(!execute_result)
            {
                if(add_colllsion_)
                {
                    remove_gripper_collision_object();
                    remove_collision_object();
                }
                res.result.data = false;
                return true;
            }

            execute_result = Joint_plan(PLANNING_GROUP,"base",place,region);
            if(!execute_result)
            {
                res.result.data = false;
                if(add_colllsion_)
                {
                    remove_gripper_collision_object();
                    remove_collision_object();
                }
                return true;
            }
            else
            {
                m_robot_state = s_place;
                res.result.data = true;
                if(add_colllsion_)
                {
                    remove_gripper_collision_object();
                    remove_collision_object();
                }
                return true;
            }
        }
        else if(m_robot_state == s_grasp_desk)
        {
            if(add_colllsion_)
                add_gripper_collision_object();
            bool execute_result;
            std::string PLANNING_GROUP = "arm";

            int region = getCurrentRegion_desk(current_pose);

            std::vector<geometry_msgs::Pose> waypoints;
            waypoints.push_back(current_pose);
            current_pose.position.z += 0.1;
            waypoints.push_back(current_pose);
            current_pose.position.y = y_up;
            current_pose.position.y += 0.1;
            waypoints.push_back(current_pose);

            execute_result = C_plan(PLANNING_GROUP,"base",waypoints);
            if(!execute_result)
            {
                if(add_colllsion_)
                {
                    remove_gripper_collision_object();
                    remove_collision_object();
                }
                res.result.data = false;
                return true;
            }

            execute_result = Joint_plan(PLANNING_GROUP,"base",place,region);
            if(!execute_result)
            {
                res.result.data = false;
                if(add_colllsion_)
                {
                    remove_gripper_collision_object();
                    remove_collision_object();
                }
                return true;
            }
            else
            {
                m_robot_state = s_place;
                res.result.data = true;
                if(add_colllsion_)
                {
                    remove_gripper_collision_object();
                    remove_collision_object();
                }
                return true;
            }
        }
        else
        {
            if(add_colllsion_)
            {
                remove_gripper_collision_object();
                remove_collision_object();
            }
            ROS_ERROR("robot is no at grasp place");
            res.result.data = false;
            return true;
        }
    }
    else if(req.mode == 3)
    {
        bool execute_result;
        execute_result = move_to_home();
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
    else
    {
        res.result.data = false;
        return true;
    }
}
 
int main(int argc, char** argv)
{
  ros::init(argc, argv, "ur_pick_srv");
  ros::NodeHandle node_handle;

  planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  planning_scene_diff_client =
        node_handle.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
  planning_scene_diff_client.waitForExistence();

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

  Eigen::Quaterniond q_ee_to_camera(-0.104917,0.6966957,0.08487588,0.704559);
  r_matrix = q_ee_to_camera.toRotationMatrix();
  for(int i=0;i<3;i++)
      for(int j=0;j<3;j++)
          ee_to_camera(i,j) = r_matrix(i,j);
  ee_to_camera(0,3) = 0.0887066;
  ee_to_camera(1,3) = 0.112934;
  ee_to_camera(2,3) = 0.00874476;
  ee_to_camera(3,0) = 0;
  ee_to_camera(3,1) = 0;
  ee_to_camera(3,2) = 0;
  ee_to_camera(3,3) = 1;

  Eigen::Quaterniond q_up_right_base_to_ee(0.25562,0.68508,-0.22921,-0.64248);
  r_matrix = q_up_right_base_to_ee.toRotationMatrix();
  for(int i=0;i<3;i++)
      for(int j=0;j<3;j++)
          up_right_base_to_ee(i,j) = r_matrix(i,j);
  up_right_base_to_ee(0,3) = -0.48149;
  up_right_base_to_ee(1,3) = -0.62734;
  up_right_base_to_ee(2,3) = 0.73776;
  up_right_base_to_ee(3,0) = 0;
  up_right_base_to_ee(3,1) = 0;
  up_right_base_to_ee(3,2) = 0;
  up_right_base_to_ee(3,3) = 1;


  Eigen::Quaterniond q_down_right_base_to_ee(0.24262,0.7015,-0.20086,-0.63929);
  r_matrix = q_down_right_base_to_ee.toRotationMatrix();
  for(int i=0;i<3;i++)
      for(int j=0;j<3;j++)
          down_right_base_to_ee(i,j) = r_matrix(i,j);
  down_right_base_to_ee(0,3) = -0.51845;
  down_right_base_to_ee(1,3) = -0.63849;
  down_right_base_to_ee(2,3) = 0.21094;
  down_right_base_to_ee(3,0) = 0;
  down_right_base_to_ee(3,1) = 0;
  down_right_base_to_ee(3,2) = 0;
  down_right_base_to_ee(3,3) = 1;

 
  Eigen::Quaterniond q_up_left_base_to_ee(0.24805,0.70014, -0.20203, -0.63833);
  r_matrix = q_up_left_base_to_ee.toRotationMatrix();
  for(int i=0;i<3;i++)
      for(int j=0;j<3;j++)
          up_left_base_to_ee(i,j) = r_matrix(i,j);
  up_left_base_to_ee(0,3) = 0.33619;
  up_left_base_to_ee(1,3) = -0.64342;
  up_left_base_to_ee(2,3) = 0.73286;
  up_left_base_to_ee(3,0) = 0;
  up_left_base_to_ee(3,1) = 0;
  up_left_base_to_ee(3,2) = 0;
  up_left_base_to_ee(3,3) = 1;


  Eigen::Quaterniond q_down_left_base_to_ee(0.23291,0.70454,-0.21623,-0.63453);
  r_matrix = q_down_left_base_to_ee.toRotationMatrix();
  for(int i=0;i<3;i++)
      for(int j=0;j<3;j++)
          down_left_base_to_ee(i,j) = r_matrix(i,j);
  down_left_base_to_ee(0,3) = 0.30212;
  down_left_base_to_ee(1,3) = -0.64518;
  down_left_base_to_ee(2,3) = 0.21125;
  down_left_base_to_ee(3,0) = 0;
  down_left_base_to_ee(3,1) = 0;
  down_left_base_to_ee(3,2) = 0;
  down_left_base_to_ee(3,3) = 1;


  Eigen::Quaterniond q_down_middle_base_to_ee(0.18998,0.70956, -0.16659,-0.65779);
  r_matrix = q_down_middle_base_to_ee.toRotationMatrix();
  for(int i=0;i<3;i++)
      for(int j=0;j<3;j++)
          down_middle_base_to_ee(i,j) = r_matrix(i,j);
  down_middle_base_to_ee(0,3) = -0.10758;
  down_middle_base_to_ee(1,3) = -0.71025;
  down_middle_base_to_ee(2,3) = 0.20461;
  down_middle_base_to_ee(3,0) = 0;
  down_middle_base_to_ee(3,1) = 0;
  down_middle_base_to_ee(3,2) = 0;
  down_middle_base_to_ee(3,3) = 1;

 
  Eigen::Quaterniond q_up_middle_base_to_ee(-0.21257,-0.66936,0.2107,0.67998);
  r_matrix = q_up_middle_base_to_ee.toRotationMatrix();
  for(int i=0;i<3;i++)
      for(int j=0;j<3;j++)
          up_middle_base_to_ee(i,j) = r_matrix(i,j);
  up_middle_base_to_ee(0,3) = -0.054825;
  up_middle_base_to_ee(1,3) = -0.6063;
  up_middle_base_to_ee(2,3) = 0.77967;
  up_middle_base_to_ee(3,0) = 0;
  up_middle_base_to_ee(3,1) = 0;
  up_middle_base_to_ee(3,2) = 0;
  up_middle_base_to_ee(3,3) = 1;

  Eigen::Quaterniond q_desk_left_base_to_ee(0.087992,0.73301,-0.094138,-0.66791);
  r_matrix = q_desk_left_base_to_ee.toRotationMatrix();
  for(int i=0;i<3;i++)
      for(int j=0;j<3;j++)
          desk_left_base_to_ee(i,j) = r_matrix(i,j);
  desk_left_base_to_ee(0,3) = 0.15951;
  desk_left_base_to_ee(1,3) = -0.84786;
  desk_left_base_to_ee(2,3) = 0.58707;
  desk_left_base_to_ee(3,0) = 0;
  desk_left_base_to_ee(3,1) = 0;
  desk_left_base_to_ee(3,2) = 0;
  desk_left_base_to_ee(3,3) = 1;

  Eigen::Quaterniond q_desk_right_base_to_ee(0.10174,0.73089,-0.1003,-0.66738);
  r_matrix = q_desk_right_base_to_ee.toRotationMatrix();
  for(int i=0;i<3;i++)
      for(int j=0;j<3;j++)
          desk_right_base_to_ee(i,j) = r_matrix(i,j);
  desk_right_base_to_ee(0,3) = -0.23952;
  desk_right_base_to_ee(1,3) = -0.78823;
  desk_right_base_to_ee(2,3) = 0.54054;
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


  bool init_result = move_to_home();
  if(init_result)
  {
      remove_desk();
      sleep(1);
      remove_huojia();
      sleep(1);
      remove_collision_object();
      sleep(1);

      ROS_INFO("Ready to move!");
      m_robot_state = s_home;
  }
  else
  {
      ROS_ERROR("failed to init!");
      return 0;
  }

  m_agv_state = unknown;

  ros::ServiceServer service = node_handle.advertiseService("ur_grasp", grasp);

  ros::waitForShutdown();

  return 0;
}
