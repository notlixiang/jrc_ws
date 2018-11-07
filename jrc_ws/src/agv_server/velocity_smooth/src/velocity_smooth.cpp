#include <iostream>
#include <fstream>
#include <iomanip>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <velocity_smooth/smooth_srv.h>
#include <velocity_smooth/smooth_srv_multi.h>

using namespace std;

#define PI 3.1415926

class Jrc_move
{
public:
    Jrc_move():w_max_(0.7),beta_max_(0.2),vx_max_(0.7),ax_max_(0.2),vy_max_(0.7),ay_max_(0.2),initial_x_(-2.071),initial_y_(0.206),initial_angle_(-1.518)
    {   
    	//>>>>>>>>>>>>>>>>>>>>>>>initializing parameter>>>>>>>>>>>>>>>>>>>>>>>>>>>>//
    	T_max_=w_max_/(2.0*beta_max_);
    	Tx_max_=vx_max_/(2.0*ax_max_);
    	Ty_max_=vy_max_/(2.0*ay_max_);
    	first_mark=false;
    	x_mark=false;
    	y_mark=false;
    	theta_first_arrive_mark=false;
    	theta_final_arrive_mark=false;
    	
    	x_first_arrive_mark=false;
    	x_final_arrive_mark=false;
    	
    	y_first_arrive_mark=false;
    	y_final_arrive_mark=false;
    	
    	arrive_mark = false;
        //Topic(cmd_vel) you want to publish
        cmd_vel_pub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        initial_pose_pub = node.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 100);
        //Topic(amcl_pose) you want to subscribe
        amcl_pose_sub = node.subscribe("/amcl_pose", 1, &Jrc_move::callback, this);
        //Server
        service = node.advertiseService("/Jrc_move", &Jrc_move::move_to_target, this);  //Jrc_move_multi
        //initial the goal
        goal_in_map.header.stamp = ros::Time(0);
        goal_in_map.header.frame_id = "map";
        goal_in_base.header.stamp = goal_in_map.header.stamp;
        goal_in_base.header.frame_id = "base_link";

        //publish the initial pose  
        geometry_msgs::PoseWithCovarianceStamped initial_pose;
        initial_pose.header.stamp = ros::Time(0);
        initial_pose.header.frame_id = "map";
        initial_pose.pose.pose.position.x = initial_x_;
        initial_pose.pose.pose.position.y = initial_y_;
       /* double q1,q2,q3,q4,yaw;
        q1=
        q2=
        q3=
        a4=
        yaw=arctan
*/
        initial_pose.pose.pose.orientation = tf::createQuaternionMsgFromYaw(initial_angle_);
        
        double start_time_point = ros::Time::now().toSec();
        time_counter = 0;
        while(time_counter < 2)
        {
            time_counter = ros::Time::now().toSec() - start_time_point;
            ros::spinOnce();
            initial_pose_pub.publish(initial_pose);
            ROS_INFO("the robot initial pose");
        }

//        //rotate left and right to get the accuracy amcl pose
//        start_time_point = ros::Time::now().toSec();
//        time_counter = 0;
//        while(time_counter < 4)
//        {
//            time_counter = ros::Time::now().toSec() - sta+rt_time_point;
//            ros::spinOnce();
//            if(time_counter < 2)
//            {
//                velocity = geometry_msgs::Twist();
//                velocity.linear.x = 0.1;
//                cmd_vel_pub.publish(velocity);
//                ROS_INFO("the robot forward");
//            }
//            else
//            {
//                velocity = geometry_msgs::Twist();
//                velocity.linear.x = -0.1;
//                cmd_vel_pub.publish(velocity);
//                ROS_INFO("the robot backward");
//            }
//        }
        cmd_vel_pub.publish(geometry_msgs::Twist());
        ros::Duration(1).sleep();


    }
    void callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amcl_pose)
    {
        robot_pose = amcl_pose->pose.pose;
        //ROS_INFO("the robot pose");
    }

    bool move_to_target(velocity_smooth::smooth_srv::Request &req,
                           velocity_smooth::smooth_srv::Response &res)
       {   cmd_vel_pub.publish(geometry_msgs::Twist());
       	theta = req.theta - initial_angle_;
       	x=req.x-initial_x_;
       	y=req.y-initial_y_;
       	theta_dir=theta < 0 ? -1 : 1;
    	x_dir=x < 0 ? -1 : 1;
    	y_dir=y < 0 ? -1 : 1;
       	k=(req.y-initial_y_)/(req.x-initial_x_);
      	if (fabs(k)>1)
       		{
       		  k=1/k;
       		  vx_max_=fabs(k)*vy_max_;
       //		  if (fabs(vx_max_)<0.02)
      // 			  vx_max_ = x_dir * 0.02;
       		}
       	else
       	    {
       	      vy_max_=fabs(k)*vx_max_;     
      //        if(fabs(vy_max_)<0.02) 
       //       vy_max_ = y_dir * 0.02;
       	    }
      	 if (fabs(vx_max_)<0.02)
      	       			  vx_max_ = 0.02;
      	 if(fabs(vy_max_)<0.02) 
      	              vy_max_ =  0.02;
      	 
       	Tx_max_=vx_max_/(2.0*ax_max_);
       	Ty_max_=vy_max_/(2.0*ay_max_);
       	//test
       //	theta = 2* PI;
       //   ROS_INFO("remain angle : %f",      theta);
       	T= 
               (sqrt(fabs(theta)/(6.0*beta_max_))) < T_max_ ? (sqrt(fabs(theta)/(6*beta_max_))): T_max_;
       	Tx= 
       	    (sqrt(fabs(x)/(6.0*ax_max_))) < Tx_max_ ? (sqrt(fabs(x)/(6*ax_max_))): Tx_max_;
       	Ty=
       	    (sqrt(fabs(y)/(6.0*ay_max_))) < Ty_max_ ? (sqrt(fabs(y)/(6*ay_max_))): Ty_max_;
           IS_LONG_WAY_theta=
         		(sqrt(fabs(theta)/(6.0*beta_max_))) < T_max_ ? false : true ;
           IS_LONG_WAY_x=
         		(sqrt(fabs(x)/(6.0*ax_max_))) < Tx_max_ ? false : true ;
           IS_LONG_WAY_y=
               (sqrt(fabs(y)/(6.0*ay_max_))) < Ty_max_ ? false : true ;
        //   if (IS_LONG_WAY_theta)
        // 	{
       //	   	T1=(theta-6*beta_max_* T_max_* T_max_)/(2.0 * beta_max_ * T_max_);
     	//	}
      	//	else    T1=0;
           theta_a_ = 3 * beta_max_ * T * T * theta_dir;
           x_a_ = 3 * ax_max_ * Tx * Tx * x_dir;
           y_a_ = 3 * ay_max_ * Ty * Ty * y_dir;
           goal_in_map.pose.position.x = req.x;
           goal_in_map.pose.position.y = req.y;
           geometry_msgs::Quaternion  quat = tf::createQuaternionMsgFromYaw(req.theta);
        //   geometry_msgs::Quaternion  quat = tf::createQuaternionMsgFromYaw(theta + initial_angle_);
           goal_in_map.pose.orientation = quat;
           error_x = error_y = 10;
           res.mark= 0;    //the position is not reached!
       //    while(fabs(error_x) > 0.01 || fabs(error_y) > 0.01 || fabs(error_theta) > 0.1)
            ros::Rate r(100); // 20 hz
            ros::Time begin = ros::Time::now();
            double time;
            data_x_y_theta.open("data_x_y_theta.txt",ofstream::app);
            data_Vx_Vy_w.open("data_Vx_Vy_w.txt",ofstream::app);
            data_vx_vy_w.open("data_vx_vy_w.txt",ofstream::app);
            while (ros::ok())     	  
           {
             ros::spinOnce();
           	 ros::Duration curr=ros::Time::now()-begin;
   			 time=curr.toSec();         	 
           	 //gain the current error of x y theta;
           	            try{
           	               listener.waitForTransform("/base_link", "/map", ros::Time(0), ros::Duration(100.0) );
           	               listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
           	                }
           	            catch (tf::TransformException ex){
           	               ROS_ERROR("%s",ex.what());
           	               ros::Duration(1.0).sleep();
           	               }          
           	            geometry_msgs::Quaternion quat0;
           	                quat0.x =  transform.getRotation().getX();
           	                quat0.y =  transform.getRotation().getY();
           	                quat0.z =  transform.getRotation().getZ();
           	                quat0.w =  transform.getRotation().getW();    	                
           	            current_theta_ = tf::getYaw(quat0);
           	            current_x_=transform.getOrigin().x();
                   	    current_y_=transform.getOrigin().y();
                   	    
                   	    data_x_y_theta << setprecision(10) << time << "   " << setprecision(5) << current_x_ << "   " << setprecision(5) << current_y_ <<"   " << 
                   	    	          setprecision(5) << current_theta_ << "\n" << endl;
                   	    
           	        //    ROS_INFO("current_theta :  current_x : current_y %f %f %f",      current_theta_,current_x_,current_y_);
           	            
           	            listener.transformPose("/base_link",ros::Time(0), goal_in_map, "/map", goal_in_base);
           	          
           	            error_x = goal_in_base.pose.position.x;
           	            error_y = goal_in_base.pose.position.y;
           	            error_theta = tf::getYaw(goal_in_base.pose.orientation);
           	         ROS_INFO("error_theta : . req.theta-current_theta_: %f %f" ,     error_theta, req.theta-current_theta_);
           	 //accelaration for rotation
           	            if(time < T)
           	           	 {
           	           	    w=(beta_max_/(2.0*T)) *(time * time) * theta_dir;
           	           		         
           	           	 }
           	           	else if(time<(2.0*T))
           	           	 {
           	           	    w=beta_max_*(time-T/2.0) * theta_dir;
           	           	 }
           	           	 else if(time<(3.0*T))
           	           	 {				  
           	           	    w=beta_max_*(-(time *time)/(2.0*T) + 3.0*time - 2.5*T) * theta_dir;
           	           	 }
           	           	 else
           	           	 {
           	           	  if (IS_LONG_WAY_theta)
           	           	  {
           	                if((fabs(error_theta-1.0*theta_a_)>0.02) && (!first_mark))  // error limit need change!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1
           	           	    {       	           	           	       
           	                	w = w_max_ * theta_dir;        	           	                      	
           	           	    }
           	                else 
           	                {   
           	                	first_mark=true;
           	                	if(t1_vector_.size() == 0)
           	                    {
           	                	   t1_vector_.push_back(error_theta);
           	                 	  // t1= time;  
           	                    }	
           	            
   							   
   //        	                	if (time < (t1+T))
   //        	                	{  
   //        	                		w=beta_max_*(-((t1+3*T-time) *(t1+3*T-time))/(2.0*T) + 3.0*(t1+3*T-time) - 2.5*T);  	                		
   //        	                	}
   //        	                	else if (time < (t1+2*T))
   //        	                	{
   //        	                		w=beta_max_*(t1+3*T-time-T/2.0);
   //        	                	}
   //        	                	else if (time < (t1+3*T))
   //        	                	{
   //        	                		 w=(beta_max_/(2.0*T)) *((t1+3*T-time)  * (t1+3*T-time) );
   //        	                	}
   //        	                	else w=0;
           	                	
           	                	if (!theta_final_arrive_mark)
           	                    {
           	                		if(fabs(error_theta)<fabs(t1_vector_[0]))
           	                        w = (theta_dir * w_max_ / t1_vector_[0]) * (error_theta );
           	                		else w= 0.1 * fabs(error_theta) / (error_theta);
           	                        if(fabs(error_theta)<0.02)  theta_first_arrive_mark=true;           	                        	                 	                	  
           	                    }
           	                    else w = 0;
           	                }
           	           	  }
           	           	  else
           	              {   
           	           		  if (error_theta_vector_.size() == 0)
           	                  {
           	            	    error_theta_vector_.push_back(error_theta);
           	            	    error_theta_mark= error_theta;
           	                  }
           	           		  if(!theta_final_arrive_mark)
           	           		  {  if (fabs(error_theta)<fabs(error_theta_mark))    	 
           	           			  w =  theta_dir * (2.0 * beta_max_ * T)*(error_theta / error_theta_mark);
           	           		     else w =  (2.0 * beta_max_ * T) * fabs(error_theta) / (error_theta);
           	           	         if(fabs(error_theta)<0.02)  theta_first_arrive_mark=true;
           	           		  }  
           	           		  else w = 0; 
           	        
           	           	  }
   								 
           	           	 }
           	            // accelaration for x_movement
           	                                if(time < Tx)
           	                 	           	 {
           	                 	           	    vx_map_=(ax_max_/(2.0*Tx)) *(time * time) * x_dir;
           	                 	           		         
           	                 	           	 }
           	                 	           	else if(time<(2.0*Tx))
           	                 	           	 {
           	                 	           	    vx_map_=ax_max_*(time-Tx/2.0) * x_dir; 
           	                 	           	 }
           	                 	           	 else if(time<(3.0*Tx))
           	                 	           	 {				  
           	                 	           	    vx_map_=ax_max_*(-(time *time)/(2.0*Tx) + 3.0*time - 2.5*Tx) * x_dir;
           	                 	           	 }
           	                 	           	 else
           	                 	           	 {
           	                 	           	  if(IS_LONG_WAY_x)
           	                 	           	  {
           	                 	                if((fabs(req.x - current_x_ -x_a_)>0.02) && (!x_mark))  // error limit need change!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1
           	                 	           	    {       	 double test;
           	                 	           	    test=req.x-current_x_-x_a_;
           	                 	         //      ROS_INFO("test %f ",   	      test);
           	                 	                	vx_map_= vx_max_ * x_dir;        	           	                      	
           	                 	           	    }
           	                 	                else 
           	                 	                {   
           	                 	                	x_mark=true;
           	                 	                	if(t2_vector_.size() == 0)
           	                 	                    {
           	                 	                	   t2_vector_.push_back(req.x-current_x_);
           	                 	                 	   //t2= time;  
           	                 	                    }	
           	                 	                    
   //        	                 	                	if (time < (t2+Tx))
   //        	                 	                	{  
   //        	                 	                		vx_map_=ax_max_*(-((t2 + 3*Tx-time) *(t2+3*Tx-time))/(2.0*Tx) + 3.0*(t2+3*Tx-time) - 2.5*Tx);  	                		
   //        	                 	                	}
   //        	                 	                	else if (time < (t2+2*Tx))
   //        	                 	                	{
   //        	                 	                		vx_map_=ax_max_*(t2 + 3*Tx-time-Tx/2.0);
   //        	                 	                	}
   //        	                 	                	else if (time < (t2+3*Tx))
   //        	                 	                	{
   //        	                 	                		vx_map_=(ax_max_/(2.0*Tx)) *((t2+3*Tx-time)  * (t2+3*Tx-time) );
   //        	                 	                	}
   //        	                 	                	else vx_map_=0;
           	                 	                	
           	                 	                	if (!x_final_arrive_mark)
           	                 	                	{
           	                 	                	  if (fabs(req.x - current_x_)<fabs(t2_vector_[0]))
           	                 	                     	vx_map_ = (x_dir * vx_max_ / t2_vector_[0]) * (req.x - current_x_);
           	                 	                	  else vx_map_= vx_max_ * fabs(req.x - current_x_) / (req.x - current_x_);
           	                 	                	  if(fabs(req.x - current_x_)<0.02)  x_first_arrive_mark=true;           	                 	                	  
           	                 	                	}
           	                 	                	else vx_map_= 0;
           	                 	                }
           	                 	           	   }
           	                 	           	   else
           	                 	           	   {
           	                 	           		 if (error_x_vector_.size() == 0)
           	                 	           		 {
           	                 	           		   error_x_vector_.push_back(error_x);
           	                 	           		   error_x_mark= req.x - current_x_;
           	                 	           		 }
           	                 	           		 if(!x_final_arrive_mark)
           	                 	           		 {
           	                 	           		   if (fabs(req.x - current_x_)<fabs(error_x_mark))    	                 	                	                  	                 	           		 
           	               	           			     vx_map_ = x_dir * (2.0 * ax_max_ * Tx) * (req.x - current_x_) / error_x_mark;
           	               	           			   else  vx_map_ = (2.0 * ax_max_ * Tx) * fabs(req.x - current_x_) / (req.x - current_x_);
           	               	           			   if(fabs(req.x - current_x_)<0.02)  x_first_arrive_mark=true;
           	                 	           		 }  
           	                 	           		 else vx_map_= 0;
           	                 	           	   }
           	         								 
           	                 	           	 }
           	                                                                        if(time < Ty)
           	                                      	                 	           	 {
           	                                      	                 	           	    vy_map_=(ay_max_/(2.0*Ty)) *(time * time) * y_dir;
           	                                      	                 	           		         
           	                                      	                 	           	 }
           	                                      	                 	           	else if(time<(2.0*Ty))
           	                                      	                 	           	 {
           	                                      	                 	           	    vy_map_=ay_max_*(time-Ty/2.0) * y_dir;
           	                                      	                 	           	 }
           	                                      	                 	           	 else if(time<(3.0*Ty))
           	                                      	                 	           	 {				  
           	                                      	                 	           	    vy_map_=ay_max_*(-(time *time)/(2.0*Ty) + 3.0*time - 2.5*Ty) * y_dir;
           	                                      	                 	           	 }
           	                                      	                 	           	 else
           	                                      	                 	           	 { 
           	                                      	                 	           	  if(IS_LONG_WAY_y)
           	                                      	                 	           	  {
           	                                      	                 	                if((fabs(req.y - current_y_ -y_a_)>0.02) && (!y_mark))  // error limit need change!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1
           	                                      	                 	           	    {       	 
           	                                      	                 	                	double testy;
           	                                      	                 	           	        testy=req.y-current_y_-y_a_;
           	                                      	                 	             //       ROS_INFO("testy %f ",  testy);
           	                                      	                 	                	    
           	                                      	                 	                	vy_map_= vy_max_ * y_dir;        	           	                      	
           	                                      	                 	           	    }
           	                                      	                 	                else 
           	                                      	                 	                {   
           	                                      	                 	                	y_mark=true;
           	                                      	                 	                	if(t3_vector_.size() == 0)
           	                                      	                 	                    {
           	                                      	                 	                	   t3_vector_.push_back(req.y-current_y_);
           	                                      	                 	                 	 //  t3= time;  
           	                                      	                 	                    }	
           	                                      	                 	                    
   //        	                                      	                 	                	if (time < (t2+Tx))
   //        	                                      	                 	                	{  
   //        	                                      	                 	                		vx_map_=ax_max_*(-((t2 + 3*Tx-time) *(t2+3*Tx-time))/(2.0*Tx) + 3.0*(t2+3*Tx-time) - 2.5*Tx);  	                		
   //        	                                      	                 	                	}
   //        	                                      	                 	                	else if (time < (t2+2*Tx))
   //        	                                      	                 	                	{
   //        	                                      	                 	                		vx_map_=ax_max_*(t2 + 3*Tx-time-Tx/2.0);
   //        	                                      	                 	                	}
   //        	                                      	                 	                	else if (time < (t2+3*Tx))
   //        	                                      	                 	                	{
   //        	                                      	                 	                		vx_map_=(ax_max_/(2.0*Tx)) *((t2+3*Tx-time)  * (t2+3*Tx-time) );
   //        	                                      	                 	                	}
   //        	                                      	                 	                	else vx_map_=0;
           	                                      	                 	                	
           	                                      	                 	                	if (!y_final_arrive_mark)
           	                                      	                 	                	{
           	                                      	                 	                		if (fabs(req.y - current_y_)<fabs(t3_vector_[0]))
           	                                      	                 	                	         vy_map_ = y_dir * (vy_max_ / t3_vector_[0]) * (req.y - current_y_ );
           	                                      	                 	                		else vy_map_ = vy_max_ * fabs(req.y - current_y_) / (req.y - current_y_);
           	                                      	                 	                	  if(fabs(req.y - current_y_)<0.02)  y_first_arrive_mark=true;
           	                                      	                 	                	  
           	                                      	                 	                	}
           	                                      	                 	                	else vy_map_= 0;
           	                                      	                 	                }
           	                                      	                 	           	  }
           	                                      	                 	              else
           	                                      	                 	              {
           	                                      	                 	            	if (error_y_vector_.size() == 0)
           	                                      	                 	            	{
           	                                      	                 	            	  error_y_vector_.push_back(error_y);
           	                                      	                 	            	  error_y_mark= req.y - current_y_;
           	                                      	                 	            	}
           	                                      	                 	            	if(!y_final_arrive_mark)
           	                                      	                 	            	{
           	                                      	                 	            		if (fabs(req.y - current_y_)<fabs(error_y_mark))
           	                                      	                 	            	 vy_map_ = y_dir * (2.0 * ay_max_ * Ty) * (req.y - current_y_) / error_y_mark;
           	                                      	                 	            		else vy_map_ = (2.0 * ay_max_ * Ty) * fabs(req.y - current_y_) / (req.y - current_y_);
           	                                      	                 	            	  if(fabs(req.y - current_y_)<0.02)  y_first_arrive_mark=true;
           	                                      	                 	            	}  
           	                                      	                 	            	else vy_map_= 0;  
           	                                      	                 	              }
           	                                      	                 	           	 }
           	                                                                        
           	             
//           	             if((theta_first_arrive_mark == true) && (x_first_arrive_mark == true) && (y_first_arrive_mark == true))
//           	             {  
//           	            	 theta_final_arrive_mark = true;
//           	            	 x_final_arrive_mark = true;
//           	            	 y_final_arrive_mark = true;
//           	            	 break;
//           	             }
           	                 if((fabs(error_x)<0.01)&&(fabs(error_y)<0.01)&&(fabs(error_theta)<0.02))                                                      
           	                 {  
           	                     theta_final_arrive_mark = true;
           	                     x_final_arrive_mark = true;
           	                     y_final_arrive_mark = true;         
           	                     break;
           	                 }
           	                 
           	              
           	    //         ROS_INFO("rotation speed :  vx_map_ speed : vy_map_speed: %f %f %f",
           	   //        			      w, vx_map_, vy_map_);
           	           //velocity.linear.x = vx_map_ * (k * sin(current_theta_) + cos(current_theta_));
           	           //velocity.linear.y = vx_map_ * (k * cos(current_theta_) - sin(current_theta_));   
           	             velocity.linear.x = vy_map_ * sin(current_theta_) + vx_map_ * cos(current_theta_);
           	             velocity.linear.y = vy_map_ * cos(current_theta_) - vx_map_ * sin(current_theta_);
           	           // velocity.linear.x= 0;
           	          //   velocity.linear.y =0;
           	                 	//      ROS_INFO("velocity.linear.x : velocity.linear.y:  %f %f",
           	                  	 //                 	            		 velocity.linear.x, velocity.linear.y);
           	             velocity.angular.z = w;         	             
           	             data_Vx_Vy_w << setprecision(10) << time << "   " << setprecision(5) << vx_map_ << "   " << setprecision(5) << vy_map_ <<"   " << 
           	            		setprecision(5) << w << "\n" << endl;           	             
           	             data_vx_vy_w << setprecision(10) << time << "   " << setprecision(5) << velocity.linear.x << "   " << setprecision(5) << velocity.linear.y <<"   " <<
           	            		setprecision(5) << w << "\n" << endl;
           	           	 cmd_vel_pub.publish(velocity);
           	           	 
           	         //   if ((vx_map_ == 0) && (vy_map_ == 0)&&(w = 0))
           	          //    break;     	  
           	            ROS_INFO(">>>>>>>>>>>>>>>>>>In the loop now:>>>>>>>>>>>>>");
           	             r.sleep();
           }
           data_x_y_theta.close();
           data_Vx_Vy_w.close();
           data_vx_vy_w.close();
           cmd_vel_pub.publish(geometry_msgs::Twist());
           ROS_INFO("finish the move");
           ros::Duration(1).sleep();
           res.mark = 1.0;               //the position is reached!!
           return true;
       }


private:
    ros::NodeHandle node;
    tf::TransformListener listener;
    tf::StampedTransform transform;
    geometry_msgs::PoseStamped goal_in_map, goal_in_base;
    ros::Publisher cmd_vel_pub;
    ros::Publisher initial_pose_pub;
    ros::Subscriber amcl_pose_sub;
    ros::ServiceServer service;
    geometry_msgs::Pose robot_pose;
    geometry_msgs::Twist velocity;
    double error_x, error_y, error_theta;
    double time_counter;
    
    double initial_x_, initial_y_, initial_angle_;
    double T, T1, T_max_, beta_max_,  w_max_, w;
    double theta_a_, theta_m_, theta_d_, theta;   
    double Tx, Tx_max_, ax_max_, vx_max_, vx_map_;
    double Ty, Ty_max_, ay_max_, vy_max_, vy_map_;
    double x_a_, x_m, x_d_, x;
    double y_a_, y_m, y_d_, y;
    
    double t1,t2,t3;
    vector <double> t1_vector_, t2_vector_, t3_vector_;
    vector <double> error_theta_vector_, error_x_vector_, error_y_vector_;
    bool IS_LONG_WAY_theta, IS_LONG_WAY_x, IS_LONG_WAY_y;  //标志位,判断过程路径是否属长
    bool first_mark, x_mark, y_mark;
    bool theta_first_arrive_mark, x_first_arrive_mark, y_first_arrive_mark;
    bool theta_final_arrive_mark, x_final_arrive_mark, y_final_arrive_mark;
    double error_theta_mark, error_x_mark, error_y_mark;
    double current_theta_, current_x_, current_y_;
    double k;
    double theta_dir, x_dir, y_dir;
    ofstream data_x_y_theta, data_Vx_Vy_w, data_vx_vy_w;
    bool arrive_mark;
};

//*********************************************************************************************************************************************//
//*******************************         MULTIPLE POINST EXIST IN SINGLE PATH         ********************************************************//
//*********************************************************************************************************************************************//

/*
class Jrc_move_multi
{
public:
    Jrc_move_multi():w_max_(0.7),beta_max_(0.3),vx_max_(0.7),ax_max_(0.5),vy_max_(0.7),ay_max_(0.5),initial_x_(-0.630),initial_y_(-0.119),initial_angle_(0.219)
    {   
    	T_max_=w_max_/(2.0*beta_max_);
    	Tx_max_=vx_max_/(2.0*ax_max_);
    	Ty_max_=vy_max_/(2.0*ay_max_);
    	first_mark=false;
    	x_mark=false;
    	y_mark=false;
    	theta_arrive_mark=false;
    	x_arrive_mark=false;
    	y_arrive_mark=false;
        //Topic(cmd_vel) you want to publish
        cmd_vel_pub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        initial_pose_pub = node.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 100);
        //Topic(amcl_pose) you want to subscribe
        amcl_pose_sub = node.subscribe("/amcl_pose", 1, &Jrc_move_multi::callback, this);
        //Server
        service = node.advertiseService("/Jrc_move_multi", &Jrc_move_multi::move_to_target, this);
        //initial the goal
        goal_in_map.header.stamp = ros::Time(0);
        goal_in_map.header.frame_id = "map";
        goal_in_base.header.stamp = goal_in_map.header.stamp;
        goal_in_base.header.frame_id = "base_link";

        //publish the initial pose  
        geometry_msgs::PoseWithCovarianceStamped initial_pose;
        initial_pose.header.stamp = ros::Time(0);
        initial_pose.header.frame_id = "map";
        initial_pose.pose.pose.position.x = initial_x_;
        initial_pose.pose.pose.position.y = initial_y_;
  
        initial_pose.pose.pose.orientation = tf::createQuaternionMsgFromYaw(initial_angle_);
        
        double start_time_point = ros::Time::now().toSec();
        time_counter = 0;
        while(time_counter < 5)
        {
            time_counter = ros::Time::now().toSec() - start_time_point;
            ros::spinOnce();
            initial_pose_pub.publish(initial_pose);
           // ROS_INFO("the robot initial pose");
        }

//        //rotate left and right to get the accuracy amcl pose
//        start_time_point = ros::Time::now().toSec();
//        time_counter = 0;
//        while(time_counter < 4)
//        {
//            time_counter = ros::Time::now().toSec() - sta+rt_time_point;
//            ros::spinOnce();
//            if(time_counter < 2)
//            {
//                velocity = geometry_msgs::Twist();
//                velocity.linear.x = 0.1;
//                cmd_vel_pub.publish(velocity);
//                ROS_INFO("the robot forward");
//            }
//            else
//            {
//                velocity = geometry_msgs::Twist();
//                velocity.linear.x = -0.1;
//                cmd_vel_pub.publish(velocity);
//                ROS_INFO("the robot backward");
//            }
//        }
        cmd_vel_pub.publish(geometry_msgs::Twist());
        ros::Duration(1).sleep();


    }
    void callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amcl_pose)
    {
        robot_pose = amcl_pose->pose.pose;
        //ROS_INFO("the robot pose");
    }

    bool move_to_target(velocity_smooth::smooth_srv::Request &req,
                        velocity_smooth::smooth_srv::Response &res)
    {   cmd_vel_pub.publish(geometry_msgs::Twist());
    	theta = req.theta - initial_angle_;
    	x=req.x-initial_x_;
    	y=req.y-initial_y_;
    	k=(req.y-initial_y_)/(req.x-initial_x_);
    	//test
    //	theta = 2* PI;
       ROS_INFO("remain angle : %f",      theta);
    	T= 
            (sqrt(theta/(6.0*beta_max_))) < T_max_ ? (sqrt(theta/(6*beta_max_))): T_max_;
    	Tx= 
    	    (sqrt(x/(6.0*ax_max_))) < Tx_max_ ? (sqrt(x/(6*ax_max_))): Tx_max_;
    	Ty=
    	    (sqrt(y/(6.0*ay_max_))) < Ty_max_ ? (sqrt(y/(6*ay_max_))): Ty_max_;
        IS_LONG_WAY_theta=
      		(sqrt(theta/(6.0*beta_max_))) < T_max_ ? false : true ;
        IS_LONG_WAY_x=
      		(sqrt(x/(6.0*ax_max_))) < Tx_max_ ? false : true ;
        IS_LONG_WAY_y=
            (sqrt(y/(6.0*ay_max_))) < Ty_max_ ? false : true ;
     //   if (IS_LONG_WAY_theta)
     // 	{
    //	   	T1=(theta-6*beta_max_* T_max_* T_max_)/(2.0 * beta_max_ * T_max_);
  	//	}
   	//	else    T1=0;
        theta_a_ = 3 * beta_max_ * T * T;
        x_a_ = 3 * ax_max_ * Tx * Tx;
        y_a_ = 3 * ay_max_ * Ty * Ty;
        goal_in_map.pose.position.x = req.x;
        goal_in_map.pose.position.y = req.y;
        geometry_msgs::Quaternion  quat = tf::createQuaternionMsgFromYaw(req.theta);
     //   geometry_msgs::Quaternion  quat = tf::createQuaternionMsgFromYaw(theta + initial_angle_);
        goal_in_map.pose.orientation = quat;
        error_x = error_y = 10;
        res.mark= 0;    //the position is not reached!
    //    while(fabs(error_x) > 0.01 || fabs(error_y) > 0.01 || fabs(error_theta) > 0.1)
         ros::Rate r(50); // 20 hz
         ros::Time begin = ros::Time::now();
         double time;
         while (ros::ok())     	  
        {
        	 ros::Duration curr=ros::Time::now()-begin;
			 time=curr.toSec();
        	 ros::spinOnce();
        	 //gain the current error of x y theta;
        	            try{
        	               listener.waitForTransform("/base_link", "/map", ros::Time(0), ros::Duration(100.0) );
        	               listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
        	                }
        	            catch (tf::TransformException ex){
        	               ROS_ERROR("%s",ex.what());
        	               ros::Duration(1.0).sleep();
        	               }          
        	            geometry_msgs::Quaternion quat0;
        	                quat0.x =  transform.getRotation().getX();
        	                quat0.y =  transform.getRotation().getY();
        	                quat0.z =  transform.getRotation().getZ();
        	                quat0.w =  transform.getRotation().getW();    	                
        	            current_theta_ = tf::getYaw(quat0);
        	            current_x_=transform.getOrigin().x();
                	    current_y_=transform.getOrigin().y();
        	            ROS_INFO("current_theta :  current_x : current_y %f %f %f",      current_theta_,current_x_,current_y_);
        	            
        	            listener.transformPose("/base_link",ros::Time(0), goal_in_map, "/map", goal_in_base);
        	          
        	            error_x = goal_in_base.pose.position.x;
        	            error_y = goal_in_base.pose.position.y;
        	            error_theta = tf::getYaw(goal_in_base.pose.orientation);
        	 //accelaration for rotation
        	            if(time < T)
        	           	 {
        	           	    w=(beta_max_/(2.0*T)) *(time * time);
        	           		         
        	           	 }
        	           	else if(time<(2.0*T))
        	           	 {
        	           	    w=beta_max_*(time-T/2.0);
        	           	 }
        	           	 else if(time<(3.0*T))
        	           	 {				  
        	           	    w=beta_max_*(-(time *time)/(2.0*T) + 3.0*time - 2.5*T);
        	           	 }
        	           	 else
        	           	 {
        	           	  if (IS_LONG_WAY_theta)
        	           	  {
        	                if((fabs(error_theta-theta_a_)>0.05) && (!first_mark))  // error limit need change!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1
        	           	    {       	           	           	       
        	                	w = w_max_;        	           	                      	
        	           	    }
        	                else 
        	                {   
        	                	first_mark=true;
        	                	if(t1_vector_.size() == 0)
        	                    {
        	                	   t1_vector_.push_back(time);
        	                 	   t1= time;  
        	                    }	
        	                   
							   
//        	                	if (time < (t1+T))
//        	                	{  
//        	                		w=beta_max_*(-((t1+3*T-time) *(t1+3*T-time))/(2.0*T) + 3.0*(t1+3*T-time) - 2.5*T);  	                		
//        	                	}
//        	                	else if (time < (t1+2*T))
//        	                	{
//        	                		w=beta_max_*(t1+3*T-time-T/2.0);
//        	                	}
//        	                	else if (time < (t1+3*T))
//        	                	{
//        	                		 w=(beta_max_/(2.0*T)) *((t1+3*T-time)  * (t1+3*T-time) );
//        	                	}
//        	                	else w=0;
        	                	
        	                	if (!theta_arrive_mark)
        	                    {
        	                        w = (w_max_ / theta_a_) * (error_theta - 0.05);
        	                        if(fabs(error_theta)<0.06)  theta_arrive_mark=true;
        	                        	                 	                	  
        	                    }
        	                    else w = 0;
        	                }
        	           	  }
        	           	  else
        	              {   
        	           		  if (error_theta_vector_.size() == 0)
        	                  {
        	            	    error_theta_vector_.push_back(error_theta);
        	            	    error_theta_mark= error_theta;
        	                  }
        	           		  if(!theta_arrive_mark)
        	           		  {
        	           			  w = (2.0 * beta_max_ * T)*(error_theta / error_theta_mark);
        	           	         if(fabs(error_theta)<0.06)  theta_arrive_mark=true;
        	           		  }  
        	           		  else w = 0; 
        	        
        	           	  }
								 
        	           	 }
        	            // accelaration for x_movement
        	                                if(time < Tx)
        	                 	           	 {
        	                 	           	    vx_map_=(ax_max_/(2.0*Tx)) *(time * time);
        	                 	           		         
        	                 	           	 }
        	                 	           	else if(time<(2.0*Tx))
        	                 	           	 {
        	                 	           	    vx_map_=ax_max_*(time-Tx/2.0);
        	                 	           	 }
        	                 	           	 else if(time<(3.0*Tx))
        	                 	           	 {				  
        	                 	           	    vx_map_=ax_max_*(-(time *time)/(2.0*Tx) + 3.0*time - 2.5*Tx);
        	                 	           	 }
        	                 	           	 else
        	                 	           	 {
        	                 	           	  if(IS_LONG_WAY_x)
        	                 	           	  {
        	                 	                if((fabs(req.x - current_x_ -x_a_)>0.02) && (!x_mark))  // error limit need change!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1
        	                 	           	    {       	 double test;
        	                 	           	    test=req.x-current_x_-x_a_;
        	                 	               ROS_INFO("test %f ",
        	                 	                	      test);
        	                 	                	vx_map_= vx_max_;        	           	                      	
        	                 	           	    }
        	                 	                else 
        	                 	                {   
        	                 	                	x_mark=true;
        	                 	                	if(t2_vector_.size() == 0)
        	                 	                    {
        	                 	                	   t2_vector_.push_back(time);
        	                 	                 	   t2= time;  
        	                 	                    }	
        	                 	                    
//        	                 	                	if (time < (t2+Tx))
//        	                 	                	{  
//        	                 	                		vx_map_=ax_max_*(-((t2 + 3*Tx-time) *(t2+3*Tx-time))/(2.0*Tx) + 3.0*(t2+3*Tx-time) - 2.5*Tx);  	                		
//        	                 	                	}
//        	                 	                	else if (time < (t2+2*Tx))
//        	                 	                	{
//        	                 	                		vx_map_=ax_max_*(t2 + 3*Tx-time-Tx/2.0);
//        	                 	                	}
//        	                 	                	else if (time < (t2+3*Tx))
//        	                 	                	{
//        	                 	                		vx_map_=(ax_max_/(2.0*Tx)) *((t2+3*Tx-time)  * (t2+3*Tx-time) );
//        	                 	                	}
//        	                 	                	else vx_map_=0;
        	                 	                	
        	                 	                	if (!x_arrive_mark)
        	                 	                	{
        	                 	                	vx_map_ = (vx_max_ / x_a_) * (req.x - current_x_ - 0.02);
        	                 	                	  if(fabs(req.x - current_x_)<0.03)  x_arrive_mark=true;
        	                 	                	  
        	                 	                	}
        	                 	                	else vx_map_= 0;
        	                 	                }
        	                 	           	   }
        	                 	           	   else
        	                 	           	   {
        	                 	           		 if (error_x_vector_.size() == 0)
        	                 	           		 {
        	                 	           		   error_x_vector_.push_back(error_x);
        	                 	           		   error_x_mark= req.x - current_x_;
        	                 	           		 }
        	                 	           		 if(!x_arrive_mark)
        	                 	           		 {
        	               	           			   vx_map_ = (2.0 * ax_max_ * Tx) * (req.x - current_x_) / error_x_mark;
        	               	           			  if(fabs(req.x - current_x_)<0.03)  x_arrive_mark=true;
        	                 	           		 }  
        	                 	           		 else vx_map_= 0;
        	                 	           	   }
        	         								 
        	                 	           	 }
        	                                                                        if(time < Ty)
        	                                      	                 	           	 {
        	                                      	                 	           	    vy_map_=(ay_max_/(2.0*Ty)) *(time * time);
        	                                      	                 	           		         
        	                                      	                 	           	 }
        	                                      	                 	           	else if(time<(2.0*Ty))
        	                                      	                 	           	 {
        	                                      	                 	           	    vy_map_=ay_max_*(time-Ty/2.0);
        	                                      	                 	           	 }
        	                                      	                 	           	 else if(time<(3.0*Ty))
        	                                      	                 	           	 {				  
        	                                      	                 	           	    vy_map_=ay_max_*(-(time *time)/(2.0*Ty) + 3.0*time - 2.5*Ty);
        	                                      	                 	           	 }
        	                                      	                 	           	 else
        	                                      	                 	           	 { 
        	                                      	                 	           	  if(IS_LONG_WAY_y)
        	                                      	                 	           	  {
        	                                      	                 	                if((fabs(req.y - current_y_ -y_a_)>0.02) && (!y_mark))  // error limit need change!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1
        	                                      	                 	           	    {       	 
        	                                      	                 	                	double testy;
        	                                      	                 	           	        testy=req.y-current_y_-y_a_;
        	                                      	                 	                    ROS_INFO("testy %f ",
        	                                      	                 	                	      testy);
        	                                      	                 	                	vy_map_= vy_max_;        	           	                      	
        	                                      	                 	           	    }
        	                                      	                 	                else 
        	                                      	                 	                {   
        	                                      	                 	                	y_mark=true;
        	                                      	                 	                	if(t3_vector_.size() == 0)
        	                                      	                 	                    {
        	                                      	                 	                	   t3_vector_.push_back(time);
        	                                      	                 	                 	   t3= time;  
        	                                      	                 	                    }	
        	                                      	                 	                    
//        	                                      	                 	                	if (time < (t2+Tx))
//        	                                      	                 	                	{  
//        	                                      	                 	                		vx_map_=ax_max_*(-((t2 + 3*Tx-time) *(t2+3*Tx-time))/(2.0*Tx) + 3.0*(t2+3*Tx-time) - 2.5*Tx);  	                		
//        	                                      	                 	                	}
//        	                                      	                 	                	else if (time < (t2+2*Tx))
//        	                                      	                 	                	{
//        	                                      	                 	                		vx_map_=ax_max_*(t2 + 3*Tx-time-Tx/2.0);
//        	                                      	                 	                	}
//        	                                      	                 	                	else if (time < (t2+3*Tx))
//        	                                      	                 	                	{
//        	                                      	                 	                		vx_map_=(ax_max_/(2.0*Tx)) *((t2+3*Tx-time)  * (t2+3*Tx-time) );
//        	                                      	                 	                	}
//        	                                      	                 	                	else vx_map_=0;
        	                                      	                 	                	
        	                                      	                 	                	if (!y_arrive_mark)
        	                                      	                 	                	{
        	                                      	                 	                	vy_map_ = (vy_max_ / y_a_) * (req.y - current_y_ - 0.02);
        	                                      	                 	                	  if(fabs(req.y - current_y_)<0.03)  y_arrive_mark=true;
        	                                      	                 	                	  
        	                                      	                 	                	}
        	                                      	                 	                	else vy_map_= 0;
        	                                      	                 	                }
        	                                      	                 	           	  }
        	                                      	                 	              else
        	                                      	                 	              {
        	                                      	                 	            	if (error_y_vector_.size() == 0)
        	                                      	                 	            	{
        	                                      	                 	            	  error_y_vector_.push_back(error_y);
        	                                      	                 	            	  error_y_mark= req.y - current_y_;
        	                                      	                 	            	}
        	                                      	                 	            	if(!y_arrive_mark)
        	                                      	                 	            	{
        	                                      	                 	            	 vy_map_ = (2.0 * ay_max_ 
        	                                      	    */
int main(int argc, char **argv)
{
	
	 ros::init(argc, argv, "velocity_smooth");
     Jrc_move Jrc_move_obj;
     ros::spin();
     return 0;

}
