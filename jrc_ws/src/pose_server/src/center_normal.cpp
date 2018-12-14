//ros
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/PoseArray.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_broadcaster.h>
//opencv
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
//pcl
#include <pcl/io/ply_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

//chrono
#include <chrono>
//C++
#include <iostream>
#include <ctime>
//self files
// #include "ppf.hpp"
// #include "object6D.hpp"
//#include "realsense_msgs/realsense_msgs.h"
// #include "kinect_msgs/kinect_msgs.h"
#include "jrc_srvs/rgbd_image.h"
// #include "jrc_srvs/rgbd.h"
#include "jrc_srvs/bbox_msgs.h"
#include "jrc_srvs/obj_6d.h"

using namespace std;
using namespace cv;
using namespace pcl;
// using namespace obj6D;

#define depth_scale 1.0
#define metric_scale 1000
#define voting_thred 2

const double camera_cx = 321.6;//311.832(realsense) //321.6(kinect)
const double camera_cy = 239.2;//241.665(realsense) //239.2(kinect)
const double camera_fx = 527.7;//620.845(realsense) //527.7(kinect)
const double camera_fy = 527.9;//620.845(realsense) //527.9(kinect)
int bbox_label = -1;
int bbox_xmin = 0;
int bbox_ymin = 0;
int bbox_xmax = 0;
int bbox_ymax = 0;
int tf_show_signal = 1;
// ppf::PPF6DDetector detector(0.05, 3, 12.0, 0.03); //0.05, 3, 12.0, 0.03
// Pose6DPtr result;
//init main variable

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr scene_rgb (new pcl::PointCloud<pcl::PointXYZRGBA> ());
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr object_rgb (new pcl::PointCloud<pcl::PointXYZRGBA> ());
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr download_object_rgb (new pcl::PointCloud<pcl::PointXYZRGBA> ());
pcl::PointCloud<pcl::PointXYZ>::Ptr target (new pcl::PointCloud<pcl::PointXYZ> ());
pcl::PointCloud<pcl::PointNormal>::Ptr target_with_normals (new pcl::PointCloud<pcl::PointNormal> ());
//pcl viewer
pcl::visualization::PCLVisualizer::Ptr viewer2(new pcl::visualization::PCLVisualizer());
Eigen::Matrix4d transformation;

inline int compute_distance(int x, int y, int centerX, int centerY) {
    return sqrt(pow(x - centerX,2) + pow(y - centerY,2));
}

class object_pose
{
private:
    ros::NodeHandle nh_;
    ros::ServiceClient camera_client;
    ros::ServiceClient bbox_client;
    ros::ServiceServer service;
    jrc_srvs::bbox_msgs bbox_srv;
    jrc_srvs::rgbd_image camera_srv;
    sensor_msgs::Image msg_rgb;
    sensor_msgs::Image msg_depth;

public:
    object_pose()
    {
        camera_client = nh_.serviceClient<jrc_srvs::rgbd_image>("kinect_server");
        service = nh_.advertiseService("object_pose_central_normal", &object_pose::obj_pose, this); 
        ROS_INFO("get service!");
    }
    ~object_pose()
    {}
    bool obj_pose(jrc_srvs::obj_6d::Request &req,jrc_srvs::obj_6d::Response &res)
    {
        Mat rgb_image;
        Mat depth_image;
        Mat depth_show;
        int min_dist_point_id = -1;
        auto start = chrono::system_clock::now();
        if(req.start)
        {
            camera_srv.request.start = true;
            if(camera_client.call(camera_srv))
            {
                try
                {
                    msg_rgb = camera_srv.response.rgb_image;
                    msg_depth = camera_srv.response.depth_image;
                    rgb_image = cv_bridge::toCvCopy(msg_rgb,sensor_msgs::image_encodings::TYPE_8UC3)->image;
                    depth_image = cv_bridge::toCvCopy(msg_depth,sensor_msgs::image_encodings::TYPE_32FC1)->image;
                    normalize(depth_image,depth_show,255,0,NORM_MINMAX);
                    depth_show.convertTo(depth_show, CV_8UC1, 1.0);
                }
                catch (cv_bridge::Exception& e)
                {
                    ROS_ERROR("cv_bridge exception: %s", e.what());
                    return 1;
                }
                depth_image.convertTo(depth_image,CV_32F);
            }
            if((!rgb_image.data)||(!depth_image.data))
            {
                cout<<"Error: Can not get rgb or depth image!"<<endl;
            }
            // cv::cvtColor(rgb_image, rgb_image, cv::COLOR_RGB2BGR);
            // imshow("rgb",rgb_image);
            // imshow("depth",depth_show);
            // waitKey(1);

            scene_rgb->clear();
            object_rgb->clear();
            target->clear();
            target_with_normals->clear();

            bbox_label = req.label;
            bbox_xmin = req.xmin;
            bbox_ymin = req.ymin;
            bbox_xmax = req.xmax;
            bbox_ymax = req.ymax;

            if(bbox_label <= 0)
            {
                cout<<"Can not detect object in list."<<endl;
                return true;
            }

            for (int r=0;r<rgb_image.rows;r++)
            {
                for (int c=0;c<rgb_image.cols;c++)
                {
                    pcl::PointXYZ p;
                    pcl::PointXYZRGBA p_rgb;
                    depth_image.at<float>(r,c) = depth_image.at<float>(r,c)*depth_scale/metric_scale;
                    if(!(depth_image.at<float>(r,c)>0&&depth_image.at<float>(r,c)<1)){continue;}
                    double scene_z = double(depth_image.at<float>(r,c));
                    double scene_x = (c - camera_cx) * scene_z / camera_fx;
                    double scene_y = (r - camera_cy) * scene_z / camera_fy;
                    p.x = scene_x;
                    p.y = scene_y;
                    p.z = scene_z;
                    p_rgb.x = scene_x;
                    p_rgb.y = scene_y;
                    p_rgb.z = scene_z;
                    p_rgb.r = rgb_image.ptr<uchar>(r)[c*3+2];
                    p_rgb.g = rgb_image.ptr<uchar>(r)[c*3+1];
                    p_rgb.b = rgb_image.ptr<uchar>(r)[c*3];
                    scene_rgb->points.push_back(p_rgb);
                }
            }

            if(scene_rgb->size() < 10)
            {
                cout<<"No points in scene!"<<endl;
                return false;
            }
            //removing nan
            std::vector<int> mapping;
            pcl::removeNaNFromPointCloud(*scene_rgb, *scene_rgb, mapping);

            //segment plane
            pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
            seg.setOptimizeCoefficients (false);
            seg.setModelType (pcl::SACMODEL_PLANE);
            seg.setMethodType (pcl::SAC_RANSAC);
            seg.setDistanceThreshold (0.003);
            seg.setMaxIterations (100);
            seg.setProbability(0.95);
            seg.setInputCloud (scene_rgb);
            pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
            pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
            seg.segment (*inliers, *coefficients);
            if (inliers->indices.size () == 0)
            {
            PCL_ERROR ("Could not estimate a planar model for the given dataset.");
            return (-1);
            }
    //        std::cerr << "Model coefficients: " << coefficients->values[0] << " "
    //                                          << coefficients->values[1] << " "
    //                                          << coefficients->values[2] << " "
    //                                          << coefficients->values[3] << std::endl;

    //        std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
            //Extract the inliers
            pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
            extract.setInputCloud (scene_rgb);
            extract.setIndices (inliers);
            extract.setNegative (true);
            //PointCloudPtr output (new PointCloud);
            extract.filter (*object_rgb);
            // object_rgb = scene_rgb;

            if(object_rgb->size() < 10)
            {
                cout<<"Little points in object pc!"<<endl;
                return false;
            }

            //*****************************
            //下采样
            //*****************************
            pcl::VoxelGrid<pcl::PointXYZRGBA> grid; //VoxelGrid 把一个给定的点云，聚集在一个局部的3D网格上,并下采样和滤波点云数据
            grid.setLeafSize (0.003, 0.003, 0.003); //设置体元网格的叶子大小
            //下采样 源点云
            grid.setInputCloud (object_rgb); //设置输入点云
            grid.filter (*download_object_rgb); //下采样和滤波，并存储在src中

            int bbox_centerX, bbox_centerY;
            bbox_centerX = (bbox_xmax + bbox_xmin) / 2;
            bbox_centerY = (bbox_ymax + bbox_ymin) / 2;
            int min_dist = INT_MAX;
            #pragma omp parallel for
            int point_count = 0;
            for(int i = 0;i<download_object_rgb->points.size();i++)
            {
                int u = (camera_fx/download_object_rgb->points.at(i).z)*download_object_rgb->points.at(i).x + camera_cx;
                int v = (camera_fy/download_object_rgb->points.at(i).z)*download_object_rgb->points.at(i).y + camera_cy;
                if ((u>bbox_xmin)&&(u<bbox_xmax)&&(v>bbox_ymin)&&(v<bbox_ymax))
                {
                    int current_dist = compute_distance(u,v,bbox_centerX,bbox_centerY);
                    if( current_dist < min_dist) 
                    {
                        min_dist = current_dist;
                        min_dist_point_id = point_count;
                    }
                    pcl::PointXYZ p_rgb;
                    p_rgb.x = download_object_rgb->points.at(i).x;
                    p_rgb.y = download_object_rgb->points.at(i).y;
                    p_rgb.z = download_object_rgb->points.at(i).z;
                    target->points.push_back(p_rgb);
                    point_count++;
                }
            }
            cout<<"get target"<<endl;
            if(target->size() < 30)
            {
                cout<<"Little points in target pc!"<<endl;
                return false;
            }
            // pcl::PLYWriter writer;
            // writer.write("/home/mzm/jrc_ws_vision_debug/src/pose_server/test.ply", *target);

            viewer2->removeAllPointClouds();
            viewer2->setBackgroundColor(255,255,255);
            viewer2->addCoordinateSystem(0.1,-1,1,1);
            viewer2->addPointCloud(download_object_rgb,"download_object_rgb");

            if(bbox_label <= 0)
            {
                res.label = -1;
            }

            cout<<"test1"<<endl;

            pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
            pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
            ne.setSearchMethod (tree);
            ne.setRadiusSearch (0.03);
            ne.setInputCloud (target);

            ne.compute (*cloud_normals);
            cout<<"test2"<<endl;
            pcl::Normal normal = cloud_normals->points.at(min_dist_point_id);
            cout<<"test3"<<endl;
            Eigen::Vector3d axis_N( normal.normal_x, normal.normal_y, normal.normal_z );
            Eigen::Vector3d axis_Z(0,0,1);
            Eigen::Vector3d cross_N_x_Z = axis_N.cross(axis_Z);
            if(cross_N_x_Z.dot(Eigen::Vector3d(1,0,0)) < 0) {
                cross_N_x_Z = axis_Z.cross(axis_N);
            } 
            cout<<"test4"<<endl;
            Eigen::Vector3d axis_X = cross_N_x_Z.normalized();
            axis_Z = axis_N.normalized();
            Eigen::Vector3d axis_Y = axis_Z.cross(axis_X).normalized();

            Eigen::Matrix3d rotation;
            rotation << axis_X[0], axis_Y[0], axis_Z[0],
                        axis_X[1], axis_Y[1], axis_Z[1],
                        axis_X[2], axis_Y[2], axis_Z[2];

            Eigen::Quaterniond q;       
            q = Eigen::Quaterniond(rotation);
            cout<<"eigen q:"<< q.coeffs()<<endl;
            Eigen::Vector4d q_centroid = q.coeffs();
            Eigen::Vector3d t_centroid(target->points.at(min_dist_point_id).x,
                                       target->points.at(min_dist_point_id).y,
                                       target->points.at(min_dist_point_id).z);

            geometry_msgs::Pose pose;
            pose.position.x = (float)t_centroid[0];
            pose.position.y = (float)t_centroid[1];
            pose.position.z = (float)t_centroid[2];
            pose.orientation.x = (float)q_centroid[0];
            pose.orientation.y = (float)q_centroid[1];
            pose.orientation.z = (float)q_centroid[2];
            pose.orientation.w = (float)q_centroid[3];

            transformation = Eigen::Matrix4d::Identity ();
            transformation.block(0,0,3,3) = rotation;
            transformation(0,3) = t_centroid[0];
            transformation(1,3) = t_centroid[1];
            transformation(2,3) = t_centroid[2];

            cout<<"position:"<<(float)t_centroid[0]<<" "<<(float)t_centroid[1]<<" "<<(float)t_centroid[2]<<endl;
            cout<<"orientation:"<<(float)q_centroid[1]<<" "<<(float)q_centroid[2]<<" "<<(float)q_centroid[3]<<" "<<(float)q_centroid[0]<<endl;

            res.obj_pose = pose;
            res.label = bbox_label;

            string path = ros::package::getPath("pose_server");
            stringstream ss;
            ss<<path<<"/model/"<<bbox_label<<".ply";
            pcl::PointCloud<pcl::PointXYZ>::Ptr model_show (new pcl::PointCloud<pcl::PointXYZ> ());
            pcl::io::loadPLYFile(ss.str().c_str(),*model_show);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ppf (new pcl::PointCloud<pcl::PointXYZ>);

            if(tf_show_signal == 1)
            {
                tf::Transform transform;
                transform.setOrigin( tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
                // cv::Matx33d rotation_cv = Eigen2cvMat(rotation);
                Eigen::Quaterniond qe;
                qe = Eigen::Quaterniond(rotation);
                Eigen::Vector4d qe_ = qe.coeffs();
                cout<<"eigen q:"<<qe.coeffs()<<endl;
                tf::Quaternion q(qe_[0],qe_[1],qe_[2],qe_[3]);
                transform.setRotation(q);
                static tf::TransformBroadcaster br;
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_depth_optical_frame", "object"));
                tf_show_signal = 0;
            }

            pcl::transformPointCloud(*model_show, *cloud_ppf, transformation);
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud_ppf, 0, 255, 0);

            viewer2->addPointCloud<pcl::PointXYZ> (cloud_ppf, single_color, "normals");
            viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "normals");
            viewer2->spinOnce();
        }
        auto end   = chrono::system_clock::now();
        auto duration = chrono::duration_cast<chrono::microseconds>(end - start);
        cout <<  "cost " 
        << double(duration.count()) * chrono::microseconds::period::num / chrono::microseconds::period::den 
        << " second" << endl;
        return true;
    }
};

int main(int argc, char** argv)
{
    //ros configs
    ros::init(argc,argv,"jd_vision_center_normal");
    object_pose p1;

    ros::Rate loop_rate(200);

    ROS_INFO("START JRC VISION CENTER NORMAL!");

    while(ros::ok())
    {
        loop_rate.sleep();
        viewer2->spinOnce();
        ros::spinOnce();
    }

    return 0;
}
