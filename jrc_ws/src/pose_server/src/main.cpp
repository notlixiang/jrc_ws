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
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
//chrono
#include <chrono>
//C++
#include <iostream>
#include <ctime>
//self files
#include "ppf.hpp"
#include "object6D.hpp"
//#include "realsense_msgs/realsense_msgs.h"
// #include "kinect_msgs/kinect_msgs.h"
#include "jrc_srvs/rgbd_image.h"
// #include "jrc_srvs/rgbd.h"
#include "jrc_srvs/bbox_msgs.h"
#include "jrc_srvs/obj_6d.h"

using namespace std;
using namespace cv;
using namespace pcl;
using namespace obj6D;

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
ppf::PPF6DDetector detector(0.05, 3, 12.0, 0.03); //0.05, 3, 12.0, 0.03
vector<Pose6DPtr> results;
//init main variable

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr scene_rgb (new pcl::PointCloud<pcl::PointXYZRGBA> ());
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr object_rgb (new pcl::PointCloud<pcl::PointXYZRGBA> ());
pcl::PointCloud<pcl::PointXYZ>::Ptr target (new pcl::PointCloud<pcl::PointXYZ> ());
pcl::PointCloud<pcl::PointNormal>::Ptr target_with_normals (new pcl::PointCloud<pcl::PointNormal> ());
//pcl viewer
pcl::visualization::PCLVisualizer::Ptr viewer1(new pcl::visualization::PCLVisualizer());
Eigen::Matrix4d transformation;

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
        service = nh_.advertiseService("object_pose", &object_pose::obj_pose, this); 
        ROS_INFO("get service!");
    }
    ~object_pose()
    {}
    bool obj_pose(jrc_srvs::obj_6d::Request &req,jrc_srvs::obj_6d::Response &res)
    {
        Mat rgb_image;
        Mat depth_image;
        Mat depth_show;
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

            if(bbox_label == 1) //leshi
                detector.setParameters(0.05, 3, 12.0, 0.03);
            if(bbox_label == 2) //milk
                detector.setParameters(0.04, 3, 12.0, 0.02);
            if(bbox_label == 3) //zhencui
                detector.setParameters(0.05, 3, 12.0, 0.03);
            if(bbox_label == 4) //yagao
                detector.setParameters(0.025, 3, 10.0, 0.02);
            if(bbox_label == 5) //coke
                detector.setParameters(0.05, 3, 12.0, 0.03);
            if(bbox_label == 6) //gaipian
                detector.setParameters(0.05, 3, 12.0, 0.03);
            if(bbox_label == 7) //juzi
                detector.setParameters(0.08, 4, 12.0, 0.03);
            if(bbox_label == 8) //yajiao
                detector.setParameters(0.05, 3, 12.0, 0.03);
            if(bbox_label == 9) //taiping
                detector.setParameters(0.05, 3, 12.0, 0.03);
            if(bbox_label == 10)//zhijin
                detector.setParameters(0.05, 3, 12.0, 0.03);
            if(bbox_label == 11)//book
                detector.setParameters(0.05, 3, 12.0, 0.03);
            if(bbox_label == 12)//aoliao
                detector.setParameters(0.04, 3, 12.0, 0.02);
            if(bbox_label == 13)//guozhen
                detector.setParameters(0.05, 3, 12.0, 0.03);
            if(bbox_label == 14)//xifalu
                detector.setParameters(0.04, 3, 12.0, 0.02);
            if(bbox_label == 15)//xiangchang
                detector.setParameters(0.05, 3, 12.0, 0.03);
            if(bbox_label == 16)//yashua
                detector.setParameters(0.04, 3, 12.0, 0.02);
 
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
            seg.setDistanceThreshold (0.01);
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

            if(object_rgb->size() < 10)
            {
                cout<<"Little points in object pc!"<<endl;
                return false;
            }


            #pragma omp parallel for
            for(int i = 0;i<object_rgb->points.size();i++)
            {
                int u = (camera_fx/object_rgb->points.at(i).z)*object_rgb->points.at(i).x + camera_cx;
                int v = (camera_fy/object_rgb->points.at(i).z)*object_rgb->points.at(i).y + camera_cy;
                if ((u>bbox_xmin)&&(u<bbox_xmax)&&(v>bbox_ymin)&&(v<bbox_ymax))
                {
                    pcl::PointXYZ p_rgb;
                    p_rgb.x = object_rgb->points.at(i).x;
                    p_rgb.y = object_rgb->points.at(i).y;
                    p_rgb.z = object_rgb->points.at(i).z;
                    target->points.push_back(p_rgb);
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

            viewer1->removeAllPointClouds();
            viewer1->setBackgroundColor(255,255,255);
            viewer1->addCoordinateSystem(0.1,-1,1,1);
            viewer1->addPointCloud(object_rgb,"object_rgb");

            if(bbox_label <= 0)
            {
                res.label = -1;
            }
            string path = ros::package::getPath("pose_server");
            stringstream ss;
            ss<<path<<"/model/"<<bbox_label<<".ply";
            pcl::PointCloud<pcl::PointXYZ>::Ptr model (new pcl::PointCloud<pcl::PointXYZ> ());
            pcl::PointCloud<pcl::PointXYZ>::Ptr model_show (new pcl::PointCloud<pcl::PointXYZ> ());
            pcl::PointCloud<pcl::PointNormal>::Ptr model_with_normals (new pcl::PointCloud<pcl::PointNormal> ());
            pcl::io::loadPLYFile(ss.str().c_str(),*model);

            cout << "Model Pre-Processing..." << endl;
            double start = clock();
            detector.preProcessing(model, *model_with_normals, *model_show, 1);
            double stop = clock();
            cout << "Model Pre-Processing complete in ：" <<
                    ((double)(stop-start))/CLOCKS_PER_SEC << " s" << endl;
            cout << "Model Training..." << endl;
            start = clock();
            detector.trainModel(model_with_normals);
            stop = clock();
            cout << "Model Training complete in ：" <<
                    ((double)(stop-start))/CLOCKS_PER_SEC << " s" << endl;

            start = clock();
            detector.preProcessing(target, *target_with_normals, 0);
            stop = clock();
            cout << "Scene Proprocessing complete in ：" <<
                    ((double)(stop-start))/CLOCKS_PER_SEC << " s" << endl;

            cout << "Starting scene matching..." << endl;
            results.clear();
            start = clock();
            detector.match(target_with_normals, results, 1.0f/3.0f);
            stop = clock();
            cout << "Scene Matching complete in ：" <<
                    ((double)(stop-start))/CLOCKS_PER_SEC << " s" << endl;
            // geometry_msgs::Pose obj_pose;
            // obj_array.poses.resize(0);
            cout<<"results.size: "<<results.size()<<endl;
            cout<<"Max numVotes: "<<results[0]->numVotes<<endl;

            int tmp_numVotes = 0;
            int max_index = 0;
            int second_index = 0;
            for (size_t i=0;i<results.size();i++)
            {
                Pose6DPtr tmp_result = results[i];
                if(tmp_result->numVotes > tmp_numVotes)
                {
                    max_index = i;
                    tmp_numVotes = tmp_result->numVotes;
                }
            }
            if(results[max_index]->numVotes<voting_thred)
                {
                    cout<<"vote num is less than threshold."<<endl;
                    return false;
                }
            Vec3d t = results[max_index]->pose_t;
            Vec4d q = results[max_index]->q;

            transformation = Eigen::Matrix4d::Identity ();
            transformation = cvMat2Eigen(results[max_index]->pose);
            Eigen::Matrix3d rotation = transformation.block(0,0,3,3);
            // cv::Matx33d rotation_cv = Eigen2cvMat(rotation);
            Eigen::Quaterniond qe;
            qe = Eigen::Quaterniond(rotation);
            Eigen::Vector4d qe_ = qe.coeffs();
            cout<<"eigen q:"<<qe.coeffs()<<endl;

            geometry_msgs::Pose pose;
            pose.position.x = (float)t[0];
            pose.position.y = (float)t[1];
            pose.position.z = (float)t[2];
            pose.orientation.x = (float)qe_[0];
            pose.orientation.y = (float)qe_[1];
            pose.orientation.z = (float)qe_[2];
            pose.orientation.w = (float)qe_[3];

            cout<<"position:"<<(float)t[0]<<" "<<(float)t[1]<<" "<<(float)t[2]<<endl;
            cout<<"orientation:"<<(float)q[1]<<" "<<(float)q[2]<<" "<<(float)q[3]<<" "<<(float)q[0]<<endl;
            // obj_array.poses.push_back(pose);
            res.obj_pose = pose;
            res.label = bbox_label;

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ppf (new pcl::PointCloud<pcl::PointXYZ>);

            if(tf_show_signal == 1)
            {
                    tf::Transform transform;
                    transform.setOrigin( tf::Vector3(transformation(0,3), transformation(1,3), transformation(2,3)) );
                    Eigen::Matrix3d rotation = transformation.block(0,0,3,3);
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
            stringstream ss2;
            ss2<<max_index;
            // viewer2->addPointCloudNormals<pcl::PointNormal> (cloud_ppf1, 10, 0.01, "normals2");
            viewer1->addPointCloud<pcl::PointXYZ> (cloud_ppf, single_color, ss2.str().c_str());
            viewer1->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, ss2.str().c_str());
            viewer1->spinOnce();
        }
        auto end   = chrono::system_clock::now();
        auto duration = chrono::duration_cast<chrono::microseconds>(end - start);
        cout <<  "cost " 
        << double(duration.count()) * chrono::microseconds::period::num / chrono::microseconds::period::den 
        << " second" << endl;
        detector.clearTrainingModels();
        return true;
    }
};

int main(int argc, char** argv)
{
    //ros configs
    ros::init(argc,argv,"jd_vision");
    object_pose p1;

    ros::Rate loop_rate(200);

    ROS_INFO("START JD VISION!");

    while(ros::ok())
    {
        loop_rate.sleep();
        viewer1->spinOnce();
        ros::spinOnce();
    }

    return 0;
}
