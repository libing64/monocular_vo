#include <iostream>
#include <Eigen/Eigen>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "mono_vo.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace std;
using namespace Eigen;
using namespace cv;

ros::Subscriber camera_info_sub;
mono_vo mono;

ros::Publisher pub_odom, pub_pose, pub_path, pub_cloud;

Quaterniond q_cam2imu;

void publish_odom(mono_vo &mono);
void publish_pose(mono_vo &mono);
void publish_path(mono_vo &mono);
void publish_cloud(mono_vo &mono);

void camera_info_callback(const sensor_msgs::CameraInfoConstPtr &msg)
{
    if (!mono.is_camera_info_init)
        mono.set_camere_info(msg);
    camera_info_sub.shutdown();
}

void image_callback(const sensor_msgs::ImageConstPtr &left_image_msg,
                    const sensor_msgs::ImageConstPtr &right_image_msg)
{
    if (mono.is_camera_info_init)
    {
        Mat left_img = cv_bridge::toCvCopy(left_image_msg, string("mono8"))->image;
        Mat right_img = cv_bridge::toCvCopy(right_image_msg, string("mono8"))->image;
        //imshow("left", left_img);
        //imshow("right", right_img);
        //waitKey(2);
        mono.timestamp = left_image_msg->header.stamp.toSec();
        mono.update(left_img);
        //mono.mono_detect(left_img, right_img);
        publish_odom(mono);
        publish_pose(mono);
        publish_cloud(mono);
    }

}

void publish_odom(mono_vo &mono)
{
    nav_msgs::Odometry odometry;
    odometry.header.frame_id = "odom";
    odometry.header.stamp = ros::Time(mono.timestamp);

    Eigen::Quaterniond q = q_cam2imu * mono.q;
    Eigen::Vector3d p = q_cam2imu.toRotationMatrix() * mono.t;

    odometry.child_frame_id = "base_link";
    odometry.pose.pose.position.x = p(0);
    odometry.pose.pose.position.y = p(1);
    odometry.pose.pose.position.z = p(2);
    odometry.pose.pose.orientation.x = q.x();
    odometry.pose.pose.orientation.y = q.y();
    odometry.pose.pose.orientation.z = q.z();
    odometry.pose.pose.orientation.w = q.w();
    pub_odom.publish(odometry);
}

void publish_pose(mono_vo &mono)
{
    static nav_msgs::Path path;
    geometry_msgs::PoseStamped pose;
    Eigen::Quaterniond q = q_cam2imu * mono.q;
    Eigen::Vector3d p = q_cam2imu.toRotationMatrix() * mono.t;

    pose.header.stamp = ros::Time(mono.timestamp);
    pose.header.frame_id = "odom";
    pose.pose.orientation.w = q.w();
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.position.x = p(0);
    pose.pose.position.y = p(1);
    pose.pose.position.z = p(2);
    pub_pose.publish(pose);

    //cout << "publish pose: " << endl;
    //_pose.print_state();
    path.header.frame_id = "odom";
    path.poses.push_back(pose);
    pub_path.publish(path);

    //send transfrom
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped tr;
    tr.header.stamp = ros::Time(mono.timestamp);
    tr.header.frame_id = "odom";
    tr.child_frame_id = "base_link";
    tr.transform.translation.x = p(0);
    tr.transform.translation.y = p(1);
    tr.transform.translation.z = p(2);
    tr.transform.rotation.x = pose.pose.orientation.x;
    tr.transform.rotation.y = pose.pose.orientation.y;
    tr.transform.rotation.z = pose.pose.orientation.z;
    tr.transform.rotation.w = pose.pose.orientation.w;
    br.sendTransform(tr);
}

void publish_cloud(mono_vo &mono)
{
    if (mono.feat3ds.size())
    {
        pcl::PointCloud<pcl::PointXYZI> cloud;
        Matrix3d Ric = q_cam2imu.toRotationMatrix();
        for (int i = 0; i < mono.feat3ds.size(); i++)
        {
            Vector3d pc;
            pc(0) = mono.feat3ds[i].x;
            pc(1) = mono.feat3ds[i].y;
            pc(2) = mono.feat3ds[i].z;
            Vector3d pi = Ric * pc;
            PointType p;
            p.x = pi(0);
            p.y = pi(1);
            p.z = pi(2);
            cloud.points.push_back(p);
        }

        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(cloud, cloud_msg);

        std_msgs::Header header;
        header.stamp = ros::Time(mono.timestamp);
        header.frame_id = "odom";
        cloud_msg.header = header;

        pub_cloud.publish(cloud_msg);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mono_vo");
    ros::NodeHandle n("~");

    camera_info_sub = n.subscribe("/camera_info", 10, camera_info_callback);

    message_filters::Subscriber<sensor_msgs::Image> left_img_sub(n, "/left_image", 1);
    message_filters::Subscriber<sensor_msgs::Image> right_img_sub(n, "/right_image", 2);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), left_img_sub, right_img_sub);
    sync.registerCallback(boost::bind(&image_callback, _1, _2));

    pub_odom = n.advertise<nav_msgs::Odometry>("/odom", 10);
    pub_pose = n.advertise<geometry_msgs::PoseStamped>("/mono_pose", 10);
    pub_path = n.advertise<nav_msgs::Path>("/mono_path", 10);
    pub_cloud = n.advertise<sensor_msgs::PointCloud2>("/cloud", 10);

    Matrix3d R_cam2imu;
    R_cam2imu << 0, 0, 1, -1, 0, 0, 0, -1, 0;
    q_cam2imu = Quaterniond(R_cam2imu);
    cout << "R_cam2imu: " << R_cam2imu << endl;
    cout << "q_cam2imu: " << q_cam2imu.coeffs().transpose() << endl;
                 
    ros::spin();
    return 0;
}