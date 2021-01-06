#ifndef __mono_vo_H
#define __mono_vo_H
#include <iostream>
#include <Eigen/Eigen>
#include <vector>
#include <map>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/CameraInfo.h>

using namespace std;
using namespace Eigen;
using namespace cv;

class mono_vo
{
private:
    int max_feat_cnt = 1000;
    int min_feat_cnt = 100;
    int min_track_cnt = 50;
    int min_feat_dist = 10;
    int min_disparity = 2;
    int max_epipolar = 5;
    bool feat_vis_enable = true;
    //camera matrix
    Matrix3d K;
    double baseline;
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
    Mat keyframe;
public:
    Quaterniond q;
    Vector3d t;
    double timestamp;

    //state of keyframe
    Quaterniond qk;
    Vector3d tk;

    int is_camera_info_init;
    vector<Point2f> feats;

    mono_vo();
    ~mono_vo();

    void set_camere_info(const sensor_msgs::CameraInfoConstPtr& msg);

    void mono_detect(Mat &img);
    void mono_visualize(Mat &img, vector<Point2f> &left_feats);
    void mono_visualize(Mat &img, Mat &right_img, vector<Point2f>&left_feats, vector<Point2f>&right_feats);

    int mono_track(Mat &keyframe, Mat &img);

    void update(Mat& img);
    void visualize_features(Mat &img, vector<Point2f> &feats, vector<Point2f> &feats_prev, vector<uchar> &status);
};

mono_vo::mono_vo()
{
    is_camera_info_init = false;
    qk = Quaterniond::Identity();
    tk = Vector3d::Zero();
    q = qk;
    t = tk;
}

mono_vo::~mono_vo()
{
}

void mono_vo::set_camere_info(const sensor_msgs::CameraInfoConstPtr& msg)
{
    K = Matrix3d::Identity();
    K(0, 0) = msg->K[0];
    K(1, 1) = msg->K[4];
    K(0, 2) = msg->K[2];
    K(1, 2) = msg->K[5];
    baseline = fabs(msg->P[3] / msg->K[0]);
    cout << "K: " << K << endl;
    cout << "baseline: " << baseline << endl;


    camera_matrix = Mat::eye(3, 3, CV_64F);
    camera_matrix.at<double>(0, 0) = msg->K[0];
    camera_matrix.at<double>(1, 1) = msg->K[4];
    camera_matrix.at<double>(0, 2) = msg->K[2];
    camera_matrix.at<double>(1, 2) = msg->K[5];

    dist_coeffs = Mat::zeros(5, 1, CV_64F);
    is_camera_info_init = true;
}

void mono_vo::mono_detect(Mat &img)
{
    Ptr<FeatureDetector> detector = cv::ORB::create(max_feat_cnt);
    vector<KeyPoint> keypoints;
    detector->detect(img, keypoints);
    KeyPoint::convert(keypoints, feats);

    img.copyTo(keyframe);
    qk = q;
    tk = t;
    cout << "mono detect: qk: " << qk.coeffs().transpose() << endl << "tk: " << t.transpose() << endl;
    cout << "feats size: " << feats.size() << endl;
    mono_visualize(img, feats);
}

void mono_vo::mono_visualize(Mat &img, vector<Point2f> &left_feats)
{
    if (feat_vis_enable)
    {
        Mat color_img = Mat(img.rows, img.cols, CV_8UC3);
        if (img.channels() == 3)
        {
            img.copyTo(color_img(Rect(0, 0, img.cols, img.rows)));
        }
        else
        {
            cvtColor(img, color_img, cv::COLOR_GRAY2RGB);
        }

        Scalar color = Scalar(0, 0, 255);
        Scalar left_color = Scalar(255, 0, 0);
        for (int i = 0; i < left_feats.size(); i++)
        {
            circle(color_img, left_feats[i], 1, left_color);
        }
        imshow("mono feature", color_img);
        waitKey(2);
    }
}

void mono_vo::mono_visualize(Mat &img, Mat &right_img, vector<Point2f> &left_feats, vector<Point2f> &right_feats)
{
    if (feat_vis_enable)
    {
        Mat merge_img = Mat(img.rows * 2, img.cols, CV_8UC3);
        if (img.channels() == 3)
        {
            img.copyTo(merge_img(Rect(0, 0, img.cols, img.rows)));
            right_img.copyTo(merge_img(Rect(0, img.rows, right_img.cols, right_img.rows)));
        }
        else
        {
            Mat img_color, right_img_color;
            cvtColor(img, img_color, cv::COLOR_GRAY2RGB);
            cvtColor(right_img, right_img_color, cv::COLOR_GRAY2RGB);
            img_color.copyTo(merge_img(Rect(0, 0, img.cols, img.rows)));
            right_img_color.copyTo(merge_img(Rect(0, img.rows, right_img.cols, right_img.rows)));
        }

        Scalar color = Scalar(0, 0, 255);
        Scalar left_color = Scalar(255, 0, 0);
        Scalar right_color = Scalar(0, 255, 0);
        Point2f delta(0, img.rows);
        for (int i = 0; i < feats.size(); i++)
        {
            //if (status[i])
            {
                line(merge_img, left_feats[i], right_feats[i] + delta, color, 1, 8);
                circle(merge_img, left_feats[i], 1, left_color);
                circle(merge_img, right_feats[i], 1, right_color);
            }
        }
        imshow("mono match", merge_img);
        waitKey(2);
    }
}

int mono_vo::mono_track(Mat &keyframe, Mat &img)
{
    vector<uchar> status;
    vector<float> err;
    vector<Point2f> feats_curr;
    cv::calcOpticalFlowPyrLK(keyframe, img, feats, feats_curr, status, err);
    int count = std::count(status.begin(), status.end(), 1);

    vector<Point2f> points_prev(count);

    vector<Point2f> points_curr(count);
    int j = 0;
    for (auto i = 0; i < status.size(); i++)
    {
        if (status[i])
        {
            points_curr[j] = feats_curr[i];
            points_prev[j] = feats[i];
            j++;
        }
    }
    Mat E, rvec, tvec;
    Mat dR;
    vector<uchar> inliers;

    E = findEssentialMat(points_prev, points_curr, camera_matrix, RANSAC, 0.99, 1.0, inliers); //threshold!!!
    int ret = recoverPose(E, points_prev, points_curr, camera_matrix, rvec, tvec, inliers);

    cout << "rvec: " << rvec << endl;
    cout << "tvec: " << tvec << endl;
    visualize_features(img, points_prev, points_curr, inliers);
    if (ret)
    {
        Matrix3d R;
        Quaterniond dq;
        Vector3d dt;
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                R(i, j) = rvec.at<double>(i, j);
            }
        }

        dt = Vector3d(0, 0, 1);
        dq = Quaterniond(R.transpose());
        t = t + q.toRotationMatrix() * dt;
        q = qk * dq;
        cout << "mono track: " << q.coeffs().transpose() << endl << "t: " << t.transpose() << endl;
    }
    int inlier_count = std::count(inliers.begin(), inliers.end(), 1);
    double inlier_rate = 1.0 * inlier_count / inliers.size();
    cout << "inlier rate: " << inlier_rate << endl;
    return inlier_count;
}

void mono_vo::update(Mat &img)
{
    //feature_extract(img);
    if (feats.empty())
    {
        q = Quaterniond::Identity();
        t = Vector3d::Zero();

        mono_detect(img);
    } else 
    {
        int inlier_count = mono_track(keyframe, img);
        if (inlier_count < min_feat_cnt)
        {
            mono_detect(img);
        }
    }
}

void mono_vo::visualize_features(Mat &img, vector<Point2f> &feats, vector<Point2f> &feats_prev, vector<uchar>& status)
{
    static Mat img_color;
    if (feat_vis_enable)
    {
        cv::cvtColor(img, img_color, cv::COLOR_GRAY2RGB);
        Scalar color = Scalar(0, 0, 255);
        Scalar color_prev = Scalar(255, 0, 0);
        Scalar color_next = Scalar(0, 255, 0);
        for (auto i = 0; i < feats.size(); i++)
        {
            if (status[i])
            {
                line(img_color, feats[i], feats_prev[i], color, 1, 8);
                circle(img_color, feats[i], 1, color);
                circle(img_color, feats_prev[i], 2, color_prev);
                circle(img_color, feats[i], 2, color_next);
            }
        }
        imshow("feats", img_color);
        waitKey(2);
    }
}

#endif
