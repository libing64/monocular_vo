#ifndef __mono_vo_H
#define __mono_vo_H
#include <iostream>
#include <Eigen/Eigen>
#include <vector>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/CameraInfo.h>

using namespace std;
using namespace Eigen;
using namespace cv;

class mono_vo
{
private:
    int max_feat_cnt = 500;
    int min_feat_cnt = 100;
    int min_track_cnt = 50;
    int min_feat_dist = 10;
    int min_disparity = 2;
    int max_epipolar = 5;
    double min_parallex = 0.1;
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
    vector<Point3f> feat3ds;
    vector<Point2f> feats;
    Mat masks;

    mono_vo();
    ~mono_vo();

    void set_camere_info(const sensor_msgs::CameraInfoConstPtr& msg);

    void mono_detect(Mat &left_img);
    void mono_visualize(Mat &left_img, vector<Point2f> &left_feats);
    void mono_visualize(Mat &left_img, Mat &right_img, vector<Point2f>&left_feats, vector<Point2f>&right_feats);

    double distance(Point2f& p1, Point2f& p2);
    double mono_parallex(vector<Point2f> &points, vector<Point2f> &points_curr, vector<uchar> &status);
    int mono_triangulate(Mat& keyframe, Mat& img);
    int mono_track(Mat &keyframe, Mat &img);

    void update(Mat& left_img);
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

void mono_vo::mono_detect(Mat &left_img)
{
    Ptr<FeatureDetector> detector = cv::ORB::create();
    vector<KeyPoint> left_keypoints;
    detector->detect(left_img, left_keypoints);
    KeyPoint::convert(left_keypoints, feats);

    left_img.copyTo(keyframe);
    feat3ds.clear();
    qk = q;
    tk = t;
    cout << "mono detect: " << qk.coeffs().transpose() << "  tk: " << t.transpose() << endl;
    cout << "feats size: " << feats.size() << endl;
    mono_visualize(left_img, feats);
}


void mono_vo::mono_visualize(Mat &left_img, vector<Point2f> &left_feats)
{
    if (feat_vis_enable)
    {
        Mat color_img = Mat(left_img.rows, left_img.cols, CV_8UC3);
        if (left_img.channels() == 3)
        {
            left_img.copyTo(color_img(Rect(0, 0, left_img.cols, left_img.rows)));
        }
        else
        {
            cvtColor(left_img, color_img, cv::COLOR_GRAY2RGB);
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

void mono_vo::mono_visualize(Mat &left_img, Mat &right_img, vector<Point2f> &left_feats, vector<Point2f> &right_feats)
{
    if (feat_vis_enable)
    {
        Mat merge_img = Mat(left_img.rows * 2, left_img.cols, CV_8UC3);
        if (left_img.channels() == 3)
        {
            left_img.copyTo(merge_img(Rect(0, 0, left_img.cols, left_img.rows)));
            right_img.copyTo(merge_img(Rect(0, left_img.rows, right_img.cols, right_img.rows)));
        }
        else
        {
            Mat left_img_color, right_img_color;
            cvtColor(left_img, left_img_color, cv::COLOR_GRAY2RGB);
            cvtColor(right_img, right_img_color, cv::COLOR_GRAY2RGB);
            left_img_color.copyTo(merge_img(Rect(0, 0, left_img.cols, left_img.rows)));
            right_img_color.copyTo(merge_img(Rect(0, left_img.rows, right_img.cols, right_img.rows)));
        }

        Scalar color = Scalar(0, 0, 255);
        Scalar left_color = Scalar(255, 0, 0);
        Scalar right_color = Scalar(0, 255, 0);
        Point2f delta(0, left_img.rows);
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

double mono_vo::distance(Point2f& p1, Point2f& p2)
{
    Point2f p = p1 - p2;
    return sqrtf(p.dot(p));
}

double mono_vo::mono_parallex(vector<Point2f> &points, vector<Point2f> &points_curr, vector<uchar>& status)
{
    int count = std::count(status.begin(), status.end(), 1);
    double parallex_sum = 0;
    for (int i = 0; i < points.size(); i++)
    {
        parallex_sum += distance(points[i], points_curr[i]);
    }
    double diagonal_len = sqrtf(keyframe.cols * keyframe.cols + keyframe.rows * keyframe.rows);
    double parallex = parallex_sum / count / diagonal_len;
    return parallex;
}

int mono_vo::mono_triangulate(Mat &keyframe, Mat &img)
{
    vector<uchar> status;
    vector<float> err;
    vector<Point2f> feats_curr;
    cv::calcOpticalFlowPyrLK(keyframe, img, feats, feats_curr, status, err);

    int count = std::count(status.begin(), status.end(), 1);
    cout << "track cnt: " << count << endl;

    if (count < min_feat_cnt)
    {
        feats.clear();//re-detect init features
        return 0;
    }

    double parallax = mono_parallex(feats, feats_curr, status);
    cout << "parallex: " << parallax << endl;
    if (parallax < min_parallex)//init feature map
    {
        return 0;
    } 

    vector<Point3f> point3ds(count);
    vector<Point2f> points(count);

    vector<Point2f> points_curr(count);
    int j = 0;
    for (auto i = 0; i < status.size(); i++)
    {
        if (status[i])
        {
            points_curr[j] = feats_curr[i];
            points[j] = feats[i];
            j++;
        }
    }

    Mat E, rvec, tvec;
    vector<uchar> inliers;

    E = findEssentialMat(points, points_curr, camera_matrix, cv::RANSAC, 0.999, 1.0, inliers);
    int ret = recoverPose(E, points, points_curr, camera_matrix, rvec, tvec, inliers);

    if (ret)
    {
        cout << "rvec: " << rvec << endl;
        cout << "tvec: " << tvec << endl;

        // Camera 1 Projection Matrix K[I|0]
        cv::Mat P1(3, 4, CV_32F, cv::Scalar(0));
        camera_matrix.copyTo(P1.rowRange(0, 3).colRange(0, 3));

        // Camera 2 Projection Matrix K[R|t]
        cv::Mat P2(3, 4, CV_32F);
        rvec.copyTo(P2.rowRange(0, 3).colRange(0, 3));
        tvec.copyTo(P2.rowRange(0, 3).col(3));
        P2 = camera_matrix * P2;

        cout << "P1: " << P1 << endl;
        cout << "P2: " << P2 << endl;

        int inlier_count = std::count(inliers.begin(), inliers.end(), 1);

        vector<Point2f> inlier_points(inlier_count);
        vector<Point2f> inlier_points_curr(inlier_count);
        int j = 0;
        for (auto i = 0; i < inliers.size(); i++)
        {
            if (inliers[i])
            {
                inlier_points_curr[j] = points_curr[i];
                inlier_points[j] = points[i];
                j++;
            }
        }

        cv::Mat points_4d;

        cv::triangulatePoints(P1, P2, inlier_points, inlier_points_curr, points_4d);
        point3ds.clear();

        for (int i = 0; i < points_4d.cols; i++)
        {
            point3ds.push_back(cv::Point3d(points_4d.at<double>(0, i) / points_4d.at<double>(3, i),
                                              points_4d.at<double>(1, i) / points_4d.at<double>(3, i),
                                              points_4d.at<double>(2, i) / points_4d.at<double>(3, i)));
        }

        Matrix3d R;
        Quaterniond dq;
        Vector3d dt;
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                R(i, j) = rvec.at<double>(i, j);
            }
            dt(i) = tvec.at<double>(i);
        }

        cout << "R: " << R << endl;
        dt = -R.transpose() * dt;
        dq = Quaterniond(R.transpose());

        cout << "delta: dq" << dq.coeffs().transpose() << "  dt: " << dt.transpose() << endl;

        cout << "keyframe: qk" << qk.coeffs().transpose() << "  tk: " << tk.transpose() << endl;

        t = tk + qk.toRotationMatrix() * dt;
        q = qk * dq;
        cout << "mono track: " << q.coeffs().transpose() << "  t: " << t.transpose() << endl;
    }


    visualize_features(img, points, points_curr, inliers);
    //return count;
    // Mat rvec, tvec;
    // Mat dR;
    // vector<uchar> inliers;
    // bool ret = cv::solvePnPRansac(point3ds, points_curr, camera_matrix, dist_coeffs,
    //                               rvec, tvec,
    //                               false, 30, 6.0, 0.99, inliers, cv::SOLVEPNP_ITERATIVE);
    // cout << "rvec: " << rvec << endl;
    // cout << "tvec: " << tvec << endl;
    // visualize_features(img, points, points_curr, inliers);
    // if (ret)
    // {
    //     cv::Rodrigues(rvec, dR);

    //     Matrix3d R;
    //     Quaterniond dq;
    //     Vector3d dt;
    //     for (int i = 0; i < 3; i++)
    //     {
    //         for (int j = 0; j < 3; j++)
    //         {
    //             R(i, j) = dR.at<double>(i, j);
    //         }
    //         dt(i) = tvec.at<double>(i);
    //     }

    //     dt = -R.transpose() * dt;
    //     dq = Quaterniond(R.transpose());
    //     t = tk + qk.toRotationMatrix() * dt;
    //     q = qk * dq;
    //     cout << "mono track: " << q.coeffs().transpose() << "  t: " << t.transpose() << endl;
    // }
    int inlier_count = std::count(inliers.begin(), inliers.end(), 1);
    return inlier_count;
}

int mono_vo::mono_track(Mat &keyframe, Mat &img)
{
    vector<uchar> status;
    vector<float> err;
    vector<Point2f> feats_curr;
    cv::calcOpticalFlowPyrLK(keyframe, img, feats, feats_curr, status, err);
    int count = std::count(status.begin(), status.end(), 1);

    vector<Point3f> point3ds(count);
    vector<Point2f> points(count);

    vector<Point2f> points_curr(count);
    int j = 0;
    for (auto i = 0; i < status.size(); i++)
    {
        if (status[i])
        {
            point3ds[j] = feat3ds[i];
            points_curr[j] = feats_curr[i];
            points[j] = feats[i];
            j++;
        }
    }
    Mat rvec, tvec;
    Mat dR;
    vector<uchar> inliers;
    bool ret = cv::solvePnPRansac(point3ds, points_curr, camera_matrix, dist_coeffs,
                                  rvec, tvec,
                                  false, 30, 6.0, 0.99, inliers, cv::SOLVEPNP_ITERATIVE);
    cout << "rvec: " << rvec << endl;
    cout << "tvec: " << tvec << endl;
    visualize_features(img, points, points_curr, inliers);
    if (ret)
    {
        cv::Rodrigues(rvec, dR);

        Matrix3d R;
        Quaterniond dq;
        Vector3d dt;
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                R(i, j) = dR.at<double>(i, j);
            }
            dt(i) = tvec.at<double>(i);
        }

        dt = -R.transpose() * dt;
        dq = Quaterniond(R.transpose());
        t = tk + qk.toRotationMatrix() * dt;
        q = qk * dq;
        cout << "stereo track: " << q.coeffs().transpose() << "  t: " << t.transpose() << endl;
    }
    int inlier_count = std::count(inliers.begin(), inliers.end(), 1);
    return inlier_count;
}

void mono_vo::update(Mat &left_img)
{
    if (feats.empty())
    {
        q = Quaterniond::Identity();
        t = Vector3d::Zero();

        mono_detect(left_img);
    } else if (feat3ds.empty())
    {
        mono_triangulate(keyframe, left_img);
    }
    else 
    {
        int inlier_count = mono_track(keyframe, left_img);
        if (inlier_count < min_feat_cnt)
        {
            mono_detect(left_img);
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
