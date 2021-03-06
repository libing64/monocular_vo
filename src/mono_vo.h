#ifndef __mono_vo_H
#define __mono_vo_H
#include <iostream>
#include <Eigen/Eigen>
#include <vector>
#include <map>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/CameraInfo.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/io.h>

typedef pcl::PointXYZI PointType;

using namespace std;
using namespace Eigen;
using namespace cv;

static bool CvKeyPointResponseCompare(const cv::KeyPoint &p1,
                                      const cv::KeyPoint &p2)
{
    return (p1.response > p2.response);
}

static double distance(Point2f &p1, Point2f &p2)
{
    Point2f p = p1 - p2;
    return sqrtf(p.dot(p));
}

class mono_vo
{
private:
    int max_feat_cnt = 1000;
    int min_feat_cnt = 1000;
    int min_track_cnt = 50;
    int min_feat_dist = 10;
    int min_disparity = 2;
    int max_epipolar = 5;
    bool feat_vis_enable = true;
    float vel = 0.8;//0.8 * 30fps = 24m/s
    double min_parallx = 20.0;
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
    vector<Point3f> feat3ds;

    bool vis_enable = false;

    mono_vo();
    ~mono_vo();

    void set_camere_info(const sensor_msgs::CameraInfoConstPtr& msg);
    Mat projection_matrix(Quaterniond q, Vector3d t, Mat camera_matrix);

    void mono_detect(Mat &img);
    void mono_visualize(Mat &img, vector<Point2f> &left_feats);
    void visualize_features(Mat &img, vector<Point2f> &feats, vector<Point2f> &feats_prev);
    void mono_visualize(Mat &img, Mat &right_img, vector<Point2f>&left_feats, vector<Point2f>&right_feats);

    void remove_outliers_with_F(vector<Point2f> &feats_prev, vector<Point2f> &feats_curr);
    void remove_outliers_with_F(vector<Point2f> &feats_prev, vector<Point2f> &feats_curr, vector<Point3f>& feat3ds);

    void remove_outliers_with_E(vector<Point2f> &feats_prev, vector<Point2f> &feats_curr);
    void remove_outliers_with_E(vector<Point2f> &feats_prev, vector<Point2f> &feats_curr, vector<Point3f> &feat3ds);

    void remove_invalid(vector<Point2f> &feats_prev, vector<Point2f> &feats_curr, vector<uchar>& status);
    void remove_invalid(vector<Point2f> &feats_prev, vector<Point2f> &feats_curr, vector<Point3f>& feat3ds, vector<uchar> &status);

    int mono_track(Mat &keyframe, Mat &img);

    void update(Mat& img);
    void visualize_features(Mat &img, vector<Point2f> &feats, vector<Point2f> &feats_prev, vector<uchar> &status);


    //add mapping
    int mono_track(Mat &keyframe, Mat &img, vector<Point2f> &feats_prev, vector<Point2f> &feats_curr);
    int mono_track(Mat &keyframe, Mat &img, vector<Point2f> &feats_prev, vector<Point2f> &feats_curr, vector<Point3f> &feat3ds_prev);
    bool add_keyframe(Mat& img);

    int mono_register(Mat &keyframe, Mat &img, vector<Point2f> &feats_prev, vector<Point2f> &feats_curr, vector<Point3f> &feat3ds_prev, Quaterniond qk, Vector3d tk, Quaterniond q, Vector3d t);
    double mono_parallex(vector<Point2f> &points, vector<Point2f> &points_curr, vector<uchar> &status);
    double mono_parallex(vector<Point2f> &points, vector<Point2f> &points_curr);
    bool is_keyframe();
    void Triangulate(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const cv::Mat &P1, const cv::Mat &P2, cv::Mat &x3D);
    void map_init(vector<Point2f>& feats_prev, vector<Point2f>& feats_curr, vector<Point3f>& feat3ds, Quaterniond& q_prev, Vector3d& t_prev, Quaterniond& q_curr, Vector3d& t_curr);
    void map_reset();
    void visualize_cloud(pcl::PointCloud<PointType>::Ptr ptr, std::string str);
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

Mat mono_vo::projection_matrix(Quaterniond q, Vector3d t, Mat camera_matrix)
{
    cv::Mat Rt1 = Mat::eye(3, 4, CV_64FC1);
    // rvec.copyTo(Rt1.rowRange(0, 3).colRange(0, 3));
    // tvec.copyTo(Rt1.rowRange(0, 3).col(3));

    //take care!! x ~ P[R t] * X; here is n->b
    Vector3d t1 = -q.inverse().toRotationMatrix() * t;
    MatrixXd R1 = q.inverse().toRotationMatrix();
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            Rt1.at<double>(i, j) = R1(i, j);
        }
        Rt1.at<double>(i, 3) = t1(i);
    }
    cout << "Rt1: " << Rt1 << endl;

    // Camera 1 Projection Matrix K[I|0]
    cv::Mat P1 = camera_matrix * Rt1;
    cout << "P1: " << P1 << endl;
    return P1;
}

void mono_vo::mono_detect(Mat &img)
{
    printf("func: %s, %d\n", __FUNCTION__, __LINE__);
    vector<KeyPoint> keypoints;
    int thresh = 10;
    Mat mask = cv::Mat(img.size(), CV_8UC1, cv::Scalar(255));
    Ptr<FastFeatureDetector> detector = FastFeatureDetector::create(thresh);
    detector->detect(img, keypoints, mask);
    sort(keypoints.begin(), keypoints.end(), CvKeyPointResponseCompare);
    for (int i = 0; i < keypoints.size(); i++)
    {
        if (feats.size() < max_feat_cnt && mask.at<unsigned char>(keypoints[i].pt.y, keypoints[i].pt.x))
        {
            feats.push_back(keypoints[i].pt);
            circle(mask, keypoints[i].pt, min_feat_dist, cv::Scalar(0), cv::FILLED);
        }
    }

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

void mono_vo::remove_outliers_with_F(vector<Point2f> &feats_prev, vector<Point2f> &feats_curr)
{
    vector<uchar> status;
    Mat F = findFundamentalMat(feats_prev, feats_curr, FM_RANSAC, 1.0, 0.99, status);
    remove_invalid(feats_prev, feats_curr, status);
    cout << "inlier percent: " << (feats_prev.size() * 1.0) / status.size() << endl;
}

void mono_vo::remove_outliers_with_F(vector<Point2f> &feats_prev, vector<Point2f> &feats_curr, vector<Point3f>& feat3ds)
{
    vector<uchar> status;
    Mat F = findFundamentalMat(feats_prev, feats_curr, FM_RANSAC, 1.0, 0.99, status);
    remove_invalid(feats_prev, feats_curr, feat3ds, status);
    cout << "inlier percent: " << (feats_prev.size() * 1.0) / status.size() << endl;
}

void mono_vo::remove_outliers_with_E(vector<Point2f> &feats_prev, vector<Point2f> &feats_curr)
{
    vector<uchar> status;
    Mat E = findEssentialMat(feats_prev, feats_curr, camera_matrix, RANSAC, 0.99, 1.0, status);
    remove_invalid(feats_prev, feats_curr, status);
    cout << "inlier percent: " << (feats_prev.size() * 1.0) / status.size() << endl;
}

void mono_vo::remove_outliers_with_E(vector<Point2f> &feats_prev, vector<Point2f> &feats_curr, vector<Point3f> &feat3ds)
{
    vector<uchar> status;
    Mat E = findEssentialMat(feats_prev, feats_curr, camera_matrix, RANSAC, 0.99, 1.0, status);
    remove_invalid(feats_prev, feats_curr, feat3ds, status);
    cout << "inlier percent: " << (feats_prev.size() * 1.0) / status.size() << endl;
}

void mono_vo::remove_invalid(vector<Point2f> &feats_prev, vector<Point2f> &feats_curr, vector<uchar> &status)
{
    int j = 0;
    for (int i = 0; i < status.size(); i++)
    {
        if (status[i])
        {
            if (i != j)
            {
                feats_prev[j] = feats_prev[i];
                feats_curr[j] = feats_curr[i];
            }
            j++;
        }
    }
    feats_prev.resize(j);
    feats_curr.resize(j);
}

void mono_vo::remove_invalid(vector<Point2f> &feats_prev, vector<Point2f> &feats_curr, vector<Point3f>& feat3ds, vector<uchar> &status)
{
    int j = 0;
    for (int i = 0; i < status.size(); i++)
    {
        if (status[i])
        {
            if (i != j)
            {
                feats_prev[j] = feats_prev[i];
                feats_curr[j] = feats_curr[i];
                feat3ds[j] = feat3ds[i];
            }
            j++;
        }
    }
    feats_prev.resize(j);
    feats_curr.resize(j);
    feat3ds.resize(j);
}

int mono_vo::mono_track(Mat &keyframe, Mat &img)
{
    vector<uchar> status;
    vector<float> err;
    vector<Point2f> feats_prev = feats;
    vector<Point2f> feats_curr;
    cv::calcOpticalFlowPyrLK(keyframe, img, feats_prev, feats_curr, status, err);
    int count = std::count(status.begin(), status.end(), 1);


    remove_invalid(feats_prev, feats_curr, status);

    remove_outliers_with_F(feats_prev, feats_curr);
    Mat E, rvec, tvec;
    Mat dR;
    vector<uchar> inliers;

    E = findEssentialMat(feats_prev, feats_curr, camera_matrix, RANSAC, 0.99, 1.0, inliers); //threshold!!!
    int ret = recoverPose(E, feats_prev, feats_curr, camera_matrix, rvec, tvec, inliers);

    cout << "rvec: " << rvec << endl;
    cout << "tvec: " << tvec << endl;
    visualize_features(img, feats_prev, feats_curr, inliers);

    int inlier_count = std::count(inliers.begin(), inliers.end(), 1);
    double inlier_rate = 1.0 * inlier_count / inliers.size();
    cout << "inlier rate: " << inlier_rate << endl;

    if (ret && inlier_rate > 0.2)
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

        dt = Vector3d(0, 0, vel);
        dq = Quaterniond(R.transpose());
        t = t + q.toRotationMatrix() * dt;
        q = qk * dq;
        cout << "mono track: " << q.coeffs().transpose() << endl << "t: " << t.transpose() << endl;
    }

    return inlier_count;
}

int mono_vo::mono_track(Mat &keyframe, Mat &img, vector<Point2f>& feats_prev, vector<Point2f>& feats_curr, vector<Point3f>& feat3ds_prev)
{
    vector<uchar> status;
    vector<float> err;
    cv::calcOpticalFlowPyrLK(keyframe, img, feats_prev, feats_curr, status, err);
    int count = std::count(status.begin(), status.end(), 1);

    remove_invalid(feats_prev, feats_curr, feat3ds_prev, status);

    //remove outliers wit F matrix
    status.clear();
    Mat F = findFundamentalMat(feats_prev, feats_curr, FM_RANSAC, 1.0, 0.99, status);
    remove_invalid(feats_prev, feats_curr, feat3ds_prev, status);

    cout << "feats_prev size: " << feats_prev.size() << endl;
    cout << "feats_curr size: " << feats_curr.size() << endl;
    cout << "feat3ds_prev size: " << feat3ds_prev.size() << endl;

    return feats_curr.size();
}

int mono_vo::mono_track(Mat &keyframe, Mat &img, vector<Point2f> &feats_prev, vector<Point2f> &feats_curr)
{

    printf("func: %s, %d\n", __FUNCTION__, __LINE__);

    vector<uchar> status;
    vector<float> err;
    cv::calcOpticalFlowPyrLK(keyframe, img, feats_prev, feats_curr, status, err);
    int count = std::count(status.begin(), status.end(), 1);

    remove_invalid(feats_prev, feats_curr, status);
    //remove outliers wit F matrix
    status.clear();
    Mat F = findFundamentalMat(feats_prev, feats_curr, FM_RANSAC, 1.0, 0.99, status);

    visualize_features(img, feats_prev, feats_curr, status);
    remove_invalid(feats_prev, feats_curr, status);

    cout << "feats_prev size: " << feats_prev.size() << endl;
    cout << "feats_curr size: " << feats_curr.size() << endl;

    return feats_curr.size();
}

int mono_vo::mono_register(Mat &keyframe, Mat &img, vector<Point2f> &feats_prev, vector<Point2f> &feats_curr, vector<Point3f> &feat3ds_prev, Quaterniond qk, Vector3d tk, Quaterniond q, Vector3d t)
{
    printf("func: %s, %d\n", __FUNCTION__, __LINE__);

    //1. mask
    //add mask and detect new features
    Mat mask = cv::Mat(img.size(), CV_8UC1, cv::Scalar(255));
    for (int i = 0; i < feats_prev.size(); i++)
    {
        circle(mask, feats_prev[i], min_feat_dist, cv::Scalar(0), cv::FILLED);
    }

    //2. detect and tracking new features
    vector<KeyPoint> keypoints;
    int thresh = 10;
    Ptr<FastFeatureDetector> detector = FastFeatureDetector::create(thresh);
    detector->detect(keyframe, keypoints, mask);

    vector<Point2f> points_prev, points_curr;
    sort(keypoints.begin(), keypoints.end(), CvKeyPointResponseCompare);
    for (int i = 0; i < keypoints.size(); i++)
    {
        if (points_prev.size() < (max_feat_cnt - feats_prev.size()) && mask.at<unsigned char>(keypoints[i].pt.y, keypoints[i].pt.x))
        {
            points_prev.push_back(keypoints[i].pt);
            circle(mask, keypoints[i].pt, min_feat_dist, cv::Scalar(0), cv::FILLED);
        }
    }

    vector<uchar> status;
    vector<float> err;
    cv::calcOpticalFlowPyrLK(keyframe, img, points_prev, points_curr, status, err);
    remove_invalid(points_prev, points_curr, status);


    //3. triangulate new features
    Mat P1 = projection_matrix(qk, tk, camera_matrix);
    Mat P2 = projection_matrix(q, t, camera_matrix);

    int inlier_count = points_prev.size();

    Mat inlier_points_prev(2, inlier_count, CV_64FC1);
    Mat inlier_points_curr(2, inlier_count, CV_64FC1);
    int j = 0;
    for (int i = 0; i < inlier_count; i++)
    {
        inlier_points_prev.at<double>(0, i) = points_prev[i].x;
        inlier_points_prev.at<double>(1, i) = points_prev[i].y;
        inlier_points_curr.at<double>(0, i) = points_curr[i].x;
        inlier_points_curr.at<double>(1, i) = points_curr[i].y;
    }

    Mat point4ds;
    cv::triangulatePoints(P1, P2, inlier_points_prev, inlier_points_curr, point4ds);
    //cout << "point4ds: " << point4ds << endl;

    //cout << "point4ds size: " << point4ds.rows << "  " << point4ds.cols << endl;

    vector<Point3f> feat3ds;
    feat3ds.clear();
    for (int i = 0; i < point4ds.cols; i++)
    {
        Point3f p;
        p.x = point4ds.at<double>(0, i) / point4ds.at<double>(3, i);
        p.y = point4ds.at<double>(1, i) / point4ds.at<double>(3, i);
        p.z = point4ds.at<double>(2, i) / point4ds.at<double>(3, i);
        feat3ds.push_back(p);
    }

    //4. visualize
    //cout << "feat3ds: " << feat3ds << endl;
    //visualize cloud
    pcl::PointCloud<PointType> cloud;
    for (int i = 0; i < feat3ds.size(); i++)
    {
        PointType p;
        p.x = feat3ds[i].x;
        p.y = feat3ds[i].y;
        p.z = feat3ds[i].z;
        p.intensity = i;
        cloud.points.push_back(p);
    }
    cout << "feat3ds size: " << feat3ds.size() << endl;
    visualize_cloud(cloud.makeShared(), "feat3ds new");

    //5. add new features to map
    for (int i = 0; i < inlier_points_curr.cols; i++)
    {
        Point2f p1, p2;
        p1.x = inlier_points_prev.at<double>(0, i);
        p1.y = inlier_points_prev.at<double>(1, i);

        p2.x = inlier_points_curr.at<double>(0, i);
        p2.y = inlier_points_curr.at<double>(1, i);

        feats_prev.push_back(p1);
        feats_curr.push_back(p2);
        feat3ds_prev.push_back(feat3ds[i]);
    }
    remove_outliers_with_F(feats_prev, feats_curr, feat3ds_prev);
    remove_outliers_with_E(feats_prev, feats_curr, feat3ds_prev);
    visualize_features(img, feats_prev, feats_curr);
    //6. update keyframe

    return feats_curr.size();
}

void mono_vo::update(Mat &img)
{
    printf("func: %s, %d\n", __FUNCTION__, __LINE__);
    //feature_extract(img);
    if (feats.empty())
    {
        mono_detect(img);
        return;
    } else if (feat3ds.empty())
    {
        vector<Point2f> feats_prev = feats;
        vector<Point2f> feats_curr;
    
        int tracking_count = mono_track(keyframe, img, feats_prev, feats_curr);
        printf("tracking feat cnt: %d\n", tracking_count);
        //whether keyframe
        double parallax = mono_parallex(feats_prev, feats_curr);
        printf("parallax: %lf\n", parallax);

        if (parallax > min_parallx)
        {
            qk = q;
            tk = t;
            vector<Point3f> feat3ds_curr;
            map_init(feats_prev, feats_curr, feat3ds_curr, qk, tk, q, t);
            img.copyTo(keyframe);
            feats = feats_curr;
            feat3ds = feat3ds_curr;
        } else 
        {
            cout << "no enough parallex for map init" << endl;
        }
    } else
    {

        vector<Point2f> feats_prev = feats;
        vector<Point3f> feat3ds_prev = feat3ds;
        vector<Point2f> feats_curr;

        int tracking_count = mono_track(keyframe, img, feats_prev, feats_curr, feat3ds_prev);
        printf("tracking feat cnt: %d\n", tracking_count);
        // cout << "feats_prev size: " << feats_prev.size() << endl;
        // cout << "feats_curr size: " << feats_curr.size() << endl;
        // cout << "feat3ds_prev size: " << feat3ds_prev.size() << endl;

        if (tracking_count < min_track_cnt)
        {
            map_reset();
            return;
        }

        printf("tracking feat cnt: %d\n", tracking_count);
        //whether keyframe
        double parallax = mono_parallex(feats_prev, feats_curr);
        printf("parallax: %lf\n", parallax);
        //1. update pose
        Mat rvec, tvec;
        Mat dR;
        vector<uchar> inliers;
        bool ret = cv::solvePnPRansac(feat3ds_prev, feats_curr, camera_matrix, dist_coeffs,
                                      rvec, tvec,
                                      false, 30, 6.0, 0.95, inliers, cv::SOLVEPNP_ITERATIVE);


        if (ret)
        {
            printf("func: %s, %d\n", __FUNCTION__, __LINE__);

            cout << "rvec: " << rvec << endl;
            cout << "tvec: " << tvec << endl;
            cv::Rodrigues(rvec, dR);

            Matrix3d R;
            Quaterniond dq;
            Vector3d tt;
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    R(i, j) = dR.at<double>(i, j);
                }
                tt(i) = tvec.at<double>(i);
            }

            Vector3d tc = -R.transpose() * tt;
            Quaterniond qc = Quaterniond(R.transpose());
            double angle = qc.angularDistance(q);
            double dist = (tc - t).norm();

            if (angle < (90.0 * M_PI / 180.0) && dist < 10.0)
            {
                t = -R.transpose() * tt;
                q = Quaterniond(R.transpose());
            } else 
            {
                cout << "fail for solvePnP" << endl;
                map_reset();
                return;
            }
            cout << "mono track: " << q.coeffs().transpose() << "  t: " << t.transpose() << endl;
        } else 
        {
            cout << "fail for solvePnP" << endl;
            map_reset();
            return;
        }
        int inlier_count = std::count(inliers.begin(), inliers.end(), 1);

        //2. if parallax > min_parallex, insert new features to map
        if (parallax > min_parallx)
        {
            printf("func: %s, %d\n", __FUNCTION__, __LINE__);
            cout << "update keyframe" << endl;
            //register new frame as keyframe, and insert new Points to map
            mono_register(keyframe, img, feats_prev, feats_curr, feat3ds_prev, qk, tk, q, t);

            //update keyframe
            img.copyTo(keyframe);
            feats = feats_curr;
            feat3ds = feat3ds_prev;
            qk = q;
            tk = t;
        } else 
        {

        }

    }
}

 void mono_vo::map_init(vector<Point2f>& feats_prev, vector<Point2f>& feats_curr, vector<Point3f>& feat3ds, 
               Quaterniond& q_prev, Vector3d& t_prev, 
               Quaterniond& q_curr, Vector3d& t_curr)
 {
     Mat E, rvec, tvec;
     Mat dR;
     vector<uchar> inliers;

    E = findEssentialMat(feats_prev, feats_curr, camera_matrix, RANSAC, 0.99, 1.0, inliers);
    int inlier_count = std::count(inliers.begin(), inliers.end(), 1);
    double inlier_rate = 1.0 * inlier_count / inliers.size();



     //cout << "rvec: " << rvec << endl;
     //cout << "tvec: " << tvec << endl;


     //cout << "inlier rate: " << inlier_rate << endl;

     if (inlier_rate > 0.2 && inlier_count >= 20)
     {
         int ret = recoverPose(E, feats_prev, feats_curr, camera_matrix, rvec, tvec, inliers);

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
        dt = -R.transpose() * dt;
        dq = Quaterniond(R.transpose());


        q = qk * dq;
        t = tk + qk.toRotationMatrix() * dt;

        int inlier_count = std::count(inliers.begin(), inliers.end(), 1);

        Mat inlier_points_prev(2, inlier_count, CV_64FC1);
        Mat inlier_points_curr(2, inlier_count, CV_64FC1);
        int j = 0;
        for (int i = 0; i < inliers.size(); i++)
        {
            if (inliers[i])
            {
                inlier_points_prev.at<double>(0, j) = feats_prev[i].x;
                inlier_points_prev.at<double>(1, j) = feats_prev[i].y;

                inlier_points_curr.at<double>(0, j) = feats_curr[i].x;
                inlier_points_curr.at<double>(1, j) = feats_curr[i].y;
                j++;
            }
        }


        cv::Mat point4ds;
        cv::Mat Rt0 = Mat::eye(3, 4, CV_64FC1);
        cv::Mat Rt1 = Mat::eye(3, 4, CV_64FC1);
        // rvec.copyTo(Rt1.rowRange(0, 3).colRange(0, 3));
        // tvec.copyTo(Rt1.rowRange(0, 3).col(3));


        //take care!! x ~ P[R t] * X; here is n->b
        Vector3d t0 = -qk.inverse().toRotationMatrix() * tk;
        Vector3d t1 = -q.inverse().toRotationMatrix() * t;
        MatrixXd R0 = qk.inverse().toRotationMatrix();
        MatrixXd R1 = q.inverse().toRotationMatrix();
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                Rt0.at<double>(i, j) = R0(i, j);
                Rt1.at<double>(i, j) = R1(i, j);
            }
            Rt0.at<double>(i, 3) = t0(i);
            Rt1.at<double>(i, 3) = t1(i);
        }
        cout << "Rt0: " << Rt0 << endl;
        cout << "Rt1: " << Rt1 << endl;


        // Camera 1 Projection Matrix K[I|0]
        cv::Mat P1 = camera_matrix * Rt0;
        cv::Mat P2 = camera_matrix * Rt1;
        cout << "P1: " << P1 << endl;
        cout << "P2: " << P2 << endl;

        cv::triangulatePoints(P1, P2, inlier_points_prev, inlier_points_curr, point4ds);
        //cout << "point4ds: " << point4ds << endl;

        //cout << "point4ds size: " << point4ds.rows << "  " << point4ds.cols << endl;

        feat3ds.clear();
        for (int i = 0; i < point4ds.cols; i++)
        {
            Point3f p;
            p.x = point4ds.at<double>(0, i) / point4ds.at<double>(3, i);
            p.y = point4ds.at<double>(1, i) / point4ds.at<double>(3, i);
            p.z = point4ds.at<double>(2, i) / point4ds.at<double>(3, i);
            feat3ds.push_back(p);
        }
        remove_invalid(feats_prev, feats_curr, inliers);
        
        printf("map init size: %d, %d, %d\n", feats_prev.size(), feats_curr.size(), feat3ds.size());

        //cout << "feat3ds: " << feat3ds << endl;
        //visualize cloud
        pcl::PointCloud<PointType> cloud;
        for (int i = 0; i < feat3ds.size(); i++)
        {
            PointType p;
            p.x = feat3ds[i].x;
            p.y = feat3ds[i].y;
            p.z = feat3ds[i].z;
            p.intensity = i;
            cloud.points.push_back(p);
        }
        cout << "feat3ds size: " << feat3ds.size() << endl;
        visualize_cloud(cloud.makeShared(), "feat3ds");
     }


 }

 void mono_vo::map_reset()
 {
     static int reset_cnt = 0;
     reset_cnt++;
     cout << "reset_cnt: " << reset_cnt << endl;
     feat3ds.clear();
     feats.clear();
     qk = q;
     tk = t;
 }

 void mono_vo::visualize_cloud(pcl::PointCloud<PointType>::Ptr ptr, std::string str)
 {
     if (vis_enable)
     {
         pcl::visualization::CloudViewer viewer(str);
         viewer.showCloud(ptr);
         while (!viewer.wasStopped())
         {
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

void mono_vo::visualize_features(Mat &img, vector<Point2f> &feats, vector<Point2f> &feats_prev)
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
            line(img_color, feats[i], feats_prev[i], color, 1, 8);
            circle(img_color, feats[i], 1, color);
            circle(img_color, feats_prev[i], 2, color_prev);
            circle(img_color, feats[i], 2, color_next);
        }
        imshow("feats", img_color);
        waitKey(2);
    }
}

double mono_vo::mono_parallex(vector<Point2f> &points, vector<Point2f> &points_curr, vector<uchar> &status)
{
    if (points.size() < 10)
    {
        return 0.0;
    }
    int count = std::count(status.begin(), status.end(), 1);
    double parallex_sum = 0;
    for (int i = 0; i < points.size(); i++)
    {
        if (status[i])
            parallex_sum += distance(points[i], points_curr[i]);
    }
    double parallex = parallex_sum / count;
    return parallex;
}

double mono_vo::mono_parallex(vector<Point2f> &points, vector<Point2f> &points_curr)
{
    if (points.size() < 10)
    {
        return 0.0;
    }
    int count = points.size();
    double parallex_sum = 0;
    for (int i = 0; i < points.size(); i++)
    {
        parallex_sum += distance(points[i], points_curr[i]);
    }
    double parallex = parallex_sum / count;
    return parallex;
}

void mono_vo::Triangulate(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const cv::Mat &P1, const cv::Mat &P2, cv::Mat &x3D)
{
    cv::Mat A(4, 4, CV_32F);

    A.row(0) = kp1.pt.x * P1.row(2) - P1.row(0);
    A.row(1) = kp1.pt.y * P1.row(2) - P1.row(1);
    A.row(2) = kp2.pt.x * P2.row(2) - P2.row(0);
    A.row(3) = kp2.pt.y * P2.row(2) - P2.row(1);

    cv::Mat u, w, vt;
    cv::SVD::compute(A, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
    x3D = vt.row(3).t();
    x3D = x3D.rowRange(0, 3) / x3D.at<float>(3);
}

#endif
