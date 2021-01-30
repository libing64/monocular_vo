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

void visualize_cloud(pcl::PointCloud<PointType>::Ptr ptr, std::string str)
{
    pcl::visualization::CloudViewer viewer(str);
    viewer.showCloud(ptr);
    while (!viewer.wasStopped())
    {
    }

}

void triangulate(MatrixXd& _x1, MatrixXd& _x2, MatrixXd& _P1, MatrixXd& _P2, MatrixXd pts3d)
{
    cv::Mat point4ds;
    // Camera 1 Projection Matrix K[I|0]
    cv::Mat P1 = Mat::zeros(3, 4, CV_64FC1);
    cv::Mat P2 = Mat::zeros(3, 4, CV_64FC1);
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            P1.at<double>(i, j) = _P1(i, j);
            P2.at<double>(i, j) = _P2(i, j);
        }
    }
    // vector<Point2f> points_prev(_x1.cols());
    // vector<Point2f> points_curr(_x2.cols());
    // for (int i = 0; i < x1.cols(); i++)
    // {
    //     Point2f p1 = Point2f(_x1(0, i), x1(1, i));
    //     Point2f p2 = Point2f(_x2(0, i), x2(1, i));
    //     points_prev[i] = p1;
    //     points_curr[i] = p2;
    // }

    Mat x1(2, _x1.cols(), CV_64FC1);
    Mat x2(2, _x2.cols(), CV_64FC1);
    for (int i = 0; i < _x1.cols(); i++)
    {
        x1.at<double>(0, i) = _x1(0, i);
        x1.at<double>(1, i) = _x1(1, i);
        x2.at<double>(0, i) = _x2(0, i);
        x2.at<double>(1, i) = _x2(1, i);
    }
    cv::triangulatePoints(P1, P2, x1, x2, point4ds);
    cout << "point4ds: " << point4ds << endl;

    cout << "point4ds size: " << point4ds.rows << "  " << point4ds.cols << endl;

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

    cout << "feat3ds: " << feat3ds << endl;
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

int main0()
{

    Matx33d K1(6137.147949, 0.000000, 644.974609,
               0.000000, 6137.147949, 573.442749,
               0.000000, 0.000000, 1.000000);
    Matx33d K2(6137.147949, 0.000000, 644.674438,
               0.000000, 6137.147949, 573.079834,
               0.000000, 0.000000, 1.000000);

    Matx34d RT1(1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1, 0);

    Matx34d RT2(0.998297, 0.0064108, -0.0579766, 143.614334,
                -0.0065818, 0.999975, -0.00275888, -5.160085,
                0.0579574, 0.00313577, 0.998314, 96.066109);




    Matx34d P1 = K1 * RT1;
    Matx34d P2 = K2 * RT2;

    float x1data[] = {438.f, 19.f};
    float x2data[] = {452.363600f, 16.452225f};
    float Xdata[] = {-81.049530f, -215.702804f, 2401.645449f};
    Mat x1(2, 1, CV_32F, x1data);
    Mat x2(2, 1, CV_32F, x2data);
    Mat res0(1, 3, CV_32F, Xdata);
    Mat res_, res;


    Mat X(4, 1, CV_64F);
    X.at<double>(0, 0) = Xdata[0];
    X.at<double>(1, 0) = Xdata[1];
    X.at<double>(2, 0) = Xdata[2];
    X.at<double>(3, 0) = 1;


    cout << "X: " << X << endl;
    cout << "P1: " << P1 << endl;
    cout << "P2: " << P2 << endl;

    Mat xx1 = P1 * X;
    Mat xx2 = P2 * X;

    cout << "xx1 : " << xx1 << endl;
    cout << "xx2: " << xx2 << endl;
    for (int i = 0; i < 3; i++)
    {
        xx1.at<double>(i, 0) /= xx1.at<double>(2, 0);
        xx2.at<double>(i, 0) /= xx2.at<double>(2, 0);
    }
    cout << "xx1 : " << xx1 << endl;
    cout << "xx2: " << xx2 << endl;

    triangulatePoints(P1, P2, x1, x2, res_);
    cv::transpose(res_, res_); // TODO cvtest (transpose doesn't support inplace)
    convertPointsFromHomogeneous(res_, res);
    res = res.reshape(1, 1);

    cout << "[2]:" << endl;
    cout << "\tres0: " << res0 << endl;
    cout << "\tres: " << res << endl;
}

int main1()
{
    double P1data[] = {250, 0, 200, 0, 0, 250, 150, 0, 0, 0, 1, 0};
    double P2data[] = {250, 0, 200, -250, 0, 250, 150, 0, 0, 0, 1, 0};
    Mat P1(3, 4, CV_64F, P1data), P2(3, 4, CV_64F, P2data);

    float x1data[] = {200.f, 0.f};
    float x2data[] = {170.f, 1.f};
    float Xdata[] = {0.f, -5.f, 25 / 3.f};
    Mat x1(2, 1, CV_32F, x1data);
    Mat x2(2, 1, CV_32F, x2data);
    Mat res0(1, 3, CV_32F, Xdata);
    Mat res_, res;

    triangulatePoints(P1, P2, x1, x2, res_);
    cv::transpose(res_, res_); // TODO cvtest (transpose doesn't support inplace)
    convertPointsFromHomogeneous(res_, res);

    res = res.reshape(1, 1);

    cout << "[1]:" << endl;
    cout << "\tres0: " << res0 << endl;
    cout << "\tres: " << res << endl;

    return 0;
}

int main()
{
    Matrix3d R = Quaterniond::Identity().toRotationMatrix();
    Vector3d t = Vector3d::Random() * 0.1;

    cout << "R: " << R << endl;
    cout << "t: " << t.transpose() << endl;

    const int N = 1000;
    MatrixXd X = MatrixXd::Random(3, N) * 10;
    for (int i = 0; i < N; i++)
    {
        X.col(i) /= X(2, i);
    }
    cout << "X: " << X << endl;
    MatrixXd X2 = R * X;
    for (int i = 0; i < X2.cols(); i++)
    {
        X2.col(i) += t;
    }


    Matrix3d K;
    K << 500, 0, 320, 0, 500, 240, 0, 0, 1;

    cout << "K: " << K << endl;
    MatrixXd Rt1 = MatrixXd::Identity(3, 4);
    MatrixXd Rt2 = MatrixXd::Identity(3, 4);
    Rt2.topLeftCorner(3, 3) = R;
    Rt2.rightCols(1) = t;

    cout << "Rt1: " << Rt1 << endl;
    cout << "Rt2: " << Rt2 << endl;

    MatrixXd P1 = K * Rt1;
    MatrixXd P2 = K * Rt2;

    cout << "P1: " << P1 << endl;
    cout << "P2: " << P2 << endl;

    MatrixXd x1 = K * X;
    MatrixXd x2 = K * X2;

    for (int i = 0; i < N; i++)
    {
        x1.col(i) /= x1(2, i);
        x2.col(i) /= x2(2, i);
    }
    cout << "x1: " << x1 << endl;
    cout << "x2: " << x2 << endl;

    MatrixXd point3ds;
    triangulate(x1, x2, P1, P2, point3ds);
    return 0;
}

