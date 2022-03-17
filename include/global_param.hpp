#ifndef global_param_h
#define global_param_h

#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <queue>
#include <map>

using namespace Eigen;
using namespace std;
using namespace cv;

struct IMU_MSG {
    double header;
    Vector3d acc;
    Vector3d gyr;
};

struct IMG_MSG {//图像特征消息
    double header;
    map<int, Matrix<double, 7, 1>> point_clouds;
};

struct IMUMeasument {
    double a_x;
    double a_y;
    double a_z;
    double w_x;
    double w_y;
    double w_z;
};

struct IMGMeasurement{
	double header;
	cv::Mat image;
};

struct MyOdometryMsg{
	double header;
	Eigen::Vector3d mP;
	Eigen::Quaterniond mQ;
	Eigen::Vector3d mV;
	Eigen::Vector3d tic;//相机与IMU外参标定
	Eigen::Quaterniond qic;
};

struct MyPointCloudMsg{
	double header;
	vector<Vector3d> vp3;
	vector<Vector2d> vp2;
	vector<Vector2d> vuv;
	vector<int> vid;
	Eigen::Vector3d mP;
	Eigen::Quaterniond mQ;
	int index;
};

typedef IMU_MSG* ImuConstPtr;
typedef IMG_MSG* ImgConstPtr;

#endif
