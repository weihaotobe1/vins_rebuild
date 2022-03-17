#pragma once

#include <string>
#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <opencv2/opencv.hpp>

#include "estimator.h"
#include "estimatorparameters.h"
#include "featuretrackerparameters.h"
#include "feature_tracker.h"
#include "global_param.hpp"
#include "pgsystem.h"
#include "system.h"
#include "keyframe.h"
#include "visualization2.h"
using namespace std;
class pgsystem;
class PoseGraph;
class KeyFrame;
class vinssystem
{
	public:
	vinssystem();
	
	
	
	
	pgsystem* pg_node;// 位姿图
	//from estimator_node.cpp
	Estimator estimator;

	std::condition_variable con;
	double current_time = -1;
	queue<ImuConstPtr> imu_buf;
	queue<ImgConstPtr> feature_buf;
	queue<MyPointCloudMsg> relo_buf;
	//queue<sensor_msgs::PointCloudConstPtr> relo_buf;
	int sum_of_wait = 0;

	std::mutex m_buf;
	std::mutex m_state;
	std::mutex i_buf;
	std::mutex m_estimator;

	double latest_time;
	Eigen::Vector3d tmp_P;
	Eigen::Quaterniond tmp_Q;
	Eigen::Vector3d tmp_V;
	Eigen::Vector3d tmp_Ba;
	Eigen::Vector3d tmp_Bg;
	Eigen::Vector3d acc_0;
	Eigen::Vector3d gyr_0;
	bool init_feature = 0;
	bool init_imu = 1;
	double last_imu_t = 0;
	
	//from feature_tracker_node.cpp
	vector<uchar> r_status;
    vector<float> r_err;
    //queue<sensor_msgs::ImageConstPtr> img_buf;
    std::thread t_process;


    FeatureTracker* trackerData;
    double first_image_time;
    int pub_count = 1;
    bool first_image_flag = true;
    double last_image_time = 0;
    bool init_pub = 0;
    
    void create(string _config_file);
    void inputIMU(const ImuConstPtr &imu_msg);
    void predict(const ImuConstPtr &imu_msg);
    void inputImage(cv::Mat srcImage, double _header);
    std::vector<std::pair<std::vector<ImuConstPtr>, ImgConstPtr>> getMeasurements();
    void process();
    void relocalization_callback(const MyPointCloudMsg &points_msg);
    void restart_callback();
};
