#pragma once

#include <vector>

#include <iostream>

#include <mutex>
#include <queue>
#include <thread>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include "keyframe.h"
#include "tic_toc.h"
#include "pose_graph.h"
#include "global_param.hpp"
#include "posegraphparameters.h"
#include "system.h"
#include "keyframe.h"
#include "pose_graph.h"
#include "MapDrawer.h"
#include <pangolin/display/display.h>
#include <pangolin/var/var.h>
#include <pangolin/pangolin.h>
#define SKIP_FIRST_CNT 10
using namespace std;
class vinssystem;
class PoseGraph;
class KeyFrame;
class pgsystem
{
	public:
	pgsystem();
	queue<IMGMeasurement> image_buf;
	queue<MyPointCloudMsg> point_buf;
	queue<MyOdometryMsg> pose_buf;
	queue<Eigen::Vector3d> odometry_buf;
	std::mutex m_buf;
	std::mutex m_process;
	std::mutex m_imagetext;
	std::mutex m_strpose;
	int frame_index  = 0;
	int sequence = 1;
	PoseGraph* posegraph;
	MapDrawer* mpmapdrawer;
	int skip_first_cnt = 0;
	int SKIP_CNT;
	int skip_cnt = 0;
	bool load_flag = 0;
	bool start_flag = 0;
	double SKIP_DIS = 0;
    std::thread measurement_process;
    std::thread draw_process;


	vinssystem* mpvinssystem;

	int VISUALIZE_IMU_FORWARD;
	int LOOP_CLOSURE;

	std::stringstream s;
	cv::Mat mimText;
	double mT;
	
	Eigen::Vector3d last_t;
	double last_image_time = -1;
	
	void create(string _config_file, vinssystem* _pvinssystem);
	void process();
	void image_callback(const cv::Mat &image, double header);
	void point_callback(const MyPointCloudMsg &point_msg);
	void pose_callback(const MyOdometryMsg &pose_msg);
	void imu_forward_callback(const MyOdometryMsg &forward_msg);
	void relo_relative_pose_callback(const MyOdometryMsg &pose_msg);
	void vio_callback(const MyOdometryMsg &pose_msg);
	void extrinsic_callback(const MyOdometryMsg &pose_msg);
	void show_callback(const cv::Mat &image);
	void Run();
	void new_sequence();
};
