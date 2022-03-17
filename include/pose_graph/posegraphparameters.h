#pragma once

#include "CameraFactory.h"
#include "CataCamera.h"
#include "PinholeCamera.h"
#include <eigen3/Eigen/Dense>
#include <iostream>
//#include <ros/ros.h>
//#include <sensor_msgs/Image.h>
//#include <sensor_msgs/PointCloud.h>
//#include <sensor_msgs/image_encodings.h>
//#include <cv_bridge/cv_bridge.h>

extern camodocal::CameraPtr m_camera;
extern Eigen::Vector3d tic;
extern Eigen::Matrix3d qic;
//extern ros::Publisher pub_match_img;
//extern ros::Publisher pub_match_points;
extern int VISUALIZATION_SHIFT_X;
extern int VISUALIZATION_SHIFT_Y;
extern std::string BRIEF_PATTERN_FILE;
extern std::string POSE_GRAPH_SAVE_PATH;
extern std::string VINS_RESULT_PATH;
extern int POSEROW;
extern int POSECOL;
//extern std::string VINS_RESULT_PATH;
extern int DEBUG_IMAGE;
extern int FAST_RELOCALIZATION;

void posegraphreadParameters(std::string _config_file);
