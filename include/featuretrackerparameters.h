#pragma once

#include <opencv2/highgui/highgui.hpp>
#include <iostream>

/**
 * 这个头文件以及库文件是为了读取特征处理相关的配置参数
 * */

extern int FEATROW;
extern int FEATCOL;
extern int FEATFOCAL_LENGTH;
//const int NUM_OF_CAM = 1;


extern std::string IMAGE_TOPIC;
//extern std::string IMU_TOPIC;
extern std::string FISHEYE_MASK;
extern std::vector<std::string> CAM_NAMES;
extern int MAX_CNT;
extern int MIN_DIST;
extern int FEATWINDOW_SIZE;
extern int FREQ;
extern double F_THRESHOLD;
extern int SHOW_TRACK;
extern int STEREO_TRACK;
extern int EQUALIZE;
extern int FISHEYE;
extern bool PUB_THIS_FRAME;

void featuretrackerreadParameters(std::string _config_file);
