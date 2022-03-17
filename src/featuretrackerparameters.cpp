#include "featuretrackerparameters.h"

std::string IMAGE_TOPIC;
//std::string IMU_TOPIC;
std::vector<std::string> CAM_NAMES;
std::string FISHEYE_MASK;
int MAX_CNT;
int MIN_DIST;
int FEATWINDOW_SIZE;
int FREQ;
double F_THRESHOLD;
int SHOW_TRACK;
int STEREO_TRACK;
int EQUALIZE;
int FEATROW;
int FEATCOL;
int FEATFOCAL_LENGTH;
int FISHEYE;
bool PUB_THIS_FRAME;



void featuretrackerreadParameters(std::string _config_file)
{
    std::string config_file;
    config_file = _config_file;
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cout << "ERROR: Wrong path to settings" << std::endl;
    }
    std::string VINS_FOLDER_PATH = "";

    fsSettings["image_topic"] >> IMAGE_TOPIC;
    //fsSettings["imu_topic"] >> IMU_TOPIC;
    MAX_CNT = fsSettings["max_cnt"];
    MIN_DIST = fsSettings["min_dist"];
    FEATROW = fsSettings["image_height"];
    FEATCOL = fsSettings["image_width"];
    FREQ = fsSettings["freq"];
    F_THRESHOLD = fsSettings["F_threshold"];
    SHOW_TRACK = fsSettings["show_track"];
    EQUALIZE = fsSettings["equalize"];
    FISHEYE = fsSettings["fisheye"];
    if (FISHEYE == 1)
        FISHEYE_MASK = VINS_FOLDER_PATH + "config/fisheye_mask.jpg";
    CAM_NAMES.push_back(config_file);

    FEATWINDOW_SIZE = 20;
    STEREO_TRACK = false;
    FEATFOCAL_LENGTH = 460;
    PUB_THIS_FRAME = false;

    if (FREQ == 0)
        FREQ = 100;

    fsSettings.release();


}
