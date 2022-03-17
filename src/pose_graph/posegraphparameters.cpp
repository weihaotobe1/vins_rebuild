#include "posegraphparameters.h"

camodocal::CameraPtr m_camera;
Eigen::Vector3d tic;
Eigen::Matrix3d qic;

int VISUALIZATION_SHIFT_X;
int VISUALIZATION_SHIFT_Y;
std::string BRIEF_PATTERN_FILE;
std::string POSE_GRAPH_SAVE_PATH;

int POSEROW;
int POSECOL;
std::string VINS_RESULT_PATH;
int DEBUG_IMAGE;
int FAST_RELOCALIZATION;

void posegraphreadParameters(std::string _config_file)
{
	std::string config_file;
    config_file = _config_file;
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cout << "ERROR: Wrong path to settings" << std::endl;
    }
    
    m_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(config_file.c_str());
    VISUALIZATION_SHIFT_X = 0;
    VISUALIZATION_SHIFT_Y = 0;
    
    int tmploop_closure;
    tmploop_closure = fsSettings["loop_closure"];
    if(tmploop_closure)
    {
		POSEROW = fsSettings["image_height"];
        POSECOL = fsSettings["image_width"];
        
        BRIEF_PATTERN_FILE = std::string("../support_files/brief_pattern.yml");
        fsSettings["pose_graph_save_path"] >> POSE_GRAPH_SAVE_PATH;
        //fsSettings["output_path"] >> VINS_RESULT_PATH;
        fsSettings["save_image"] >> DEBUG_IMAGE;
        FAST_RELOCALIZATION = fsSettings["fast_relocalization"];
	}
}
