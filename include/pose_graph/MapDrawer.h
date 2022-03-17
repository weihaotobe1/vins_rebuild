//
// Created by root on 18-5-20.
//

#ifndef VINS_PC_MAPDRAWER_H
#define VINS_PC_MAPDRAWER_H


#include <pangolin/display/opengl_render_state.h>
#include <opencv2/core/mat.hpp>
#include <string>
#include <mutex>
#include "global_param.hpp"
#include <vector>
#include <Eigen/Dense>
#include "pose_graph/utility.h"

using namespace Eigen;
class MapDrawer {
public:
    MapDrawer(const std::string &strSettingPath);



    void DrawMapPoints();
    void DrawKeyFrames();
    void DrawGrids();
    void GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M);
    void SetAllFrames(std::vector<std::pair<int, MyOdometryMsg>> frames_to_draw);
    void AddOneFrame(std::pair<int, MyOdometryMsg> oneframe_to_draw);
    void SetAllPoints(std::vector<Vector3d> points_3d);
    void SetKeyFrames(std::vector<Eigen::Vector3d> _vposes);
    void AddOneKeyFrame(Eigen::Vector3d _pose);
    float mScaleic;

private:

    float mKeyFrameSize;
    float mKeyFrameLineWidth;
    float mGraphLineWidth;
    float mPointSize;
    float mCameraSize;
    float mCameraLineWidth;
    vector<cv::Mat> cvTwcs;
    vector<cv::Mat> cvpoints_3d;
    vector<cv::Mat> cvKftwcs;

    cv::Mat mCameraPose;

    std::mutex mMutexCamera;
    std::mutex mMutexPoints;
};


#endif //VINS_PC_MAPDRAWER_H
