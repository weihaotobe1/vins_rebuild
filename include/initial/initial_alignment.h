#pragma once
#include <eigen3/Eigen/Dense>
#include <iostream>
#include "factor/imu_factor.h"
#include "pose_graph/utility.h"

#include <map>
#include "feature_manager.h"

using namespace Eigen;
using namespace std;

/// 某一帧图像信息
class ImageFrame
{
    public:
        ImageFrame(){};
        ImageFrame(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>& _points, double _t):t{_t},is_key_frame{false}
        {
            points = _points;
        };
        map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>> > > points;
        double t;
        Matrix3d R;
        Vector3d T;
        IntegrationBase *pre_integration;
        bool is_key_frame;
};

bool VisualIMUAlignment(map<double, ImageFrame> &all_image_frame, Vector3d* Bgs, Vector3d &g, VectorXd &x);
