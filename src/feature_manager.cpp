#include "feature_manager.h"

int FeaturePerId::endFrame()
{
    return start_frame + feature_per_frame.size() - 1;
}

FeatureManager::FeatureManager(Matrix3d _Rs[])
    : Rs(_Rs)
{
    for (int i = 0; i < NUM_OF_CAM; i++)
        ric[i].setIdentity();
}

/// 特征数据库设置相机和imu之间的旋转
void FeatureManager::setRic(Matrix3d _ric[])
{
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ric[i] = _ric[i];
    }
}

/// 清除所有的特征
void FeatureManager::clearState()
{
    feature.clear();
}

/// 统计特征点跟踪帧数大于等于4的特征？
int FeatureManager::getFeatureCount()
{
    int cnt = 0;
    for (auto &it : feature)
    {//
        it.used_num = it.feature_per_frame.size();

        if (it.used_num >= 2 && it.start_frame < ESTWINDOW_SIZE - 2)
        {
//            std::cout << "used num : " << it.used_num << ", it.start_frame : " << it.start_frame << std::endl;
            cnt++;
        }
    }
    return cnt;
}

/// 关键帧判断，开源代码中视差计算实际上不包含旋转补偿,直接计算倒数第二帧与倒数第三帧之间的norm坐标系差值
bool FeatureManager::addFeatureCheckParallax(int frame_count, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, double td)
{
    printf("input feature: %d", (int)image.size());
    printf("num of feature: %d", getFeatureCount());
    double parallax_sum = 0;
    int parallax_num = 0;
    last_track_num = 0;
    for (auto &id_pts : image)
    {
        FeaturePerFrame f_per_fra(id_pts.second[0].second, td);

        int feature_id = id_pts.first;
        auto it = find_if(feature.begin(), feature.end(), [feature_id](const FeaturePerId &it)
                          {
            return it.feature_id == feature_id;
                          });

        if (it == feature.end())
        {///新增加特征到feature list中
            feature.push_back(FeaturePerId(feature_id, frame_count));//frame count 始终是10
            feature.back().feature_per_frame.push_back(f_per_fra);
        }
        else if (it->feature_id == feature_id)
        {/// 新增加观测到某一特征中
            it->feature_per_frame.push_back(f_per_fra);
            last_track_num++;
        }
    }

    /// 关键帧判断1： 如果上一帧到当前帧跟踪点数小于20，则该帧为关键帧
    if (frame_count < 2 || last_track_num < 20)
        return true;

    /// 关键帧判断2： 计算视差
    for (auto &it_per_id : feature)
    {
        // 第一个判断表示该特征包含倒数第三帧，第二个判断表示该特征必须包含倒数第二帧的观测
        if (it_per_id.start_frame <= frame_count - 2 &&
            it_per_id.start_frame + int(it_per_id.feature_per_frame.size()) - 1 >= frame_count - 1)
        {
            parallax_sum += compensatedParallax2(it_per_id, frame_count);
            parallax_num++;
        }
    }

    if (parallax_num == 0)
    {
        return true;
    }
    else
    {
        printf("parallax_sum: %lf, parallax_num: %d", parallax_sum, parallax_num);
        printf("current parallax: %lf", parallax_sum / parallax_num * ESTFOCAL_LENGTH);
        return parallax_sum / parallax_num >= MIN_PARALLAX;///视察足够大，则说明倒数第二帧为关键帧？
    }
}

void FeatureManager::debugShow()
{
    printf("debug show");
    for (auto &it : feature)
    {
        assert(it.feature_per_frame.size() != 0);
        assert(it.start_frame >= 0);
        assert(it.used_num >= 0);

        printf("%d,%d,%d ", it.feature_id, it.used_num, it.start_frame);
        int sum = 0;
        for (auto &j : it.feature_per_frame)
        {
            printf("%d,", int(j.is_used));
            sum += j.is_used;
            printf("(%lf,%lf) ",j.point(0), j.point(1));
        }
        assert(it.used_num == sum);
    }
}

/// 返回第l帧和第r的特征点的归一化坐标值
vector<pair<Vector3d, Vector3d>> FeatureManager::getCorresponding(int frame_count_l, int frame_count_r)
{
    vector<pair<Vector3d, Vector3d>> corres;
    for (auto &it : feature)
    {
        if (it.start_frame <= frame_count_l && it.endFrame() >= frame_count_r)
        {
            Vector3d a = Vector3d::Zero(), b = Vector3d::Zero();
            int idx_l = frame_count_l - it.start_frame;
            int idx_r = frame_count_r - it.start_frame;

            a = it.feature_per_frame[idx_l].point;

            b = it.feature_per_frame[idx_r].point;
            
            corres.push_back(make_pair(a, b));
        }
    }
    return corres;
}

/// 给特征设置深度值 (如何保证x的维度和特征的数量相等？)
void FeatureManager::setDepth(const VectorXd &x)
{
    int feature_index = -1;
    for (auto &it_per_id : feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < ESTWINDOW_SIZE - 2))
            continue;//这一部分特征会被进行赋值

        it_per_id.estimated_depth = 1.0 / x(++feature_index);
        //ROS_INFO("feature id %d , start_frame %d, depth %f ", it_per_id->feature_id, it_per_id-> start_frame, it_per_id->estimated_depth);
        if (it_per_id.estimated_depth < 0)
        {
            it_per_id.solve_flag = 2;//由于特征表示在首次观测的相机坐标系下，因此深度值不能为负数
        }
        else
            it_per_id.solve_flag = 1;
    }
}

/// 删除三角化失败的特征
void FeatureManager::removeFailures()
{
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;
        if (it->solve_flag == 2)
            feature.erase(it);
    }
}

/// 将特征点的深度设为一个统一的值(与setDepth只是少了一个solve_flag的设置)
void FeatureManager::clearDepth(const VectorXd &x)
{
    int feature_index = -1;
    for (auto &it_per_id : feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < ESTWINDOW_SIZE - 2))
            continue;
        it_per_id.estimated_depth = 1.0 / x(++feature_index);
    }
}

/// 获取所有特征的深度值(特征点深度在最开开始设置为-1.0)
VectorXd FeatureManager::getDepthVector()
{
    VectorXd dep_vec(getFeatureCount());
    int feature_index = -1;
    for (auto &it_per_id : feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < ESTWINDOW_SIZE - 2))
            continue;
#if 1
        dep_vec(++feature_index) = 1. / it_per_id.estimated_depth;
#else
        dep_vec(++feature_index) = it_per_id->estimated_depth;
#endif
    }
    return dep_vec;
}

/// 特征点进行三角化（对所有的能够进行三角化的特征进行三角化）
void FeatureManager::triangulate(Vector3d Ps[], Vector3d tic[], Matrix3d ric[])
{
    for (auto &it_per_id : feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < ESTWINDOW_SIZE - 2))
            continue;

        if (it_per_id.estimated_depth > 0)//如果已经三角化成功，则不对该点进行三角化
            continue;
        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;

        assert(NUM_OF_CAM == 1);
        Eigen::MatrixXd svd_A(2 * it_per_id.feature_per_frame.size(), 4);
        int svd_idx = 0;

        Eigen::Matrix<double, 3, 4> P0;//第一帧
        Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[0];//p_CinG = p_IinG + R_ItoG * t_CinI
        Eigen::Matrix3d R0 = Rs[imu_i] * ric[0];// R_CtoG=R_ItoG * R_CtoI
        P0.leftCols<3>() = Eigen::Matrix3d::Identity();
        P0.rightCols<1>() = Eigen::Vector3d::Zero();

        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;

            Eigen::Vector3d t1 = Ps[imu_j] + Rs[imu_j] * tic[0];
            Eigen::Matrix3d R1 = Rs[imu_j] * ric[0];//当前帧的相机位姿
            Eigen::Vector3d t = R0.transpose() * (t1 - t0);// p_C1inC0 = R_GtoC0*(p_C1inG-p_C0inG)
            Eigen::Matrix3d R = R0.transpose() * R1;// R_C1toC0=R_GtoC0 * R_C1toG
            Eigen::Matrix<double, 3, 4> P;
            P.leftCols<3>() = R.transpose();//注意P的构建方式
            P.rightCols<1>() = -R.transpose() * t;
            Eigen::Vector3d f = it_per_frame.point.normalized();
            svd_A.row(svd_idx++) = f[0] * P.row(2) - f[2] * P.row(0);
            svd_A.row(svd_idx++) = f[1] * P.row(2) - f[2] * P.row(1);

            if (imu_i == imu_j)//第一帧观测为世界坐标系
                continue;
        }
        assert(svd_idx == svd_A.rows());
        Eigen::Vector4d svd_V = Eigen::JacobiSVD<Eigen::MatrixXd>(svd_A, Eigen::ComputeThinV).matrixV().rightCols<1>();
        double svd_method = svd_V[2] / svd_V[3];
        //it_per_id->estimated_depth = -b / A;
        //it_per_id->estimated_depth = svd_V[2] / svd_V[3];

        it_per_id.estimated_depth = svd_method;
        //it_per_id->estimated_depth = INIT_DEPTH;

        if (it_per_id.estimated_depth < 0.1)
        {
            it_per_id.estimated_depth = INIT_DEPTH;
        }
    }
}

void FeatureManager::removeOutlier()
{
    //ROS_BREAK();
    int i = -1;
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;
        i += it->used_num != 0;
        if (it->used_num != 0 && it->is_outlier == true)
        {
            feature.erase(it);
        }
    }
}

/// 删除滑动窗口第一帧，并且特征点的深度值进行调整
void FeatureManager::removeBackShiftDepth(Eigen::Matrix3d marg_R, Eigen::Vector3d marg_P, Eigen::Matrix3d new_R, Eigen::Vector3d new_P)
{
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame != 0)
            it->start_frame--;
        else
        {
            Eigen::Vector3d uv_i = it->feature_per_frame[0].point;  
            it->feature_per_frame.erase(it->feature_per_frame.begin());
            if (it->feature_per_frame.size() < 2)
            {
                feature.erase(it);
                continue;
            }
            else
            {
                Eigen::Vector3d pts_i = uv_i * it->estimated_depth;
                Eigen::Vector3d w_pts_i = marg_R * pts_i + marg_P;
                Eigen::Vector3d pts_j = new_R.transpose() * (w_pts_i - new_P);
                double dep_j = pts_j(2);
                if (dep_j > 0)
                    it->estimated_depth = dep_j;
                else
                    it->estimated_depth = INIT_DEPTH;
            }
        }
    }
}

/// 假如删除滑动窗口的第一帧，则start_frame减一
void FeatureManager::removeBack()
{
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame != 0)
            it->start_frame--;
        else
        {// 等于0，则删除第一帧对应的观测，如果没有任何观测信息，则直接从feature数据库中删除该特征
            it->feature_per_frame.erase(it->feature_per_frame.begin());
            if (it->feature_per_frame.size() == 0)
                feature.erase(it);
        }
    }
}

/// 假如删除滑动窗口的最后一帧
// TODO: 这一部分是啥意思？
void FeatureManager::removeFront(int frame_count)
{
    for (auto it = feature.begin(), it_next = feature.begin(); it != feature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame == frame_count)
        {
            it->start_frame--;
        }
        else
        {
            int j = ESTWINDOW_SIZE - 1 - it->start_frame;
            if (it->endFrame() < frame_count - 1)
                continue;
            it->feature_per_frame.erase(it->feature_per_frame.begin() + j);
            if (it->feature_per_frame.size() == 0)
                feature.erase(it);
        }
    }
}

/// 带旋转补偿的视差计算(检查倒数第二帧是否是关键帧，在norm点上计算倒数第三帧与第二帧之间的差值)
double FeatureManager::compensatedParallax2(const FeaturePerId &it_per_id, int frame_count)
{
    //check the second last frame is keyframe or not
    //parallax betwwen seconde last frame and third last frame
    const FeaturePerFrame &frame_i = it_per_id.feature_per_frame[frame_count - 2 - it_per_id.start_frame];//倒数第三帧
    const FeaturePerFrame &frame_j = it_per_id.feature_per_frame[frame_count - 1 - it_per_id.start_frame];//倒数第二帧

    double ans = 0;
    Vector3d p_j = frame_j.point;

    double u_j = p_j(0);
    double v_j = p_j(1);

    Vector3d p_i = frame_i.point;
    Vector3d p_i_comp;

    //int r_i = frame_count - 2;
    //int r_j = frame_count - 1;
    //p_i_comp = ric[camera_id_j].transpose() * Rs[r_j].transpose() * Rs[r_i] * ric[camera_id_i] * p_i;
    p_i_comp = p_i;
    double dep_i = p_i(2);
    double u_i = p_i(0) / dep_i;
    double v_i = p_i(1) / dep_i;
    double du = u_i - u_j, dv = v_i - v_j;

    double dep_i_comp = p_i_comp(2);
    double u_i_comp = p_i_comp(0) / dep_i_comp;
    double v_i_comp = p_i_comp(1) / dep_i_comp;
    double du_comp = u_i_comp - u_j, dv_comp = v_i_comp - v_j;

    ans = max(ans, sqrt(min(du * du + dv * dv, du_comp * du_comp + dv_comp * dv_comp)));

    return ans;
}
