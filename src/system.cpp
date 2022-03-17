#include "system.h"

vinssystem::vinssystem()
{
	
}

/// 系统初始化以及参数设置
void vinssystem::create(string _config_file)
{
	pg_node = new pgsystem();
	pg_node->create(_config_file, this);
	
	estimatorreadParameters(_config_file);//状态估计相关参数读取
	featuretrackerreadParameters(_config_file);//特征处理相关参数读取
	
	estimator.setParameter();
	
	trackerData = new FeatureTracker[NUM_OF_CAM];
    for (int i = 0; i < NUM_OF_CAM; i++)
        trackerData[i].readIntrinsicParameter(CAM_NAMES[i]);
    if(FISHEYE)
    {
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            trackerData[i].fisheye_mask = cv::imread(FISHEYE_MASK, 0);
            if(!trackerData[i].fisheye_mask.data)
            {
                cout << "load mask fail" << endl;
            }
            else
                cout << "load mask success" << endl;
        }
    }

    FILE* fptmpodo = fopen("odo.txt", "w");
    fclose(fptmpodo);
	
	t_process = std::thread(&vinssystem::process, this);
}

/// 输入IMU观测值
void vinssystem::inputIMU(const ImuConstPtr &imu_msg)
{
    if (imu_msg->header <= last_imu_t)
    {
        //ROS_WARN("imu message in disorder!");
        return;
    }
    last_imu_t = imu_msg->header;

    m_buf.lock();
    imu_buf.push(imu_msg);
    m_buf.unlock();
    con.notify_one();

    last_imu_t = imu_msg->header;

    {
        std::lock_guard<std::mutex> lg(m_state);
        predict(imu_msg);
        double header = imu_msg->header;
        //header.frame_id = "world";
        /// 可视化，输出imu频率的位姿数据
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
            pubLatestOdometry(tmp_P, tmp_Q, tmp_V, header, pg_node);
    }
}

/// 基于IMU数据对状态进行积分
void vinssystem::predict(const ImuConstPtr &imu_msg)
{
    double t = imu_msg->header;
    if (init_imu)
    {
        latest_time = t;
        init_imu = 0;
        return;
    }
    double dt = t - latest_time;
    latest_time = t;

    double dx = imu_msg->acc[0];
    double dy = imu_msg->acc[1];
    double dz = imu_msg->acc[2];
    Eigen::Vector3d linear_acceleration{dx, dy, dz};

    double rx = imu_msg->gyr[0];
    double ry = imu_msg->gyr[1];
    double rz = imu_msg->gyr[2];
    Eigen::Vector3d angular_velocity{rx, ry, rz};

    Eigen::Vector3d un_acc_0 = tmp_Q * (acc_0 - tmp_Ba) - estimator.g;

    Eigen::Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - tmp_Bg;
    tmp_Q = tmp_Q * Utility::deltaQ(un_gyr * dt);

    Eigen::Vector3d un_acc_1 = tmp_Q * (linear_acceleration - tmp_Ba) - estimator.g;

    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);

    tmp_P = tmp_P + dt * tmp_V + 0.5 * dt * dt * un_acc;
    tmp_V = tmp_V + dt * un_acc;

    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}

/// 系统reset
void vinssystem::restart_callback()
{

        printf("restart the estimator!\n");
        m_buf.lock();
        while(!feature_buf.empty())
            feature_buf.pop();
        while(!imu_buf.empty())
            imu_buf.pop();
        m_buf.unlock();

        m_estimator.lock();
        estimator.clearState();//清除状态
        estimator.setParameter();
        m_estimator.unlock();

        current_time = -1;//变量参数设置
        last_imu_t = 0;

    return;
}

/// 输入图像并进行处理
void vinssystem::inputImage(cv::Mat srcImage, double _header)
{
    pg_node->image_callback(srcImage.clone(), _header);
    cout << srcImage.rows << endl;
    cout << srcImage.cols << endl;
    
    if(first_image_flag) // 第一张图片
    {
        first_image_flag = false;
        first_image_time = _header;
        last_image_time = _header;
        return;
    }
    // detect unstable camera stream
    /// 假设检测到图像乱序，或者时间戳跳跃，则直接重启整个系统
    if (_header - last_image_time > 1.0 || _header < last_image_time)
    {
        printf("image discontinue! reset the feature tracker!\n");
        first_image_flag = true; 
        last_image_time = 0;
        pub_count = 1;
        //std_msgs::Bool restart_flag;
        //restart_flag.data = true;
        //pub_restart.publish(restart_flag);
        restart_callback();
        return;
    }
    last_image_time = _header;

    // frequency control
    /// 频率控制，以一定频率处理图像
    if (round(1.0 * pub_count / (_header - first_image_time)) <= FREQ)
    {
        PUB_THIS_FRAME = true;
        // reset the frequency control (重置频率控制)
        if (abs(1.0 * pub_count / (_header - first_image_time) - FREQ) < 0.01 * FREQ)
        {
            first_image_time = _header;
            pub_count = 0;
        }
    }
    else
        PUB_THIS_FRAME = false;

    /// 图像特征跟踪
    cv::Mat show_img = srcImage.clone();
    TicToc t_r;
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        printf("processing camera %d\n", i);
        if (i != 1 || !STEREO_TRACK)//处理单目图像i=0或者MONOCULAR_TRACK
            trackerData[i].readImage(srcImage.rowRange(FEATROW * i, FEATROW * (i + 1)), _header);
        else
        {// 双目图像的右图处理(看起来是两张图像合并到一起了,列合并)
            if (EQUALIZE)
            {//直方图均衡
                cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
                clahe->apply(srcImage.rowRange(FEATROW * i, FEATROW * (i + 1)), trackerData[i].cur_img);
            }
            else
                trackerData[i].cur_img = srcImage.rowRange(FEATROW * i, FEATROW * (i + 1));
        }
    }

    /// 等待特征跟踪处理完
    for (unsigned int i = 0;; i++)
    {
        bool completed = false;
        for (int j = 0; j < NUM_OF_CAM; j++)
            if (j != 1 || !STEREO_TRACK)
                completed |= trackerData[j].updateID(i);
        if (!completed)
            break;
    }

   if (PUB_THIS_FRAME)
   {
        pub_count++;
        //sensor_msgs::PointCloudPtr feature_points(new sensor_msgs::PointCloud);
        //sensor_msgs::ChannelFloat32 id_of_point;
        //sensor_msgs::ChannelFloat32 u_of_point;
        //sensor_msgs::ChannelFloat32 v_of_point;
        //sensor_msgs::ChannelFloat32 velocity_x_of_point;
        //sensor_msgs::ChannelFloat32 velocity_y_of_point;

        IMG_MSG* feature_points = new IMG_MSG();// 图像特征信息
        feature_points->header = _header;
        

        //vector<set<int>> hash_ids(NUM_OF_CAM);
        map<int, Eigen::Matrix<double, 7, 1>> image_msg;
        
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            auto &un_pts = trackerData[i].cur_un_pts;
            auto &cur_pts = trackerData[i].cur_pts;
            auto &ids = trackerData[i].ids;
            auto &pts_velocity = trackerData[i].pts_velocity;
            for (unsigned int j = 0; j < ids.size(); j++)
            {
                if (trackerData[i].track_cnt[j] > 1)
                {
                    Eigen::Matrix<double, 7, 1> img_msg_matrix;
                    int p_id = ids[j];
                    //hash_ids[i].insert(p_id);
                    //geometry_msgs::Point32 p;
                    img_msg_matrix(0, 0) = un_pts[j].x;
                    img_msg_matrix(1, 0) = un_pts[j].y;
                    img_msg_matrix(2, 0) = 1;

                    //feature_points->points.push_back(p);
                    //id_of_point.values.push_back(p_id * NUM_OF_CAM + i);
                    img_msg_matrix(3, 0) = cur_pts[j].x;
                    img_msg_matrix(4, 0) = cur_pts[j].y;
                    img_msg_matrix(5, 0) = pts_velocity[j].x;
                    img_msg_matrix(6, 0) = pts_velocity[j].y;
                    image_msg[p_id * NUM_OF_CAM + i] = img_msg_matrix;
                    //u_of_point.values.push_back(cur_pts[j].x);
                    //v_of_point.values.push_back(cur_pts[j].y);
                    //velocity_x_of_point.values.push_back(pts_velocity[j].x);
                    //velocity_y_of_point.values.push_back(pts_velocity[j].y);
                }
            }
        }

		feature_points->point_clouds = image_msg;
        // skip the first image; since no optical speed on frist image
        if (!init_pub)
        {
            init_pub = 1;
        }
        else
        {
			if (!init_feature)
			{
				//skip the first detected feature, which doesn't contain optical flow speed
				init_feature = 1;
				return;
			}
			m_buf.lock();
			feature_buf.push(feature_points);//特征跟踪结果
			m_buf.unlock();
			con.notify_one();
		}
            

        /// 特征跟踪结果进行可视化
        if (SHOW_TRACK)
        {
			cv::Mat tmp_img;

            for (int i = 0; i < NUM_OF_CAM; i++)
            {
                if(show_img.channels() == 1)
					cv::cvtColor(show_img, tmp_img, CV_GRAY2RGB);
				else
					tmp_img = show_img;
					
                for (unsigned int j = 0; j < trackerData[i].cur_pts.size(); j++)
                {
                    double len = std::min(1.0, 1.0 * trackerData[i].track_cnt[j] / FEATWINDOW_SIZE);
                    cv::circle(tmp_img, trackerData[i].cur_pts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
                }
            }

            //pub_match.publish(ptr->toImageMsg());
            pg_node->show_callback(tmp_img);///位姿图优化结果可视化
        }
    }
    //ROS_INFO("whole feature tracker processing costs: %f", t_r.toc());
}

/// 读取图像之间的imu数据和图像数据
std::vector<std::pair<std::vector<ImuConstPtr>, ImgConstPtr>> vinssystem::getMeasurements()
{
    std::vector<std::pair<std::vector<ImuConstPtr>, ImgConstPtr>> measurements;

    while (true)
    {
        if (imu_buf.empty() || feature_buf.empty())
            return measurements;

        // imu的数据超过图像数据
        if (!(imu_buf.back()->header > feature_buf.front()->header + estimator.td))
        {
            //ROS_WARN("wait for imu, only should happen at the beginning");
            sum_of_wait++;
            return measurements;
        }

        // 第一帧imu数据时间戳应该大于图像时间戳
        if (!(imu_buf.front()->header < feature_buf.front()->header + estimator.td))
        {
            cout <<"throw img, only should happen at the beginning" << endl;
            feature_buf.pop();
            continue;
        }
        ImgConstPtr img_msg = feature_buf.front();
        feature_buf.pop();

        std::vector<ImuConstPtr> IMUs;
        while (imu_buf.front()->header < img_msg->header + estimator.td)
        {
            IMUs.emplace_back(imu_buf.front());
            imu_buf.pop();
        }
        IMUs.emplace_back(imu_buf.front());
        if (IMUs.empty())
            cout << "no imu between two image" << endl;
        measurements.emplace_back(IMUs, img_msg);
    }
    return measurements;
}

/// 重定位启动
void vinssystem::relocalization_callback(const MyPointCloudMsg &points_msg)
{
    //printf("relocalization callback! \n");
    m_buf.lock();
    relo_buf.push(points_msg);
    m_buf.unlock();
}

// thread: visual-inertial odometry
/// VIO处理线程
void vinssystem::process()
{
    while (true)
    {
        /// 获取图像和IMU对齐的数据
        std::vector<std::pair<std::vector<ImuConstPtr>, ImgConstPtr>> measurements;
        std::unique_lock<std::mutex> lk(m_buf);
        con.wait(lk, [&]
                 {
            return (measurements = getMeasurements()).size() != 0;
                 });
        lk.unlock();
        m_estimator.lock();

        /// 处理Imu和图像数据
        for (auto &measurement : measurements)
        {
            auto img_msg = measurement.second;//图像数据
            double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;
            for (auto &imu_msg : measurement.first)// imu数据
            {
                double t = imu_msg->header;
                double img_t = img_msg->header + estimator.td;
                if (t <= img_t)//imu的时间戳小于图像时间戳
                { 
                    if (current_time < 0)
                        current_time = t;
                    double dt = t - current_time;
                    assert(dt >= 0);
                    current_time = t;
                    dx = imu_msg->acc[0];
                    dy = imu_msg->acc[1];
                    dz = imu_msg->acc[2];
                    rx = imu_msg->gyr[0];
                    ry = imu_msg->gyr[1];
                    rz = imu_msg->gyr[2];
                    /// 将imu数据送入estimator中进行处理
                    estimator.processIMU(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                    //printf("imu: dt:%f a: %f %f %f w: %f %f %f\n",dt, dx, dy, dz, rx, ry, rz);
                }
                else
                {//对大于图像时间戳的数据进行线性插值,插值出图像时刻的Imu数据
                    double dt_1 = img_t - current_time;
                    double dt_2 = t - img_t;
                    current_time = img_t;
                    assert(dt_1 >= 0);
                    assert(dt_2 >= 0);
                    assert(dt_1 + dt_2 > 0);
                    double w1 = dt_2 / (dt_1 + dt_2);
                    double w2 = dt_1 / (dt_1 + dt_2);
                    dx = w1 * dx + w2 * imu_msg->acc[0];
                    dy = w1 * dy + w2 * imu_msg->acc[1];
                    dz = w1 * dz + w2 * imu_msg->acc[2];
                    rx = w1 * rx + w2 * imu_msg->gyr[0];
                    ry = w1 * ry + w2 * imu_msg->gyr[1];
                    rz = w1 * rz + w2 * imu_msg->gyr[2];
                    estimator.processIMU(dt_1, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                    //printf("dimu: dt:%f a: %f %f %f w: %f %f %f\n",dt_1, dx, dy, dz, rx, ry, rz);
                }
            }//将imu数据加入到estimator中

            // set relocalization frame
            /// 设置重定位相关处理
            MyPointCloudMsg relo_msg;
            relo_msg.header = -10;
            while (!relo_buf.empty())
            {
                relo_msg = relo_buf.front();
                relo_buf.pop();
            }
            if (relo_msg.header != -10)
            {
                vector<Vector3d> match_points;
                double frame_stamp = relo_msg.header;
                for (unsigned int i = 0; i < relo_msg.vp3.size(); i++)
                {
                    Vector3d u_v_id = relo_msg.vp3[i];
                    match_points.push_back(u_v_id);
                }
                Vector3d relo_t = relo_msg.mP;
                Quaterniond relo_q;
                relo_q = relo_msg.mQ;
                Matrix3d relo_r = relo_q.toRotationMatrix();
                int frame_index;
                frame_index = relo_msg.index;
                /// 重定位的帧的位姿以及特征点匹配
                estimator.setReloFrame(frame_stamp, frame_index, match_points, relo_t, relo_r);
            }

            cout << "processing vision data with stamp " <<  img_msg->header << endl;

            TicToc t_s;
            map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> image;
            for(auto& img_pts : img_msg->point_clouds)
            {
				int v = img_pts.first + 0.5;
				int feature_id = v / NUM_OF_CAM;
                int camera_id = v % NUM_OF_CAM;
                Eigen::Matrix<double, 7, 1> tmp_pt = img_pts.second;
                double x = tmp_pt(0, 0);
                double y = tmp_pt(1, 0);
                double z = tmp_pt(2, 0);
                double p_u = tmp_pt(3, 0);
                double p_v = tmp_pt(4, 0);
                double velocity_x = tmp_pt(5, 0);
                double velocity_y = tmp_pt(6, 0);
                assert(z == 1);
                Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
                xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
                image[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
			}
            estimator.processImage(image, img_msg->header);

            if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
            {
                FILE* fptmpodo = fopen("odo.txt", "a");
                fprintf(fptmpodo, "%.9lf %10.6lf %10.6lf %10.6lf\n", estimator.Headers[ESTWINDOW_SIZE], estimator.Ps[ESTWINDOW_SIZE].x(), estimator.Ps[ESTWINDOW_SIZE].y(), estimator.Ps[ESTWINDOW_SIZE].z());
                fclose(fptmpodo);
            }

            /// 以下操作为可视化操作
            //double whole_t = t_s.toc();
            //printStatistics(estimator, whole_t);
            //std_msgs::Header header = img_msg->header;
            //header.frame_id = "world";
			double header = img_msg->header;
            //pubOdometry(estimator, header, pg_node);
            //pubKeyPoses(estimator, header);
            //pubCameraPose(estimator, header);
            //pubPointCloud(estimator, header);
            //pubTF(estimator, header, pg_node);
            pubOdometryTF(estimator, header, pg_node);
            pubKeyframe(estimator, pg_node);
            if (relo_msg.header != -10)
                pubRelocalization(estimator, pg_node);
            //ROS_ERROR("end: %f, at %f", img_msg->header.stamp.toSec(), ros::Time::now().toSec());
        }
        m_estimator.unlock();

        m_buf.lock();
        m_state.lock();
        //if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
        //    update();
        m_state.unlock();
        m_buf.unlock();
    }
}
