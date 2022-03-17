#include "pgsystem.h"

pgsystem::pgsystem()
{
	frame_index = 0;
	sequence = 1;
	skip_first_cnt = 0;
	skip_cnt = 0;
	load_flag = 0;
	start_flag = 0;
	SKIP_DIS = 0;
	last_t = Eigen::Vector3d(-100, -100, -100);
	last_image_time = -1;

}

void pgsystem::image_callback(const cv::Mat &image, double header)
{
    //ROS_INFO("image_callback!");
    if(!LOOP_CLOSURE)
        return;
    IMGMeasurement image_msg;
    image_msg.header = header;
    image_msg.image = image.clone();
    m_buf.lock();
    image_buf.push(image_msg);
    m_buf.unlock();
    //printf(" image time %f \n", image_msg->header.stamp.toSec());

    // detect unstable camera stream
    if (last_image_time == -1)
        last_image_time = image_msg.header;
    else if (image_msg.header - last_image_time > 1.0 || image_msg.header < last_image_time)
    {
        printf("image discontinue! detect a new sequence!");
        new_sequence();
    }
    last_image_time = image_msg.header;

}

void pgsystem::new_sequence()
{
    printf("new sequence\n");
    sequence++;
    printf("sequence cnt %d \n", sequence);
    if (sequence > 5)
    {
        printf("only support 5 sequences since it's boring to copy code for more sequences.\n");
        return;
    }
    //posegraph.posegraph_visualization->reset();
    //posegraph.publish();
    m_buf.lock();
    while(!image_buf.empty())
        image_buf.pop();
    while(!point_buf.empty())
        point_buf.pop();
    while(!pose_buf.empty())
        pose_buf.pop();
    while(!odometry_buf.empty())
        odometry_buf.pop();
    m_buf.unlock();
}

void pgsystem::show_callback(const cv::Mat &image)
{
    cv::Mat imText;
    int baseline=0;
    m_strpose.lock();
    cv::Size textSize = cv::getTextSize(s.str(),cv::FONT_HERSHEY_PLAIN,1,1,&baseline);
    cv::Scalar initcolor=cv::Scalar(0,0,0,255);
    imText = cv::Mat(image.rows+(textSize.height+10)*4,image.cols,image.type(),initcolor);
    image.copyTo(imText.rowRange(0,image.rows).colRange(0,image.cols));
    //imText.rowRange(image.rows,imText.rows) = cv::Mat::zeros((textSize.height+10)*4,image.cols,image.type(),cv::Scalar(0,0,0,255));
//    cv::putText(imText,s.str(),cv::Point(5,imText.rows-5-(textSize.height+10)*3),cv::FONT_HERSHEY_PLAIN,5,cv::Scalar(255,0,255,255),5,8);
//    cv::putText(imText,s1.str(),cv::Point(5,imText.rows-5-(textSize.height+10)*2),cv::FONT_HERSHEY_PLAIN,5,cv::Scalar(255,0,255,255),5,8);
    cv::putText(imText,s.str(),cv::Point(5,imText.rows-5-(textSize.height+10)),cv::FONT_HERSHEY_PLAIN,3,cv::Scalar(255,0,255,255),3,8);
    m_strpose.unlock();
    //  cv::putText(imText,s3.str(),cv::Point(5,imText.rows-5),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(255,0,255,255),1,8);
    m_imagetext.lock();
    imText.copyTo(mimText);
    m_imagetext.unlock();
}

void pgsystem::point_callback(const MyPointCloudMsg &point_msg)
{
    //ROS_INFO("point_callback!");
    if(!LOOP_CLOSURE)
        return;
    m_buf.lock();
    point_buf.push(point_msg);
    vector<Vector3d> vtmp_p3d;
    for(int i = 0; i < point_msg.vp3.size(); i++)
    {
        Vector3d tmp_p3d1;
        tmp_p3d1 = posegraph->w_r_vio * point_msg.vp3[i] + posegraph->w_t_vio;
        Vector3d tmp_p3d2;
        tmp_p3d2 = posegraph->r_drift * tmp_p3d1 + posegraph->t_drift;
        vtmp_p3d.push_back(tmp_p3d2);
    }
    mpmapdrawer->SetAllPoints(vtmp_p3d);
    m_buf.unlock();
    /*
    for (unsigned int i = 0; i < point_msg->points.size(); i++)
    {
        printf("%d, 3D point: %f, %f, %f 2D point %f, %f \n",i , point_msg->points[i].x, 
                                                     point_msg->points[i].y,
                                                     point_msg->points[i].z,
                                                     point_msg->channels[i].values[0],
                                                     point_msg->channels[i].values[1]);
    }
    */
}

void pgsystem::pose_callback(const MyOdometryMsg &pose_msg)
{
    //ROS_INFO("pose_callback!");
    if(!LOOP_CLOSURE)
        return;
    m_buf.lock();
    pose_buf.push(pose_msg);
    m_buf.unlock();
    /*
    printf("pose t: %f, %f, %f   q: %f, %f, %f %f \n", pose_msg->pose.pose.position.x,
                                                       pose_msg->pose.pose.position.y,
                                                       pose_msg->pose.pose.position.z,
                                                       pose_msg->pose.pose.orientation.w,
                                                       pose_msg->pose.pose.orientation.x,
                                                       pose_msg->pose.pose.orientation.y,
                                                       pose_msg->pose.pose.orientation.z);
    */
}

void pgsystem::imu_forward_callback(const MyOdometryMsg &forward_msg)
{
    if (VISUALIZE_IMU_FORWARD)
    {
        Vector3d vio_t = forward_msg.mP;
        Quaterniond vio_q;
        vio_q = forward_msg.mQ;


        vio_t = posegraph->w_r_vio * vio_t + posegraph->w_t_vio;
        vio_q = posegraph->w_r_vio *  vio_q;

        vio_t = posegraph->r_drift * vio_t + posegraph->t_drift;
        vio_q = posegraph->r_drift * vio_q;

        Vector3d vio_t_cam;
        Quaterniond vio_q_cam;
        vio_t_cam = vio_t + vio_q * tic;
        vio_q_cam = vio_q * qic;        

        //cameraposevisual.reset();
        //cameraposevisual.add_pose(vio_t_cam, vio_q_cam);
        //cameraposevisual.publish_by(pub_camera_pose_visual, forward_msg->header);
    }
}

void pgsystem::relo_relative_pose_callback(const MyOdometryMsg &pose_msg)
{
    Vector3d relative_t = pose_msg.mP;
    Quaterniond relative_q;
    relative_q = pose_msg.mQ;

    double relative_yaw = pose_msg.mV[0];
    int index = pose_msg.mV[1];
    //printf("receive index %d \n", index );
    Eigen::Matrix<double, 8, 1 > loop_info;
    loop_info << relative_t.x(), relative_t.y(), relative_t.z(),
                 relative_q.w(), relative_q.x(), relative_q.y(), relative_q.z(),
                 relative_yaw;
    posegraph->updateKeyFrameLoop(index, loop_info);

}

void pgsystem::vio_callback(const MyOdometryMsg &pose_msg)
{

    m_process.lock();
    tic = pose_msg.tic;
    qic = pose_msg.qic.toRotationMatrix();
    m_process.unlock();
    //ROS_INFO("vio_callback!");
    Vector3d vio_t = pose_msg.mP;
    Quaterniond vio_q;
    vio_q = pose_msg.mQ;

    posegraph->m_allframes.lock();
    vio_t = posegraph->w_r_vio * vio_t + posegraph->w_t_vio;
    vio_q = posegraph->w_r_vio *  vio_q;

    MyOdometryMsg tmp_viopose;
    tmp_viopose.header = pose_msg.header;
    tmp_viopose.mP = vio_t;
    tmp_viopose.mQ = vio_q;
    tmp_viopose.tic = pose_msg.tic;
    tmp_viopose.qic = pose_msg.qic;
    pair<int, MyOdometryMsg> tmp_idx_viopose;
    tmp_idx_viopose.first = sequence;
    tmp_idx_viopose.second = tmp_viopose;
    posegraph->vallframes_noloop.push_back(tmp_idx_viopose);


    FILE* fpnoloop = fopen("noloop_frames.txt", "a");
    fprintf(fpnoloop, "%.6lf %10.6lf %10.6lf %10.6lf\n", pose_msg.header, tmp_viopose.mP[0], tmp_viopose.mP[1], tmp_viopose.mP[2]);
    fclose(fpnoloop);

    vio_t = posegraph->r_drift * vio_t + posegraph->t_drift;
    vio_q = posegraph->r_drift * vio_q;

    tmp_idx_viopose.second.mP = vio_t;
    tmp_idx_viopose.second.mQ = vio_q;
    mpmapdrawer->AddOneFrame(tmp_idx_viopose);

    FILE* fploop = fopen("loop_frames.txt", "a");
    fprintf(fploop, "%.6lf %10.6lf %10.6lf %10.6lf\n", pose_msg.header, vio_t[0], vio_t[1], vio_t[2]);
    fclose(fploop);

    m_strpose.lock();
    s.str("");
    s << "P: " << "P:X:" << setprecision(3)
                 << vio_t[0] << " Y:"
                 << vio_t[1] << " Z:"
                 << vio_t[2] << " ";
    m_strpose.unlock();
    posegraph->m_allframes.unlock();

    Vector3d vio_t_cam;
    Quaterniond vio_q_cam;
    vio_t_cam = vio_t + vio_q * tic;
    vio_q_cam = vio_q * qic;        

    if (!VISUALIZE_IMU_FORWARD)
    {
        //cameraposevisual.reset();
        //cameraposevisual.add_pose(vio_t_cam, vio_q_cam);
        //cameraposevisual.publish_by(pub_camera_pose_visual, pose_msg->header);
    }

    odometry_buf.push(vio_t_cam);
    if (odometry_buf.size() > 10)
    {
        odometry_buf.pop();
    }

    //visualization_msgs::Marker key_odometrys;
    //key_odometrys.header = pose_msg->header;
    //key_odometrys.header.frame_id = "world";
    //key_odometrys.ns = "key_odometrys";
    //key_odometrys.type = visualization_msgs::Marker::SPHERE_LIST;
    //key_odometrys.action = visualization_msgs::Marker::ADD;
    //key_odometrys.pose.orientation.w = 1.0;
    //key_odometrys.lifetime = ros::Duration();

    //static int key_odometrys_id = 0;
    //key_odometrys.id = 0; //key_odometrys_id++;
    //key_odometrys.scale.x = 0.1;
    //key_odometrys.scale.y = 0.1;
    //key_odometrys.scale.z = 0.1;
    //key_odometrys.color.r = 1.0;
    //key_odometrys.color.a = 1.0;

    for (unsigned int i = 0; i < odometry_buf.size(); i++)
    {
        //geometry_msgs::Point pose_marker;
        Vector3d vio_t;
        vio_t = odometry_buf.front();
        odometry_buf.pop();
        //pose_marker.x = vio_t.x();
        //pose_marker.y = vio_t.y();
        //pose_marker.z = vio_t.z();
        //key_odometrys.points.push_back(pose_marker);
        odometry_buf.push(vio_t);
    }
    //pub_key_odometrys.publish(key_odometrys);

    if (!LOOP_CLOSURE)
    {
        //geometry_msgs::PoseStamped pose_stamped;
        //pose_stamped.header = pose_msg->header;
        //pose_stamped.header.frame_id = "world";
        //pose_stamped.pose.position.x = vio_t.x();
        //pose_stamped.pose.position.y = vio_t.y();
        //pose_stamped.pose.position.z = vio_t.z();
        //no_loop_path.header = pose_msg->header;
        //no_loop_path.header.frame_id = "world";
        //no_loop_path.poses.push_back(pose_stamped);
        //pub_vio_path.publish(no_loop_path);
    }
}

void pgsystem::extrinsic_callback(const MyOdometryMsg &pose_msg)
{
    m_process.lock();
    tic = pose_msg.mP;
    qic = pose_msg.mQ.toRotationMatrix();
    m_process.unlock();
}

void pgsystem::process()
{
    if (!LOOP_CLOSURE)
        return;
    while (true)
    {
        IMGMeasurement image_msg;
        MyPointCloudMsg point_msg;
        MyOdometryMsg pose_msg;
        pose_msg.header = -10;

        // find out the messages with same time stamp
        m_buf.lock();
        if(!image_buf.empty() && !point_buf.empty() && !pose_buf.empty())
        {
            if (image_buf.front().header > pose_buf.front().header)
            {
                pose_buf.pop();
                printf("throw pose at beginning\n");
            }
            else if (image_buf.front().header > point_buf.front().header)
            {
                point_buf.pop();
                printf("throw point at beginning\n");
            }
            else if (image_buf.back().header >= pose_buf.front().header 
                && point_buf.back().header >= pose_buf.front().header)
            {
                pose_msg = pose_buf.front();
                pose_buf.pop();
                while (!pose_buf.empty())
                    pose_buf.pop();
                while (image_buf.front().header < pose_msg.header)
                    image_buf.pop();
                image_msg = image_buf.front();
                image_buf.pop();

                while (point_buf.front().header < pose_msg.header)
                    point_buf.pop();
                point_msg = point_buf.front();
                point_buf.pop();
            }
        }
        m_buf.unlock();

        if (pose_msg.header != -10)
        {
            //printf(" pose time %f \n", pose_msg->header.stamp.toSec());
            //printf(" point time %f \n", point_msg->header.stamp.toSec());
            //printf(" image time %f \n", image_msg->header.stamp.toSec());
            // skip fisrt few
            printf("pg1\n");
            if (skip_first_cnt < SKIP_FIRST_CNT)
            {
                skip_first_cnt++;
                continue;
            }

            if (skip_cnt < SKIP_CNT)
            {
                skip_cnt++;
                continue;
            }
            else
            {
                skip_cnt = 0;
            }
            printf("pg2\n");
            /*cv_bridge::CvImageConstPtr ptr;
            if (image_msg->encoding == "8UC1")
            {
                sensor_msgs::Image img;
                img.header = image_msg->header;
                img.height = image_msg->height;
                img.width = image_msg->width;
                img.is_bigendian = image_msg->is_bigendian;
                img.step = image_msg->step;
                img.data = image_msg->data;
                img.encoding = "mono8";
                ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
            }
            else
                ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8);*/
            
            cv::Mat image = image_msg.image.clone();
            // build keyframe
            Vector3d T = pose_msg.mP;
            Matrix3d R = pose_msg.mQ.toRotationMatrix();
            if((T - last_t).norm() > SKIP_DIS)
            {
                vector<cv::Point3f> point_3d; 
                vector<cv::Point2f> point_2d_uv; 
                vector<cv::Point2f> point_2d_normal;
                vector<double> point_id;

                for (unsigned int i = 0; i < point_msg.vp3.size(); i++)
                {
                    Vector3d tmpP3 = point_msg.vp3[i];
                    cv::Point3f p_3d;
                    p_3d.x = tmpP3[0];
                    p_3d.y = tmpP3[1];
                    p_3d.z = tmpP3[2];
                    point_3d.push_back(p_3d);

                    Vector2d tmpP2; Vector2d tmpuv2;
                    tmpP2 = point_msg.vp2[i];
                    tmpuv2 = point_msg.vuv[i];
                    cv::Point2f p_2d_uv, p_2d_normal;
                    double p_id;
                    p_2d_normal.x = tmpP2[0];
                    p_2d_normal.y = tmpP2[1];
                    p_2d_uv.x = tmpuv2[0];
                    p_2d_uv.y = tmpuv2[1];
                    p_id = point_msg.vid[i];
                    point_2d_normal.push_back(p_2d_normal);
                    point_2d_uv.push_back(p_2d_uv);
                    point_id.push_back(p_id);

                    //printf("u %f, v %f \n", p_2d_uv.x, p_2d_uv.y);
                }
                printf("pg3\n");
                KeyFrame* keyframe = new KeyFrame(pose_msg.header, frame_index, T, R, image,
                                   point_3d, point_2d_uv, point_2d_normal, point_id, sequence);   
                m_process.lock();
                start_flag = 1;
                posegraph->addKeyFrame(keyframe, 1, mpvinssystem);
                m_process.unlock();
                frame_index++;
                last_t = T;
            }
        }

        std::chrono::milliseconds dura(5);
        std::this_thread::sleep_for(dura);
    }
}

void pgsystem::create(string _config_file, vinssystem* _pvinssystem)
{


    // read param
    //n.getParam("visualization_shift_x", VISUALIZATION_SHIFT_X);
    //n.getParam("visualization_shift_y", VISUALIZATION_SHIFT_Y);
    mpvinssystem = _pvinssystem;
    mpmapdrawer = new MapDrawer(_config_file);
    posegraph = new PoseGraph(mpmapdrawer);

    SKIP_CNT = 0;
    SKIP_DIS = 0;
    std::string config_file;
    config_file = _config_file;
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    //double camera_visual_size = fsSettings["visualize_camera_size"];
    //cameraposevisual.setScale(camera_visual_size);
    //cameraposevisual.setLineWidth(camera_visual_size / 10.0);
    posegraphreadParameters(config_file);

    LOOP_CLOSURE = fsSettings["loop_closure"];
    std::string IMAGE_TOPIC;
    int LOAD_PREVIOUS_POSE_GRAPH;
    if (LOOP_CLOSURE)
    {
        POSEROW = fsSettings["image_height"];
        POSECOL = fsSettings["image_width"];
        //std::string pkg_path = ros::package::getPath("pose_graph");
        string vocabulary_file = std::string("../support_files/brief_k10L6.bin");
        cout << "vocabulary_file" << vocabulary_file << endl;
        posegraph->loadVocabulary(vocabulary_file);

        BRIEF_PATTERN_FILE = std::string("../support_files/brief_pattern.yml");
        cout << "BRIEF_PATTERN_FILE" << BRIEF_PATTERN_FILE << endl;
        //m_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(config_file.c_str());

        fsSettings["image_topic"] >> IMAGE_TOPIC;        
        fsSettings["pose_graph_save_path"] >> POSE_GRAPH_SAVE_PATH;
        fsSettings["output_path"] >> VINS_RESULT_PATH;
        fsSettings["save_image"] >> DEBUG_IMAGE;

        // create folder if not exists
        FileSystemHelper::createDirectoryIfNotExists(POSE_GRAPH_SAVE_PATH.c_str());
        FileSystemHelper::createDirectoryIfNotExists(VINS_RESULT_PATH.c_str());

        VISUALIZE_IMU_FORWARD = fsSettings["visualize_imu_forward"];
        LOAD_PREVIOUS_POSE_GRAPH = fsSettings["load_previous_pose_graph"];
        FAST_RELOCALIZATION = fsSettings["fast_relocalization"];
        VINS_RESULT_PATH = VINS_RESULT_PATH + "vins_result_loop.csv";
        std::ofstream fout(VINS_RESULT_PATH, std::ios::out);
        fout.close();
        fsSettings.release();

        if (LOAD_PREVIOUS_POSE_GRAPH)
        {
            printf("load pose graph\n");
            m_process.lock();
            posegraph->loadPoseGraph(mpvinssystem);
            m_process.unlock();
            printf("load pose graph finish\n");
            load_flag = 1;
        }
        else
        {
            printf("no previous pose graph\n");
            load_flag = 1;
        }
    }

    fsSettings.release();

    //ros::Subscriber sub_imu_forward = n.subscribe("/vins_estimator/imu_propagate", 2000, imu_forward_callback);
    //ros::Subscriber sub_vio = n.subscribe("/vins_estimator/odometry", 2000, vio_callback);
    //ros::Subscriber sub_image = n.subscribe(IMAGE_TOPIC, 2000, image_callback);
    //ros::Subscriber sub_pose = n.subscribe("/vins_estimator/keyframe_pose", 2000, pose_callback);
    //ros::Subscriber sub_extrinsic = n.subscribe("/vins_estimator/extrinsic", 2000, extrinsic_callback);
    //ros::Subscriber sub_point = n.subscribe("/vins_estimator/keyframe_point", 2000, point_callback);
    //ros::Subscriber sub_relo_relative_pose = n.subscribe("/vins_estimator/relo_relative_pose", 2000, relo_relative_pose_callback);

    //pub_match_img = n.advertise<sensor_msgs::Image>("match_image", 1000);
    //pub_camera_pose_visual = n.advertise<visualization_msgs::MarkerArray>("camera_pose_visual", 1000);
    //pub_key_odometrys = n.advertise<visualization_msgs::Marker>("key_odometrys", 1000);
    //pub_vio_path = n.advertise<nav_msgs::Path>("no_loop_path", 1000);
    //pub_match_points = n.advertise<sensor_msgs::PointCloud>("match_points", 100);

    FILE* fpnoloop = fopen("noloop_frames.txt", "w");
    fclose(fpnoloop);
    FILE* fploop = fopen("loop_frames.txt", "w");
    fclose(fploop);
    //std::thread keyboard_command_process;

    measurement_process = std::thread(&pgsystem::process, this);
    //keyboard_command_process = std::thread(command);
    mT = 50;
    draw_process = std::thread(&pgsystem::Run, this);

    //ros::spin();
}

void pgsystem::Run()
{
    pangolin::CreateWindowAndBind("Map Viewer",1035,768);
    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
    /*pangolin::Var<bool> menuFollowCamera("menu.Follow Camera",true,true);
    pangolin::Var<bool> menuShowPoints("menu.Show Points",true,true);
    pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames",true,true);
    pangolin::Var<bool> menuShowGraph("menu.Show Graph",true,true);
    pangolin::Var<bool> menuLocalizationMode("menu.Localization Mode",false,true);*/
    pangolin::Var<bool> menuReset("menu.Reset",false,false);
    pangolin::Var<bool> menunewsequence("menu.New Sequence",false,false);
    pangolin::Var<bool> menusavepg("menu.Save Posegraph", false, false);

    float mViewpointF = 500;
    float mViewpointX = 38.0;
    float mViewpointY = 30.0;
    float mViewpointZ = 75.0;

    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(860,768,mViewpointF,mViewpointF,512,389,0.1,1000),
            pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 38.0,30.0,0,0.0,1.0, 0.0)
    );

    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -860.0f/768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    pangolin::OpenGlMatrix Twc;
    Twc.SetIdentity();
    cv::namedWindow("Current Frame", CV_WINDOW_NORMAL);
    cv::resizeWindow("Current Frame", 480, 320);
    bool bwrittenmeas = false;
    while(true) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        mpmapdrawer->GetCurrentOpenGLCameraMatrix(Twc);
        s_cam.Follow(Twc);
        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        mpmapdrawer->DrawGrids();
        mpmapdrawer->DrawMapPoints();
        mpmapdrawer->DrawKeyFrames();
        pangolin::FinishFrame();
        m_imagetext.lock();
        if (!mimText.empty()) {
            cv::imshow("Current Frame", mimText);
            cv::waitKey(mT);
        }
        m_imagetext.unlock();
        if(menunewsequence == true)
        {
            new_sequence();
            menunewsequence = false;
        }
        if(menusavepg == true)
        {
            m_process.lock();
            posegraph->savePoseGraph();
            m_process.unlock();
            menusavepg = false;
        }
        if(menuReset == true)
        {
            new_sequence();

            mpvinssystem->first_image_flag = true;
            mpvinssystem->last_image_time = 0;
            mpvinssystem->pub_count = 1;
            mpvinssystem->restart_callback();
            menuReset = false;
        }
        usleep(100000);
    }
}