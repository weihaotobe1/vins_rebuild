//
// Created by root on 18-5-20.
//

#include <opencv2/core/persistence.hpp>
#include <GL/glew.h>
#include "MapDrawer.h"
#include <pangolin/pangolin.h>
MapDrawer::MapDrawer(const std::string &strSettingPath)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    mKeyFrameSize = fSettings["Viewer.KeyFrameSize"];
    mKeyFrameLineWidth = fSettings["Viewer.KeyFrameLineWidth"];
    mGraphLineWidth = fSettings["Viewer.GraphLineWidth"];
    mPointSize = fSettings["Viewer.PointSize"];
    mCameraSize = fSettings["Viewer.CameraSize"];
    mCameraLineWidth = fSettings["Viewer.CameraLineWidth"];
    mScaleic = fSettings["Viewer.Scale"];

}

void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M)
{
    if(!mCameraPose.empty())
    {
        cv::Mat Rwc(3,3,CV_32F);
        cv::Mat twc(3,1,CV_32F);
        {
            std::unique_lock<std::mutex> lock(mMutexCamera);
            //cv::Mat InitialTwc = mpMap->GetInitialTransition();
            //cv::Mat InitialTcw = InitialTwc.inv();
            //cv::Mat mtransformedTcw = mCameraPose * InitialTwc;
            cv::Mat mtransformedTcw = cvTwcs[cvTwcs.size()-1].inv();
            Rwc = mtransformedTcw.rowRange(0,3).colRange(0,3).t();
            twc = -Rwc*mtransformedTcw.rowRange(0,3).col(3);
        }

        M.m[0] = Rwc.at<float>(0,0);
        M.m[1] = Rwc.at<float>(1,0);
        M.m[2] = Rwc.at<float>(2,0);
        M.m[3]  = 0.0;

        M.m[4] = Rwc.at<float>(0,1);
        M.m[5] = Rwc.at<float>(1,1);
        M.m[6] = Rwc.at<float>(2,1);
        M.m[7]  = 0.0;

        M.m[8] = Rwc.at<float>(0,2);
        M.m[9] = Rwc.at<float>(1,2);
        M.m[10] = Rwc.at<float>(2,2);
        M.m[11]  = 0.0;

        M.m[12] = twc.at<float>(0)/mScaleic;
        M.m[13] = twc.at<float>(1)/mScaleic;
        M.m[14] = twc.at<float>(2)/mScaleic;
        M.m[15]  = 1.0;
    }
    else
        M.SetIdentity();
}

void MapDrawer::SetAllFrames(std::vector<std::pair<int, MyOdometryMsg>> frames_to_draw)
{
    unique_lock<mutex> lock(mMutexCamera);
    if(cvTwcs.size() > 0)
        cvTwcs.clear();
    for(int i = 0; i < frames_to_draw.size(); i++)
    {
        cv::Mat cvRwc;
        cv::Mat cvtwc;
        Matrix3d eiRwc;
        Vector3d eitwc;


        eiRwc = frames_to_draw[i].second.mQ.toRotationMatrix() * frames_to_draw[i].second.qic.toRotationMatrix();
        eitwc = frames_to_draw[i].second.mP + frames_to_draw[i].second.mQ.toRotationMatrix() * frames_to_draw[i].second.tic;



        cvRwc = Utility::toCvMat(eiRwc);
        cvtwc = Utility::toCvMat(eitwc);
        //eigen2cv(frames_to_draw[i].P_draw,twc);
        //eigen2cv(frames_to_draw[i].R_draw,Rwc);
        cv::Mat Twc = cv::Mat::eye(4,4,CV_32F);
        cvRwc.copyTo(Twc.rowRange(0,3).colRange(0,3));
        cvtwc.copyTo(Twc.rowRange(0,3).col(3));
        cvTwcs.push_back(Twc.clone());
    }
}
void MapDrawer::AddOneFrame(std::pair<int, MyOdometryMsg> oneframe_to_draw)
{
    unique_lock<mutex> lock(mMutexCamera);
    cv::Mat cvRwc;
    cv::Mat cvtwc;
    Matrix3d eiRwc;
    Vector3d eitwc;


    eiRwc = oneframe_to_draw.second.mQ.toRotationMatrix() * oneframe_to_draw.second.qic.toRotationMatrix();
    eitwc = oneframe_to_draw.second.mP + oneframe_to_draw.second.mQ.toRotationMatrix() * oneframe_to_draw.second.tic;



    cvRwc = Utility::toCvMat(eiRwc);
    cvtwc = Utility::toCvMat(eitwc);
    //eigen2cv(frames_to_draw[i].P_draw,twc);
    //eigen2cv(frames_to_draw[i].R_draw,Rwc);
    cv::Mat Twc = cv::Mat::eye(4,4,CV_32F);
    cvRwc.copyTo(Twc.rowRange(0,3).colRange(0,3));
    cvtwc.copyTo(Twc.rowRange(0,3).col(3));
    cvTwcs.push_back(Twc.clone());
}
void MapDrawer::SetAllPoints(std::vector<Vector3d> points_3d)
{
    std::unique_lock<std::mutex> lock(mMutexPoints);
    cvpoints_3d.clear();
    for(int i = 0; i < points_3d.size(); i++)
    {
        cv::Mat cvpt;
        cvpt = Utility::toCvMatd(points_3d[i]);
        //eigen2cv(points_3d[i],cvpt);
        cvpoints_3d.push_back(cvpt.clone());
    }
}

void MapDrawer::DrawMapPoints()
{

    std::unique_lock<std::mutex> lock(mMutexPoints);
    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(1.0,0.0,0.0);

    for(int i = 0; i < cvpoints_3d.size() ; i++)
    {
        cv::Mat pos = cvpoints_3d[i].clone();
        glVertex3f(pos.at<double>(0)/mScaleic,pos.at<double>(1)/mScaleic,pos.at<double>(2)/mScaleic);
    }

    glEnd();
}

void MapDrawer::DrawGrids()
{
    //glLineWidth(mKeyFrameLineWidth);
    //glColor3f(0.5f,0.5f,0.5f);
    //glBegin(GL_LINES);
    const float axis_length = 25;
    const float grid_width = 15;
    ///////////Draw Axis////////////////////////////////////////////////
    glLineWidth(mKeyFrameLineWidth*2);
    glColor3f(1.0f,0.0f,0.0f);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(axis_length,0,0);
    glEnd();
    glLineWidth(mKeyFrameLineWidth*2);
    glColor3f(0.0f,1.0f,0.0f);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(0,axis_length,0);
    glEnd();
    glLineWidth(mKeyFrameLineWidth*2);
    glColor3f(0.0f,0.0f,1.0f);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(0,0,axis_length);
    glEnd();
    ///////////Draw Grids///////////////////////////////////////////////
    glLineWidth(mKeyFrameLineWidth);
    glColor3f(0.5f,0.5f,0.5f);
    glBegin(GL_LINES);
    glVertex3f(-grid_width,-grid_width,0);
    glVertex3f(-grid_width,grid_width,0);
    glVertex3f(-2.0/3.0*grid_width,-grid_width,0);
    glVertex3f(-2.0/3.0*grid_width,grid_width,0);
    glVertex3f(-1.0/3.0*grid_width,-grid_width,0);
    glVertex3f(-1.0/3.0*grid_width,grid_width,0);
    glVertex3f(0,-grid_width,0);
    glVertex3f(0,grid_width,0);
    glVertex3f(1.0/3.0*grid_width,-grid_width,0);
    glVertex3f(1.0/3.0*grid_width,grid_width,0);
    glVertex3f(2.0/3.0*grid_width,-grid_width,0);
    glVertex3f(2.0/3.0*grid_width,grid_width,0);
    glVertex3f(grid_width,-grid_width,0);
    glVertex3f(grid_width,grid_width,0);

    glVertex3f(-grid_width,-grid_width,0);
    glVertex3f(grid_width,-grid_width,0);
    glVertex3f(-grid_width,-2.0/3.0*grid_width,0);
    glVertex3f(grid_width,-2.0/3.0*grid_width,0);
    glVertex3f(-grid_width,-1.0/3.0*grid_width,0);
    glVertex3f(grid_width,-1.0/3.0*grid_width,0);
    glVertex3f(-grid_width,0,0);
    glVertex3f(grid_width,0,0);
    glVertex3f(-grid_width,1.0/3.0*grid_width,0);
    glVertex3f(grid_width,1.0/3.0*grid_width,0);
    glVertex3f(-grid_width,2.0/3.0*grid_width,0);
    glVertex3f(grid_width,2.0/3.0*grid_width,0);
    glVertex3f(-grid_width,grid_width,0);
    glVertex3f(grid_width,grid_width,0);
    //////////////////Draw Texts//////////////////////
    const float textscale = grid_width / 3.0;
    glVertex3f(4*textscale,0.2*textscale,0*textscale);
    glVertex3f(5*textscale,0.8*textscale,0*textscale);
    glVertex3f(4*textscale,0.9*textscale,0*textscale);
    glVertex3f(5*textscale,0.1*textscale,0*textscale);
    glVertex3f(-0.9*textscale,5*textscale,0*textscale);
    glVertex3f(-0.5*textscale,4.5*textscale,0*textscale);
    glVertex3f(-0.1*textscale,5*textscale,0*textscale);
    glVertex3f(-0.5*textscale,4.5*textscale,0*textscale);
    glVertex3f(-0.5*textscale,4*textscale,0*textscale);
    glVertex3f(-0.5*textscale,4.5*textscale,0*textscale);
    glVertex3f(0*textscale,0.3*textscale,5*textscale);
    glVertex3f(0*textscale,0.9*textscale,5*textscale);
    glVertex3f(0*textscale,0.9*textscale,5*textscale);
    glVertex3f(0*textscale,0.2*textscale,4*textscale);
    glVertex3f(0*textscale,0.2*textscale,4*textscale);
    glVertex3f(0*textscale,0.8*textscale,4*textscale);

    glEnd();

    glLineWidth(mKeyFrameLineWidth*2);
    glColor4f(1.0f,0.0f,0.0f,1.0);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(axis_length,0,0);
    glEnd();

    /*char* fontpath = "/home/goodstudent/Documents/Pangolin/src/_embed_/fonts/AnonymousPro.ttf";
    pangolin::GlText txt;

    pangolin::GlFont glFont(fontpath, 16.0);
    glColor4f(0.0, 0.0, 0.5, 0.8);
    txt = glFont.Text("0");
    txt.Draw(0.0, 0.0, -1.5);
    glEnd();

    glColor4f(0.0, 0.0, 0.5, 0.8);
    txt = glFont.Text("100");
    txt.Draw(1.0/3.0*grid_width-2, 0.0, -1.5);
    glEnd();

    glColor4f(0.0, 0.0, 0.5, 0.8);
    txt = glFont.Text("200");
    txt.Draw(2.0/3.0*grid_width-2, 0.0, -1.5);
    glEnd();

    glColor4f(0.0, 0.0, 0.5, 0.8);
    txt = glFont.Text("300");
    txt.Draw(grid_width-2, 0.0, -1.5);
    glEnd();

    glColor4f(0.0, 0.0, 0.5, 0.8);
    txt = glFont.Text("-100");
    txt.Draw(-1.0/3.0*grid_width-2, 0.0, -1.5);
    glEnd();

    glColor4f(0.0, 0.0, 0.5, 0.8);
    txt = glFont.Text("-200");
    txt.Draw(-2.0/3.0*grid_width-2, 0.0, -1.5);
    glEnd();

    glColor4f(0.0, 0.0, 0.5, 0.8);
    txt = glFont.Text("-300");
    txt.Draw(-grid_width-2, 0.0, -1.5);
    glEnd();

    glColor4f(0.0, 0.0, 0.5, 0.8);
    txt = glFont.Text("100");
    txt.Draw(0.0, 1.0/3.0*grid_width-0.5, -1.5);
    glEnd();

    glColor4f(0.0, 0.0, 0.5, 0.8);
    txt = glFont.Text("200");
    txt.Draw(0.0, 2.0/3.0*grid_width-0.5, -1.5);
    glEnd();

    glColor4f(0.0, 0.0, 0.5, 0.8);
    txt = glFont.Text("300");
    txt.Draw(0.0, grid_width-0.5, -1.5);
    glEnd();

    glColor4f(0.0, 0.0, 0.5, 0.8);
    txt = glFont.Text("-100");
    txt.Draw(0.0, -1.0/3.0*grid_width-0.5, -1.5);
    glEnd();

    glColor4f(0.0, 0.0, 0.5, 0.8);
    txt = glFont.Text("-200");
    txt.Draw(0.0, -2.0/3.0*grid_width-0.5, -1.5);
    glEnd();

    glColor4f(0.0, 0.0, 0.5, 0.8);
    txt = glFont.Text("-300");
    txt.Draw(0.0, -grid_width-0.5, -1.5);
    glEnd();

    if(fps != -1)
    {
        pangolin::GlFont glFont2(fontpath, 22.0);
        glColor4f(0.0, 0.0, 0.5, 0.8);
        txt = glFont2.Text((string("Fps:")+to_string(fps)).c_str());
        txt.Draw(-25, 76, 3);
        glEnd();
    }
    if(tracked_num != -1)
    {
        pangolin::GlFont glFont2(fontpath, 22.0);
        glColor4f(0.0, 0.0, 0.5, 0.8);
        txt = glFont2.Text((string("tracked features:")+to_string(tracked_num)).c_str());
        txt.Draw(-25, 70, 0.0);
        glEnd();
    }*/
}

void MapDrawer::DrawKeyFrames()
{
    unique_lock<mutex> lock(mMutexCamera);
    const float &w = mKeyFrameSize;
    const float h = w*0.75;
    const float z = w*0.6;


    //if(bDrawKF)
    {
        // camera pose, GPS good, current keyframes
        for(size_t i=cvTwcs.size() - 1; i<cvTwcs.size(); i++)
        {
            if(cvTwcs[i].empty() || cvTwcs[i].dims < 2)
                continue;

            cv::Mat Twc= cvTwcs[i].t();
            cv::Mat t = Twc.row(3).colRange(0,3);
            t = t/mScaleic;
            t.copyTo(Twc.row(3).colRange(0,3));

            glPushMatrix();

            glMultMatrixf(Twc.ptr<GLfloat>(0));

            glLineWidth(mKeyFrameLineWidth);
            glColor3f(0.0f,1.0f,1.0f);
            glBegin(GL_LINES);
            glVertex3f(0,0,0);
            glVertex3f(w,h,z);
            glVertex3f(0,0,0);
            glVertex3f(w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,h,z);

            glVertex3f(w,h,z);
            glVertex3f(w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(-w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(w,h,z);

            glVertex3f(-w,-h,z);
            glVertex3f(w,-h,z);
            glEnd();

            glPopMatrix();
        }



    }



    if(cvTwcs.size() > 1)
    {
        for(size_t i = 0; i < cvTwcs.size() - 1; i++)
        {
            if(cvTwcs[i].empty() || cvTwcs[i + 1].empty())
            {
                continue;
            }

            cv::Mat Twc1 = cvTwcs[i].clone();
            cv::Mat Ow1 = Twc1.rowRange(0,3).col(3);
            cv::Mat Twc2 = cvTwcs[i + 1].clone();
            cv::Mat Ow2 = Twc2.rowRange(0,3).col(3);
            float tmp_z1 = Ow1.at<float>(2, 0);
            float tmp_z2 = Ow2.at<float>(2, 0);
            glLineWidth(2 * mGraphLineWidth);
            if(abs(0.5 * (tmp_z1 + tmp_z2)) < 2.5)
                glColor4f(1.0f,0.0f,1.0f,0.6f);
            else if (abs(0.5 * (tmp_z1 + tmp_z2)) < 7.5)
                glColor4f(1.0f,0.0f,1.0f,0.6f);
            else if (abs(0.5 * (tmp_z1 + tmp_z2)) < 12.5)
                glColor4f(1.0f,0.0f,1.0f,0.6f);
            else
                glColor4f(1.0f,0.0f,1.0f,0.6f);
            glBegin(GL_LINES);
            Ow1 = Ow1/mScaleic;
            Ow2 = Ow2/mScaleic;
            glVertex3f(Ow1.at<float>(0),Ow1.at<float>(1),Ow1.at<float>(2));
            glVertex3f(Ow2.at<float>(0),Ow2.at<float>(1),Ow2.at<float>(2));
            glEnd();
        }
    }

    if(cvKftwcs.size() > 1)
    {
        for(size_t i = 0; i < cvKftwcs.size() - 1; i++)
        {
            if(cvKftwcs[i].empty() || cvKftwcs[i + 1].empty())
            {
                continue;
            }

            cv::Mat Ow1 = cvKftwcs[i].clone();

            cv::Mat Ow2 = cvKftwcs[i + 1].clone();

            glLineWidth(2 * mGraphLineWidth);
            glColor4f(0.0f,1.0f,0.0f,0.6f);
            glBegin(GL_LINES);
            Ow1 = Ow1/mScaleic;
            Ow2 = Ow2/mScaleic;
            glVertex3f(Ow1.at<float>(0),Ow1.at<float>(1),Ow1.at<float>(2));
            glVertex3f(Ow2.at<float>(0),Ow2.at<float>(1),Ow2.at<float>(2));
            glEnd();
        }
    }
}

void MapDrawer::SetKeyFrames(std::vector<Eigen::Vector3d> _vposes)
{
    unique_lock<mutex> lock(mMutexCamera);
    cvKftwcs.clear();
    for(int i = 0; i < _vposes.size(); i++)
    {
        cv::Mat cvtwc;
        cvtwc = Utility::toCvMat(_vposes[i]);
        cvKftwcs.push_back(cvtwc.clone());
    }

}

void MapDrawer::AddOneKeyFrame(Eigen::Vector3d _pose)
{
    unique_lock<mutex> lock(mMutexCamera);
    cv::Mat cvtwc;
    cvtwc = Utility::toCvMat(_pose);
    cvKftwcs.push_back(cvtwc.clone());
}