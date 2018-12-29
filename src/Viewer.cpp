//
// Created by cbt on 18-12-29.
//

/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include <opencv/cv.hpp>
#include <thread>

#include "Viewer.h"
#include "MapPoint.h"

using namespace std;
namespace myslam
{

Viewer::Viewer()
{

    float fps = 10;
    if(fps<1)
        fps=30;
    mT = 1e3/fps;

    mImageWidth = 1241;
    mImageHeight = 376;
    if(mImageWidth<1 || mImageHeight<1)
    {
        mImageWidth = 640;
        mImageHeight = 480;
    }

    mViewpointX = 0;
    mViewpointY = -100;
    mViewpointZ = -0.1f;
    mViewpointF = 2000;

    mKeyFrameSize = 0.6;
    mKeyFrameLineWidth = 2;
    mGraphLineWidth = 1;
    mPointSize = 2;
    mCameraSize = 0.7;
    mCameraLineWidth = 3;

}

// pangolin库的文档：http://docs.ros.org/fuerte/api/pangolin_wrapper/html/namespacepangolin.html
void Viewer::Run()
{
    //这个变量配合SetFinish函数用于指示该函数是否执行完毕
    mbFinished = false;
    mbStopped = false;

    pangolin::CreateWindowAndBind("Stereo Odometry by lzz : Map Viewer",1024,768);

    // 3D Mouse handler requires depth testing to be enabled
    // 启动深度测试，OpenGL只绘制最前面的一层，绘制时检查当前像素前面是否有别的像素，如果别的像素挡住了它，那它就不会绘制
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    // 在OpenGL中使用颜色混合
    glEnable(GL_BLEND);
    // 选择混合选项
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // 新建按钮和选择框，第一个参数为按钮的名字，第二个为默认状态，第三个为是否有选择框
    pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
    pangolin::Var<bool> menuFollowCamera("menu.Follow Camera",true,true);
    pangolin::Var<bool> menuShowPoints("menu.Show Points",true,true);
    pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames",true,true);
    pangolin::Var<bool> menuShowGraph("menu.Show Graph",true,true);


    // Define Camera Render Object (for view / scene browsing)
    // 定义相机投影模型：ProjectionMatrix(w, h, fu, fv, u0, v0, zNear, zFar)
    // 定义观测方位向量：观测点位置：(mViewpointX mViewpointY mViewpointZ)
    //                观测目标位置：(0, 0, 0)
    //                观测的方位向量：(0.0,-1.0, 0.0)
    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024,768,mViewpointF,mViewpointF,512,389,0.1,1000),
            pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0)
    );

    // Add named OpenGL viewport to window and provide 3D Handler
    // 定义显示面板大小，orbslam中有左右两个面板，昨天显示一些按钮，右边显示图形
    // 前两个参数（0.0, 1.0）表明宽度和面板纵向宽度和窗口大小相同
    // 中间两个参数（pangolin::Attach::Pix(175), 1.0）表明右边所有部分用于显示图形
    // 最后一个参数（-1024.0f/768.0f）为显示长宽比
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    pangolin::OpenGlMatrix Twc;
    Twc.SetIdentity();

//    cv::namedWindow("Stereo Odometry by lzz");

    bool bFollow = true;
    bool bLocalizationMode = false;
    int nMapIndex = 0;

    while(1)
    {
        // 清除缓冲区中的当前可写的颜色缓冲 和 深度缓冲
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // 步骤1：得到最新的相机位姿
        GetCurrentOpenGLCameraMatrix(Twc);

        // 步骤2：根据相机的位姿调整视角
        // menuFollowCamera为按钮的状态，bFollow为真实的状态
        if(menuFollowCamera && bFollow)
        {
            s_cam.Follow(Twc);
        }
        else if(menuFollowCamera && !bFollow)
        {
            s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0));
            s_cam.Follow(Twc);
            bFollow = true;
        }
        else if(!menuFollowCamera && bFollow)
        {
            bFollow = false;
        }


        d_cam.Activate(s_cam);
        // 步骤3：绘制地图和图像
        // 设置为白色，glClearColor(red, green, blue, alpha），数值范围(0, 1)
        glClearColor(1.0f,1.0f,1.0f,1.0f);
        DrawCurrentCamera(Twc);
        if(menuShowKeyFrames || menuShowGraph)
            DrawKeyFrames(menuShowKeyFrames,menuShowGraph);
        if(menuShowPoints)
            DrawMapPoints();

        pangolin::FinishFrame();

//        cv::Mat im = mpFrameDrawer->DrawFrame();
//        cv::imshow("CTI-ORB: Current Frame",im);
        cv::waitKey(mT);


        if(Stop())
        {
            while(isStopped())
            {
                //usleep(3000);
                std::this_thread::sleep_for(std::chrono::milliseconds(3));

            }
        }

        if(CheckFinish())
            break;
    }

    SetFinish();
}

void Viewer::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool Viewer::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void Viewer::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool Viewer::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

void Viewer::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(!mbStopped)
        mbStopRequested = true;
}

bool Viewer::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool Viewer::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);

    if(mbFinishRequested)
        return false;
    else if(mbStopRequested)
    {
        mbStopped = true;
        mbStopRequested = false;
        return true;
    }

    return false;

}

void Viewer::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopped = false;
}


void Viewer::DrawMapPoints()
{
    //取出所有的地图点
    const auto &vpMPs = map->map_points_;
    //取出mvpReferenceMapPoints，也即局部地图d点

    if(vpMPs.empty())
        return;

    // for AllMapPoints
    //显示所有的地图点（不包括局部地图点），大小为2个像素，黑色
    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.0,0.0,0.0);

    for(auto &pt : vpMPs)
    {
        auto pos = pt.second->mPos3d;
        glVertex3f(pos[0], pos[1], pos[2]);
    }
    glEnd();

}


// 将相机位姿mCameraPose由Mat类型转化为OpenGlMatrix类型
void Viewer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M)
{
    if(!mCameraPose.empty())
    {
        cv::Mat Rwc(3,3,CV_32F);
        cv::Mat twc(3,1,CV_32F);
        {
            unique_lock<mutex> lock(mMutexCamera);
            Rwc = mCameraPose.rowRange(0,3).colRange(0,3).t();
            twc = -Rwc*mCameraPose.rowRange(0,3).col(3);
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

        M.m[12] = twc.at<float>(0);
        M.m[13] = twc.at<float>(1);
        M.m[14] = twc.at<float>(2);
        M.m[15]  = 1.0;
    }
    else
        M.SetIdentity();
}


//关于gl相关的函数，可直接google, 并加上msdn关键词
void Viewer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc)
{
    //相机模型大小：宽度占总宽度比例为0.08
    const float &w = mCameraSize;
    const float h = w*0.75;
    const float z = w*0.6;

    //百度搜索：glPushMatrix 百度百科
    glPushMatrix();

    //将4*4的矩阵Twc.m右乘一个当前矩阵
    //（由于使用了glPushMatrix函数，因此当前帧矩阵为世界坐标系下的单位矩阵）
    //因为OpenGL中的矩阵为列优先存储，因此实际为Tcw，即相机在世界坐标下的位姿
#ifdef HAVE_GLES
    glMultMatrixf(Twc.m);
#else
    glMultMatrixd(Twc.m);
#endif

    //设置绘制图形时线的宽度
    glLineWidth(mCameraLineWidth);
    //设置当前颜色为绿色(相机图标显示为绿色)
    glColor3f(0.0f,1.0f,0.0f);
    //用线将下面的顶点两两相连
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


//关于gl相关的函数，可直接google, 并加上msdn关键词
void Viewer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph)
{
    //历史关键帧图标：宽度占总宽度比例为0.05
    const float &w = mKeyFrameSize;
    const float h = w*0.75;
    const float z = w*0.6;

    //步骤1：取出所有的关键帧
    const auto &vpKFs = map->keyFrames_;

    //步骤2：显示所有关键帧图标
    //通过显示界面选择是否显示历史关键帧图标
    if(bDrawKF)
    {
        for(auto &kf : vpKFs)
        {
            auto &pKF = kf.second;
            //转置, OpenGL中的矩阵为列优先存储
            cv::Mat Twc = pKF->GetPoseInverse().t();

            glPushMatrix();

            //（由于使用了glPushMatrix函数，因此当前帧矩阵为世界坐标系下的单位矩阵）
            //因为OpenGL中的矩阵为列优先存储，因此实际为Tcw，即相机在世界坐标下的位姿
            glMultMatrixf(Twc.ptr<GLfloat>(0));

            //设置绘制图形时线的宽度
            glLineWidth(mKeyFrameLineWidth);
            //设置当前颜色为蓝色(关键帧图标显示为蓝色) (临时关键帧为红色)

            glColor3f(0.0f,0.0f,1.0f);
            //用线将下面的顶点两两相连
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

}


void Viewer::SetPose(cv::Mat pose)
{
    unique_lock<mutex> lock(mMutexCamera);
    mCameraPose = pose.clone();
}

}
