//
// Created by cbt on 18-12-29.
//

#ifndef MYSLAM_VIEWER_H
#define MYSLAM_VIEWER_H

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


#ifndef VIEWER_H
#define VIEWER_H


#include "common_include.h"
#include "Map.h"
#include <pangolin/pangolin.h>
#include <mutex>
namespace myslam
{

    class Tracking;
    class FrameDrawer;
    class MapDrawer;
    class System;

    class Viewer
    {
    public:
        Viewer();

        // Main thread function. Draw points, keyframes, the current camera pose and the last processed
        // frame. Drawing is refreshed according to the camera fps. We use Pangolin.
        void Run();

        void RequestFinish();

        void RequestStop();

        bool isFinished();

        bool isStopped();

        void Release();

        void SetMap(Map::Ptr m){ map = m;}

        void SetPose(cv::Mat);

        void SetFrame(const Frame::Ptr &frame);

    private:

        bool Stop();

        Map::Ptr map;

        // 1/fps in ms
        double mT;
        float mImageWidth, mImageHeight;

        float mViewpointX, mViewpointY, mViewpointZ, mViewpointF;

        bool CheckFinish();
        void SetFinish();
        bool mbFinishRequested;
        bool mbFinished;
        std::mutex mMutexFinish;

        bool mbStopped;
        bool mbStopRequested;
        bool mbReuseMap;
        std::string mMapfile;
        std::mutex mMutexStop;


        void DrawMapPoints();
        void DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph);
        void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc);
        void SetCurrentCameraPose(const cv::Mat &Tcw);
        void GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M);

        float mKeyFrameSize;
        float mKeyFrameLineWidth;
        float mGraphLineWidth;
        float mPointSize;
        float mCameraSize;
        float mCameraLineWidth;

        cv::Mat mCameraPose;
        std::mutex mMutexCamera;

        Frame::Ptr mFrame;
        std::mutex mMutexFrame;

    };

}


#endif // VIEWER_H





#endif //MYSLAM_VIEWER_H
