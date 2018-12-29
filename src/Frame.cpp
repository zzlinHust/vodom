//
// Created by cbt on 18-12-21.
//

#include "Frame.h"


using namespace std;

namespace myslam
{
Frame::Frame(unsigned int id, Input::Ptr input, Camera::Ptr camera) :mId(id), T_c_w_(Sophus::SE3()), mCamera(camera),
          mImagePyr(input->mLeft.imgPyr), mKeyPoints(input->mLeft.keyPoints), mDescriptor(input->mLeft.descriptor),
          mDepth(input->mLeft.depth)
{
     /** 1. 分到格子中，获取某一区域内特征点索引 **/
     DistributeKeyPoints2Grid();

    mMapPoints.resize(mKeyPoints.size());

}


Frame::~Frame() {}

Frame::Ptr Frame::CreateFrame(Input::Ptr input, Camera::Ptr camera)
{
     static unsigned int factory = 0;
     return std::make_shared<Frame>(factory++, input, camera);
}

void Frame::DistributeKeyPoints2Grid()
{
     static float grid_height_inv = GRID_ROWS / float(mImagePyr[0].rows);
     static float grid_width_inv  = GRID_COLS / float(mImagePyr[0].cols);

     for(size_t i = 0 ; i < mKeyPoints.size() ; ++i)
     {
          auto &kp = mKeyPoints[i];
          mGrid[int(kp.pt.y * grid_height_inv)][int(kp.pt.x * grid_width_inv)].push_back(i);
     }
}

Eigen::Vector3d Frame::GetCameraCenter()
{
    return T_c_w_.inverse().translation();
}

cv::Mat Frame::GetPoseInverse()
{
    auto m = T_c_w_.inverse().matrix();
    cv::Mat cvMat(4,4,CV_32F);
    for(int i=0;i<4;i++)
        for(int j=0; j<4; j++)
            cvMat.at<float>(i,j) = m(i,j);
    return cvMat.clone();
}

cv::Mat Frame::GetPose()
{
    auto m = T_c_w_.matrix();
    cv::Mat cvMat(4,4,CV_32F);
    for(int i=0;i<4;i++)
        for(int j=0; j<4; j++)
            cvMat.at<float>(i,j) = m(i,j);
    return cvMat.clone();
}

}