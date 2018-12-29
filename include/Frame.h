//
// Created by cbt on 18-12-21.
//

#ifndef MYSLAM_FRAME_H
#define MYSLAM_FRAME_H

#include "common_include.h"
#include "Input.h"
#include "MapPoint.h"
#include "Camera.h"

namespace myslam
{
class MapPoint;

class Frame
{
#define GRID_ROWS 48
#define GRID_COLS 64
public:
    typedef std::shared_ptr<Frame> Ptr;

    Frame(unsigned int id, Input::Ptr input, Camera::Ptr camera);

    ~Frame();

    static Frame::Ptr CreateFrame(Input::Ptr input, Camera::Ptr camera);

    void SetPose(Sophus::SE3 pose) { T_c_w_ = pose; }

    Eigen::Vector3d GetCameraCenter();

    cv::Mat GetPoseInverse();

    cv::Mat GetPose();

    /** **/
    unsigned int mId;
    Sophus::SE3 T_c_w_; // pose


    /** camera intrinsics **/
    Camera::Ptr mCamera;
//    cv::Mat mK;
//    float mbf;

    /** input img info **/
    std::vector<cv::Mat> mImagePyr;
    std::vector<cv::KeyPoint> mKeyPoints;
    cv::Mat mDescriptor;
    std::vector<float> mDepth;


    /** map points **/
    std::vector<std::shared_ptr<MapPoint>> mMapPoints;
    std::vector<size_t> mGrid[GRID_ROWS][GRID_COLS];


private:
    void DistributeKeyPoints2Grid();


};
}
#endif //MYSLAM_FRAME_H
