//
// Created by cbt on 18-12-21.
//

#ifndef MYSLAM_FRAME_H
#define MYSLAM_FRAME_H

#include "common_include.h"
#include "Input.h"
#include "MapPoint.h"

namespace myslam
{
class MapPoint;

class Frame
{
public:
    typedef std::shared_ptr<Frame> Ptr;

    Frame(unsigned int id, Input::Ptr input);

    ~Frame();

    static Frame::Ptr CreateFrame(Input::Ptr input);

    void SetPose(Sophus::SE3 pose);

    /** **/
    unsigned int mId;
    Sophus::SE3 T_c_w_; // pose


    /** camera intrinsics **/
    cv::Mat mK;
    float mbf;

    /** input img info **/
    cv::Mat mImage;
    std::vector<cv::KeyPoint> mKeyPoints;
    cv::Mat mDescriptor;
    std::vector<float> mDepth;

    /** map points **/
//    std::vector<MapPoint::Ptr> mMapPoints;


private:


};
}
#endif //MYSLAM_FRAME_H
