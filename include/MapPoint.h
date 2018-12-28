//
// Created by cbt on 18-12-28.
//

#ifndef MYSLAM_MAPPOPINT_H
#define MYSLAM_MAPPOPINT_H

#include "common_include.h"
#include "Frame.h"

namespace myslam
{
class Frame;

class MapPoint
{
public:
    typedef std::shared_ptr<MapPoint> Ptr;

//    MapPoint();

    MapPoint(unsigned int id , Eigen::Vector3d position , Eigen::Vector3d norm);

    ~MapPoint();

    static MapPoint::Ptr createMapPoint();

    unsigned int mId;
    Eigen::Vector3d mPos3d;
    Eigen::Vector3d mDirObv;
    cv::Mat mDescriptor;
//    Frame::Ptr mRefFrame;


private:

};

}
#endif //MYSLAM_MAPPOPINT_H
