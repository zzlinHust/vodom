//
// Created by cbt on 18-5-30.
//

#ifndef MYSLAM_MAP_H
#define MYSLAM_MAP_H
#include "myslam/mapPoint.h"
#include "myslam/frame.h"
#include "myslam/mapPoint.h"
namespace myslam
{

class Map
{
public:
    typedef std::shared_ptr<Map> Ptr;

    std::unordered_map< unsigned long , MapPoint::Ptr > map_points_;
    std::unordered_map< unsigned long , Frame::Ptr > keyFrames_;

    Map();


    void insertKeyFrame(Frame::Ptr frame);
    void insertMapPoint(MapPoint::Ptr mapPoint);

};


}

#endif //MYSLAM_MAP_H
