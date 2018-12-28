//
// Created by cbt on 18-5-30.
//

#ifndef MYSLAM_MAP_H
#define MYSLAM_MAP_H

#include <unordered_map>
#include "common_include.h"
#include "MapPoint.h"
#include "Frame.h"

namespace myslam
{
class Frame;
class MapPoint;

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
