//
// Created by cbt on 18-5-30.
//

#ifndef VISUALODOMETRY_MAPPOINT_H
#define VISUALODOMETRY_MAPPOINT_H

#include "common_include.h"

namespace myslam
{
class Frame;

class MapPoint
{
public:
    typedef std::shared_ptr<MapPoint> Ptr;
    unsigned long id_;
    Vector3d pos_;          // position in world
    Vector3d norm_;         // Normal of viewing direction
    Mat descriptor_;        // Descriptor for matching
    int observed_times_;    // being observed by feature matching algorithm
    int correct_times_;     // being an inliner in pose estimation


    MapPoint();
    MapPoint( long id , Vector3d position , Vector3d norm);
    ~MapPoint();

    static MapPoint::Ptr createMapPoint();

};



}

#endif //VISUALODOMETRY_MAPPOINT_H
