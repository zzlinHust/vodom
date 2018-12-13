//
// Created by cbt on 18-5-30.
//
#include "myslam/mapPoint.h"
#include "myslam/common_include.h"

namespace myslam
{

    MapPoint::MapPoint() : id_(-1) , pos_(Vector3d(0,0,0)) , norm_(Vector3d(0,0,0)) ,
                           observed_times_(0) , correct_times_(0) {};

    MapPoint::MapPoint( long id , Vector3d position , Vector3d norm)
            : id_(id) , pos_(position) , norm_(norm) , observed_times_(0) , correct_times_(0) {}

    MapPoint::~MapPoint(){}

    MapPoint::Ptr MapPoint::createMapPoint()
    {
        static long factory_id = 0;
        return MapPoint::Ptr( new MapPoint( factory_id++, Vector3d(0,0,0), Vector3d(0,0,0) ) );
    }

}
