//
// Created by cbt on 18-12-28.
//

#include "MapPoint.h"
using Eigen::Vector3d;

namespace myslam
{
MapPoint::MapPoint( unsigned int  id , Eigen::Vector3d position , Eigen::Vector3d dir)
        : mId(id) , mPos3d(position) , mDirObv(dir)
{}

MapPoint::~MapPoint(){}

MapPoint::Ptr MapPoint::createMapPoint()
{
    static unsigned int factory_id = 0;
    return MapPoint::Ptr( new MapPoint( factory_id++, Vector3d(0,0,0), Vector3d(0,0,0) ) );
}

}
