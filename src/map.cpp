//
// Created by cbt on 18-5-30.
//

#include "myslam/map.h"


namespace myslam
{

    Map::Map() {};


    void Map::insertKeyFrame(Frame::Ptr frame)
    {

        std::cout << "Key frame size = " << keyFrames_.size() << std::endl;

        if(keyFrames_.find(frame->id_) == keyFrames_.end())
        {
            keyFrames_.insert( std::make_pair(frame->id_,frame) );
        }
        else
        {
            keyFrames_[frame->id_] = frame;
        }
    }


    void Map::insertMapPoint(MapPoint::Ptr mapPoint)
    {
        if ( map_points_.find(mapPoint->id_) == map_points_.end() )
        {
            map_points_.insert( std::make_pair(mapPoint->id_, mapPoint) );
        }
        else
        {
            map_points_[mapPoint->id_] = mapPoint;
        }
    }

}