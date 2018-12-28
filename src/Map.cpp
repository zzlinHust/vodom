//
// Created by cbt on 18-5-30.
//

#include "Map.h"

using namespace std;

namespace myslam
{

    Map::Map() {};


    void Map::insertKeyFrame(Frame::Ptr frame)
    {

        std::cout << "Key frame size = " << keyFrames_.size() << std::endl;

        if(keyFrames_.find(frame->mId) == keyFrames_.end())
        {
            keyFrames_.insert( std::make_pair(frame->mId,frame) );
        }
        else
        {
            keyFrames_[frame->mId] = frame;
        }
    }


    void Map::insertMapPoint(MapPoint::Ptr mapPoint)
    {
        if ( map_points_.find(mapPoint->mId) == map_points_.end() )
        {
            map_points_.insert( std::make_pair(mapPoint->mId, mapPoint) );
        }
        else
        {
            map_points_[mapPoint->mId] = mapPoint;
        }
    }

}