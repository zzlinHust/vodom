//
// Created by cbt on 18-5-30.
//

#ifndef VISUALODOMETRY_FRAME_H
#define VISUALODOMETRY_FRAME_H

#include "myslam/common_include.h"
#include "myslam/camera.h"

namespace myslam
{

class Frame
{
class MapPoint;

public:
    typedef std::shared_ptr<Frame> Ptr;
    unsigned long id_;    // id of this frame
    double time_stamp_;   // when it is record
    SE3 T_c_w_;           // transform from world to camera
    Camera::Ptr camera_;  // Pinhole RGB-D Camera model
    Mat color_ , depth_;  // color and depth image

public:
    Frame();
    Frame(unsigned long id , double time_stamp = 0 , SE3 T_c_w = SE3() ,
        Camera::Ptr camera = nullptr , Mat color = Mat() , Mat depth = Mat());
    ~Frame();

    static Frame::Ptr createFrame();

    /** find the depth in the depth map **/
    double findDepth(const cv::KeyPoint &kp);

    /** get camera center **/
    Vector3d getCameraCenter() const;

    /** check if a point is in this frame **/
    bool isInFrame(const Vector3d &pt_world);

};

}

#endif //VISUALODOMETRY_FRAME_H
