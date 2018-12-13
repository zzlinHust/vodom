//
// Created by cbt on 18-5-29.
//

#ifndef VISUALODOMETRY_CAMERA_H
#define VISUALODOMETRY_CAMERA_H

#include "myslam/common_include.h"

namespace myslam
{

    /**
     * 存储相机内参
     * 坐标转换
     *
     */
class Camera
{
public:
    typedef std::shared_ptr<Camera> Ptr;
    float fx_, fy_, cx_, cy_, depth_scale_;

    cv::Mat K;

    Camera();
    Camera( float fx, float fy, float cx, float cy, float depth_scale = 0 ) :
            fx_(fx), fy_(fy), cx_(cx), cy_(cy), depth_scale_(depth_scale){}
    ~Camera();

    Vector3d world2Camera( const Vector3d &p_w , const SE3 &T_c_w );
    Vector3d camera2World( const Vector3d &p_c , const SE3 &T_c_w );
    Vector2d camera2Pixel( const Vector3d &p_c );
    Vector3d pixel2Camera( const Vector2d &p_p , double depth = 1 );
    Vector3d pixel2World ( const Vector2d &p_p , const SE3 &T_c_W , double depth = 1 );
    Vector2d world2Pixel ( const Vector3d &p_w , const SE3 &T_c_w );
};

}

#endif //VISUALODOMETRY_CAMERA_H
