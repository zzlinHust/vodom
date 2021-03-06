//
// Created by cbt on 18-5-29.
//

#include "Camera.h"
#include "config.h"
using Eigen::Vector3d;
using Eigen::Vector2d;
using Sophus::SE3;

namespace myslam
{

Camera::Camera()
{
    fx_ = Config::get<float>("camera.fx");
    fy_ = Config::get<float>("camera.fy");
    cx_ = Config::get<float>("camera.cx");
    cy_ = Config::get<float>("camera.cy");
    bf_ = Config::get<float>("camera.depth_scale");

    K = ( cv::Mat_<double>(3,3)<<
                                fx_, 0  , cx_,
                                0  , fy_, cy_,
                                0  , 0  , 1  );
}

Camera::~Camera(){}


Vector3d Camera::world2Camera( const Vector3d &p_w , const SE3 &T_c_w )
{
    return T_c_w * p_w;
}

Vector3d Camera::camera2World( const Vector3d &p_c , const SE3 &T_c_w )
{
    return T_c_w.inverse() * p_c;
}

Vector2d Camera::camera2Pixel( const Vector3d &p_c )
{
    return Eigen::Vector2d(fx_ * p_c( 0,0 ) / p_c ( 2,0 ) + cx_ , fy_ * p_c( 1,0 ) / p_c ( 2,0 ) + cy_ );
}

Vector3d Camera::pixel2Camera( const Vector2d &p_p , double depth )
{
    return Vector3d(
            ( p_p ( 0,0 ) - cx_ ) * depth / fx_ ,
            ( p_p ( 1,0 ) - cy_ ) * depth / fy_ ,
            depth
    );
}


Vector3d Camera::pixel2World( const Vector2d &p_p , const SE3 &T_c_W , double depth )
{
    return camera2World( pixel2Camera(p_p , depth) , T_c_W );
}


Vector2d Camera::world2Pixel( const Vector3d &p_w , const SE3 &T_c_w )
{
    return camera2Pixel( world2Camera( p_w , T_c_w) );
}


}

