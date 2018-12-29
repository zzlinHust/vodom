//
// Created by cbt on 18-5-29.
//

#ifndef VISUALODOMETRY_CAMERA_H
#define VISUALODOMETRY_CAMERA_H

#include "common_include.h"

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
    float fx_, fy_, cx_, cy_, bf_;

    cv::Mat K;

    Camera();
    Camera( float fx, float fy, float cx, float cy, float bf = 0 ) :
            fx_(fx), fy_(fy), cx_(cx), cy_(cy), bf_(bf){}
    ~Camera();


    Eigen::Vector3d world2Camera( const Eigen::Vector3d &p_w , const Sophus::SE3 &T_c_w );
    Eigen::Vector3d camera2World( const Eigen::Vector3d &p_c , const Sophus::SE3 &T_c_w );
    Eigen::Vector2d camera2Pixel( const Eigen::Vector3d &p_c );
    Eigen::Vector3d pixel2Camera( const Eigen::Vector2d &p_p , double depth = 1 );
    Eigen::Vector3d pixel2World ( const Eigen::Vector2d &p_p , const Sophus::SE3 &T_c_W , double depth = 1 );
    Eigen::Vector2d world2Pixel ( const Eigen::Vector3d &p_w , const Sophus::SE3 &T_c_w );
};

}

#endif //VISUALODOMETRY_CAMERA_H
