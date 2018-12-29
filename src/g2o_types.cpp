//
// Created by cbt on 18-5-31.
//

#include "g2o_types.h"
using Eigen::Vector3d;

namespace myslam
{
void EdgeProjectXYZRGBD::computeError()
{
    const g2o::VertexSBAPointXYZ* point = static_cast<const g2o::VertexSBAPointXYZ*> ( _vertices[0] );
    const g2o::VertexSE3Expmap* pose = static_cast<const g2o::VertexSE3Expmap*> ( _vertices[1] );
    _error = _measurement - pose->estimate().map ( point->estimate() );
}

void EdgeProjectXYZRGBD::linearizeOplus()
{
    g2o::VertexSE3Expmap* pose = static_cast<g2o::VertexSE3Expmap *> ( _vertices[1] );
    g2o::SE3Quat T ( pose->estimate() );
    g2o::VertexSBAPointXYZ* point = static_cast<g2o::VertexSBAPointXYZ*> ( _vertices[0] );
    Eigen::Vector3d xyz = point->estimate();
    Eigen::Vector3d xyz_trans = T.map ( xyz );
    double x = xyz_trans[0];
    double y = xyz_trans[1];
    double z = xyz_trans[2];

    _jacobianOplusXi = - T.rotation().toRotationMatrix();

    _jacobianOplusXj ( 0,0 ) = 0;
    _jacobianOplusXj ( 0,1 ) = -z;
    _jacobianOplusXj ( 0,2 ) = y;
    _jacobianOplusXj ( 0,3 ) = -1;
    _jacobianOplusXj ( 0,4 ) = 0;
    _jacobianOplusXj ( 0,5 ) = 0;

    _jacobianOplusXj ( 1,0 ) = z;
    _jacobianOplusXj ( 1,1 ) = 0;
    _jacobianOplusXj ( 1,2 ) = -x;
    _jacobianOplusXj ( 1,3 ) = 0;
    _jacobianOplusXj ( 1,4 ) = -1;
    _jacobianOplusXj ( 1,5 ) = 0;

    _jacobianOplusXj ( 2,0 ) = -y;
    _jacobianOplusXj ( 2,1 ) = x;
    _jacobianOplusXj ( 2,2 ) = 0;
    _jacobianOplusXj ( 2,3 ) = 0;
    _jacobianOplusXj ( 2,4 ) = 0;
    _jacobianOplusXj ( 2,5 ) = -1;
}

void EdgeProjectXYZRGBDPoseOnly::computeError()
{
    const g2o::VertexSE3Expmap* pose = static_cast<const g2o::VertexSE3Expmap*> ( _vertices[0] );
    _error = _measurement - pose->estimate().map ( point_ );
}

void EdgeProjectXYZRGBDPoseOnly::linearizeOplus()
{
    g2o::VertexSE3Expmap* pose = static_cast<g2o::VertexSE3Expmap*> ( _vertices[0] );
    g2o::SE3Quat T ( pose->estimate() );
    Vector3d xyz_trans = T.map ( point_ );
    double x = xyz_trans[0];
    double y = xyz_trans[1];
    double z = xyz_trans[2];

    _jacobianOplusXi ( 0,0 ) = 0;
    _jacobianOplusXi ( 0,1 ) = -z;
    _jacobianOplusXi ( 0,2 ) = y;
    _jacobianOplusXi ( 0,3 ) = -1;
    _jacobianOplusXi ( 0,4 ) = 0;
    _jacobianOplusXi ( 0,5 ) = 0;

    _jacobianOplusXi ( 1,0 ) = z;
    _jacobianOplusXi ( 1,1 ) = 0;
    _jacobianOplusXi ( 1,2 ) = -x;
    _jacobianOplusXi ( 1,3 ) = 0;
    _jacobianOplusXi ( 1,4 ) = -1;
    _jacobianOplusXi ( 1,5 ) = 0;

    _jacobianOplusXi ( 2,0 ) = -y;
    _jacobianOplusXi ( 2,1 ) = x;
    _jacobianOplusXi ( 2,2 ) = 0;
    _jacobianOplusXi ( 2,3 ) = 0;
    _jacobianOplusXi ( 2,4 ) = 0;
    _jacobianOplusXi ( 2,5 ) = -1;
}

void EdgeProjectXYZ2UVPoseOnly::computeError()
{
    const g2o::VertexSE3Expmap* pose = static_cast<const g2o::VertexSE3Expmap*> ( _vertices[0] );
    _error = _measurement - camera_->camera2Pixel (
            pose->estimate().map(point_) );
}

void EdgeProjectXYZ2UVPoseOnly::linearizeOplus()
{
    g2o::VertexSE3Expmap* pose = static_cast<g2o::VertexSE3Expmap*> ( _vertices[0] );
    g2o::SE3Quat T ( pose->estimate() );
    Vector3d xyz_trans = T.map ( point_ );
    double x = xyz_trans[0];
    double y = xyz_trans[1];
    double z = xyz_trans[2];
    double z_2 = z*z;

    _jacobianOplusXi ( 0,0 ) =  x*y/z_2 *camera_->fx_;
    _jacobianOplusXi ( 0,1 ) = - ( 1+ ( x*x/z_2 ) ) *camera_->fx_;
    _jacobianOplusXi ( 0,2 ) = y/z * camera_->fx_;
    _jacobianOplusXi ( 0,3 ) = -1./z * camera_->fx_;
    _jacobianOplusXi ( 0,4 ) = 0;
    _jacobianOplusXi ( 0,5 ) = x/z_2 * camera_->fx_;

    _jacobianOplusXi ( 1,0 ) = ( 1+y*y/z_2 ) *camera_->fy_;
    _jacobianOplusXi ( 1,1 ) = -x*y/z_2 *camera_->fy_;
    _jacobianOplusXi ( 1,2 ) = -x/z *camera_->fy_;
    _jacobianOplusXi ( 1,3 ) = 0;
    _jacobianOplusXi ( 1,4 ) = -1./z *camera_->fy_;
    _jacobianOplusXi ( 1,5 ) = y/z_2 *camera_->fy_;
}

EdgeDirect::EdgeDirect(Eigen::Vector3d pos, float fx, float fy, float cx, float cy, const cv::Mat *img) : pos_world(pos),
                       fx_(fx), fy_(fy), cx_(cx), cy_(cy), image_(img)
{

}

void EdgeDirect::computeError()
{
    const g2o::VertexSE3Expmap *vertex = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
    Eigen::Vector3d pos_local = vertex->estimate().map(pos_world);
    float inv_z = 1.0f / float(pos_local[2]);
    float x = float(pos_local[0]) * fx_ * inv_z + cx_;
    float y = float(pos_local[1]) * fy_ * inv_z + cy_;

    if(x < 0)
    {
        _error(0,0) = 0;
        this->setLevel(1);
    }
    else
    {
        _error(0,0) = getPixelValue ( x,y ) - _measurement;
    }
}

void EdgeDirect::linearizeOplus()
{

    if(level() == 1)
    {
        _jacobianOplusXi = Eigen::Matrix<double, 1, 6>::Zero();
        return;
    }

    const g2o::VertexSE3Expmap *vertex = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
    Eigen::Vector3d pos_local = vertex->estimate().map(pos_world);

    double x = pos_local[0];
    double y = pos_local[1];
    double iz = 1.0 / pos_local[2];
    double iz2 = iz * iz;
    float u = fx_ * x * iz + cx_;
    float v = fy_ * y * iz + cy_;

    Eigen::Matrix<double, 2, 6> J_pixel_xi;

    J_pixel_xi ( 0,0 ) = -x * y * iz2 * fx_;
    J_pixel_xi ( 0,1 ) = (1 + x * x * iz2) * fx_;
    J_pixel_xi ( 0,2 ) = -y * iz * fx_;
    J_pixel_xi ( 0,3 ) = iz * fx_;
    J_pixel_xi ( 0,4 ) = 0;
    J_pixel_xi ( 0,5 ) = -x * iz2 * fx_;

    J_pixel_xi ( 1,0 ) = - (1 + y * y * iz2) * fy_;
    J_pixel_xi ( 1,1 ) = x * y * iz2 * fy_;
    J_pixel_xi ( 1,2 ) = x * iz * fy_;
    J_pixel_xi ( 1,3 ) = 0;
    J_pixel_xi ( 1,4 ) = iz * fy_;
    J_pixel_xi ( 1,5 ) = -y * iz2 * fy_;

    Eigen::Matrix<double, 1, 2> pixel_gradient;

    pixel_gradient ( 0,0 ) = ( getPixelValue ( u+1,v )-getPixelValue ( u-1,v ) ) * 0.5; // 像素梯度
    pixel_gradient ( 0,1 ) = ( getPixelValue ( u,v+1 )-getPixelValue ( u,v-1 ) ) * 0.5;
    _jacobianOplusXi = pixel_gradient * J_pixel_xi;

//    log("jacobi", "finish");

}
}