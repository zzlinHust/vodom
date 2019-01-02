//
// Created by cbt on 18-5-31.
//

#ifndef MYSLAM_G2O_TYPES_H
#define MYSLAM_G2O_TYPES_H

#include "common_include.h"
#include "Camera.h"
//#include "fortest.h"

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>


namespace myslam
{




class EdgeProjectXYZ2UVPoseOnly : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, g2o::VertexSE3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeProjectXYZ2UVPoseOnly(Eigen::Vector3d pt_3d, Camera::Ptr cam) : point_(pt_3d), camera_(cam){}

    virtual void computeError();
    virtual void linearizeOplus();

    virtual bool read( std::istream& in ){}
    virtual bool write(std::ostream& os) const {};

    Eigen::Vector3d point_;
    Camera::Ptr camera_;
};




class EdgeDirect : public g2o::BaseUnaryEdge<1, double, g2o::VertexSE3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeDirect(){}
    EdgeDirect(Eigen::Vector3d pos, float fx, float fy, float cx, float cy, const cv::Mat *img);

    void computeError() override ;
    void linearizeOplus() override ;

    virtual bool read( std::istream& in ){}
    virtual bool write(std::ostream& os) const {};


public:
    Eigen::Vector3d pos_world;         // 3D point in world frame
    float cx_, cy_, fx_, fy_;// Camera intrinsics
    const cv::Mat* image_ = nullptr;          // reference image

protected:
    inline float getPixelValue ( float x, float y )
    {
        uchar* data = & image_->data[ int ( y ) * image_->step + int ( x ) ];
        float xx = x - floor ( x );
        float yy = y - floor ( y );
        return float (
                ( 1-xx ) * ( 1-yy ) * data[0] + \
                xx* ( 1-yy ) * data[1] + \
                ( 1-xx ) * yy * data[image_->step] + \
                xx * yy * data[image_->step+1] );
    }
};


}



#endif //MYSLAM_G2O_TYPES_H
