//
// Created by cbt on 18-5-30.
//

#include "myslam/frame.h"


    Frame::Frame() : id_(-1), time_stamp_(-1), camera_(nullptr) {}

    Frame::Frame(unsigned long id, double time_stamp , SE3 T_c_w , Camera::Ptr camera , Mat color , Mat depth ) :
            id_(id), time_stamp_(time_stamp), T_c_w_(T_c_w), camera_(camera), color_(color), depth_(depth) {}


    Frame::~Frame(){}

    Frame::Ptr Frame::createFrame()
    {
        static long factory_id = 0;
        return Ptr ( new Frame(factory_id++) );
    }

    double Frame::findDepth(const cv::KeyPoint &kp)
    {
        int x = cvRound(kp.pt.x);
        int y = cvRound(kp.pt.y);
        ushort d = depth_.ptr<ushort>(y)[x];

        if( d != 0 ) return double(d) / camera_->depth_scale_;
        else
        {
            int dx[4] = {-1,0,1,0};
            int dy[4] = {0,-1,0,1};

            for( int i = 0 ; i < 4 ; ++i)
            {
                d = depth_.ptr<ushort>( y + dy[i] )[x + dx[i]];
                if( d != 0 ) return double(d) / camera_->depth_scale_;
            }
        }

        return -1.0;
    }

    Vector3d Frame::getCameraCenter() const
    {
        return T_c_w_.inverse().translation();
    }

    bool Frame::isInFrame(const Vector3d &pt_world)
    {

        /// camera == nullptr ???
        Vector3d p_cam = camera_->world2Camera( pt_world, T_c_w_ );
        if ( p_cam(2,0)<0 )
            return false;
        Vector2d pixel = camera_->world2Pixel( pt_world, T_c_w_ );
        return pixel(0,0)>0 && pixel(1,0)>0 \
               && pixel(0,0)<color_.cols \
               && pixel(1,0)<color_.rows;
    }

