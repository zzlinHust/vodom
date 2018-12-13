//
// Created by cbt on 18-5-30.
//

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <algorithm>
#include <boost/timer.hpp>

#include "myslam/config.h"
#include "myslam/visual_odometry.h"
#include "myslam/g2o_types.h"


namespace myslam
{

VisualOdometry::VisualOdometry() : state_(INITIALIZING) , ref_(nullptr) , curr_(nullptr) , map_(new Map) , num_lost_(0),
                                   num_inliers_(0)
{
    num_of_features_    = Config::get<int> ( "number_of_features" );
    scale_factor_       = Config::get<double> ( "scale_factor" );
    level_pyramid_      = Config::get<int> ( "level_pyramid" );
    match_ratio_        = Config::get<float> ( "match_ratio" );
    max_num_lost_       = Config::get<int> ( "max_num_lost" );
    min_inliers_        = Config::get<int> ( "min_inliers" );
    key_frame_min_rot   = Config::get<double> ( "keyframe_rotation" );
    key_frame_min_trans = Config::get<double> ( "keyframe_translation" );
    orb_ = cv::ORB::create ( num_of_features_, scale_factor_, level_pyramid_ );

}


VisualOdometry::~VisualOdometry() {}


bool VisualOdometry::addFrame(Frame::Ptr frame)
{
    switch ( state_ )
    {
        case INITIALIZING:
        {
            state_ = OK;
            curr_ = ref_ = frame;
            map_->insertKeyFrame(frame);
            extractKeyPoints();
            computeDescriptors();
            setRef3DPoints();  /// compute the 3d position of features in ref frame
            break;
        }

        case OK:
        {
            curr_ = frame;
            extractKeyPoints();
            computeDescriptors();
            featureMatching();
            poseEstimationPnP();

            if( checkEstimatedPose() ) // a good estimation
            {
                curr_->T_c_w_ = T_c_r_estimated_ * ref_->T_c_w_;
                ref_ = curr_;
                setRef3DPoints();
                num_lost_ = 0;
                if( checkKeyFrame() )  // is a key-frame
                {
                    addKeyFrame();
                }
            }
            else // bad estimation due to various reasons
            {
                ++num_lost_;
                if(num_lost_ > max_num_lost_)
                    state_ = LOST;

                return false;
            }
            break;
        }

        case LOST:
        {
            std::cout << "VO has lost." << std::endl;
            break;
        }

        default:
            break;
    }

    return true;
}


void VisualOdometry::extractKeyPoints()
{
    orb_->detect( curr_->color_ , key_points_curr_);
}


void VisualOdometry::computeDescriptors()
{
    orb_->compute( curr_->color_ , key_points_curr_ , descriptor_curr_ );
}


void VisualOdometry::featureMatching()
{
    /** match desp_ref and desp_curr , use OpenCV's brute force match 暴力匹配 **/
    std::vector<cv::DMatch> matches;
    cv::BFMatcher matcher ( cv::NORM_HAMMING );
    matcher.match( descriptor_ref_ , descriptor_curr_ , matches );

    /** select the best matches **/
    float min_dis = std::min_element(
            matches.begin() , matches.end(),
            [] (const cv::DMatch &m1 , const cv::DMatch &m2)    /// lamda
            {
                return m1.distance < m2.distance;
            })->distance;

    feature_matches_.clear();   //////   ?????????????????????? shao le
    for ( cv::DMatch &m : matches)
        if( m.distance < std::max<float>( min_dis*match_ratio_ , 30.0))
            feature_matches_.push_back(m);


    std::cout << "good matches : " << feature_matches_.size() << std::endl;

}


void VisualOdometry::setRef3DPoints()
{
    // select the features with depth measurement
    pts_3d_ref_.clear();
    descriptor_ref_ = Mat();

    for( size_t i = 0 ; i < key_points_curr_.size() ; ++i)
    {
        double d = ref_->findDepth(key_points_curr_[i]);
        if( d > 0 )
        {
            Vector3d p_cam = ref_->camera_->pixel2Camera(
                    Vector2d( key_points_curr_[i].pt.x , key_points_curr_[i].pt.y) , d );
            pts_3d_ref_.push_back( cv::Point3f( p_cam(0,0), p_cam(1,0), p_cam(2,0) ) );
            descriptor_ref_.push_back(descriptor_curr_.row(i));
        }
    }

}


void VisualOdometry::poseEstimationPnP()
{
    // construct the 3d 2d observations
    std::vector<cv::Point3f> pts3d;
    std::vector<cv::Point2f> pts2d;

    for( cv::DMatch &m : feature_matches_ )
    {
        pts3d.push_back( pts_3d_ref_[m.queryIdx] );
        pts2d.push_back( key_points_curr_[m.trainIdx].pt );
    }

    cv::Mat rvec , tvec , inliers;

    cv::solvePnPRansac( pts3d , pts2d , ref_->camera_->K , Mat() , rvec , tvec , false , 100 , 4.0 , 0.99 , inliers );
    num_inliers_ = inliers.rows;

    std::cout << "pnp inliers" << num_inliers_ << std::endl;

    T_c_r_estimated_ = SE3(
            SO3( rvec.at<double>(0,0), rvec.at<double>(1,0), rvec.at<double>(2,0) ),
            Vector3d( tvec.at<double>(0,0), tvec.at<double>(1,0), tvec.at<double>(2,0) )
    );



    // using bundle adjustment to optimize the pose
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,2>> Block;
    Block::LinearSolverType *linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>();
    Block *solver_ptr = new Block(linearSolver);
    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm( solver );

    g2o::VertexSE3Expmap *pose = new g2o::VertexSE3Expmap();
    pose->setId(0);
    pose->setEstimate( g2o::SE3Quat(
            T_c_r_estimated_.rotation_matrix() , T_c_r_estimated_.translation()
    ));
    optimizer.addVertex( pose );


    //edges
    for( int i = 0 ; i < inliers.rows ; ++i )
    {
        int index = inliers.at<int>(i,0);

        //3D -> 2D projection
        EdgeProjectXYZ2UVPoseOnly *edge = new EdgeProjectXYZ2UVPoseOnly();
        edge->setId(i);
        edge->setVertex( 0 , pose );
        edge->camera_ = curr_->camera_.get();
        edge->point_ = Vector3d ( pts3d[index].x , pts3d[index].y , pts3d[index].z );
        edge->setMeasurement( Vector2d(pts2d[index].x , pts2d[index].y) );
        edge->setInformation( Eigen::Matrix2d::Identity() );
        optimizer.addEdge( edge );
    }


    optimizer.initializeOptimization();
    optimizer.optimize(10);

    T_c_r_estimated_ = SE3(
            pose->estimate().rotation() ,
            pose->estimate().translation()
    );




}


bool VisualOdometry::checkEstimatedPose()
{
    // check if the estimated pose is good
    if ( num_inliers_ < min_inliers_ )
    {
        std::cout << "reject because inlier is too small: " << num_inliers_ << std::endl;
        return false;
    }
    // if the motion is too large, it is probably wrong
    Sophus::Vector6d d = T_c_r_estimated_.log();
    if ( d.norm() > 5.0 )
    {
        std::cout << "reject because motion is too large: " << d.norm() << std::endl;
        return false;
    }
    return true;
}

bool VisualOdometry::checkKeyFrame()
{
    Sophus::Vector6d d = T_c_r_estimated_.log();
    Vector3d trans = d.head<3>();
    Vector3d rot = d.tail<3>();
    if ( rot.norm() >key_frame_min_rot || trans.norm() >key_frame_min_trans )
        return true;
    return false;
}

void VisualOdometry::addKeyFrame()
{
    std::cout << "adding a key-frame" << std::endl;
    map_->insertKeyFrame ( curr_ );
}



}