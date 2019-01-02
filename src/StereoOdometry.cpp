//
// Created by cbt on 18-5-30.
//


#include "StereoOdometry.h"

//#include "g2o_types.h"
using namespace std;

namespace myslam
{

StereoOdometry::StereoOdometry() : state_(INITIALIZING) , ref_(nullptr) , curr_(nullptr) , map_(new Map) , num_lost_(0),
                                   num_inliers_(0)
{

    FeatureExtractionParam param;
    param.levels = 8;
    param.thresh_FAST = 20;
    param.thresh_FAST_min = 7;
    param.feature_num = 1000;
    param.scale_factor = 1.2;
    CameraParam cameraParam;
    cameraParam.b = 386.1448;
    cameraParam.fx = 718.856;
    cameraParam.fy = 718.856;
    cameraParam.cx = 607.1928;
    cameraParam.cy = 185.2157;
    cameraParam.s = 0.0;

    mExtraction = std::make_shared<myslam::FeatureExtraction>(param);

    mK = cv::Mat::eye(3,3,CV_32F);
    mK.at<float>(0,0) = 718.856;
    mK.at<float>(1,1) = 718.856;
    mK.at<float>(0,2) = 607.1928;
    mK.at<float>(1,2) = 185.2157;

    // 图像矫正系数
    // [k1 k2 p1 p2 k3]
    mDistCoef = cv::Mat(4,1,CV_32F);
    mDistCoef.at<float>(0) = 0.0;
    mDistCoef.at<float>(1) = 0.0;
    mDistCoef.at<float>(2) = 0.0;
    mDistCoef.at<float>(3) = 0.0;
    const float k3 = 0.0;
    if(k3 != 0)
    {
        mDistCoef.resize(5);
        mDistCoef.at<float>(4) = k3;
    }
    mbf = 386.1448;

    mCamera = make_shared<Camera>(cameraParam.fx, cameraParam.fy, cameraParam.cx, cameraParam.cy, mbf);
    mMatcher = make_shared<Matcher>();

    mViewer = new Viewer;
    mViewer->SetMap(map_);

//    viewer = new thread(&Viewer::Run, mViewer);
}


StereoOdometry::~StereoOdometry() {

}

void StereoOdometry::GrabImage(cv::Mat img_l, cv::Mat img_r)
{
    myslam::Input::Ptr input = std::make_shared<myslam::Input>(img_l, img_r, mK, mDistCoef, mbf, mExtraction);
    addFrame(Frame::CreateFrame(input, mCamera));
}

bool StereoOdometry::addFrame(Frame::Ptr frame)
{
    switch ( state_ )
    {
        case INITIALIZING:
        {
            state_ = OK;
            curr_ = pre_ = frame;
            frame->SetPose(Sophus::SE3());
            addKeyFrame();
            break;
        }

        case OK:
        {
            curr_ = frame;

            // TODO: matching and estimate
            /** 1. 直接法跟踪特征点并获取初始位姿 **/
            mMatcher->DirectMethodMatching(curr_, pre_, mExtraction);

            /** 2. 特征点匹配 **/
            FeatureMatching();

            /** 3. 重投影位姿优化 **/

            addKeyFrame();
            mViewer->SetPose(curr_->GetPose());
            /*
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
             */
            pre_ = curr_;
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

void StereoOdometry::addKeyFrame()
{
    for(int i = 0 ; i < curr_->mKeyPoints.size() ; ++i)
    {
        float z = curr_->mDepth[i];
        if(z > 0)
        {
            auto &pt = curr_->mKeyPoints[i].pt;
            Eigen::Vector3d p_w = curr_->mCamera->pixel2World(
                    Eigen::Vector2d(pt.x, pt.y), curr_->T_c_w_, z);
            auto point = MapPoint::createMapPoint(p_w);
            point->mDescriptor = curr_->mDescriptor.row(i).clone();
            point->mDirObv = p_w - curr_->GetCameraCenter();
            point->mDirObv.normalize();

            map_->insertMapPoint(point);
            curr_->mMapPoints[i] = point;
        }
    }
    map_->insertKeyFrame(curr_);
    ref_ = curr_;
}

void StereoOdometry::FeatureMatching()
{
    /**
     *  根据直接法得到的初始位姿,将上一帧三维点投影到当前帧，在区域内查找出最佳匹配的点。
     */
    const auto &map_points = pre_->mMapPoints;
    const auto &key_points = curr_->mKeyPoints;
    const auto &descriptor = curr_->mDescriptor;

    /** 1. 投影3d点到当前帧,获取候选匹配特征点 **/
    vector<vector<pair<float, int>>> matches(key_points.size());   // 与当前帧的特征可能匹配的地图点
    for(int i = 0 ; i < map_points.size() ; ++i)
    {
        auto &point = map_points[i];
        Eigen::Vector2d uv = pre_->mCamera->world2Pixel(point->mPos3d, curr_->T_c_w_);

        vector<size_t> candidates;
        curr_->GetKeyPointsInArea(cv::Point2f(uv[0], uv[1]), 15, candidates);

        int best_dist = 100;
        int best_index = -1;
        for( const auto &id : candidates)
        {
            int dist = Matcher::HammingDistance(point->mDescriptor, descriptor.row(id));
            if(dist < best_dist)
            {
                best_dist = dist;
                best_index = id;
            }
        }
        if(best_index != -1)
            matches[best_index].push_back(make_pair(best_dist, i));
    }

    feature_matches_.resize(key_points.size(), -1);
    for(int i = 0 ; i < key_points.size() ; ++i)
    {
        if(matches[i].empty()) continue;

        int best_match = std::min_element(
            matches[i].begin() , matches[i].end(),
            [] (const pair<float, int> &m1 , pair<float, int> &m2)
            {
                return m1.first < m2.first;
            })->second;
        feature_matches_[i] = best_match;
    }
}

void StereoOdometry::PoseOptimization()
{
    // 初始化g2o
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,1>> DirectBlock;  // 求解的向量是6＊1的
    DirectBlock::LinearSolverType* linearSolver = new g2o::LinearSolverDense< DirectBlock::PoseMatrixType > ();
    DirectBlock* solver_ptr = new DirectBlock ( linearSolver );

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( solver_ptr ); // L-M
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm ( solver );
    optimizer.setVerbose( false );

    g2o::SE3Quat se3pose( curr_->T_c_w_.rotation_matrix(), curr_->T_c_w_.translation() )
    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
    pose->setEstimate ( se3pose );
    pose->setId ( 0 );
    optimizer.addVertex ( pose );

    const auto &point3d = pre_->mMapPoints;
    int id = 0;
    for(int j = 0 ; j < feature_matches_.size() ; ++j)
    {
        int index = feature_matches_[j];
        if( index < 0) continue;

        const auto &pt =  curr_->mKeyPoints[j].pt;
        EdgeProjectXYZ2UVPoseOnly *edge = new EdgeProjectXYZ2UVPoseOnly(point3d[index]->mPos3d, mCamera);
        edge->setVertex(0, pose);
        edge->setMeasurement( Eigen::Vector2d(pt.x, pt.y) );
        edge->setInformation( Eigen::Matrix<double,1,1>::Identity());
        edge->setId(id++);
        optimizer.addEdge ( edge );
    }

    optimizer.initializeOptimization();
//        log("match", "begin opt");
    optimizer.optimize ( 30 );

    se3pose = pose->estimate();


}


//void StereoOdometry::featureMatching()
//{
//    /** match desp_ref and desp_curr , use OpenCV's brute force match 暴力匹配 **/
//    std::vector<cv::DMatch> matches;
//    cv::BFMatcher matcher ( cv::NORM_HAMMING );
//    matcher.match( descriptor_ref_ , descriptor_curr_ , matches );
//
//    /** select the best matches **/
//    float min_dis = std::min_element(
//            matches.begin() , matches.end(),
//            [] (const cv::DMatch &m1 , const cv::DMatch &m2)    /// lamda
//            {
//                return m1.distance < m2.distance;
//            })->distance;
//
//    feature_matches_.clear();
//    for ( cv::DMatch &m : matches)
//        if( m.distance < std::max<float>( min_dis*match_ratio_ , 30.0))
//            feature_matches_.push_back(m);
//
//
//    std::cout << "good matches : " << feature_matches_.size() << std::endl;
//
//}


/*
void StereoOdometry::poseEstimationPnP()
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


bool StereoOdometry::checkEstimatedPose()
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

bool StereoOdometry::checkKeyFrame()
{
    Sophus::Vector6d d = T_c_r_estimated_.log();
    Vector3d trans = d.head<3>();
    Vector3d rot = d.tail<3>();
    if ( rot.norm() >key_frame_min_rot || trans.norm() >key_frame_min_trans )
        return true;
    return false;
}

void StereoOdometry::addKeyFrame()
{
    std::cout << "adding a key-frame" << std::endl;
    map_->insertKeyFrame ( curr_ );
}
*/




}