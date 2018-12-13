//
// Created by cbt on 18-5-30.
//

#ifndef VISUALODOMETRY_VISUAL_ODOMETRY_H
#define VISUALODOMETRY_VISUAL_ODOMETRY_H


#include "map.h"
#include "common_include.h"
#include "opencv2/features2d/features2d.hpp"


namespace myslam
{

class VisualOdometry
{
public:
    typedef std::shared_ptr<VisualOdometry> Ptr;
    enum VOState
    {
        INITIALIZING = -1,
        OK = 0,
        LOST
    };

    VOState state_;      // current vo status
    Map::Ptr map_;       // map with all frames and map points
    Frame::Ptr ref_;     // reference frame
    Frame::Ptr curr_;    // current frame

    cv::Ptr<cv::ORB> orb_;   // orb detector and computer
    std::vector<cv::Point3f> pts_3d_ref_;  // 3d points in reference frame
    std::vector<cv::KeyPoint> key_points_curr_; // key points in current frame
    Mat descriptor_curr_; // descriptor in current frame
    Mat descriptor_ref_;  // descriptor in reference frame
    std::vector<cv::DMatch> feature_matches_;

    SE3 T_c_r_estimated_;  // the estimated pose of current frame
    int num_inliers_;      // number of inlier features in icp
    int num_lost_;         // number of lost times


    int num_of_features_;
    double scale_factor_;    // scale in image pyramid
    int level_pyramid_;       // number of pyramid levels
    float match_ratio_;       // ratio for selecting good matches
    int max_num_lost_;        // max number of continuous lost times
    int min_inliers_;         // minimal inliers

    double key_frame_min_rot;  // minimal rotation of two key-frames
    double key_frame_min_trans; // mimimal translation of two key-frames

public:
    /** functions **/

    VisualOdometry();
    ~VisualOdometry();

    bool addFrame(Frame::Ptr frame);


protected:
    /** inner operation **/
    void extractKeyPoints();
    void computeDescriptors();
    void featureMatching();
    void poseEstimationPnP();
    void setRef3DPoints();

    void addKeyFrame();
    bool checkEstimatedPose();
    bool checkKeyFrame();

};

}



#endif //VISUALODOMETRY_VISUAL_ODOMETRY_H
