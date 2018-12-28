//
// Created by cbt on 18-5-30.
//

#ifndef VISUALODOMETRY_VISUAL_ODOMETRY_H
#define VISUALODOMETRY_VISUAL_ODOMETRY_H


#include "common_include.h"
#include "Map.h"
#include "FeatureExtraction.h"
#include "Frame.h"
#include "Input.h"

namespace myslam
{

class StereoOdometry
{
public:
    typedef std::shared_ptr<StereoOdometry> Ptr;
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

    std::vector<cv::Point3f> pts_3d_local_;  // 3d points in reference frame
    std::vector<cv::KeyPoint> key_points_curr_; // key points in current frame
    std::vector<int> feature_matches_;

    Sophus::SE3 T_c_r_estimated_;  // the estimated pose of current frame
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

    StereoOdometry();
    ~StereoOdometry();

    bool addFrame(Frame::Ptr frame);


private:
    /** inner operation **/

    void BruteForceMatching();  /// 仅匹配，无位姿

    void OpticalFlowMatching(); /// 匹配，并有初始位姿

    void DirectMethodMatching(); /// 匹配，并有初始位姿


    void TrackKeyFrame();

    void TrackLastFrame();

    void TrackLocalMap();

    void Stereoinitialization();


    void poseEstimationPnP();
    void setRef3DPoints();

    void addKeyFrame();
    bool checkEstimatedPose();
    bool checkKeyFrame();

};

}



#endif //VISUALODOMETRY_VISUAL_ODOMETRY_H
