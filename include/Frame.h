//
// Created by cbt on 18-12-21.
//

#ifndef MYSLAM_FRAME_H
#define MYSLAM_FRAME_H

#include "common_include.h"
#include "FeatureExtraction.h"

namespace myslam
{


class Frame
{
public:
    Frame(cv::Mat &imgL, cv::Mat &imgR);

    ~Frame();

    Eigen::Matrix3f mK;
    cv::Mat mImgL, mImgR;

    std::vector<cv::KeyPoint> mKeyPointsL, mKeyPointsR;
    cv::Mat mDescL, mDescR;

    std::vector<int> mMatch;
    std::vector<float> mDepth;

    FeatureExtraction extraction;

private:

//    static float fx;
//    static float fy;
//    static float cx;
//    static float cy;
//    static float bl;

    void ComputeDepth();

    void StereoMatch();

    int HammingDistance(const cv::Mat &a, const cv::Mat &b);


};
}
#endif //MYSLAM_FRAME_H
