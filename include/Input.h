//
// Created by cbt on 18-12-24.
//

#ifndef MYSLAM_INPUT_H
#define MYSLAM_INPUT_H

#include "common_include.h"

namespace myslam
{

class Input
{
public:
    typedef std::shared_ptr<Input> Ptr;

    Input(cv::Mat left, cv::Mat right);

    void StereoRectify() {} // 这里不需要

    void UnDistortKeyPoints();

    ~Input() {}

    cv::Mat mK;         // 相机内参
    cv::Mat mDistCoef;  // 畸变参数

/** 左图数据： 图像、图像金字塔、特征、与右目的匹配、特征深度 **/
    cv::Mat mImgLeft;
    std::vector<cv::Mat> mImgLeftPyr;
    std::vector<cv::KeyPoint> mKeyPointsLeft;
    std::vector<int> mMatch;
    std::vector<float> mDepth;

/** 右图数据： 图像、图像金字塔、特征 **/
    cv::Mat mImgRight;
    std::vector<cv::Mat> mImgRightPyr;
    std::vector<cv::KeyPoint> mKeyPointsRight;
};
}
#endif //MYSLAM_INPUT_H
