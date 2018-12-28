//
// Created by cbt on 18-12-24.
//

#ifndef MYSLAM_INPUT_H
#define MYSLAM_INPUT_H

#include "common_include.h"
#include "FeatureExtraction.h"

namespace myslam
{
class FeatureExtraction;
class Input
{
public:
    typedef std::shared_ptr<Input> Ptr;

    Input(cv::Mat left, cv::Mat right, cv::Mat K, cv::Mat distCoef, float bf, FeatureExtraction::Ptr extractor);

    void StereoRectify() {} // 这里不需要

    void UnDistortKeyPoints();

    ~Input() {}

    cv::Mat mK;         // 相机内参
    cv::Mat mDistCoef;  // 畸变参数
    float mbf;
    FeatureExtraction::Ptr mExtractor;


    size_t nLevels;
    float scaleFactor;
    /** 左图数据： 图像金字塔、特征点、描述子、与右目的匹配、特征深度 **/
    struct
    {
        std::vector<cv::Mat> imgPyr;
        std::vector<cv::KeyPoint> keyPoints;
        cv::Mat descriptor;
        std::vector<float> match; // 极线上对应
        std::vector<float> depth;
    }mLeft;


/** 右图数据： 图像金字塔、特征点、描述子 **/
    struct
    {
        std::vector<cv::Mat> imgPyr;
        std::vector<cv::KeyPoint> keyPoints;
        cv::Mat descriptor;
    }mRight;

private:

    void ExtractImageFeature(bool isLeft);

};
}
#endif //MYSLAM_INPUT_H
