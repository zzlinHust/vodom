//
// Created by cbt on 18-12-24.
//
#include "fortest.h"

#include <thread>
#include <opencv2/imgproc/imgproc.hpp>
#include "Input.h"
#include "Matcher.h"


using namespace std;

namespace myslam
{

Input::Input(cv::Mat left, cv::Mat right, cv::Mat K, cv::Mat distCoef, float bf, FeatureExtraction::Ptr extractor):
        mK(K), mDistCoef(distCoef), mbf(bf), mExtractor(extractor)
{
    nLevels = extractor->GetLevels();
    scaleFactor = extractor->GetScaleFactor();

    /** 1. 转为灰度图 **/
    mLeft.imgPyr.resize(extractor->GetLevels());
    mRight.imgPyr.resize(extractor->GetLevels());
    if (left.channels() == 3)
    {
        cv::cvtColor(left, mLeft.imgPyr[0], CV_RGB2GRAY);
        cv::cvtColor(right, mRight.imgPyr[0], CV_RGB2GRAY);
    }
    else if (left.channels() == 4)
    {
        cv::cvtColor(left, mLeft.imgPyr[0], CV_RGBA2GRAY);
        cv::cvtColor(right, mRight.imgPyr[0], CV_RGBA2GRAY);
    }
    else
    {
        mLeft.imgPyr[0] = left;
        mRight.imgPyr[0] = right;
    }

    /** 2. 左右目特征提取 **/
    thread threadL(&Input::ExtractImageFeature, this, true);
    thread threadR(&Input::ExtractImageFeature, this, false);
    threadL.join();
    threadR.join();

    /** 3. 双目匹配及深度计算 **/
    Matcher matcher;
    matcher.StereoMatch(this);

    cv::Mat out,outr;
    cv::drawKeypoints(mLeft.imgPyr[0],mLeft.keyPoints,out);
    cv::drawKeypoints(mRight.imgPyr[0],mRight.keyPoints,outr);
    cv::imshow("out",out);
    cv::imshow("outr",outr);
    cv::waitKey();
}


void Input::ExtractImageFeature(bool isLeft)
{
    if(isLeft) mExtractor->Extract(mLeft.imgPyr, mLeft.keyPoints, mLeft.descriptor);
    else mExtractor->Extract(mRight.imgPyr, mRight.keyPoints, mRight.descriptor);
}

void Input::UnDistortKeyPoints()
{

}
}