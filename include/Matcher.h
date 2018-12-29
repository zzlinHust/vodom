//
// Created by cbt on 18-12-25.
//

#ifndef MYSLAM_MATCHER_H
#define MYSLAM_MATCHER_H

#include "common_include.h"
#include "Input.h"
#include "Frame.h"
#include "g2o_types.h"

namespace myslam
{
class Matcher
{
public:
    typedef std::shared_ptr<Matcher> Ptr;

    Matcher(MatcherParam param);

    Matcher(){};

    ~Matcher(){}

    static int HammingDistance(const cv::Mat &a, const cv::Mat &b);

    static void StereoMatch(Input *input);

    static void DirectMethodMatching(Frame::Ptr cur_, Frame::Ptr pre_, FeatureExtraction::Ptr extract); /// 匹配，并有初始位姿


private:
    static float ComputeSTAD(const cv::Mat &temp, const cv::Mat &win, const float avg_diff, float thresh = 30);// avg_diff = win_avg - temp_avg;

//    static int DirectMethodSingleLevel()
};



}

#endif //MYSLAM_MATCHER_H
