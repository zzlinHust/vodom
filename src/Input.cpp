//
// Created by cbt on 18-12-24.
//

#include "Input.h"
#include "opencv2/imgproc/imgproc.hpp"

using namespace std;

namespace myslam
{

Input::Input(cv::Mat left, cv::Mat right)
{
    if (left.channels() == 3) {
        cv::cvtColor(left, mImgLeft, CV_RGB2GRAY);
        cv::cvtColor(right, mImgRight, CV_RGB2GRAY);
    } else if (left.channels() == 4) {
        cv::cvtColor(left, mImgLeft, CV_RGBA2GRAY);
        cv::cvtColor(right, mImgRight, CV_RGBA2GRAY);
    }
    else
    {
        mImgLeft = left;
        mImgRight = right;
    }

//    StereoRectify();
}


void Input::UnDistortKeyPoints()
{

}
}