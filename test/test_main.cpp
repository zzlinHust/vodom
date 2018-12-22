#include <iostream>
#include "FeatureExtraction.h"
#include "ORBextractor.h"
#include "Frame.h"

#include <opencv2/opencv.hpp>

int main(int argc, char **argv)
{
    std::string img_l("/home/cbt/SLAM_PROJECT/dataset/sequences/00/image_0/000000.png");
    std::string img_r("/home/cbt/SLAM_PROJECT/dataset/sequences/00/image_1/000000.png");

    cv::Mat left = cv::imread(img_l,cv::IMREAD_GRAYSCALE );
    cv::Mat right = cv::imread(img_r,cv::IMREAD_GRAYSCALE );

    FeatureExtractionParam param;
    param.levels = 8;
    param.thresh_FAST = 20;
    param.thresh_FAST_min = 7;
    param.feature_num = 1000;
    param.scale_factor = 1.2;


    myslam::Frame frame(left, right);


    cv::waitKey();

    return 0;
}