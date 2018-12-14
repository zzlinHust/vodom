#include <iostream>
#include "FeatureExtraction.h"

#include <opencv2/opencv.hpp>

int main(int argc, char **argv)
{


    std::string path("/media/cbt/data0/0LZZ-private/slam/DepthBlue/PA5/code/1.png");
    std::string paa("/home/cbt/SLAM_PROJECT/dataset/sequences/00/image_0/000000.png");
    cv::Mat img = cv::imread(paa);

    FeatureExtractionParam param;
    param.levels = 8;
    param.thresh_FAST = 20;
    param.thresh_FAST_min = 3;
    param.feature_num = 1000;
    param.scale_factor = 1.2;
    myslam::FeatureExtraction extraction(param);

    std::vector<cv::KeyPoint> kp;
    cv::Mat desc;
    extraction.Extract(img,kp,desc);

    cv::imshow("left",img);
    cv::waitKey();
    return 0;
}