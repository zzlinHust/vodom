//
// Created by cbt on 18-12-13.
//

#ifndef MYSLAM_FEATUREEXTRACTION_H
#define MYSLAM_FEATUREEXTRACTION_H

#include "common_include.h"
#include "Parameters.h"

namespace myslam
{
class FeatureExtraction
{
public:
    typedef std::shared_ptr<FeatureExtraction> Ptr;

    std::vector<cv::Mat> mImagePyramid;
    FeatureExtractionParam mParam;

    FeatureExtraction(const FeatureExtractionParam &_param);

    ~FeatureExtraction();

    void Extract(cv::Mat _img , std::vector<cv::KeyPoint> &_keyPoints , cv::OutputArray _descriptor);


private:

    std::vector<float> mScale;

    void ExtractSingleLevel(std::vector<cv::KeyPoint> &_keyPoints , cv::Mat &_descriptor , int level);

    void ComputePyramid(cv::Mat src);

    void Detect(std::vector<cv::KeyPoint> &_keyPoints , int level);

    void SortKeyPoint(std::vector<cv::KeyPoint> &_keyPoints, int levels);

    void ComputeDescriptor(std::vector<cv::KeyPoint> &_keyPoints, int levels);



};
}




#endif //MYSLAM_FEATUREEXTRACTION_H
