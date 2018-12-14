//
// Created by cbt on 18-12-13.
//

#ifndef MYSLAM_FEATUREEXTRACTION_H
#define MYSLAM_FEATUREEXTRACTION_H

#include "common_include.h"
#include "Parameters.h"

namespace myslam
{

struct QuadTreeNode
{
    std::vector<cv::KeyPoint> keyPoints;
    Eigen::Vector2i UL, UR, BL, BR;
    void DevideToNextLevel(QuadTreeNode node[4]);
};


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
    std::vector<float> mSigma;
    std::vector<int> mFeaturesPyramid;

    int mPatchRadium;
    int mPatchSize;
    int mEdgePreserve;

    void ExtractSingleLevel(std::vector<cv::KeyPoint> &_keyPoints , cv::Mat &_descriptor , int level);

    void ComputePyramid(cv::Mat src);

    void Detect(std::vector<cv::KeyPoint> &_keyPoints , int level);

    void SortKeyPoint(std::vector<cv::KeyPoint> &_keyPoints, cv::Point2f minP, cv::Point2f maxP, int level);

    void ComputeDescriptor(std::vector<cv::KeyPoint> &_keyPoints, int level);

};
}




#endif //MYSLAM_FEATUREEXTRACTION_H
