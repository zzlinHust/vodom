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
    cv::Point2i UL, UR, BL, BR;
    void DivideToNextLevel(QuadTreeNode node[4]);
};


class FeatureExtraction
{
public:
    typedef std::shared_ptr<FeatureExtraction> Ptr;

    std::vector<cv::Mat> mImagePyramid;
    FeatureExtractionParam mParam;

    FeatureExtraction(const FeatureExtractionParam &_param);

    FeatureExtraction();

    ~FeatureExtraction();

    void Extract(cv::Mat _img , std::vector<cv::KeyPoint> &_keyPoints , cv::OutputArray _descriptor);

private:

    std::vector<float> mScale;
    std::vector<float> mSigma;
    std::vector<unsigned int> mFeaturesPyramid;
    std::vector<int> umax;

    unsigned int mPatchRadium;
    unsigned int mPatchSize;
    unsigned int mEdgePreserve;

    void ExtractSingleLevel(std::vector<cv::KeyPoint> *_keyPoints , cv::Mat *_descriptor , int level);

    void ComputePyramid(cv::Mat &src);

    void Detect(std::vector<cv::KeyPoint> &_keyPoints , int level);

    void SortKeyPoint(std::vector<cv::KeyPoint> &_keyPoints, cv::Point2i minP, cv::Point2i maxP, int level);

    void ComputeAngle(std::vector<cv::KeyPoint> &_keyPoints, int level);

    void ComputeDescriptor(std::vector<cv::KeyPoint> &_keyPoints, cv::Mat &_descriptor, int level);

};
}

#endif //MYSLAM_FEATUREEXTRACTION_H
