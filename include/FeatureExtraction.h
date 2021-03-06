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

    FeatureExtraction(const FeatureExtractionParam &_param);

    FeatureExtraction();

    ~FeatureExtraction();

    void Extract(std::vector<cv::Mat> &imgPyr , std::vector<cv::KeyPoint> &_keyPoints , cv::OutputArray _descriptor);

    size_t GetLevels() { return nLevels; }

    float GetScaleFactor() { return scaleFactor; }
    float GetScale(int i) { return mScale[i]; }

private:

    size_t nFeatures;
    size_t nLevels;
    unsigned int thresh_FAST;
    unsigned int thresh_FAST_min;
    unsigned int radius_FAST;
    float scaleFactor;

    std::vector<float> mScale;
    std::vector<float> mSigma;
    std::vector<unsigned int> mFeaturesPyramid;
    std::vector<int> umax;

    unsigned int mPatchRadium;
    unsigned int mPatchSize;
    unsigned int mEdgePreserve;

    void ExtractSingleLevel(cv::Mat &img, std::vector<cv::KeyPoint> *_keyPoints , cv::Mat *_descriptor , int level);

    void ComputePyramid(std::vector<cv::Mat> &imgPyr);

    void Detect(cv::Mat &img, std::vector<cv::KeyPoint> &_keyPoints , int level);

    void SortKeyPoint(std::vector<cv::KeyPoint> &_keyPoints, cv::Point2i minP, cv::Point2i maxP, int level);

    void ComputeAngle(cv::Mat &img, std::vector<cv::KeyPoint> &_keyPoints);

    void ComputeDescriptor(cv::Mat &img, std::vector<cv::KeyPoint> &_keyPoints, cv::Mat &_descriptor, int level);

};
}

#endif //MYSLAM_FEATUREEXTRACTION_H
