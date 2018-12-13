//
// Created by cbt on 18-12-13.
//

#include "FeatureExtraction.h"
#include <thread>
#include <opencv2/imgproc.hpp>
#include <opencv/cv.hpp>

using cv::Mat;
using namespace std;

namespace myslam
{
const int PATCH_SIZE = 31;
const int HALF_PATCH_SIZE = 15;
const int EDGE_THRESHOLD = 19;

FeatureExtraction::FeatureExtraction(const FeatureExtractionParam &param) : mParam(param)
{
}

FeatureExtraction::~FeatureExtraction(){}


void FeatureExtraction::Extract(cv::Mat _img, std::vector<cv::KeyPoint> &_keyPoints, cv::OutputArray &_descriptor)
{

    ComputePyramid(_img);

    vector<vector<cv::KeyPoint>>  allKeyPoints(mParam.levels);
    vector<cv::Mat> allDescriptor(mParam.levels);
//    vector<thread> allThreads(mParam.levels);
//
//    for(int i = 0 ; i < mParam.levels ; ++i)
//        allThreads[i] = thread(&FeatureExtraction::extractSingleLevel, this, allKeyPoints[i], allDescriptor[i], i);
//
//    for(int i = 0 ; i < mParam.levels ; ++i)
//    {
//        if(allThreads[i].joinable())
//            allThreads[i].join();
//    }
    for(int i = 0 ; i < mParam.levels ; ++i)
        ExtractSingleLevel(allKeyPoints[i], allDescriptor[i], i);

    auto des_it = allDescriptor.begin();
    for(int i = 0 ; i < mParam.levels ; ++i)
    {
        _keyPoints.insert(_keyPoints.end(), allKeyPoints[i].begin(), allKeyPoints[i].end());
        if(des_it->empty())
            des_it = allDescriptor.erase(des_it); // delete empty descriptor
        else
            ++des_it;
    }
    cv::vconcat(allDescriptor, _descriptor);
}

void FeatureExtraction::ExtractSingleLevel(std::vector<cv::KeyPoint> &_keyPoints, cv::Mat &_descriptor, int level)
{

    Detect(_keyPoints, level);

    cv::Mat img;
    cv::drawKeypoints(mImagePyramid[level],_keyPoints, img, cv::Scalar::all(-1),cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    SortKeyPoint(_keyPoints, level);

    cv::drawKeypoints(mImagePyramid[level],_keyPoints, img, cv::Scalar::all(-1),cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);


    cv::waitKey();
}

void FeatureExtraction::ComputePyramid(cv::Mat src)
{
    mImagePyramid.resize(mParam.levels);
    mScale.resize(mParam.levels);
    mScale[0] = 1.0f;
    mImagePyramid[0] = src;
    float scale = 1.0f / mParam.scale_factor;
    for(int i = 1 ; i < mParam.levels ; ++i)
    {
        mScale[i] = scale * mScale[i-1];
        cv::Size sz(cvRound((float)src.cols * mScale[i]), cvRound((float)src.rows * mScale[i]));
        cv::resize(mImagePyramid[i-1], mImagePyramid[i], sz, 0, 0, cv::INTER_LINEAR);
    }


}

void FeatureExtraction::Detect(std::vector<cv::KeyPoint> &_keyPoints, int level)
{

    const int minX = 3 , minY = 3;
    const int maxX = mImagePyramid[level].cols - 3;
    const int maxY = mImagePyramid[level].rows - 3;
    const float winSize = 30;
    const float width = maxX - minX;
    const float height = maxY - minY;
    const int cellCols = int(width / winSize);
    const int cellRows = int(height / winSize);
    const int cellWidth = int(ceil(width / cellCols));
    const int cellHeight = int(ceil(height / cellRows));
    const int scaledPatchSize = PATCH_SIZE / mScale[level];

    for (int i = 0 ; i < cellRows ; ++i)
    {
        int Y_begin = minY + i * cellHeight;
        int Y_end = Y_begin + cellHeight ;

        if(Y_begin >= maxY) continue;
        if(Y_end > maxY) Y_end = maxY;

        for(int j = 0 ; j < cellCols ; ++j)
        {
            int X_begin = minX + j * cellWidth;
            int X_end = X_begin + cellWidth;
            if(X_begin >= maxX) continue;
            if(X_end > maxX) X_end = maxX;

            cv::Mat cellImg = mImagePyramid[level].rowRange(Y_begin-3,Y_end+3).colRange(X_begin-3,X_end+3);
            vector<cv::KeyPoint> kpCell;

            cv::FAST(cellImg, kpCell, mParam.thresh_FAST, true);
            if(kpCell.empty())
                cv::FAST(cellImg, kpCell, mParam.thresh_FAST_min, true);

            if(kpCell.size())
            {
                for(auto it = kpCell.begin() ; it != kpCell.end() ; ++it)
                {
                    it->pt.x += X_begin - 3;
                    it->pt.y += Y_begin - 3;
                    it->octave = level;
                    it->size =
                    _keyPoints.push_back(*it);
                }
            }
        }

    }
}


void FeatureExtraction::SortKeyPoint(std::vector<cv::KeyPoint> &_keyPoints, int levels)
{

}

}