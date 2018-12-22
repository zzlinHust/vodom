//
// Created by cbt on 18-12-21.
//

#include "Frame.h"
#include "Rectifier.h"
#include "FeatureExtraction.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv/cv.hpp>

using namespace std;

namespace myslam
{
Frame::Frame(cv::Mat &imgL, cv::Mat &imgR)
{

    std::cout << "Frame in" << std::endl;
    if (imgL.channels() == 3) {
        cv::cvtColor(imgL, mImgL, CV_RGB2GRAY);
        cv::cvtColor(imgR, mImgR, CV_RGB2GRAY);
    } else if (imgL.channels() == 4) {
        cv::cvtColor(imgL, mImgL, CV_RGBA2GRAY);
        cv::cvtColor(imgR, mImgR, CV_RGBA2GRAY);
    }
    else
    {
        mImgL = imgL;
        mImgR = imgR;
    }
    Rectifier::StereoRectify();// 此处没有用=.=



    extraction.Extract(mImgL, mKeyPointsL, mDescL);// TODO： 用线程加速
    extraction.Extract(mImgR, mKeyPointsR, mDescR);

    std::cout << "StereoMatch in" << std::endl;
    StereoMatch(); // TODO:
    std::cout << "ComputeDepth in" << std::endl;

    ComputeDepth(); // TODO:

}

Frame::~Frame() {}

int Frame::HammingDistance(const cv::Mat &a, const cv::Mat &b)
{
    const uint64_t *pa = a.ptr<uint64_t>();
    const uint64_t *pb = b.ptr<uint64_t>();

    int count = 0;
    for(int i = 0 ; i < 4 ; ++i, ++pa, ++pb)
    {
        uint64_t temp = (*pa) ^ (*pb);
        while (temp)
        {
            ++count;
            temp &= (temp - 1);
        }
    }
    return count;
}

void Frame::StereoMatch()
{
    //输入： 左目图像和特征 、 右目图像和特征
    //目的： 在右图中找到与左图特征点匹配的特征点
    mMatch.resize(mKeyPointsL.size(), -1);

    /** 1. 确定每个特征极线搜索区域 **/
    /* 极线长度可以根据双目实际参数以及深度检测范围来设置，这里默认[0,cols-1],即一整行 */

    static vector<float> search_rad;
    if(search_rad.empty())
    {
        search_rad.resize(extraction.mParam.levels);
        search_rad[0] = 2;
        for(int i = 1 ; i < extraction.mParam.levels ; ++i)
            search_rad[i] = extraction.mParam.scale_factor * search_rad[i-1];
    }

    vector<vector<int>> search_range(mImgL.rows); // search_range[i]的意义为右图中可能与左图第i行特征点匹配的所有特征。
    for( int index = 0 ; index < mKeyPointsR.size() ; ++index )
    {
        const auto &kp = mKeyPointsR[index];
        int minRow = floor(kp.pt.y - search_rad[kp.octave]);
        int maxRow = ceil(kp.pt.y + search_rad[kp.octave]);
        for(int j = minRow ; j <= maxRow ; ++j)
            search_range[j].push_back(index);
    }

    /** 2. 通过Hamming距离获取最佳匹配 **/
    for(int i = 0 ; i < mKeyPointsL.size() ; ++i)
    {
        const auto &kp = mKeyPointsL[i];
        const cv::Mat &desL = mDescL.row(i);
        const auto &possibleKps = search_range[kp.pt.y];
        static int _hamming_thresh = 100; /// param

        int best_i = -1;
        int min_dist = _hamming_thresh; /* 特征点Hamming距离必须小于一个阈值 */
        for(const auto &kpIndex : possibleKps )
        {
            const cv::Mat &desR = mDescR.row(i);
            int dist = HammingDistance(desL, desR);
            if(dist < min_dist)
            {
                min_dist = dist;
                best_i = kpIndex;
            }
        }
        if(best_i != -1)
            mMatch[i] = best_i;
    }
}

void Frame::ComputeDepth()
{
    // 输入： 左右图像及其匹配的特征点
    mDepth.resize(mKeyPointsL.size(), -1);
    std::vector<cv::DMatch> match;

    for(int i = 0 ; i < mMatch.size() ; ++i)
    {
        if(mMatch[i] != -1)
        {
            cv::DMatch m;
            m.trainIdx = i;
            m.queryIdx = mMatch[i];
        }
    }

    cv::Mat test;
    cv::drawMatches(mImgL, mKeyPointsL, mImgR, mKeyPointsR, match,test);
    cv::imshow("tesd",test);

}


}