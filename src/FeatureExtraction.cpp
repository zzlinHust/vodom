//
// Created by cbt on 18-12-13.
//

#include "FeatureExtraction.h"
#include <thread>
#include <list>
#include <opencv2/imgproc.hpp>
#include <opencv/cv.hpp>

using cv::Mat;
using namespace std;

namespace myslam
{
;

FeatureExtraction::FeatureExtraction(const FeatureExtractionParam &param) : mParam(param)
{
    size_t n = size_t(mParam.levels);
    mPatchRadium = mParam.rad_FAST_orient;
    mPatchSize = 2 * mPatchRadium + 1;
    mEdgePreserve = mPatchRadium + 1;

    mImagePyramid.resize(n);
    mFeaturesPyramid.resize(n);
    mScale.resize(n);
    mSigma.resize(n);

    float factor = mParam.scale_factor;
    float scale = 1.0f / factor;
    float nfeatures = mParam.feature_num * (1.0f - scale) / (1.0f - std::pow(scale, param.levels));
    mScale[0] = 1.0f;
    mSigma[0] = 1.0f;
    int sum = mFeaturesPyramid[0] = static_cast<unsigned int>(cvRound(nfeatures));

    for (int i = 1; i < n; ++i)
    {
        nfeatures *= scale;
        mFeaturesPyramid[i] = static_cast<unsigned int>(cvRound(nfeatures));
        mScale[i] = scale * mScale[i-1];
        mSigma[i] = factor * factor * mSigma[i-1];
        sum += mFeaturesPyramid[i];
    }
    mFeaturesPyramid[n-1] -= sum - mParam.feature_num;
    if (mFeaturesPyramid[n-1] < 0) mFeaturesPyramid[n-1] = 0;
}

FeatureExtraction::~FeatureExtraction(){}


void FeatureExtraction::Extract(cv::Mat _img, std::vector<cv::KeyPoint> &_keyPoints, cv::OutputArray &_descriptor)
{

    ComputePyramid(_img);

    vector<vector<cv::KeyPoint>> allKeyPoints(mParam.levels);
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



    cv::waitKey();
}

void FeatureExtraction::ComputePyramid(cv::Mat src)
{
    mImagePyramid[0] = src;
    for(int i = 1 ; i < mParam.levels ; ++i)
    {
        cv::Size sz(cvRound((float)src.cols * mScale[i]), cvRound((float)src.rows * mScale[i]));
        cv::resize(mImagePyramid[i-1], mImagePyramid[i], sz, 0, 0, cv::INTER_LINEAR);
    }
}

void FeatureExtraction::Detect(std::vector<cv::KeyPoint> &_keyPoints, int level)
{
    const int minX = mEdgePreserve , minY = mEdgePreserve;
    const int maxX = mImagePyramid[level].cols - mEdgePreserve;
    const int maxY = mImagePyramid[level].rows - mEdgePreserve;
    const float winSize = 30;
    const float width = maxX - minX;
    const float height = maxY - minY;
    const int cellCols = int(width / winSize);
    const int cellRows = int(height / winSize);
    const int cellWidth = int(ceil(width / cellCols));
    const int cellHeight = int(ceil(height / cellRows));
    const float scaledPatchSize = mPatchSize / mScale[level];

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
                    it->size = scaledPatchSize;
                    _keyPoints.push_back(*it);
                }
            }
        }
    }
//    cv::Mat img;
//    cv::drawKeypoints(mImagePyramid[level],_keyPoints, img, cv::Scalar::all(-1),cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
//    cv::imshow("d",img);
//    cv::waitKey();

    SortKeyPoint(_keyPoints, cv::Point(minX, minY), cv::Point(maxX, maxY), level);

//    cv::drawKeypoints(mImagePyramid[level],_keyPoints, img, cv::Scalar::all(-1),cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
//    cv::imshow("d",img);
//    cv::waitKey();
}


void FeatureExtraction::SortKeyPoint(std::vector<cv::KeyPoint> &_keyPoints, cv::Point2i minP, cv::Point2i maxP, int level)
{
    const unsigned int initNodesNum = (unsigned int)std::round(float(maxP.x - minP.x) / float(maxP.y - minP.y));
    const float hX = static_cast<float>(maxP.x - minP.x) / initNodesNum;
//    printf("level: %d  ,node nums : %d  ,hx : %.3f\n",level,initNodesNum, hX);
//    std::cout << "range : " <<  minP << maxP << std::endl;

    std::vector<QuadTreeNode> leavesNode;
    std::vector<QuadTreeNode> rootNode;
    std::vector<QuadTreeNode> nextLevelNode;
    leavesNode.reserve(mFeaturesPyramid[level]);
    rootNode.reserve(mFeaturesPyramid[level]);
    nextLevelNode.reserve(mFeaturesPyramid[level]);

    rootNode.resize(initNodesNum);
    for(int i = 0 ; i < initNodesNum ; ++i )
    {
        QuadTreeNode &node = rootNode[i];
        node.keyPoints.reserve(_keyPoints.size());
        node.UL = cv::Point2i(int(hX * i), 0);
        node.UR = cv::Point2i(int(hX * (i+1)), 0);
        node.BL = cv::Point2i(node.UL.x, maxP.y - minP.y);
        node.BR = cv::Point2i(node.UR.x, maxP.y - minP.y);
        node.keyPoints.reserve(_keyPoints.size());
//        std::cout << "init " << i << "  " << rootNode[i].UL << rootNode[i].UR << rootNode[i].BL << rootNode[i].BR << std::endl;
    }

    for(const auto &kp : _keyPoints)
    {
//        if((kp.pt.x - minP.x )/ hX > 4) std::cerr << kp.pt.x << "   " << hX << std::endl;
        rootNode[(kp.pt.x - minP.x) / hX].keyPoints.push_back(kp);
    }
//    std::cout << "end init root node" << std::endl;


    auto *father = &rootNode;
    auto *son = &nextLevelNode;

    while (1) /// 通过四叉树分裂均匀选取角点
    {
        for(auto &fatherNode : *father)
        {
            if(fatherNode.keyPoints.size() == 1)  // 不能再分裂，归到叶子节点
            {
                leavesNode.push_back(fatherNode);
            }
            else if (fatherNode.keyPoints.size() > 1) // 分裂成子节点
            {
                QuadTreeNode sonNode[4];
                fatherNode.DivideToNextLevel(sonNode);
                for (int i = 0 ; i < 4 ; ++i)
                {
                    if(sonNode[i].keyPoints.size() == 1)
                        leavesNode.push_back(sonNode[i]);
                    else if(sonNode[i].keyPoints.size() > 1)
                        son->push_back(sonNode[i]);
                    else{}
                }
            }
            else{}
        }

        if(son->size() >= mFeaturesPyramid[level] || son->size() == father->size()) break;
        else if(leavesNode.size() + son->size() * 3 > mFeaturesPyramid[level])
        {/// 当再次分裂（假设都能一分四）节点数将大于欲选取角点数时，优先对角点多的区域进行划分
            bool finish = false;
            while (!finish)
            {
                father->clear();// 父节点清空，子节点作为新的父节点。
                std::swap(father,son);

                std::sort(father->begin(),father->end(), \
                          [](const QuadTreeNode &a, const QuadTreeNode &b) \
                          { return a.keyPoints.size() < b.keyPoints.size();});
                size_t nodes_left = father->size();
                for (auto it = father->rbegin() ; it != father->rend() ; ++it)
                {
                    if(finish)
                    {
                        son->push_back(*it);
                    }
                    else
                    {
                        QuadTreeNode sonNode[4];
                        it->DivideToNextLevel(sonNode);
                        --nodes_left;
                        for (int i = 0 ; i < 4 ; ++i)
                        {
                            if(sonNode[i].keyPoints.size() == 1)
                                leavesNode.push_back(sonNode[i]);
                            else if(sonNode[i].keyPoints.size() > 1)
                                son->push_back(sonNode[i]);
                            else{}
                        }
                        if(leavesNode.size() + son->size() + nodes_left >= mFeaturesPyramid[level])
                        {
                           finish = true;
                        }
                    }
                }
                if(son->size() == father->size())
                    finish = true;
            }
            break;
        }
        else
        {
            father->clear();// 父节点清空，子节点作为新的父节点。
            std::swap(father,son);
        }
    }

//    std::cout << "sichashu end !!!" << std::endl;

    _keyPoints.clear();
    _keyPoints.reserve(leavesNode.size() + son->size());


    for (auto &node : leavesNode)
        _keyPoints.push_back(*node.keyPoints.begin());
    for(auto &node : *son)
    {

        auto it = node.keyPoints.begin();
        auto end = node.keyPoints.end();
        cv::KeyPoint &kp = *it;
        float max_respons = it->response;
        while ((++it) != end)
        {
            if(it->response > max_respons)
            {
                max_respons = it->response;
                kp = *it;
            }
        }
        _keyPoints.push_back(kp);
    }
//    std::cout << std::endl;
}

void FeatureExtraction::ComputeAngle(std::vector<cv::KeyPoint> &_keyPoints, int level)
{

}


void FeatureExtraction::ComputeDescriptor(std::vector<cv::KeyPoint> &_keyPoints, int level)
{

}







void QuadTreeNode::DivideToNextLevel(QuadTreeNode *node)
{
    int dx = static_cast<int>(0.5 * (UR.x - UL.x));
    int dy = static_cast<int>(0.5 * (BR.y - UR.y));

    node[0].UL = UL;
    node[0].UR = node[1].UL = cv::Point2i(UL.x + dx, UL.y);
    node[0].BL = node[2].UL = cv::Point2i(UL.x, UL.y + dy);
    node[0].BR = node[1].BL = node[2].UR = node[3].UL = cv::Point2i(UL.x + dx, UL.y + dy);
    node[1].UR = UR;
    node[1].BR = node[3].UR = cv::Point2i(UR.x, UR.y + dy);
    node[2].BL = BL;
    node[2].BR = node[3].BL = cv::Point2i(BL.x + dx, BL.y);
    node[3].BR = BR;

    int cx = node[0].BR.x, cy = node[0].BR.y;
    for(auto &kp : keyPoints)
    {
        if(kp.pt.x > cx)
        {
            if(kp.pt.y > cy)
                node[3].keyPoints.push_back(kp);
            else
                node[1].keyPoints.push_back(kp);
        }
        else
        {
            if(kp.pt.y > cy)
                node[2].keyPoints.push_back(kp);
            else
                node[0].keyPoints.push_back(kp);
        }
    }
}
}