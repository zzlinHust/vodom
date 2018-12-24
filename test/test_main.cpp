#include <iostream>
#include "FeatureExtraction.h"
#include "ORBextractor.h"
#include "Frame.h"
#include "Input.h"

#include <opencv2/opencv.hpp>


int HammingDistance(const cv::Mat &a, const cv::Mat &b)
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
void match(cv::Mat &mDescL,cv::Mat &mDescR,std::vector<cv::KeyPoint> &mKeyPointsL ,
           std::vector<cv::KeyPoint> &mKeyPointsR, std::vector<int> &mMatch , cv::Mat &mImgL  );



int main(int argc, char **argv)
{
    std::string img_l("/home/cbt/SLAM_PROJECT/dataset/sequences/00/image_0/000000.png");
    std::string img_r("/home/cbt/SLAM_PROJECT/dataset/sequences/00/image_1/000000.png");

    cv::Mat left = cv::imread(img_l,cv::IMREAD_GRAYSCALE );
    cv::Mat right = cv::imread(img_r,cv::IMREAD_GRAYSCALE );

    myslam::Input::Ptr slamInput = std::make_shared<myslam::Input>(left, right);



    FeatureExtractionParam param;
    param.levels = 8;
    param.thresh_FAST = 20;
    param.thresh_FAST_min = 7;
    param.feature_num = 1000;
    param.scale_factor = 1.2;


    myslam::Frame frame(left, right);


    std::vector<cv::KeyPoint> kpl , kpr;
    cv::Mat del,der;
    ORB_SLAM2::ORBextractor orBextractor(1000,1.2,8,20,7);
    orBextractor(left,cv::Mat(),kpl,del);
    orBextractor(right,cv::Mat(),kpr,der);
    cv::Mat rob_match;
    cv::vconcat(left,right,rob_match);

    std::vector<int> ma;
    match(del,der,kpl,kpr,ma,left);

    cv::Point2f ceb(0,left.rows);
    for(int i = 0 ; i < ma.size() ; ++i)
    {
        if(ma[i] != -1 && fabs(kpl[i].pt.x-kpr[ma[i]].pt.x) < 20 )
        {
            cv::circle(rob_match,kpl[i].pt,2,cv::Scalar(-1));
            cv::circle(rob_match,kpr[ma[i]].pt+ceb,2,cv::Scalar(-1));
            cv::line(rob_match,kpl[i].pt, kpr[ma[i]].pt+ceb,cv::Scalar(128));
        }
    }


    cv::imshow("orb",rob_match);
    cv::waitKey();

    return 0;
}

void match(cv::Mat &mDescL,cv::Mat &mDescR,std::vector<cv::KeyPoint> &mKeyPointsL ,
           std::vector<cv::KeyPoint> &mKeyPointsR, std::vector<int>& mMatch , cv::Mat &mImgL )
{
    mMatch.resize(mKeyPointsL.size(), -1);

    /** 1. 确定每个特征极线搜索区域 **/
    /* 极线长度可以根据双目实际参数以及深度检测范围来设置，这里默认[0,cols-1],即一整行 */

    myslam::FeatureExtraction extraction;
    static std::vector<float> search_rad;
    if(search_rad.empty())
    {
        search_rad.resize(extraction.mParam.levels);
        search_rad[0] = 2;
        for(int i = 1 ; i < extraction.mParam.levels ; ++i)
            search_rad[i] = extraction.mParam.scale_factor * search_rad[i-1];
    }

    std::vector<std::vector<int>> search_range(mImgL.rows); // search_range[i]的意义为右图中可能与左图第i行特征点匹配的所有特征。
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
            const cv::Mat &desR = mDescR.row(kpIndex);
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