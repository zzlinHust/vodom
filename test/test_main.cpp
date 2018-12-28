#include <iostream>


#include <opencv2/opencv.hpp>
#include <thread>

#include "StereoOdometry.h"
#include "fortest.h"


using namespace std;

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps);


int main(int argc, char **argv)
{
    std::string data_path("/home/cbt/SLAM_PROJECT/dataset/sequences/00");

    vector<string> vstrImageLeft;
    vector<string> vstrImageRight;
    vector<double> vTimestamps;
    LoadImages(data_path, vstrImageLeft, vstrImageRight, vTimestamps);


    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = 718.856;
    K.at<float>(1,1) = 718.856;
    K.at<float>(0,2) = 607.1928;
    K.at<float>(1,2) = 185.2157;

    // 图像矫正系数
    // [k1 k2 p1 p2 k3]
    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = 0.0;
    DistCoef.at<float>(1) = 0.0;
    DistCoef.at<float>(2) = 0.0;
    DistCoef.at<float>(3) = 0.0;
    const float k3 = 0.0;
    if(k3 != 0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }

    double bf = 386.1448;

    FeatureExtractionParam param;
    param.levels = 8;
    param.thresh_FAST = 20;
    param.thresh_FAST_min = 7;
    param.feature_num = 1000;
    param.scale_factor = 1.2;
    CameraParam cameraParam;
    cameraParam.b = 386.1448;
    cameraParam.fx = 718.856;
    cameraParam.fy = 718.856;
    cameraParam.cx = 607.1928;
    cameraParam.cy = 185.2157;
    cameraParam.s = 0.0;

    myslam::StereoOdometry::Ptr odom = make_shared<myslam::StereoOdometry>();
    myslam::FeatureExtraction::Ptr extraction = std::make_shared<myslam::FeatureExtraction>(param);


    const size_t nImages = vstrImageLeft.size();
    cv::Mat imLeft, imRight;
    for(int ni = 0; ni < nImages ; ++ni)
    {
        imLeft = cv::imread(vstrImageLeft[ni], CV_LOAD_IMAGE_UNCHANGED);
        imRight = cv::imread(vstrImageRight[ni], CV_LOAD_IMAGE_UNCHANGED);

        if(imLeft.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(vstrImageLeft[ni]) << endl;
            return 1;
        }

        /// track

        myslam::log("main", std::string("image ") + to_string(ni) + string("  in "));
        myslam::Input::Ptr input = std::make_shared<myslam::Input>(imLeft, imRight, K, DistCoef, bf, extraction);
        myslam::log("main", "finish");

        //odom->addFrame(myslam::Frame::CreateFrame(input));

        //myslam::log("main", "track finish \n");
        if(ni == 10000) break;
    }

    return 0;
}

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }

    string strPrefixLeft = strPathToSequence + "/image_0/";
    string strPrefixRight = strPathToSequence + "/image_1/";

    const size_t nTimes = vTimestamps.size();
    vstrImageLeft.resize(nTimes);
    vstrImageRight.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageLeft[i] = strPrefixLeft + ss.str() + ".png";
        vstrImageRight[i] = strPrefixRight + ss.str() + ".png";
    }
}
