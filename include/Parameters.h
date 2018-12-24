//
// Created by cbt on 18-12-13.
//

#ifndef MYSLAM_PARAMETERS_H
#define MYSLAM_PARAMETERS_H

struct CameraParam
{
    double fx;
    double fy;
    double cx;
    double cy;
    double s; // 倾斜参数,一般为0

    double b; // 双目基线长
};

struct RectifyParam
{
    double k1;
    double k2;
    double k3;
    double p1;
    double p2;

    /** 双目需要进行极线校正，这里先不处理. P Q R  **/

};

struct FeatureExtractionParam
{
    unsigned int feature_num = 1000; // 每帧提取特征数量
    unsigned int levels = 8;         // 金字塔层数
    unsigned int thresh_FAST = 20;   // FAST阈值
    unsigned int thresh_FAST_min = 3;// 最低FAST阈值
    unsigned int rad_FAST_orient = 15;// FAST角点方向半径
    float scale_factor = 1.2;// 金字塔尺度因子
};



#endif //MYSLAM_PARAMETERS_H
