//
// Created by cbt on 18-12-13.
//

#ifndef MYSLAM_PARAMETERS_H
#define MYSLAM_PARAMETERS_H

struct FeatureExtractionParam
{
    int feature_num = 1000; // 每帧提取特征数量
    int levels = 8;         // 金字塔层数
    int thresh_FAST = 20;   // FAST阈值
    int thresh_FAST_min = 3;// 最低FAST阈值
    int rad_FAST_orient = 15;// FAST角点方向半径
    float scale_factor = 1.2;// 金字塔尺度因子
};


#endif //MYSLAM_PARAMETERS_H
