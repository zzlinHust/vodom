//
// Created by cbt on 18-12-13.
//

#ifndef MYSLAM_PARAMETERS_H
#define MYSLAM_PARAMETERS_H

struct FeatureExtractionParam
{
    int feature_num = 1000;
    int levels = 8;
    int thresh_FAST = 20;
    int thresh_FAST_min = 3;
    float scale_factor = 1.2;
};


#endif //MYSLAM_PARAMETERS_H
