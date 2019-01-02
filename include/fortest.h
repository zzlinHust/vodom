//
// Created by cbt on 18-12-25.
//

#ifndef MYSLAM_FORTEST_H
#define MYSLAM_FORTEST_H

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <iostream>
#include <chrono>

#include "StereoOdometry.h"
#include "g2o_types.h"

namespace myslam
{
void log(const std::string &where, const std::string &msg);

void debug_direct_method(Frame::Ptr &pre, Frame::Ptr &cur, int level, g2o::SE3Quat se3, float scale);

}

#endif //MYSLAM_FORTEST_H