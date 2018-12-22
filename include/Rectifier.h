//
// Created by cbt on 18-12-20.
//

#ifndef MYSLAM_RECTIFIER_H
#define MYSLAM_RECTIFIER_H

#include "Parameters.h"

/**
 *  畸变校准与双目极线矫正.
 */
namespace myslam
{
class Rectifier
{
public:
    Rectifier() = delete;
    ~Rectifier() = delete;

    static void Distort(){}

    static void StereoRectify(){}

    static RectifyParam param;
};
}

#endif //MYSLAM_RECTIFIER_H
