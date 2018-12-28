//
// Created by cbt on 18-12-21.
//

#include "Frame.h"


using namespace std;

namespace myslam
{
Frame::Frame(unsigned int id, Input::Ptr input) :mId(id), T_c_w_(Sophus::SE3()), mK(input->mK), mbf(input->mbf),
          mImage(input->mLeft.imgPyr[0]), mKeyPoints(input->mLeft.keyPoints), mDescriptor(input->mLeft.descriptor),
          mDepth(input->mLeft.depth)
{

}


Frame::~Frame() {}

Frame::Ptr Frame::CreateFrame(Input::Ptr input)
{
     static unsigned int factory = 0;
     return std::make_shared<Frame>(factory++, input);
}

}