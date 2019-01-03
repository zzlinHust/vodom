//
// Created by cbt on 18-12-28.
//
#include "fortest.h"

using namespace std;

namespace myslam
{

void log(const std::string &where, const std::string &msg)
{
    static chrono::steady_clock::time_point init = chrono::steady_clock::now();
    static chrono::steady_clock::time_point last = chrono::steady_clock::now();

    chrono::steady_clock::time_point now = chrono::steady_clock::now();

    chrono::duration<double> cur_time = chrono::duration_cast<chrono::duration<double>>(now - init);
    chrono::duration<double> duration = chrono::duration_cast<chrono::duration<double>>(now - last);
    fprintf(stderr, "[%-4.3fs]  [%3.3fms] : %.10s , %s \n",cur_time.count(), 1000.0 * duration.count(), where.c_str(), msg.c_str());

    last = now;
}

void debug_direct_method(Frame::Ptr &pre, Frame::Ptr &cur, int level, g2o::SE3Quat se3, float scale)
{
    const float fx = scale * cur->mCamera->fx_;
    const float fy = scale * cur->mCamera->fy_;
    const float cx = scale * cur->mCamera->cx_;
    const float cy = scale * cur->mCamera->cy_;

    cout << "level : " << level << endl;

    cv::Mat img1, img2, img3;

    img1 = pre->mImagePyr[level].clone();
    img2 = cur->mImagePyr[level].clone();

    if(level == 0)
    {
        for(int i = 0 ; i < cur->mDepth.size() ; ++i)
        {
            float z = cur->mDepth[i];
            if(z > 0)
                cv::circle(img2, cur->mKeyPoints[i].pt, 5, cv::Scalar(255));

        }

        auto &depth = pre->mDepth;
        auto &pos_3d = pre->mMapPoints;
        for(int i = 0 ; i < depth.size() ; ++i )
        {
            if(depth[i] > 0)
            {
                auto pt = scale * pre->mKeyPoints[i].pt;
                Eigen::Vector3d pos_local = se3.map(pos_3d[i]->mPos3d);
                double x = pos_local[0];
                double y = pos_local[1];
                double iz = 1.0 / pos_local[2];
                float u = fx * x * iz + cx;
                float v = fy * y * iz + cy;

                cv::Point2f kp2(u,v+img1.rows);

                cv::vconcat(img1,img2,img3);

                cv::circle(img3,pt,5,cv::Scalar(200));
                cv::circle(img3, kp2, 1, cv::Scalar(200));
                cv::line(img3,pt, kp2, cv::Scalar(255));

                cv::Point2f ppp(40,15);
                cv::rectangle(img3,kp2-ppp,kp2+ppp,cv::Scalar(200));

                cv::imshow("con",img3);
                cv::waitKey();

            }
        }
    }
}

void show_matches(Frame::Ptr &pre, Frame::Ptr &cur, std::vector<int> &matches)
{
    cv::Mat &img_cur = cur->mImagePyr[0];
    cv::Mat &img_pre = pre->mImagePyr[0];
    cv::Mat show;

    auto &kp_pre = pre->mKeyPoints;
    auto &kp_cur = cur->mKeyPoints;

    cv::Point2f  ppp(0,img_pre.rows);

    for (int i = 0; i < kp_cur.size(); ++i)
    {
        if(matches[i] != -1)
        {
            auto &pt_cur = kp_cur[i].pt;
            auto pt_pre = kp_pre[matches[i]].pt + ppp;

            cv::vconcat(img_cur, img_pre, show);
            cv::circle(show, pt_cur, 5, cv::Scalar(200));
            cv::circle(show, pt_pre, 5, cv::Scalar(200));
            cv::line(show, pt_cur, pt_pre, cv::Scalar(200));
            cv::imshow("keypoints matches",show);
            cv::waitKey();
        }
    }
}

}