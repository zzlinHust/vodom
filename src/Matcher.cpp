//
// Created by cbt on 18-12-25.
//
#include <algorithm>
#include "Input.h"
#include "Matcher.h"
#include "g2o_types.h"

#include "fortest.h"

using namespace std;

namespace myslam
{

uint MatSum(const cv::Mat &mat)
{
    uint sum = 0;
    int rows = mat.rows;
    int cols = mat.cols;

    for(int i = 0 ; i < rows ; ++i)
    {
        const uchar *data = mat.ptr<uchar>(i);
        for(int j = 0 ; j < cols ; ++j)
        {
            sum += *data++;
        }
    }
    return sum;
}



Matcher::Matcher(MatcherParam param)
{

}

/** 计算描述子汉明距离 **/
int Matcher::HammingDistance(const cv::Mat &a, const cv::Mat &b)
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

float Matcher::ComputeSTAD(const cv::Mat &temp, const cv::Mat &win, const float avg_diff, float thresh)
{
    assert(temp.size == win.size || temp.type() == win.type());
    float sum = 0;
    int rows = win.rows;
    int cols = win.cols;

    for(int i = 0 ; i < rows ; ++i)
    {
        const uchar *t = temp.ptr<uchar>(i);
        const uchar *w = win.ptr<uchar>(i);
        for(int j = 0 ; j < cols ; ++j)
        {
            float dist = fabs(*w++ - *t++ + avg_diff);
            sum += dist > thresh ? thresh : dist;
        }
    }

    return sum;
}

/** 双目匹配与深度计算 **/
void Matcher::StereoMatch(Input* input)
{
    static uint _hamming_thresh = 100; /// param
    static int epipolar_earch_len = 5;
    static int win = 5;
    static int win_size = 2 * win +1;
    static int win_total = win_size * win_size;
    static float win_total_inv = 1.0f / win_total;
    static float AD_thresh = 60;
    static float disparity_min = 2;
    static float disparity_max = 500;
    static float select_ratio = 0.65;

    //输入： 左目图像和特征 、 右目图像和特征
    //目的： 在右图中找到与左图特征点匹配的特征点
    std::vector<float> &mMatch = input->mLeft.match;
    std::vector<cv::KeyPoint> &mKeyPointsL = input->mLeft.keyPoints;
    std::vector<cv::KeyPoint> &mKeyPointsR = input->mRight.keyPoints;
    cv::Mat &mDescL = input->mLeft.descriptor;
    cv::Mat &mDescR = input->mRight.descriptor;

    cv::Mat &mImgL = input->mLeft.imgPyr[0];
    cv::Mat &mImgR = input->mRight.imgPyr[0];

    mMatch.resize(mKeyPointsL.size(), -1);
    input->mLeft.depth.resize(mKeyPointsL.size(), -1);
    /** 1. 确定每个特征极线搜索区域 **/
    /* 极线长度可以根据双目实际参数以及深度检测范围来设置，这里默认[0,cols-1],即一整行 */
    /**
     * 极线搜索区域： 可以根据实际相机参数进行设置，如通过左右目重叠视角、深度检测范围等实际场景设置，
     *              这样一个是能够精准匹配减少计算量，另一个是减少错误匹配的概率。
     * STAD结果筛选： 这里仅选取一定比率的优质匹配点就算深度，还能综合其他方式，如结合视差、直接STAD阈值、自适应阈值等
     *                     根据实际场景权衡精度与鲁棒性
     * **/

    static vector<float> search_rad;
    if(search_rad.empty())
    {
        search_rad.resize(input->nLevels);
        search_rad[0] = 2;
        for(int i = 1 ; i < input->nLevels ; ++i)
            search_rad[i] = input->scaleFactor * search_rad[i-1];
    }

    vector<vector<int>> search_range(static_cast<unsigned long>(mImgL.rows)); // search_range[i]的意义为右图中可能与左图第i行特征点匹配的所有特征。
    for( int index = 0 ; index < mKeyPointsR.size() ; ++index )
    {
        const auto &kp = mKeyPointsR[index];
        int minRow = cvRound(floor(kp.pt.y - search_rad[kp.octave]));
        int maxRow = cvRound(ceil(kp.pt.y + search_rad[kp.octave]));
        for(int j = minRow ; j <= maxRow ; ++j)
            search_range[j].push_back(index);
    }

    /** 2. 通过Hamming距离获取最佳匹配,极线上块匹配筛选求精，最后亚像素优化 **/
    vector<pair<float, int>> dist_score;
    for(int i = 0 ; i < mKeyPointsL.size() ; ++i)
    {
        /**  2.1 特征匹配  **/
        const auto &kp = mKeyPointsL[i];
        const cv::Mat &desL = mDescL.row(i);
        const auto &possibleKps = search_range[kp.pt.y];

        int best_match = -1;
        int min_dist = _hamming_thresh; /* 特征点Hamming距离必须小于一个阈值 */
        for(const auto &kpIndex : possibleKps )
        {
            if(fabs(mKeyPointsR[kpIndex].octave - kp.octave) > 1) continue;
            const cv::Mat &desR = mDescR.row(kpIndex);
            int dist = HammingDistance(desL, desR);
            if(dist < min_dist)
            {
                min_dist = dist;
                best_match = kpIndex;
            }
        }
        if(best_match == -1)
            continue;


        /**  2.2 STAD 块匹配  **/
        auto kpR = mKeyPointsR[best_match];
        float scale = input->mExtractor->GetScale(kpR.octave);
        int col_begin = kpR.pt.x * scale - win - epipolar_earch_len;
        int col_end = kpR.pt.x * scale + win + epipolar_earch_len + 1;
        cv::Mat &search_img = input->mRight.imgPyr[kpR.octave];

        if(col_begin < 0 || col_end >= search_img.cols)
            return;

        cv::Range row_range(kp.pt.y*scale - win, kp.pt.y * scale + win + 1);
        cv::Range col_range(kp.pt.x*scale - win, kp.pt.x * scale + win + 1);
        cv::Mat leftTemplate = input->mLeft.imgPyr[kpR.octave].rowRange(row_range).colRange(col_range);
        float tempAvg = MatSum(leftTemplate) * win_total_inv;
//        cout << "temp avg : " << tempAvg << endl;

        cv::Mat search_roi = search_img.rowRange(row_range).colRange(col_begin, col_end);
        vector<int> colSum(col_end - col_begin, 0);
        for(int k = 0 ; k < search_roi.cols ; ++k)
            colSum[k] = MatSum(search_roi.col(k));

        int matchSum = std::accumulate(colSum.begin(),colSum.begin()+win_size,0);
        vector<float> dist_STAD(win_size, -1);
        float min_STAD_index = -1;
        float min_STAD_sum = INFINITY;
        for(int j = 0 ; j < win_size ; ++j)
        {
            if(j)
            {
                matchSum += colSum[win_size + j];
                matchSum -= colSum[j];
            }
            float avg_diff = matchSum * win_total_inv - tempAvg;
            dist_STAD[j] = ComputeSTAD(leftTemplate, search_roi.colRange(j, win_size + j), avg_diff, AD_thresh);
            if(dist_STAD[j] < min_STAD_sum)
            {
                min_STAD_sum = dist_STAD[j];
                min_STAD_index = j;
            }
//            cout << j << "  " << matchSum << endl;
        }

        if(min_STAD_index == 0 || min_STAD_index == win_size - 1)
            continue;

        /**  2.3 disparity refinement  **/
        /* 抛物线拟合，三个点代入 y = a(x-(x0-dx))^2 + c ， 求 dx */
        float yl = dist_STAD[min_STAD_index-1]; // x0-1
        float ym = dist_STAD[min_STAD_index];   // x0
        float yr = dist_STAD[min_STAD_index+1]; // x0+1
        float dx = (yr - yl) / (2.0f * (yl + yr - 2.0f * ym));
        float best_xR = (min_STAD_index - dx + win + col_begin) / scale;
        float disparity = kp.pt.x - best_xR;

        /**  2.4 深度计算  **/
        if(disparity > disparity_min && disparity < disparity_max)
        {
            mMatch[i] = best_xR;
            input->mLeft.depth[i] = input->mbf / disparity;
            dist_score.emplace_back(min_STAD_sum, i);
        }
    }

    /** 筛选准确匹配 **/
    sort(dist_score.begin(), dist_score.end());
    for(auto it = dist_score.begin() + dist_score.size() * select_ratio ; it != dist_score.end() ; ++it)
    {
        mMatch[it->second] = -1;
        input->mLeft.depth[it->second] = -1;
    }

    /*  for test
    for(int i = 0 ; i < dist_score.size() ; ++i)
    {
        auto &kpl = mKeyPointsL[dist_score[i].second];
        float xr = mMatch[dist_score[i].second]; // 测试时注意上面的需mMatch需保留

        cv::Mat test;
        cv::Point2f tes(mImgL.cols,0);
        cv::hconcat(mImgL,mImgR,test);

        cv::Point2f left = kpl.pt;
        cv::Point2f r1 = cv::Point2f(xr, kpl.pt.y);

        cv::circle(test, left, 5, cv::Scalar(128));
        cv::circle(test, r1 + tes,5,cv::Scalar(128));
        cv::line(test,left, r1+tes,cv::Scalar(150));
        if(i < dist_score.size() * select_ratio)
            cout << i << "  d : " << kpl.pt.x - xr << "   ||  Dist : " << dist_score[i].first << endl << endl;
        else
            cerr << i << "  d : " << kpl.pt.x - xr << "   ||  Dist : " << dist_score[i].first << endl << endl;
        cv::imshow("tesd",test);
        cv::waitKey();
    }
     */
}


void Matcher::DirectMethodMatching(Frame::Ptr cur_, Frame::Ptr pre_, FeatureExtraction::Ptr extract)
{
    const auto &cur_pyr = cur_->mImagePyr;
    const auto &cur_kpt = cur_->mKeyPoints;
    const auto &cur_des = cur_->mDescriptor;

    const auto &pre_pyr = pre_->mImagePyr;
    const auto &pre_kpt = pre_->mKeyPoints;
    const auto &pre_des = pre_->mDescriptor;
    const auto &pos_world = pre_->mMapPoints;

    auto &Tcw = cur_->T_c_w_;

    const float fx = cur_->mCamera->fx_;
    const float fy = cur_->mCamera->fy_;
    const float cx = cur_->mCamera->cx_;
    const float cy = cur_->mCamera->cy_;

    size_t ii = 1; // 保证金字塔至少3层
    size_t ini = cur_pyr.size() - 1;

    if(cur_pyr.size() > 4)
    {
        ii = 2;
        ini = (ini >> 1) << 1;
    }

    Tcw = pre_->T_c_w_;
    g2o::SE3Quat se3( Tcw.rotation_matrix(), Tcw.translation() );
    for(size_t i = ini ;  ; i -= ii )
    {
        float scale = extract->GetScale(i);
        // 直接法跟踪特征点
        float pfx = scale * fx;
        float pfy = scale * fy;
        float pcx = scale * cx;
        float pcy = scale * cy;

        // 初始化g2o
        typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,1>> DirectBlock;  // 求解的向量是6＊1的
        DirectBlock::LinearSolverType* linearSolver = new g2o::LinearSolverDense< DirectBlock::PoseMatrixType > ();
        DirectBlock* solver_ptr = new DirectBlock ( linearSolver );

        g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( solver_ptr ); // L-M
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm ( solver );
        optimizer.setVerbose( false );

        g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
        pose->setEstimate ( se3 );
        pose->setId ( 0 );
        optimizer.addVertex ( pose );


        int id = 0;
        for(int j = 0 ; j < pos_world.size() ; ++j)
        {
            const auto &pos = pos_world[j];
            auto pt = pre_kpt[j].pt;
            if(pos)
            {
                EdgeDirect *edge = new EdgeDirect(pos->mPos3d,fx,fy,cx,cy,&cur_pyr[i]);
                edge->setVertex(0, pose);
                edge->setMeasurement(double(pre_pyr[i].at<uchar>(scale * pt)));
                edge->setInformation( Eigen::Matrix<double,1,1>::Identity());
                edge->setId(id++);
                optimizer.addEdge ( edge );
            }
        }

        optimizer.initializeOptimization();
//        log("match", "begin opt");
        optimizer.optimize ( 30 );

        se3 = pose->estimate();

        if(!i) break;
    }

    Tcw = Sophus::SE3(se3.rotation(), se3.translation());
//    cout << Tcw.translation().transpose() << endl;
//    Tcw = se3.to_homogeneous_matrix();

//    Tcw = se3;
}

}