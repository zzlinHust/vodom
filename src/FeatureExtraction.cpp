//
// Created by cbt on 18-12-13.
//

#include "FeatureExtraction.h"
#include <thread>
#include <opencv2/imgproc.hpp>
#include <opencv/cv.hpp>

using namespace std;

namespace myslam
{
const float toRad = (float)(CV_PI/180.f);
static int bit_pattern_31_[256*4] =
{
    8,-3, 9,5/*mean (0), correlation (0)*/,
    4,2, 7,-12/*mean (1.12461e-05), correlation (0.0437584)*/,
    -11,9, -8,2/*mean (3.37382e-05), correlation (0.0617409)*/,
    7,-12, 12,-13/*mean (5.62303e-05), correlation (0.0636977)*/,
    2,-13, 2,12/*mean (0.000134953), correlation (0.085099)*/,
    1,-7, 1,6/*mean (0.000528565), correlation (0.0857175)*/,
    -2,-10, -2,-4/*mean (0.0188821), correlation (0.0985774)*/,
    -13,-13, -11,-8/*mean (0.0363135), correlation (0.0899616)*/,
    -13,-3, -12,-9/*mean (0.121806), correlation (0.099849)*/,
    10,4, 11,9/*mean (0.122065), correlation (0.093285)*/,
    -13,-8, -8,-9/*mean (0.162787), correlation (0.0942748)*/,
    -11,7, -9,12/*mean (0.21561), correlation (0.0974438)*/,
    7,7, 12,6/*mean (0.160583), correlation (0.130064)*/,
    -4,-5, -3,0/*mean (0.228171), correlation (0.132998)*/,
    -13,2, -12,-3/*mean (0.00997526), correlation (0.145926)*/,
    -9,0, -7,5/*mean (0.198234), correlation (0.143636)*/,
    12,-6, 12,-1/*mean (0.0676226), correlation (0.16689)*/,
    -3,6, -2,12/*mean (0.166847), correlation (0.171682)*/,
    -6,-13, -4,-8/*mean (0.101215), correlation (0.179716)*/,
    11,-13, 12,-8/*mean (0.200641), correlation (0.192279)*/,
    4,7, 5,1/*mean (0.205106), correlation (0.186848)*/,
    5,-3, 10,-3/*mean (0.234908), correlation (0.192319)*/,
    3,-7, 6,12/*mean (0.0709964), correlation (0.210872)*/,
    -8,-7, -6,-2/*mean (0.0939834), correlation (0.212589)*/,
    -2,11, -1,-10/*mean (0.127778), correlation (0.20866)*/,
    -13,12, -8,10/*mean (0.14783), correlation (0.206356)*/,
    -7,3, -5,-3/*mean (0.182141), correlation (0.198942)*/,
    -4,2, -3,7/*mean (0.188237), correlation (0.21384)*/,
    -10,-12, -6,11/*mean (0.14865), correlation (0.23571)*/,
    5,-12, 6,-7/*mean (0.222312), correlation (0.23324)*/,
    5,-6, 7,-1/*mean (0.229082), correlation (0.23389)*/,
    1,0, 4,-5/*mean (0.241577), correlation (0.215286)*/,
    9,11, 11,-13/*mean (0.00338507), correlation (0.251373)*/,
    4,7, 4,12/*mean (0.131005), correlation (0.257622)*/,
    2,-1, 4,4/*mean (0.152755), correlation (0.255205)*/,
    -4,-12, -2,7/*mean (0.182771), correlation (0.244867)*/,
    -8,-5, -7,-10/*mean (0.186898), correlation (0.23901)*/,
    4,11, 9,12/*mean (0.226226), correlation (0.258255)*/,
    0,-8, 1,-13/*mean (0.0897886), correlation (0.274827)*/,
    -13,-2, -8,2/*mean (0.148774), correlation (0.28065)*/,
    -3,-2, -2,3/*mean (0.153048), correlation (0.283063)*/,
    -6,9, -4,-9/*mean (0.169523), correlation (0.278248)*/,
    8,12, 10,7/*mean (0.225337), correlation (0.282851)*/,
    0,9, 1,3/*mean (0.226687), correlation (0.278734)*/,
    7,-5, 11,-10/*mean (0.00693882), correlation (0.305161)*/,
    -13,-6, -11,0/*mean (0.0227283), correlation (0.300181)*/,
    10,7, 12,1/*mean (0.125517), correlation (0.31089)*/,
    -6,-3, -6,12/*mean (0.131748), correlation (0.312779)*/,
    10,-9, 12,-4/*mean (0.144827), correlation (0.292797)*/,
    -13,8, -8,-12/*mean (0.149202), correlation (0.308918)*/,
    -13,0, -8,-4/*mean (0.160909), correlation (0.310013)*/,
    3,3, 7,8/*mean (0.177755), correlation (0.309394)*/,
    5,7, 10,-7/*mean (0.212337), correlation (0.310315)*/,
    -1,7, 1,-12/*mean (0.214429), correlation (0.311933)*/,
    3,-10, 5,6/*mean (0.235807), correlation (0.313104)*/,
    2,-4, 3,-10/*mean (0.00494827), correlation (0.344948)*/,
    -13,0, -13,5/*mean (0.0549145), correlation (0.344675)*/,
    -13,-7, -12,12/*mean (0.103385), correlation (0.342715)*/,
    -13,3, -11,8/*mean (0.134222), correlation (0.322922)*/,
    -7,12, -4,7/*mean (0.153284), correlation (0.337061)*/,
    6,-10, 12,8/*mean (0.154881), correlation (0.329257)*/,
    -9,-1, -7,-6/*mean (0.200967), correlation (0.33312)*/,
    -2,-5, 0,12/*mean (0.201518), correlation (0.340635)*/,
    -12,5, -7,5/*mean (0.207805), correlation (0.335631)*/,
    3,-10, 8,-13/*mean (0.224438), correlation (0.34504)*/,
    -7,-7, -4,5/*mean (0.239361), correlation (0.338053)*/,
    -3,-2, -1,-7/*mean (0.240744), correlation (0.344322)*/,
    2,9, 5,-11/*mean (0.242949), correlation (0.34145)*/,
    -11,-13, -5,-13/*mean (0.244028), correlation (0.336861)*/,
    -1,6, 0,-1/*mean (0.247571), correlation (0.343684)*/,
    5,-3, 5,2/*mean (0.000697256), correlation (0.357265)*/,
    -4,-13, -4,12/*mean (0.00213675), correlation (0.373827)*/,
    -9,-6, -9,6/*mean (0.0126856), correlation (0.373938)*/,
    -12,-10, -8,-4/*mean (0.0152497), correlation (0.364237)*/,
    10,2, 12,-3/*mean (0.0299933), correlation (0.345292)*/,
    7,12, 12,12/*mean (0.0307242), correlation (0.366299)*/,
    -7,-13, -6,5/*mean (0.0534975), correlation (0.368357)*/,
    -4,9, -3,4/*mean (0.099865), correlation (0.372276)*/,
    7,-1, 12,2/*mean (0.117083), correlation (0.364529)*/,
    -7,6, -5,1/*mean (0.126125), correlation (0.369606)*/,
    -13,11, -12,5/*mean (0.130364), correlation (0.358502)*/,
    -3,7, -2,-6/*mean (0.131691), correlation (0.375531)*/,
    7,-8, 12,-7/*mean (0.160166), correlation (0.379508)*/,
    -13,-7, -11,-12/*mean (0.167848), correlation (0.353343)*/,
    1,-3, 12,12/*mean (0.183378), correlation (0.371916)*/,
    2,-6, 3,0/*mean (0.228711), correlation (0.371761)*/,
    -4,3, -2,-13/*mean (0.247211), correlation (0.364063)*/,
    -1,-13, 1,9/*mean (0.249325), correlation (0.378139)*/,
    7,1, 8,-6/*mean (0.000652272), correlation (0.411682)*/,
    1,-1, 3,12/*mean (0.00248538), correlation (0.392988)*/,
    9,1, 12,6/*mean (0.0206815), correlation (0.386106)*/,
    -1,-9, -1,3/*mean (0.0364485), correlation (0.410752)*/,
    -13,-13, -10,5/*mean (0.0376068), correlation (0.398374)*/,
    7,7, 10,12/*mean (0.0424202), correlation (0.405663)*/,
    12,-5, 12,9/*mean (0.0942645), correlation (0.410422)*/,
    6,3, 7,11/*mean (0.1074), correlation (0.413224)*/,
    5,-13, 6,10/*mean (0.109256), correlation (0.408646)*/,
    2,-12, 2,3/*mean (0.131691), correlation (0.416076)*/,
    3,8, 4,-6/*mean (0.165081), correlation (0.417569)*/,
    2,6, 12,-13/*mean (0.171874), correlation (0.408471)*/,
    9,-12, 10,3/*mean (0.175146), correlation (0.41296)*/,
    -8,4, -7,9/*mean (0.183682), correlation (0.402956)*/,
    -11,12, -4,-6/*mean (0.184672), correlation (0.416125)*/,
    1,12, 2,-8/*mean (0.191487), correlation (0.386696)*/,
    6,-9, 7,-4/*mean (0.192668), correlation (0.394771)*/,
    2,3, 3,-2/*mean (0.200157), correlation (0.408303)*/,
    6,3, 11,0/*mean (0.204588), correlation (0.411762)*/,
    3,-3, 8,-8/*mean (0.205904), correlation (0.416294)*/,
    7,8, 9,3/*mean (0.213237), correlation (0.409306)*/,
    -11,-5, -6,-4/*mean (0.243444), correlation (0.395069)*/,
    -10,11, -5,10/*mean (0.247672), correlation (0.413392)*/,
    -5,-8, -3,12/*mean (0.24774), correlation (0.411416)*/,
    -10,5, -9,0/*mean (0.00213675), correlation (0.454003)*/,
    8,-1, 12,-6/*mean (0.0293635), correlation (0.455368)*/,
    4,-6, 6,-11/*mean (0.0404971), correlation (0.457393)*/,
    -10,12, -8,7/*mean (0.0481107), correlation (0.448364)*/,
    4,-2, 6,7/*mean (0.050641), correlation (0.455019)*/,
    -2,0, -2,12/*mean (0.0525978), correlation (0.44338)*/,
    -5,-8, -5,2/*mean (0.0629667), correlation (0.457096)*/,
    7,-6, 10,12/*mean (0.0653846), correlation (0.445623)*/,
    -9,-13, -8,-8/*mean (0.0858749), correlation (0.449789)*/,
    -5,-13, -5,-2/*mean (0.122402), correlation (0.450201)*/,
    8,-8, 9,-13/*mean (0.125416), correlation (0.453224)*/,
    -9,-11, -9,0/*mean (0.130128), correlation (0.458724)*/,
    1,-8, 1,-2/*mean (0.132467), correlation (0.440133)*/,
    7,-4, 9,1/*mean (0.132692), correlation (0.454)*/,
    -2,1, -1,-4/*mean (0.135695), correlation (0.455739)*/,
    11,-6, 12,-11/*mean (0.142904), correlation (0.446114)*/,
    -12,-9, -6,4/*mean (0.146165), correlation (0.451473)*/,
    3,7, 7,12/*mean (0.147627), correlation (0.456643)*/,
    5,5, 10,8/*mean (0.152901), correlation (0.455036)*/,
    0,-4, 2,8/*mean (0.167083), correlation (0.459315)*/,
    -9,12, -5,-13/*mean (0.173234), correlation (0.454706)*/,
    0,7, 2,12/*mean (0.18312), correlation (0.433855)*/,
    -1,2, 1,7/*mean (0.185504), correlation (0.443838)*/,
    5,11, 7,-9/*mean (0.185706), correlation (0.451123)*/,
    3,5, 6,-8/*mean (0.188968), correlation (0.455808)*/,
    -13,-4, -8,9/*mean (0.191667), correlation (0.459128)*/,
    -5,9, -3,-3/*mean (0.193196), correlation (0.458364)*/,
    -4,-7, -3,-12/*mean (0.196536), correlation (0.455782)*/,
    6,5, 8,0/*mean (0.1972), correlation (0.450481)*/,
    -7,6, -6,12/*mean (0.199438), correlation (0.458156)*/,
    -13,6, -5,-2/*mean (0.211224), correlation (0.449548)*/,
    1,-10, 3,10/*mean (0.211718), correlation (0.440606)*/,
    4,1, 8,-4/*mean (0.213034), correlation (0.443177)*/,
    -2,-2, 2,-13/*mean (0.234334), correlation (0.455304)*/,
    2,-12, 12,12/*mean (0.235684), correlation (0.443436)*/,
    -2,-13, 0,-6/*mean (0.237674), correlation (0.452525)*/,
    4,1, 9,3/*mean (0.23962), correlation (0.444824)*/,
    -6,-10, -3,-5/*mean (0.248459), correlation (0.439621)*/,
    -3,-13, -1,1/*mean (0.249505), correlation (0.456666)*/,
    7,5, 12,-11/*mean (0.00119208), correlation (0.495466)*/,
    4,-2, 5,-7/*mean (0.00372245), correlation (0.484214)*/,
    -13,9, -9,-5/*mean (0.00741116), correlation (0.499854)*/,
    7,1, 8,6/*mean (0.0208952), correlation (0.499773)*/,
    7,-8, 7,6/*mean (0.0220085), correlation (0.501609)*/,
    -7,-4, -7,1/*mean (0.0233806), correlation (0.496568)*/,
    -8,11, -7,-8/*mean (0.0236505), correlation (0.489719)*/,
    -13,6, -12,-8/*mean (0.0268781), correlation (0.503487)*/,
    2,4, 3,9/*mean (0.0323324), correlation (0.501938)*/,
    10,-5, 12,3/*mean (0.0399235), correlation (0.494029)*/,
    -6,-5, -6,7/*mean (0.0420153), correlation (0.486579)*/,
    8,-3, 9,-8/*mean (0.0548021), correlation (0.484237)*/,
    2,-12, 2,8/*mean (0.0616622), correlation (0.496642)*/,
    -11,-2, -10,3/*mean (0.0627755), correlation (0.498563)*/,
    -12,-13, -7,-9/*mean (0.0829622), correlation (0.495491)*/,
    -11,0, -10,-5/*mean (0.0843342), correlation (0.487146)*/,
    5,-3, 11,8/*mean (0.0929937), correlation (0.502315)*/,
    -2,-13, -1,12/*mean (0.113327), correlation (0.48941)*/,
    -1,-8, 0,9/*mean (0.132119), correlation (0.467268)*/,
    -13,-11, -12,-5/*mean (0.136269), correlation (0.498771)*/,
    -10,-2, -10,11/*mean (0.142173), correlation (0.498714)*/,
    -3,9, -2,-13/*mean (0.144141), correlation (0.491973)*/,
    2,-3, 3,2/*mean (0.14892), correlation (0.500782)*/,
    -9,-13, -4,0/*mean (0.150371), correlation (0.498211)*/,
    -4,6, -3,-10/*mean (0.152159), correlation (0.495547)*/,
    -4,12, -2,-7/*mean (0.156152), correlation (0.496925)*/,
    -6,-11, -4,9/*mean (0.15749), correlation (0.499222)*/,
    6,-3, 6,11/*mean (0.159211), correlation (0.503821)*/,
    -13,11, -5,5/*mean (0.162427), correlation (0.501907)*/,
    11,11, 12,6/*mean (0.16652), correlation (0.497632)*/,
    7,-5, 12,-2/*mean (0.169141), correlation (0.484474)*/,
    -1,12, 0,7/*mean (0.169456), correlation (0.495339)*/,
    -4,-8, -3,-2/*mean (0.171457), correlation (0.487251)*/,
    -7,1, -6,7/*mean (0.175), correlation (0.500024)*/,
    -13,-12, -8,-13/*mean (0.175866), correlation (0.497523)*/,
    -7,-2, -6,-8/*mean (0.178273), correlation (0.501854)*/,
    -8,5, -6,-9/*mean (0.181107), correlation (0.494888)*/,
    -5,-1, -4,5/*mean (0.190227), correlation (0.482557)*/,
    -13,7, -8,10/*mean (0.196739), correlation (0.496503)*/,
    1,5, 5,-13/*mean (0.19973), correlation (0.499759)*/,
    1,0, 10,-13/*mean (0.204465), correlation (0.49873)*/,
    9,12, 10,-1/*mean (0.209334), correlation (0.49063)*/,
    5,-8, 10,-9/*mean (0.211134), correlation (0.503011)*/,
    -1,11, 1,-13/*mean (0.212), correlation (0.499414)*/,
    -9,-3, -6,2/*mean (0.212168), correlation (0.480739)*/,
    -1,-10, 1,12/*mean (0.212731), correlation (0.502523)*/,
    -13,1, -8,-10/*mean (0.21327), correlation (0.489786)*/,
    8,-11, 10,-6/*mean (0.214159), correlation (0.488246)*/,
    2,-13, 3,-6/*mean (0.216993), correlation (0.50287)*/,
    7,-13, 12,-9/*mean (0.223639), correlation (0.470502)*/,
    -10,-10, -5,-7/*mean (0.224089), correlation (0.500852)*/,
    -10,-8, -8,-13/*mean (0.228666), correlation (0.502629)*/,
    4,-6, 8,5/*mean (0.22906), correlation (0.498305)*/,
    3,12, 8,-13/*mean (0.233378), correlation (0.503825)*/,
    -4,2, -3,-3/*mean (0.234323), correlation (0.476692)*/,
    5,-13, 10,-12/*mean (0.236392), correlation (0.475462)*/,
    4,-13, 5,-1/*mean (0.236842), correlation (0.504132)*/,
    -9,9, -4,3/*mean (0.236977), correlation (0.497739)*/,
    0,3, 3,-9/*mean (0.24314), correlation (0.499398)*/,
    -12,1, -6,1/*mean (0.243297), correlation (0.489447)*/,
    3,2, 4,-8/*mean (0.00155196), correlation (0.553496)*/,
    -10,-10, -10,9/*mean (0.00239541), correlation (0.54297)*/,
    8,-13, 12,12/*mean (0.0034413), correlation (0.544361)*/,
    -8,-12, -6,-5/*mean (0.003565), correlation (0.551225)*/,
    2,2, 3,7/*mean (0.00835583), correlation (0.55285)*/,
    10,6, 11,-8/*mean (0.00885065), correlation (0.540913)*/,
    6,8, 8,-12/*mean (0.0101552), correlation (0.551085)*/,
    -7,10, -6,5/*mean (0.0102227), correlation (0.533635)*/,
    -3,-9, -3,9/*mean (0.0110211), correlation (0.543121)*/,
    -1,-13, -1,5/*mean (0.0113473), correlation (0.550173)*/,
    -3,-7, -3,4/*mean (0.0140913), correlation (0.554774)*/,
    -8,-2, -8,3/*mean (0.017049), correlation (0.55461)*/,
    4,2, 12,12/*mean (0.01778), correlation (0.546921)*/,
    2,-5, 3,11/*mean (0.0224022), correlation (0.549667)*/,
    6,-9, 11,-13/*mean (0.029161), correlation (0.546295)*/,
    3,-1, 7,12/*mean (0.0303081), correlation (0.548599)*/,
    11,-1, 12,4/*mean (0.0355151), correlation (0.523943)*/,
    -3,0, -3,6/*mean (0.0417904), correlation (0.543395)*/,
    4,-11, 4,12/*mean (0.0487292), correlation (0.542818)*/,
    2,-4, 2,1/*mean (0.0575124), correlation (0.554888)*/,
    -10,-6, -8,1/*mean (0.0594242), correlation (0.544026)*/,
    -13,7, -11,1/*mean (0.0597391), correlation (0.550524)*/,
    -13,12, -11,-13/*mean (0.0608974), correlation (0.55383)*/,
    6,0, 11,-13/*mean (0.065126), correlation (0.552006)*/,
    0,-1, 1,4/*mean (0.074224), correlation (0.546372)*/,
    -13,3, -9,-2/*mean (0.0808592), correlation (0.554875)*/,
    -9,8, -6,-3/*mean (0.0883378), correlation (0.551178)*/,
    -13,-6, -8,-2/*mean (0.0901035), correlation (0.548446)*/,
    5,-9, 8,10/*mean (0.0949843), correlation (0.554694)*/,
    2,7, 3,-9/*mean (0.0994152), correlation (0.550979)*/,
    -1,-6, -1,-1/*mean (0.10045), correlation (0.552714)*/,
    9,5, 11,-2/*mean (0.100686), correlation (0.552594)*/,
    11,-3, 12,-8/*mean (0.101091), correlation (0.532394)*/,
    3,0, 3,5/*mean (0.101147), correlation (0.525576)*/,
    -1,4, 0,10/*mean (0.105263), correlation (0.531498)*/,
    3,-6, 4,5/*mean (0.110785), correlation (0.540491)*/,
    -13,0, -10,5/*mean (0.112798), correlation (0.536582)*/,
    5,8, 12,11/*mean (0.114181), correlation (0.555793)*/,
    8,9, 9,-6/*mean (0.117431), correlation (0.553763)*/,
    7,-4, 8,-12/*mean (0.118522), correlation (0.553452)*/,
    -10,4, -10,9/*mean (0.12094), correlation (0.554785)*/,
    7,3, 12,4/*mean (0.122582), correlation (0.555825)*/,
    9,-7, 10,-2/*mean (0.124978), correlation (0.549846)*/,
    7,0, 12,-2/*mean (0.127002), correlation (0.537452)*/,
    -1,-6, 0,-11/*mean (0.127148), correlation (0.547401)*/
};

FeatureExtraction::FeatureExtraction()
{
    FeatureExtractionParam &param = mParam;
    param.levels = 8;
    param.thresh_FAST = 20;
    param.thresh_FAST_min = 7;
    param.feature_num = 1000;
    param.scale_factor = 1.2;
}

FeatureExtraction::FeatureExtraction(const FeatureExtractionParam &param) : mParam(param)
{
    size_t n = size_t(mParam.levels);
    mPatchRadium = mParam.rad_FAST_orient;
    mPatchSize = 2 * mPatchRadium + 1;
    mEdgePreserve = mPatchRadium + 1;

    mImagePyramid.resize(n);
    mFeaturesPyramid.resize(n);
    mScale.resize(n);
    mSigma.resize(n);

    float factor = mParam.scale_factor;
    float scale = 1.0f / factor;
    float nfeatures = mParam.feature_num * (1.0f - scale) / (1.0f - std::pow(scale, param.levels));
    mScale[0] = 1.0f;
    mSigma[0] = 1.0f;
    int sum = mFeaturesPyramid[0] = static_cast<unsigned int>(cvRound(nfeatures));

    for (int i = 1; i < n; ++i)
    {
        nfeatures *= scale;
        mFeaturesPyramid[i] = static_cast<unsigned int>(cvRound(nfeatures));
        mScale[i] = scale * mScale[i-1];
        mSigma[i] = factor * factor * mSigma[i-1];
        sum += mFeaturesPyramid[i];
    }
    mFeaturesPyramid[n-1] -= sum - mParam.feature_num;
    if (mFeaturesPyramid[n-1] < 0) mFeaturesPyramid[n-1] = 0;


    const unsigned int half_patch_size = mPatchSize / 2;
    umax.resize(half_patch_size + 1);
    int v, v0, vmax = cvFloor(half_patch_size * sqrt(2.f) / 2 + 1);
    int vmin = cvCeil(half_patch_size * sqrt(2.f) / 2);
    const double hp2 = half_patch_size * half_patch_size;
    for (v = 0 ; v <= vmax ; ++v)
        umax[v] = cvRound(sqrt(hp2 - v * v));

    // Make sure we are symmetric
    for (v = half_patch_size , v0 = 0 ; v >= vmin ; --v)
    {
        while (umax[v0] == umax[v0 + 1])
            ++v0;
        umax[v] = v0;
        ++v0;
    }
}

FeatureExtraction::~FeatureExtraction(){}


void FeatureExtraction::Extract(cv::Mat _img, std::vector<cv::KeyPoint> &_keyPoints, cv::OutputArray &_descriptor)
{

    std::cout << "Extract in" << std::endl;
    ComputePyramid(_img);
    std::cout << "ComputePyramid out" << std::endl;
    vector<vector<cv::KeyPoint>> allKeyPoints(mParam.levels);
    vector<cv::Mat> allDescriptor(mParam.levels);
    vector<thread> allThreads(mParam.levels);

//    for(int i = 0 ; i < mParam.levels ; ++i) /// 使用多线程加速并行提取每层金字塔特征
//        allThreads[i] = thread(&FeatureExtraction::ExtractSingleLevel, this, &allKeyPoints[i], &allDescriptor[i], i);
//
//    for(int i = 0 ; i < mParam.levels ; ++i)
//    {
//        if(allThreads[i].joinable())
//            allThreads[i].join();
//    }
    for(int i = 0 ; i < mParam.levels ; ++i)
        ExtractSingleLevel(&allKeyPoints[i], &allDescriptor[i], i);
    std::cout << "Extract finish" << std::endl;
    auto des_it = allDescriptor.begin();
    for(int i = 0 ; i < mParam.levels ; ++i)
    {
        _keyPoints.insert(_keyPoints.end(), allKeyPoints[i].begin(), allKeyPoints[i].end());
        if(des_it->empty())
            des_it = allDescriptor.erase(des_it); // delete empty descriptor
        else
            ++des_it;
    }
    cv::vconcat(allDescriptor, _descriptor);
}

void FeatureExtraction::ExtractSingleLevel(std::vector<cv::KeyPoint> *_keyPoints, cv::Mat *_descriptor, int level)
{
    /** 检测关键点 **/
    std::cout << "Detect in" << std::endl;
    Detect(*_keyPoints, level);

    std::cout << "ComputeAngle in" << std::endl;
    /** 计算关键点方向 **/
    ComputeAngle(*_keyPoints, level);

    std::cout << "ComputeDescriptor in" << std::endl;

    /** 计算描述子 **/
    ComputeDescriptor(*_keyPoints, *_descriptor, level);
}

void FeatureExtraction::ComputePyramid(cv::Mat &src)
{
    cv::imshow("src",src);
    cv::waitKey();
    mImagePyramid[0] = src;
    for(int i = 1 ; i < mParam.levels ; ++i)
    {
        cv::Size sz(cvRound((float)src.cols * mScale[i]), cvRound((float)src.rows * mScale[i]));
        cv::resize(mImagePyramid[i-1], mImagePyramid[i], sz, 0, 0, cv::INTER_LINEAR);
    }
}

/**
 * FAST角点检测、关键点筛选。
 *
 * FAST角点通常会“扎堆”出现，为了使特征点在图像中均匀分布，使用了两个技巧：
 *   1. 将整个图像划分成一定大小格子，分别在每个格子上提取角点，使提取的角点均匀；
 *   2. 使用四叉树不断划分图像，从低层的叶子节点开始，选出目标数量的高响应关键点，使筛选出的关键点均匀分布。
 *
 * Attention：
 * 若使用设置的FAST阈值提取不到角点，则降低阈值重新提取，保证能够提取到足够数量的角点；
 * 对于图像，需要注意留边： FAST的方向计算需取半径内 patch 所有像素点进行计算。因此，图像上下左右必须至少分别预留半径长的像素；
 * 对于划分的格子，FAST选取半径为3的圆上的16个像素点，因此每个格子需要进行padding，保证能提取格子内所有像素的角点。
 */
void FeatureExtraction::Detect(std::vector<cv::KeyPoint> &_keyPoints, int level)
{
    /* 计算留边后的图像边界和自适应的格子大小 */
    const int minX = mEdgePreserve , minY = mEdgePreserve;
    const int maxX = mImagePyramid[level].cols - mEdgePreserve;
    const int maxY = mImagePyramid[level].rows - mEdgePreserve;
    const float winSize = 30;
    const float width = maxX - minX;
    const float height = maxY - minY;
    const int cellCols = int(width / winSize);
    const int cellRows = int(height / winSize);
    const int cellWidth = int(ceil(width / cellCols));
    const int cellHeight = int(ceil(height / cellRows));
    const float scaledPatchSize = mPatchSize / mScale[level];

    /* 对每一格子进行角点提取 */
    for (int i = 0 ; i < cellRows ; ++i)
    {   /* 计算格子的边界，注意边界值 */
        int Y_begin = minY + i * cellHeight;
        int Y_end = Y_begin + cellHeight ;
        if(Y_begin >= maxY) continue;
        if(Y_end > maxY) Y_end = maxY;

        for(int j = 0 ; j < cellCols ; ++j)
        {
            int X_begin = minX + j * cellWidth;
            int X_end = X_begin + cellWidth;
            if(X_begin >= maxX) continue;
            if(X_end > maxX) X_end = maxX;

            /** 注意格子加边3！！ **/
            cv::Mat cellImg = mImagePyramid[level].rowRange(Y_begin-3,Y_end+3).colRange(X_begin-3,X_end+3);
            vector<cv::KeyPoint> kpCell;

            /* 若格子提取不到角点，则降低阈值重新提取 */
            cv::FAST(cellImg, kpCell, mParam.thresh_FAST, true);
            if(kpCell.empty())
                cv::FAST(cellImg, kpCell, mParam.thresh_FAST_min, true);

            if(kpCell.size())
            {  /* 添加关键点信息 */
                for(auto it = kpCell.begin() ; it != kpCell.end() ; ++it)
                {
                    it->pt.x += X_begin - 3;
                    it->pt.y += Y_begin - 3;
                    it->octave = level;
                    it->size = scaledPatchSize;
                    _keyPoints.push_back(*it);
                }
            }
        }
    }
//    cv::Mat img;
//    cv::drawKeypoints(mImagePyramid[level],_keyPoints, img, cv::Scalar::all(-1),cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
//    cv::imshow("d",img);
//    cv::waitKey();
//
    /** 对提取出来的关键点进行筛选，均匀筛选出优质角点 **/
    SortKeyPoint(_keyPoints, cv::Point(minX, minY), cv::Point(maxX, maxY), level);
//
//    cv::drawKeypoints(mImagePyramid[level],_keyPoints, img, cv::Scalar::all(-1),cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
//    cv::imshow("d",img);
//    cv::waitKey();
}

/**
 *  使用四叉树均匀筛选目标数量的关键点
 *
 *  这里使用了三个容器 leavesNode、father、son对四叉树的节点进行了分类。低深度的不能继续划分的叶子节点归到leavesNode；当前四叉树最后一层
 *  的所有节点归到father；刚分裂的非叶子节点归到son。
 */
void FeatureExtraction::SortKeyPoint(std::vector<cv::KeyPoint> &_keyPoints, cv::Point2i minP, cv::Point2i maxP, int level)
{
    const unsigned int initNodesNum = (unsigned int)std::round(float(maxP.x - minP.x) / float(maxP.y - minP.y));
    const float hX = static_cast<float>(maxP.x - minP.x) / initNodesNum;

    std::vector<QuadTreeNode> leavesNode; // 只有一个关键点，不能继续划分的节点
    std::vector<QuadTreeNode> rootNode;
    std::vector<QuadTreeNode> nextLevelNode;
    leavesNode.reserve(mFeaturesPyramid[level]);
    rootNode.reserve(mFeaturesPyramid[level]);
    nextLevelNode.reserve(mFeaturesPyramid[level]);

    /* 构建根节点 */
    rootNode.resize(initNodesNum);
    for(int i = 0 ; i < initNodesNum ; ++i )
    {
        QuadTreeNode &node = rootNode[i];
        node.keyPoints.reserve(_keyPoints.size());
        node.UL = cv::Point2i(int(hX * i), 0);
        node.UR = cv::Point2i(int(hX * (i+1)), 0);
        node.BL = cv::Point2i(node.UL.x, maxP.y - minP.y);
        node.BR = cv::Point2i(node.UR.x, maxP.y - minP.y);
    }
    for(const auto &kp : _keyPoints) // 将角点分配到初始根节点中
        rootNode[(kp.pt.x - minP.x) / hX].keyPoints.push_back(kp);


    auto *father = &rootNode;   // 能够继续划分的父节点
    auto *son = &nextLevelNode; // 子节点
    while (1)
    {
        for(auto &fatherNode : *father) // 分裂父节点
        {
            if(fatherNode.keyPoints.size() == 1) // 不能再分裂，归到叶子节点
            {
                leavesNode.push_back(fatherNode);
            }
            else if (fatherNode.keyPoints.size() > 1)
            {/* 父节点分裂成子节点，只有一个关键点的归到叶子节点，多个关键点的归到子节点 */
                QuadTreeNode sonNode[4];
                fatherNode.DivideToNextLevel(sonNode);
                for (int i = 0 ; i < 4 ; ++i)
                {
                    if(sonNode[i].keyPoints.size() == 1)
                        leavesNode.push_back(sonNode[i]);
                    else if(sonNode[i].keyPoints.size() > 1)
                        son->push_back(sonNode[i]);
                    else{}
                }
            }
            else{}
        }

        if(son->size() >= mFeaturesPyramid[level] || son->size() == father->size()) break;
        else if(leavesNode.size() + son->size() * 3 > mFeaturesPyramid[level])
        {/// 当再次分裂（假设都能一分四）节点数将大于欲选取角点数时，优先对角点多的区域进行划分
            bool finish = false;
            while (!finish)
            {
                father->clear();// 父节点(已划分完，没用了)清空，子节点需要继续划分，作为新的父节点。
                std::swap(father,son);

                std::sort(father->begin(),father->end(), \
                          [](const QuadTreeNode &a, const QuadTreeNode &b) \
                          { return a.keyPoints.size() < b.keyPoints.size();});
                size_t nodes_left = father->size();
                for (auto it = father->rbegin() ; it != father->rend() ; ++it)
                {
                    if(finish) // 当节点数量达到目标数量时，停止划分，父节点直接转到子节点
                    {
                        son->push_back(*it);
                    }
                    else
                    {
                        QuadTreeNode sonNode[4];
                        it->DivideToNextLevel(sonNode);
                        --nodes_left;
                        for (int i = 0 ; i < 4 ; ++i)
                        {
                            if(sonNode[i].keyPoints.size() == 1)
                                leavesNode.push_back(sonNode[i]);
                            else if(sonNode[i].keyPoints.size() > 1)
                                son->push_back(sonNode[i]);
                            else{}
                        }
                        if(leavesNode.size() + son->size() + nodes_left >= mFeaturesPyramid[level])
                        {
                           finish = true;
                        }
                    }
                }
                if(son->size() == father->size())
                    finish = true;
            }
            break;
        }
        else
        {
            father->clear();// 父节点清空，子节点作为新的父节点。
            std::swap(father,son);
        }
    }

    _keyPoints.clear();
    for (auto &node : leavesNode) // 叶子节点仅有一个关键点，直接扔到结果中
        _keyPoints.push_back(node.keyPoints.front());
    for(auto &node : *son)
    {/* 选取节点中响应最大的角点作为关键点 */
        auto it = node.keyPoints.begin();
        auto end = node.keyPoints.end();
        cv::KeyPoint &kp = *it;
        float max_respons = it->response;
        while ((++it) != end)
        {
            if(it->response > max_respons)
            {
                max_respons = it->response;
                kp = *it;
            }
        }
        _keyPoints.push_back(kp);
    }
}

/**
 *  灰度质心法计算方向
 */
void FeatureExtraction::ComputeAngle(std::vector<cv::KeyPoint> &_keyPoints, int level)
{
    cv::Mat &image = mImagePyramid[level];
    const int half_patch_size = mPatchSize / 2;
    for(auto &kp : _keyPoints)
    {
        int m_01 = 0, m_10 = 0;
        const uchar* center = &image.at<uchar>(cvRound(kp.pt.y), cvRound(kp.pt.x));

        // Treat the center line differently, v=0
        for (int u = -half_patch_size ; u <= half_patch_size ; ++u)
            m_10 += u * center[u];

        // Go line by line in the circuI853lar patch
        int step = (int)image.step1();
        for (int v = 1 ; v <= half_patch_size ; ++v)
        {
            // Proceed over the two lines
            int v_sum = 0;
            int d = umax[v];
            for (int u = -d; u <= d; ++u)
            {
                int val_plus = center[u + v*step], val_minus = center[u - v*step];
                v_sum += (val_plus - val_minus);
                m_10 += u * (val_plus + val_minus);
            }
            m_01 += v * v_sum;
        }
        kp.angle = cv::fastAtan2((float)m_01, (float)m_10);
    }
}

void FeatureExtraction::ComputeDescriptor(std::vector<cv::KeyPoint> &_keyPoints, cv::Mat &_descriptor, int level)
{
    int nKeyPoints = int(_keyPoints.size());
    if( 0 == nKeyPoints )
    {
        _descriptor.release();
        return;
    }

    cv::Mat img;
    GaussianBlur(mImagePyramid[level], img, cv::Size(7,7), 2, 2, cv::BORDER_REFLECT_101);

    _descriptor = cv::Mat::zeros(nKeyPoints, 32, CV_8UC1);

    const double factor = 1.0 / mScale[level];
    for(int i = 0 ; i < nKeyPoints ; ++i)
    {
        auto &kpt = _keyPoints[i];
        float angle = kpt.angle * toRad;
        float a = cos(angle), b = sin(angle);

        const uchar* center = &img.at<uchar>(cvRound(kpt.pt.y), cvRound(kpt.pt.x));
         const int step = (int)img.step;

        uchar *desc = _descriptor.ptr(i);
        const int *pattern = bit_pattern_31_;

        #define  GET_VALUE(idx) \
        center[cvRound(pattern[idx]*b + pattern[idx+1]*a)*step + \
               cvRound(pattern[idx]*a - pattern[idx+1]*b)]

        for (int j = 0; j < 32 ; ++j, pattern += 32)
        {
            int t0, t1, val;
            t0 = GET_VALUE(0); t1 = GET_VALUE(2);
            val = t0 < t1;
            t0 = GET_VALUE(4); t1 = GET_VALUE(6);
            val |= (t0 < t1) << 1;
            t0 = GET_VALUE(8); t1 = GET_VALUE(10);
            val |= (t0 < t1) << 2;
            t0 = GET_VALUE(12); t1 = GET_VALUE(14);
            val |= (t0 < t1) << 3;
            t0 = GET_VALUE(16); t1 = GET_VALUE(18);
            val |= (t0 < t1) << 4;
            t0 = GET_VALUE(20); t1 = GET_VALUE(22);
            val |= (t0 < t1) << 5;
            t0 = GET_VALUE(24); t1 = GET_VALUE(26);
            val |= (t0 < t1) << 6;
            t0 = GET_VALUE(28); t1 = GET_VALUE(30);
            val |= (t0 < t1) << 7;

            desc[j] = (uchar)val;
        }
        if(level)
        {
            kpt.pt *= factor;
        }
    }
}

void QuadTreeNode::DivideToNextLevel(QuadTreeNode *node)
{
    int dx = static_cast<int>(0.5 * (UR.x - UL.x));
    int dy = static_cast<int>(0.5 * (BR.y - UR.y));

    node[0].UL = UL;
    node[0].UR = node[1].UL = cv::Point2i(UL.x + dx, UL.y);
    node[0].BL = node[2].UL = cv::Point2i(UL.x, UL.y + dy);
    node[0].BR = node[1].BL = node[2].UR = node[3].UL = cv::Point2i(UL.x + dx, UL.y + dy);
    node[1].UR = UR;
    node[1].BR = node[3].UR = cv::Point2i(UR.x, UR.y + dy);
    node[2].BL = BL;
    node[2].BR = node[3].BL = cv::Point2i(BL.x + dx, BL.y);
    node[3].BR = BR;

    int cx = node[0].BR.x, cy = node[0].BR.y;
    for(auto &kp : keyPoints)
    {
        if(kp.pt.x > cx)
        {
            if(kp.pt.y > cy)
                node[3].keyPoints.push_back(kp);
            else
                node[1].keyPoints.push_back(kp);
        }
        else
        {
            if(kp.pt.y > cy)
                node[2].keyPoints.push_back(kp);
            else
                node[0].keyPoints.push_back(kp);
        }
    }
}
}