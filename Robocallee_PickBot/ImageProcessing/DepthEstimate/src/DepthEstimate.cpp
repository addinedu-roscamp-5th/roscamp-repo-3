#include "DepthEstimate.hpp"

using namespace depth;
using namespace cv;
using namespace std;

DepthEstimate::DepthEstimate()
{

}

DepthEstimate::~DepthEstimate()
{

}

bool DepthEstimate::MonoDepthEstimate(
            const cv::Mat& left,
            const cv::Mat& right,
            const cv::Mat& K,
            const cv::Mat& D,
            cv::Mat& DepthMap)
{
    if(left.empty() || right.empty() || K.empty() || D.empty()) return false;

     // 1. ORB 특징점 추출 및 매칭
    Ptr<ORB> orb = ORB::create(2000);
    vector<KeyPoint> kp1, kp2;
    Mat desc1, desc2;
    orb->detectAndCompute(left, noArray(), kp1, desc1);
    orb->detectAndCompute(right, noArray(), kp2, desc2);

    BFMatcher matcher(NORM_HAMMING);
    vector<DMatch> matches;
    matcher.match(desc1, desc2, matches);

    // 2. 좋은 매칭만 필터링
    double max_dist = 0;
    for (auto& m : matches) max_dist = max(max_dist, (double)m.distance);
    vector<Point2f> pts1, pts2;
    for (auto& m : matches) {
        if (m.distance < 0.6 * max_dist) {
            pts1.push_back(kp1[m.queryIdx].pt);
            pts2.push_back(kp2[m.trainIdx].pt);
        }
    }

    if (pts1.size() < 8) {
        cerr << "매칭된 점이 너무 적습니다." << endl;
        return false;
    }

    // 3. Essential matrix 및 R, T 추정
    Mat E = findEssentialMat(pts1, pts2, K, RANSAC, 0.999, 1.0);
    Mat R, T;
    recoverPose(E, pts1, pts2, K, R, T);

    // 4. Stereo Rectification
    Mat R1, R2, P1, P2, Q;
    stereoRectify(K, D, K, D, left.size(), R, T, R1, R2, P1, P2, Q);

    // 5. 정렬된 이미지 생성
    Mat map1x, map1y, map2x, map2y;
    initUndistortRectifyMap(K, D, R1, P1, left.size(), CV_32FC1, map1x, map1y);
    initUndistortRectifyMap(K, D, R2, P2, right.size(), CV_32FC1, map2x, map2y);
    Mat img1r, img2r;
    remap(left, img1r, map1x, map1y, INTER_LINEAR);
    remap(right, img2r, map2x, map2y, INTER_LINEAR);

    // 6. Disparity 계산
    Ptr<StereoSGBM> stereo = StereoSGBM::create(
        0, 16 * 5, 5,
        8 * 3 * 5 * 5,
        32 * 3 * 5 * 5,
        1, 10, 100, 32, StereoSGBM::MODE_SGBM
    );
    Mat disp_raw, disp_f;
    stereo->compute(img1r, img2r, disp_raw);
    disp_raw.convertTo(disp_f, CV_32F, 1.0 / 16.0);  // 정규화된 disparity

    // 7. 3D 좌표로 변환
    Mat point_cloud;
    reprojectImageTo3D(disp_f, point_cloud, Q, true);  // true: handle missing values

    // 8. Z만 추출하여 Depth Map 구성
    vector<Mat> xyz;
    split(point_cloud, xyz);  // xyz[2] = Z 값
    Mat depth_map = xyz[2];   // 단위는 meter 또는 mm (Q 행렬에 따라 다름)

    // 출력
    DepthMap = depth_map;

    cout << DepthMap << endl;
    
    cv::Mat visualDepth;
    cv::Mat depthSafe = DepthMap.clone();

    for (int y = 0; y < depthSafe.rows; ++y) {
        for (int x = 0; x < depthSafe.cols; ++x) {
            float& val = depthSafe.at<float>(y, x);
            if (std::isinf(val) || val <= 0.0f)
                val = 0.0f;  // 또는 NaN, 또는 평균값 등으로 대체
        }
    }

    cv::normalize(depthSafe, visualDepth, 0, 255, cv::NORM_MINMAX);
    visualDepth.convertTo(visualDepth, CV_8UC1);
    cv::applyColorMap(visualDepth, visualDepth, cv::COLORMAP_JET);
    cv::imshow("Depth", visualDepth);

    cv::waitKey();

    return true;
}

