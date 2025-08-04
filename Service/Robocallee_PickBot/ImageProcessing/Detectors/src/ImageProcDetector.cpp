#include "ImageProcDetector.hpp"

using namespace OB;
using namespace cv;

ImageProcDetector::ImageProcDetector()
{

}

ImageProcDetector::~ImageProcDetector()
{
    
}

bool ImageProcDetector::initDetector()
{
    return true;
}

bool ImageProcDetector::FindObject(cv::Mat& image, std::vector<object2d>& objects)
{
   if (image.empty()) return false;
    if (!objects.empty()) return false;

    try
    {
        // 1. 전처리
        cv::Mat gray, thresh;
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        cv::GaussianBlur(gray, gray, cv::Size(5, 5), 0);
        cv::threshold(gray, thresh, 0, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);

        // 2. 윤곽선 검출
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(thresh, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        int index = 0; // 객체 인덱스

        for (const auto& contour : contours)
        {
            double peri = cv::arcLength(contour, true);
            std::vector<cv::Point> approx;
            cv::approxPolyDP(contour, approx, 0.02 * peri, true);

            if (approx.size() == 4 && cv::isContourConvex(approx))
            {
                double area = cv::contourArea(approx);
                if (area < 1000) continue; // 필터링

                // 3. 시계방향 정렬
                std::vector<cv::Point> ordered(4);
                cv::Point center(0, 0);
                for (const auto& pt : approx) center += pt;
                center *= 0.25f;

                for (const auto& pt : approx)
                {
                    if (pt.x < center.x && pt.y < center.y) ordered[0] = pt; // 좌상
                    else if (pt.x > center.x && pt.y < center.y) ordered[1] = pt; // 우상
                    else if (pt.x > center.x && pt.y > center.y) ordered[2] = pt; // 우하
                    else if (pt.x < center.x && pt.y > center.y) ordered[3] = pt; // 좌하
                }

                // 4. 객체 정보 구성
                object2d object;
                object.objectID = std::to_string(index); // ⬅️ 인덱스 기반 ID
                object.CenterPt = center;
                object.points = ordered;

                objects.push_back(object);
                index++;

                for (size_t i = 0; i < ordered.size(); ++i)
                {
                    cv::circle(image, ordered[i], 5, cv::Scalar(0, 0, 255), -1); // 빨간 점
                    cv::putText(image, std::to_string(i), ordered[i] + cv::Point(5, -5),
                                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 1);
                }

                // 객체 중심점도 시각화
                cv::circle(image, center, 4, cv::Scalar(0, 255, 0), -1); // 녹색 중심
                cv::putText(image, "ID: " + object.objectID, center + cv::Point(5, 15),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

                // cv::imshow("Detected Corners", image);
                // cv::waitKey(0);
            }
        }

        return !objects.empty();
    }
    catch (const cv::Exception& e)
    {
        std::cerr << "[OpenCV 예외] ArUcoMakerDetector::FindObject: " << e.what() << std::endl;
        return false;
    }
}

bool ImageProcDetector::EstimatePose(
    const cv::Mat& image,                       // 입력 이미지 (Grayscale or Color)
    const cv::Mat& cameraMatrix,                // 내부 파라미터 K
    const cv::Mat& distCoeffs,                  // 왜곡 계수 D
    const double targetsize,                    // mm
    std::vector<cv::Mat>& rvecs,                // 출력: 회전 벡터들
    std::vector<cv::Mat>& tvecs                 // 출력: 위치 벡터들
)
{
    return true;
}