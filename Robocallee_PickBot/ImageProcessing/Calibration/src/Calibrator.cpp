#include "Calibrator.hpp"

using namespace std;
using namespace cv;
using namespace Calib;

Calibrator::Calibrator()
{
}

Calibrator::Calibrator(const string filePath)
{
	loadCameraParams(filePath);
}

Calibrator::~Calibrator()
{

}

double Calibrator::CameraCalibration(
	std::vector<cv::Mat>& imgs,
	const cv::Size board,
	const float resolution,
	const bool subpix
)
{
	double rms = -1;

	vvp3f objectPoints;
	vvp2f imagePoints;

	if (!GetMultiChessboardPoints(imgs, objectPoints, imagePoints, board, resolution, subpix)) return rms;

	return MultiCameraCalibration(objectPoints, imagePoints, this->cameraMatrix_, this->distCoeffs_, board);
}

double Calibrator::MultiCameraCalibration(
	vvp3f& objectPoints,
	vvp2f& imagePoints,
	cv::Mat& cameraMatrix,
	cv::Mat& distCoeffs,
	cv::Size size
)
{
	double rms = -1;

	if (objectPoints.empty() || imagePoints.empty() || objectPoints.size() != imagePoints.size()) return rms;

	rms = cv::calibrateCamera(objectPoints, imagePoints, size, cameraMatrix, distCoeffs, rvec_, tvec_);

	cameraMatrix_ = cameraMatrix.clone();
	distCoeffs = distCoeffs_.clone();
	
	return rms;
}

double Calibrator::CameraCalibration(
	vp3f& objectPoints,
	vp2f& imagePoints,
	cv::Mat& cameraMatrix,
	cv::Mat& distCoeffs,
	cv::Size size
)
{		
	double rms = -1;

	if (objectPoints.empty() || imagePoints.empty() || objectPoints.size() != imagePoints.size()) return rms;

	rms = cv::calibrateCamera(objectPoints, imagePoints, size, cameraMatrix, distCoeffs, rvec_, tvec_);

	cameraMatrix_ = cameraMatrix.clone();
	distCoeffs = distCoeffs_.clone();
	
	return rms;
}

bool Calibrator::GetMultiChessboardPoints(
	vMat& imgs,
	vvp3f& objectPoints,
	vvp2f& imagePoints,
	const cv::Size board,
	const float resolution,
	const bool subpix
)
{
	if (imgs.empty()) return false;

	if (!imagePoints.empty()) imagePoints.clear();

	if (!objectPoints.empty()) objectPoints.clear();

	vp3f objp;
	for (int i = 0; i < board.height; ++i)
	{
		for (int j = 0; j < board.width; ++j)
		{
			objp.emplace_back(j * resolution, i * resolution, 0);
		}
	}

	for (auto img : imgs)
	{
		if (img.empty()) continue;

		vp2f corners;
		if (!cv::findChessboardCorners(img, board, corners)) continue;

		if (subpix)
		{
			cv::Mat gray;
			cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
			try
			{
				cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001));
			}
			catch (const cv::Exception& e)
			{
        		std::cerr << "OpenCV 예외 발생!\n";
        		std::cerr << "오류 메시지: " << e.what() << std::endl;
    		}
			
		}

		objectPoints.push_back(objp);
		imagePoints.push_back(corners);
	}

	return true;
}

bool Calibrator::GetChessboardPoints(
	Mat& imgs,
	vp3f& objectPoints,
	vp2f& imagePoints,
	const cv::Size board,
	const float resolution,
	const bool subpix
)
{
	if (imgs.empty()) return false;

	if (!imagePoints.empty()) imagePoints.clear();

	if (!objectPoints.empty()) objectPoints.clear();

	vp3f objp;
	for (int i = 0; i < board.height; ++i)
	{
		for (int j = 0; j < board.width; ++j)
		{
			objp.emplace_back(j * resolution, i * resolution, 0);
		}
	}

	vp2f corners;
	if (!cv::findChessboardCorners(imgs, board, corners))

	if (subpix)
	{
		cv::Mat gray;
		cv::cvtColor(imgs, gray, cv::COLOR_BGR2GRAY);
		cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001));
	}

	objectPoints.assign(objp.begin(),objp.end());
	imagePoints.assign(corners.begin(),corners.end());

	return true;
}


bool Calibrator::saveCameraParams(const std::string& filename)
{
	cv::FileStorage fs(filename, cv::FileStorage::WRITE);
	if (!fs.isOpened()) return false;

	fs << "camera_matrix" << cameraMatrix_;
	fs << "distortion_coefficients" << distCoeffs_;
	fs << "rvec" << rvec_;
	fs << "rvec" << tvec_;
	fs << "cam2gripper" << cam2gripper_;

	fs.release();

	return true;
}

bool Calibrator::loadCameraParams(const std::string& filename)
{
	cv::FileStorage fs(filename, cv::FileStorage::READ);
	if (!fs.isOpened()) return false;

	fs["camera_matrix"] >> cameraMatrix_;
	fs["distortion_coefficients"] >> distCoeffs_;
	fs["rvec"] >> rvec_;
	fs["cam2gripper"] >> cam2gripper_;

	fs.release();

	return true;
}

bool Calibrator::MultisolvePnP(
		const vvp3f& objects,
		const vvp2f& images,
		const cv::Mat& cameraMatrix,
		const cv::Mat& distCoeffs,
		vector<cv::Mat>& rvecs,
		vector<cv::Mat>& tvecs
	)
{
	if(objects.empty() || images.empty()) return false;
	
	size_t size = objects.size();

	for(size_t i = 0; i < size; ++i)
	{
		Mat rvec , tvec;
		cv::solvePnP(objects[i], images[i], cameraMatrix, distCoeffs, rvec, tvec);

		rvecs.push_back(rvec);
		tvecs.push_back(tvec);
	}

	return true;
}

bool Calibrator::solvePnP(
		const vp3f& objects,
		const vp2f& images,
		const cv::Mat& cameraMatrix,
		const cv::Mat& distCoeffs,
		cv::Mat& rvec,
		cv::Mat& tvec
	)
{
	if(objects.empty() || images.empty()) return false;	
	
	cv::solvePnP(objects, images, cameraMatrix, distCoeffs, rvec, tvec);

	return true;
}

double Calibrator::ReprojectionError(
		const vp3f& objectPoints,
		const vp2f& imagePoints,
		const cv::Mat& rvec,
		const cv::Mat& tvec,
		const cv::Mat& cameraMatrix,
		const cv::Mat& distCoeffs
	)
{
	std::vector<cv::Point2f> projectedPoints;
	cv::projectPoints(objectPoints, rvec, tvec, cameraMatrix, distCoeffs, projectedPoints);

	double totalError = 0.0;
	for (size_t i = 0; i < imagePoints.size(); ++i)
	{
		double err = cv::norm(imagePoints[i] - projectedPoints[i]);
		totalError += err * err;  // 제곱합
	}

	double rms = std::sqrt(totalError / imagePoints.size());  // Root Mean Square Error
	
	return rms;
}

bool TransformCameraPose(
            std::vector<cv::Point>& point2d,        // mm
            std::vector<cv::Point3d>& point3d,      // mm
            const cv::Mat& cameraMatrix,
            const cv::Mat& distCoeffs,
            const double depth,
            const bool useUnDist = false
)
{
	if(point2d.empty() || cameraMatrix.empty() || distCoeffs.empty()) return false;
	
	point3d.clear();
	point3d.reserve(point2d.size());
	
	if(useUnDist)
	{
		std::vector<cv::Point2f> point2f(point2d.begin(), point2d.end());
		std::vector<cv::Point2f> undistorted2f;

		// 왜곡 보정 → 내부 파라미터까지 적용됨 → z=1 평면상의 좌표
		cv::undistortPoints(point2f, undistorted2f, cameraMatrix, distCoeffs);

		for (const auto& pt : undistorted2f)
		{
			// 동차 좌표계로 확장
			cv::Point3d vec3D(pt.x, pt.y, 1.0);

			// 단위 벡터로 정규화
			double norm = cv::norm(vec3D);
			cv::Point3d unitVec = vec3D * (1.0 / norm) * depth;
			
			point3d.emplace_back(unitVec); // 방향 성분만 있는 방향 벡터
		}
	}

	else
	{
		cv::Mat invk = cameraMatrix.inv();
		for (const auto& pt : point2d)
		{
			cv::Mat uv = (cv::Mat_<double>(3,1) << pt.x, pt.y, 1.0);
			cv::Mat dir = cameraMatrix.inv() * uv;
			
			double x = dir.at<double>(0, 0) / dir.at<double>(2, 0);
			double y = dir.at<double>(1, 0) / dir.at<double>(2, 0);

			cv::Point3d vec3D(x, y, 1.0);

			// 단위 벡터로 정규화
			double norm = cv::norm(vec3D);
			cv::Point3d unitVec = vec3D * (1.0 / norm) * depth;
			
			point3d.emplace_back(unitVec);  // 방향 성분만 있는 방향 벡터
		}
	}

	return true;
}