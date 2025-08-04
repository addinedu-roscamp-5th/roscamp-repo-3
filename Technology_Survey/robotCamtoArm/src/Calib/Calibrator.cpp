#include "../../include/Calib/Calibrator.hpp"

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

	if (!GetChessboardPoints(imgs, objectPoints, imagePoints, board, resolution, subpix)) return rms;

	return CameraCalibration(objectPoints, imagePoints, this->cameraMatrix_, this->distCoeffs_, board);

}

double Calibrator::CameraCalibration(
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

bool Calibrator::GetChessboardPoints(
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
	fs["rvec"] >> rvec_;

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

	for(int i = 0; i < size ; ++i)
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
	