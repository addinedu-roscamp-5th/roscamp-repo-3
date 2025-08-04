#pragma once
#include <memory>
#include <vector>
#include <string>
#include "opencv2/opencv.hpp"

#define ERROR -1;

namespace Calib
{
	template<typename T>
	using vec = std::vector<T>;

	template<typename T>
	using vvec = std::vector<std::vector<T>>;

	using vvp3f = vvec<cv::Point3f>;
	using vvp3d = vvec<cv::Point3d>;

	using vvp2f = vvec<cv::Point2f>;
	using vvp2d = vvec<cv::Point2d>;

	using vp3f = vec<cv::Point3f>;
	using vp3d = vec<cv::Point3d>;

	using vp2f = vec<cv::Point2f>;
	using vp2d = vec<cv::Point2d>;

	using vMat = vec<cv::Mat>;

	class Calibrator
	{
	private:
		cv::Mat					cameraMatrix_;
		cv::Mat					distCoeffs_;
		cv::Mat					rvec_;
		cv::Mat					tvec_;
		
	public:
		using s_ptr = std::shared_ptr<Calibrator>;

		Calibrator();
		Calibrator(const std::string filePath);
		~Calibrator();

		const cv::Mat GetCameraMatrix() const { return cameraMatrix_; }
		const cv::Mat GetdistCoeffs() const { return distCoeffs_; }

		double CameraCalibration(
			vMat& imgs,
			const cv::Size board,
			const float resolution,
			const bool subpix = true
		);

		double CameraCalibration(
			vvp3f& objectPoints,
			vvp2f& imagePoints,
			cv::Mat& cameraMatrix,
			cv::Mat& distCoeffs,
            cv::Size size
		);

		double CameraCalibration(
			vp3f& objectPoints,
			vp2f& imagePoints,
			cv::Mat& cameraMatrix,
			cv::Mat& distCoeffs,
            cv::Size size
		);

		bool GetChessboardPoints(
            vMat& imgs,
            vvp3f& objectPoints,
            vvp2f& imagePoints,
            const cv::Size board,
            const float resolution,
            const bool subpix = true
        );

		bool GetChessboardPoints(
			cv::Mat& imgs,
			vp3f& objectPoints,
			vp2f& imagePoints,
			const cv::Size board,
			const float resolution,
			const bool subpix = true
		);

		double ReprojectionError(
			const vp3f& objectPoints,
    		const vp2f& imagePoints,
    		const cv::Mat& rvec,
    		const cv::Mat& tvec,
    		const cv::Mat& cameraMatrix,
    		const cv::Mat& distCoeffs
		);

		bool saveCameraParams(const std::string& filename);

		bool loadCameraParams(const std::string& filename);

		bool HomogeneousTransform(cv::Point point2d , cv::Point3d point3d);

		bool MultisolvePnP(
			const vvp3f& objects,
			const vvp2f& images,
			const cv::Mat& cameraMatrix,
			const cv::Mat& distCoeffs,
			vMat& rvecs,
			vMat& tvecs
		);

		bool solvePnP(
			const vp3f& objects,
			const vp2f& images,
			const cv::Mat& cameraMatrix,
			const cv::Mat& distCoeffs,
			cv::Mat& rvec,
			cv::Mat& tvec
		);
	};
};