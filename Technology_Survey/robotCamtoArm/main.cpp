#pragma once
#include <iostream>
#include <memory>
#include <string>
#include <sstream>
#include <regex>

#include "include/Logger.hpp"
#include "include/Detectors/ArUcoMakerDetector.hpp"
#include "include/Calib/Calibrator.hpp"
#include "include/Robot/RobotArm.hpp"
#include "include/Robot/openRobotArm.hpp"
#include "include/yamlReader.hpp"

using namespace std;
using namespace cv;
using namespace RA;
using namespace OB;
using namespace Calib;
using namespace ARM;
using namespace yaml;

#define JETCOBOT "/dev/ttyJETCOBOT"
#define BAUDRATE 1000000

#define Calib_File "Calib/CameraCalib.XML"
std::vector<double> parse_numbers_from_filename(const std::string& filename)
{
    std::vector<double> result;

    // 정규표현식으로 [] 안의 숫자 부분만 추출
    std::regex regex_pattern(R"(\[([^\]]+)\])");
    std::smatch match;

    if (std::regex_search(filename, match, regex_pattern))
    {
        std::string numbers_str = match[1];  // 대괄호 내부 문자열

        std::stringstream ss(numbers_str);
        std::string token;

        while (std::getline(ss, token, ','))
        {
            try
            {
                double value = std::stod(token);
                result.push_back(value);
            } catch (std::invalid_argument& e)
            {
                std::cerr << "변환 실패: " << token << std::endl;
            }
        }
    }

    return result;
}


int main()
{
    //Target 관련 변수
    const cv::Size boardSize(9, 6);
	const float resolution = 20.0f; //mm
    const double Depth = 160.0; //mm
    const double targetsize = 50.0; //mm
    
    std::string folder_path = "Images/*.png"; // .jpg, .bmp
	std::vector<cv::String> image_files;
	std::vector<cv::Mat> images;

    // Calibrator::s_ptr calib = std::make_shared<Calibrator>(Calib_File);
    Calibrator::s_ptr calib = std::make_shared<Calibrator>();
    cv::glob(folder_path, image_files);

    yamlReader::s_ptr yaml = make_shared<yamlReader>();
    std::vector<CalibData> calibdata;
    yaml->load_from_yaml("capture_data.yaml",calibdata);

    RobotArm::s_ptr Arm = std::make_shared<RobotArm>();

    for (const auto& file: image_files) 
	{
        Mat img = cv::imread(file);    
        images.push_back(img);
	}
    
    double err = calib->CameraCalibration(images,boardSize,resolution);
    
    double reerr = 0.0;
    for(auto& cal : calibdata)
	{
        Mat FK;
        Arm->compute_forward_kinematics(cal.radians, ARM::mycobot280_dh_params, FK);
        cal.FK = FK.clone();
        
        calib->GetChessboardPoints(cal.Image, cal.objectpt, cal.imagept, boardSize, resolution); //m 단위
        calib->solvePnP(cal.objectpt, cal.imagept, calib->GetCameraMatrix(), calib->GetdistCoeffs(), cal.rvec, cal.tvec); //m 단위
        reerr += calib->ReprojectionError(cal.objectpt, cal.imagept, cal.rvec, cal.tvec, calib->GetCameraMatrix(), calib->GetdistCoeffs());
	}

    reerr /= calibdata.size();

    cout << reerr << endl;

    Calib::vMat g2b_rvec,g2b_tvec;
    Calib::vMat t2c_rvec, t2c_tvec;
    Mat c2g_rvec, c2g_tvec, c2g;

    for(auto p : calibdata)
    {
        Mat rvec , tvec , invFK;
        // Arm->homogeneousInverse(p.FK, invFK);
        Arm->decomposeTransform(p.FK, rvec, tvec);
        g2b_rvec.push_back(rvec);
        g2b_tvec.push_back(tvec);

        t2c_rvec.push_back(p.rvec);
        t2c_tvec.push_back(p.tvec);
    }

    cv::calibrateHandEye(g2b_rvec,g2b_tvec,t2c_rvec,t2c_tvec, c2g_rvec, c2g_tvec, CALIB_HAND_EYE_PARK);
    Arm->composeTransform(c2g_rvec, c2g_tvec, c2g);

    cout << c2g << endl;

    /////////////////////////////// Robot /////////////////////////////// 
    openRobotArm::s_ptr RobotArm = make_shared<openRobotArm>();
    RobotArm->Connet(JETCOBOT,BAUDRATE);

    vec<float> Angles;
    vec<double> dAngles;
    vec<float> croodi;
    RobotArm->GetAngles(Angles);
    RobotArm->GetCoordi(croodi);

    dAngles.assign(Angles.begin(), Angles.end());

    vec<double> radians;
    Arm->degreesToRadians(dAngles,radians);
    Mat MycobotFK , MycobotinvFK;
    Arm->compute_forward_kinematics(radians, ARM::mycobot280_dh_params, MycobotFK);
    Arm->homogeneousInverse(MycobotFK, MycobotinvFK);

    shared_ptr<cv::VideoCapture> cap = make_shared<cv::VideoCapture>();
    cap->open(0);

    Mat img;
    cap->read(img);
    if(img.empty()) return -1;

    imwrite("saveimg.bmp",img);
    
    objectDetector::s_ptr detector = std::make_shared<ArUcoMakerDetector>();
    detector->initDetector();

    vMat Trvec , Ttvec; 
    detector->EstimatePose(img, calib->GetCameraMatrix(), calib->GetdistCoeffs(), targetsize, Trvec, Ttvec);
    
    cout << Ttvec[0] << endl;

    Mat Homo;
    Arm->toHomogeneous(Ttvec[0] , Homo);
    
    Mat Gripper = c2g * Homo;
    // Gripper.rowRange(0,3) *= 1000.0;
    Mat base = MycobotFK * Gripper;

    cout << Gripper << endl;
    cout << base << endl;

    float x = base.at<double>(0, 0);
    float y = base.at<double>(1, 0);
    float z = base.at<double>(2, 0);

    vec<float> tmep;
    RobotArm->GetCoordi(tmep);

    //vec<float> Send = { x, y, z, 0.0, 0.0, 0.0 };
    vec<float> Send;
    RobotArm->GetCoordi(Send);
    Send[0] = x;
    Send[1] = y;
    Send[2] = z;
    RobotArm->SetCoordi(Send , 10);

    return 0;
}