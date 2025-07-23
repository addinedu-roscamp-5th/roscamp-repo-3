
#include <iostream>
#include <sys/socket.h>  // socket 함수 위해 필요

#include "../include/GwanjeCam.hpp"

using namespace std;
using namespace cv;


GwanjeCam::GwanjeCam()
: detector(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_50), cv::aruco::DetectorParameters())
{
    // 3D 객체 점 초기화
    objPoints = {
        {-markerLength / 2,  markerLength / 2, 0},
        { markerLength / 2,  markerLength / 2, 0},
        { markerLength / 2, -markerLength / 2, 0},
        {-markerLength / 2, -markerLength / 2, 0}
    };

    initCamera();
    initSocket();
    initROS2();
}


GwanjeCam::~GwanjeCam()
{
    cap.release();
    close(sock);  // 소켓 닫기
    cv::destroyAllWindows();
}



void GwanjeCam::initCamera()
{
    cv::FileStorage fs1("calib_data.yaml", cv::FileStorage::READ);
    cv::FileStorage fs2("cam_to_world.yaml", cv::FileStorage::READ);

    if (!fs1.isOpened() || !fs2.isOpened()) {
        std::cerr << "Calibration file open 실패" << std::endl;
        exit(-1);
    }

    fs1["camera_matrix"] >> cameraMatrix;
    fs1["dist_coeffs"] >> distCoeffs;
    fs2["T_affine"] >> T_affine;
    fs2["R_affine"] >> R_affine;

    fs1.release();
    fs2.release();

    cap.open(dev_num);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);

    if (!cap.isOpened()) {
        std::cerr << "카메라를 열 수 없습니다." << std::endl;
        exit(-1);
    }
}

void GwanjeCam::initSocket()
{
    sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        std::cerr << "소켓 생성 실패" << std::endl;
        exit(-1);
    }

    sockaddr_in serv_addr{};
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(9000);
    inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr);

    if (connect(sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
        std::cerr << "서버 연결 실패" << std::endl;
        exit(-1);
    }
}

void GwanjeCam::initROS2()
{
    pub_node = rclcpp::Node::make_shared("pose_publisher");
    my_publisher = pub_node->create_publisher<std_msgs::msg::String>("detected_markers", 10);
    // rclcpp::spin(pub_node);  // 얘가 여기서 spin하고 있으면 안 됨. thread로 빼든가 해야함
 
} 


void GwanjeCam::monitor()
{
    
    // 카메라 초기화
    cap.open(dev_num);
    cap.set(CAP_PROP_FRAME_WIDTH, WIDTH);
    cap.set(CAP_PROP_FRAME_HEIGHT, HEIGHT);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    // 카메라 열기 확인
    if (!cap.isOpened()) {
        cerr << "카메라를 열 수 없습니다." << endl;
        exit(-1);
    }


    while (true)
    {
        Mat frame, undistorted;
        cap >> frame;
        if (frame.empty()) break;

        undistort(frame, undistorted, cameraMatrix, distCoeffs);


        detector.detectMarkers(undistorted, corners, ids, rejected);

        if (!ids.empty()) {
            for (size_t i = 0; i < ids.size(); ++i) {
                Vec3d rvec, tvec;
                solvePnP(objPoints, corners[i], cameraMatrix, distCoeffs, rvec, tvec);

                Mat rotMat;
                Rodrigues(rvec, rotMat);
                Mat trans = Mat(tvec).reshape(1, 3);
                Mat trans_applied = R_affine * trans + T_affine;
                Mat rot_table = R_affine * rotMat;

                double rx, ry, rz;
                ry = asin(-rot_table.at<double>(2, 0));
                if (cos(ry) > 1e-6) {
                    rx = atan2(rot_table.at<double>(2, 1), rot_table.at<double>(2, 2));
                    rz = atan2(rot_table.at<double>(1, 0), rot_table.at<double>(0, 0));
                } else {
                    rx = atan2(-rot_table.at<double>(1, 2), rot_table.at<double>(1, 1));
                    rz = 0;
                }

                double x = round(trans_applied.at<double>(0) / 1000.0 * 100) / 100.0;
                double y = round(trans_applied.at<double>(1) / 1000.0 * 100) / 100.0;

                stringstream trans_ss, rot_ss;
                trans_ss << "Trans: (" << x << ", " << y << ")m";
                rot_ss << "Rot: (" << round(rz * 10000) / 10000.0 << ")rad";

                putText(undistorted, trans_ss.str(), corners[i][0] + Point2f(-150, 10),
                        FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 2);
                putText(undistorted, rot_ss.str(), corners[i][0] + Point2f(-150, 50),
                        FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 2);

                for (int j = 0; j < 4; ++j)
                    circle(undistorted, corners[i][j], 4, Scalar(255, 0, 0), FILLED);
                }

        
        }


        // ROS2 메시지 생성. 좌표 및 Path 정보 추가 예정
        std_msgs::msg::String msg;
        msg.data = "Detected markers: " + std::to_string(ids.size());
        my_publisher->publish(msg);


        // 노트북 화면에 띄우기
        // imshow("World coords", undistorted);

        
        // 소켓 버전 
        cv::imencode(".jpg", undistorted, sock_buf);
        std::string encoded(sock_buf.begin(), sock_buf.end());

        sock_len = htonl(sock_buf.size());
        send(sock, &sock_len, sizeof(sock_len), 0);
        send(sock, sock_buf.data(), sock_buf.size(), 0);
        

        if (waitKey(1) == 'q') break;
  
  
    }


}


