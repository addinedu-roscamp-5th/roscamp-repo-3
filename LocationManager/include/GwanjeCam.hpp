#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


#include <fcntl.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <iostream>
#include <vector>


#define WIDTH 1920
#define HEIGHT 1080


class GwanjeCam
{
public:

    GwanjeCam();
    ~GwanjeCam();

    void monitor();
    rclcpp::Node::SharedPtr get_node() { return pub_node; }



private:
    int dev_num = 0;
    // int dev_num = 2;

    int sock;
    uint32_t sock_len;
    std::vector<uchar> sock_buf;

    cv::VideoCapture cap;
    cv::Mat cameraMatrix, distCoeffs;
    cv::Mat R_affine, T_affine;

    cv::aruco::ArucoDetector detector;
    const float markerLength = 50.0f;
    std::vector<cv::Point3f> objPoints;
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners, rejected;

    // ROS2 노드 멤버는 보통 shared_ptr로 선언
    rclcpp::Node::SharedPtr pub_node;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr my_publisher;

    void initCamera();
    void initSocket();
    void initROS2();


};
