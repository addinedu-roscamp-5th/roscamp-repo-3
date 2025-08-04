#include <memory>

#include "opencv2/opencv.hpp"

#define DH_PARAM 4
constexpr double DEG2RAD = M_PI / 180.0;

namespace ARM
{
    const std::vector<std::array<double, 4>> mycobot280_dh_params =
    {
        // {a, alpha, d, theta_offset}
        {   0.0,  M_PI_2, 131.22,     0.0 },       // joint 1
        {-110.4,     0.0,    0.0, -M_PI_2 },       // joint 2
        { -96.0,     0.0,    0.0,     0.0 },       // joint 3
        {   0.0,  M_PI_2,   63.4, -M_PI_2 },       // joint 4
        {   0.0, -M_PI_2,  75.05,  M_PI_2 },       // joint 5
        {   0.0,     0.0,   45.6,     0.0 }        // joint 6
    };

//     const std::vector<std::array<double, 4>> mycobot280_dh_params =
// {
//     // {a, alpha, d, theta_offset}  // 단위: [m, rad, m, rad]
//     {   0.0,     M_PI_2, 0.13122,     0.0     },   // joint 1
//     {-0.1104,       0.0,     0.0,    -M_PI_2  },   // joint 2
//     {-0.0960,       0.0,     0.0,     0.0     },   // joint 3
//     {   0.0,     M_PI_2, 0.0634,     -M_PI_2  },   // joint 4
//     {   0.0,    -M_PI_2, 0.07505,     M_PI_2  },   // joint 5
//     {   0.0,        0.0, 0.0456,      0.0     }    // joint 6
// };

    enum Params {a = 0, alpha, d, theta_offset};

    class RobotArm
    {
    private:

    public:
        using s_ptr = std::shared_ptr<RobotArm>;
        
        RobotArm();
        ~RobotArm();
        
        bool dh_to_transform(const double a, const  double alpha, const double d, const double theta , cv::Mat& T);
        
        bool compute_forward_kinematics(
            const std::vector<double>& joint_angles,
            const std::vector<std::array<double, DH_PARAM>> DH,
            cv::Mat& FK
        );

        bool decomposeTransform(const cv::Mat& T_se3 , cv::Mat& rvec , cv::Mat& tvec);

        bool composeTransform(const cv::Mat& rvec ,const cv::Mat& tvec,  cv::Mat& T_se3);

        bool degreesToRadians(const std::vector<double>& degrees ,std::vector<double> &radians);

        bool toHomogeneous(const cv::Mat& vecNx1, cv::Mat& vecNpx1);

        bool getTransformMatrixFromCoords(const std::vector<float>& coords , cv::Mat& T);

        bool homogeneousInverse(const cv::Mat& T, cv::Mat& invT);
    };
};