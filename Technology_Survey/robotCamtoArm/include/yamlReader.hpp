#include <fstream>
#include <vector>
#include <string>
#include "opencv2/opencv.hpp"
#include <yaml-cpp/yaml.h>

namespace yaml
{
      typedef struct CalibData
    {
        cv::Mat Image;
        cv::Mat FK;
        cv::Mat rvec;
        cv::Mat tvec;
        std::vector<double> angles;
        std::vector<double> radians;
        std::vector<double> coords;
        std::vector<cv::Point3f> objectpt;
        std::vector<cv::Point2f> imagept;
    }CalibData;

    class yamlReader
    {
    private:
        YAML::Node          root_;

    public:
        using s_ptr = std::shared_ptr<yamlReader>;
        
        yamlReader();
        ~yamlReader();

        bool load_from_yaml(const std::string& filepath, std::vector<CalibData>& data);

    };
};
