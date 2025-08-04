#pragma once

#include "Integrated.hpp"
#include <opencv2/opencv.hpp>

#include <yaml-cpp/yaml.h>

namespace OG
{
    typedef struct YAMLInfo
    {
        std::string         Image_path_;
        double              resolution_;
        std::vector<double> origin_;
        bool                negate_;
        double              occ_thresh_;
        double              free_thresh_;

    } YAMLFile;

    class OccupancyGrid
    {
    private:
        YAML::Node          Config_;
        YAMLInfo            yamlinfo_;
        
        cv::Mat             Map_;

    public:
        OccupancyGrid();
        ~OccupancyGrid();

        bool CaliGridToWorld(int mx, int my, double& wx, double& wy);

        bool CaliWorldToGrid(double wx, double wy, int& mx, int& my);

        bool LoadOccupancyGrid(const std::string path , const std::string filename);
        
        cv::Mat GetMap(){return Map_;}

        cv::Mat GetDeepCopyMap(){return Map_.clone();}
        
        YAMLInfo GetYAMLInfo(){return yamlinfo_;}
    };
}
