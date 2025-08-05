#pragma once

#include "Commondefine.hpp"
#include "Integrated.hpp"
#include <opencv2/opencv.hpp>

#include <yaml-cpp/yaml.h>

namespace OG
{
    class OccupancyGrid
    {
    private:
        YAML::Node                        Config_;
        Commondefine::YAMLFile            yamlinfo_;
        
        cv::Mat                           Map_;
    public:
        OccupancyGrid();
        ~OccupancyGrid();

        bool CaliGridToWorld(int mx, int my, double& wx, double& wy);

        bool CaliWorldToGrid(double wx, double wy, int& mx, int& my);

        bool LoadOccupancyGrid(const std::string path , const std::string filename);

        Integrated::vec<Integrated::vec<bool>> GetBoolMap();
        
        cv::Mat GetMap(){return Map_;}

        cv::Mat GetDeepCopyMap(){return Map_.clone();}
        
        Commondefine::YAMLFile  GetYAMLInfo(){return yamlinfo_;}
    };
}
