#include "OccupancyGrid.hpp"

using namespace std;
using namespace cv;
using namespace YAML;
using namespace OG;


OccupancyGrid::OccupancyGrid()
{

}

OccupancyGrid::~OccupancyGrid()
{
    
}

bool OccupancyGrid::CaliGridToWorld(int mx, int my, double& wx, double& wy)
{
    int hight = Map_.rows;
    wx = yamlinfo_.origin_[0] + (mx + 0.5) * yamlinfo_.resolution_;
    wy = yamlinfo_.origin_[1] + (hight - my - 1 + 0.5) * yamlinfo_.resolution_;

    return true;
}

bool OccupancyGrid::CaliWorldToGrid(double wx, double wy, int& mx, int& my)
{
    int hight = Map_.rows;
    mx = floor((wx - yamlinfo_.origin_[0]) / yamlinfo_.resolution_);
    my = floor((hight - wy - 1 - yamlinfo_.origin_[1]) / yamlinfo_.resolution_);

    return true;
}

bool OccupancyGrid::LoadOccupancyGrid(const std::string path , const std::string filename)
{
    try
    {
        string Yaml = path+ "/" + filename;
        Config_ = YAML::LoadFile(Yaml);

        yamlinfo_.Image_path_ = Config_["image"].as<std::string>();
        yamlinfo_.resolution_ = Config_["resolution"].as<double>();
        yamlinfo_.origin_ = Config_["origin"].as<std::vector<double>>();
        yamlinfo_.negate_ = Config_["negate"].as<int>();
        yamlinfo_.occ_thresh_ = Config_["occupied_thresh"].as<double>();
        yamlinfo_.free_thresh_ = Config_["free_thresh"].as<double>();

        string Image = path + "/" + yamlinfo_.Image_path_;

        Map_ = cv::imread(Image , IMREAD_UNCHANGED);
        if(Map_.empty())
        {
            std::cerr << "Failed to load map image." << std::endl;
            return false;
        }
    }
    catch (const YAML::BadFile& e)
    {
        std::cerr << "YAML file not found or failed to open: " << e.what() << std::endl;
        return false;
    }
    catch (const YAML::ParserException& e)
    {
        std::cerr << "YAML parse error: " << e.what() << std::endl;
        return false;
    }
    catch (const YAML::Exception& e)
    {
        std::cerr << "YAML unknown error: " << e.what() << std::endl;
        return false;
    }

    return true;
}

