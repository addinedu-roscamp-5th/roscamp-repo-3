#include "OccupancyGrid.hpp"

using namespace std;
using namespace cv;
using namespace YAML;
using namespace OG;
using namespace Commondefine;
using namespace Integrated;


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

Integrated::vec<Integrated::vec<bool>> OccupancyGrid::GetBoolMap()
{
    vector<vector<bool>> result;

    // 행렬이 비었는지 확인
    if(Map_.empty()) return {};

    int rows = Map_.rows / 10;
    int cols = Map_.cols / 10;

    result.resize(rows, vector<bool>(cols, false));

    for (int i = 0; i < rows; ++i)
    {
        for (int j = 0; j < cols; ++j)
        {
            // IMREAD_UNCHANGED 이므로 채널 수 확인 필요
            uchar value;
            if (Map_.channels() == 1)
                value = Map_.at<uchar>(i, j); // 그레이스케일 또는 8bit 단일 채널
            else if (Map_.channels() == 3)
            {
                // RGB 이미지일 경우, 각 채널의 합이 0보다 크면 true
                Vec3b pixel = Map_.at<Vec3b>(i, j);
                value = (pixel[0] + pixel[1] + pixel[2]) > 0 ? 1 : 0;
            }
            else
            {
                // 다른 포맷은 사용자 정의 처리 필요
                value = 0;
            }

            result[i][j] = (value == 0);
        }
    }

    return result;
}