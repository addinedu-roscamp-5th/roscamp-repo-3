#include "../include/yamlReader.hpp"

using namespace yaml;

yamlReader::yamlReader()
{

}
yamlReader::~yamlReader()
{

}

bool yamlReader::load_from_yaml(const std::string& filepath, std::vector<CalibData>& data)
{
    root_ = YAML::LoadFile(filepath);
    const YAML::Node& captures = root_["captures"];
    if (!captures || !captures.IsSequence())
    {
        std::cerr << "YAML 파일에 'captures' 시퀀스가 없습니다\n";
        return false;
    }

    for (const auto& node : captures)
    {
        CalibData calibData;
        std::string path = node["image"].as<std::string>(); 
        calibData.Image = cv::imread(path);
        calibData.angles = node["angles"].as<std::vector<double>>();
        calibData.radians = node["radians"].as<std::vector<double>>();
        calibData.coords = node["coords"].as<std::vector<double>>();
        data.push_back(calibData);
    }

    return true;
}