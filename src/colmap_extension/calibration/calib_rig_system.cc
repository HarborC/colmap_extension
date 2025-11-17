#include "calib_rig_system.h"

#include <fstream>
#include <iostream>

// Use rapidjson bundled with cereal thirdparty for JSON parsing
#include "../../thirdparty/cereal/external/rapidjson/document.h"
#include "../../thirdparty/cereal/external/rapidjson/istreamwrapper.h"

namespace calibration_utils {

CalibRigSystem::CalibRigSystem(const std::string &json_path) {
  using namespace rapidjson;
  std::ifstream ifs(json_path);
  if (!ifs.is_open()) {
    std::cerr << "Failed to open JSON file: " << json_path << std::endl;
    std::exit(1);
  }

  rapidjson::IStreamWrapper isw(ifs);
  rapidjson::Document doc;
  doc.ParseStream(isw);

  if (doc.HasParseError()) {
    std::cerr << "JSON parse error, offset = " << doc.GetErrorOffset()
              << std::endl;
    std::exit(1);
  }

  if (!doc.IsArray()) {
    std::cerr << "Root of JSON must be an array." << std::endl;
    std::exit(1);
  }

  for (auto &v : doc.GetArray()) {
    if (!v.IsObject()) {
      std::cerr << "Array element is not an object, skip." << std::endl;
      continue;
    }

    std::string id;
    if (v.HasMember("id") && v["id"].IsString()) {
      id = v["id"].GetString();
    } else {
      std::cerr << "Element missing string field 'id', skip." << std::endl;
      continue;
    }

    std::string config_path;
    if (v.HasMember("config_path") && v["config_path"].IsString()) {
      config_path = v["config_path"].GetString();
    } else {
      std::cerr << "Element missing string field 'config_path', skip."
                << std::endl;
      continue;
    }

    std::cout << "id = " << id << ", config_path = " << config_path
              << std::endl;
    auto b = CalibBoardFactory::FromConfig(config_path, id);
    if (b) {
      boards_.emplace(id, b);
      board_poses_.emplace(id, colmap::Rigid3d(Eigen::Quaterniond::Identity(),
                                               Eigen::Vector3d(0, 0, 0)));
    }
  }
}

} // namespace calibration_utils
