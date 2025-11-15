#include "calib_board_factory.h"

#include <algorithm>
#include <cctype>
#include <iostream>
#include <memory>
#include <string>

#include <opencv2/opencv.hpp>

#include "aprilgrid/aprilgrid.h"

namespace calibration_utils {

CalibBoard::Ptr CalibBoardFactory::FromConfig(const std::string &config_path,
                                              const CalibBoard::ID &id) {
  cv::FileStorage fs(config_path, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    std::cerr << "CalibBoardFactory: failed to open config: " << config_path
              << std::endl;
    return nullptr;
  }

  // Determine type
  if (fs["target_type"].empty()) {
    std::cerr << "CalibBoardFactory: missing target_type in config: "
              << config_path << std::endl;
    return nullptr;
  }

  std::string target_type;
  fs["target_type"] >> target_type;

  if (target_type == "aprilgrid") {
    return std::make_shared<AprilGridBoard>(id, config_path);
  }

  std::cerr << "CalibBoardFactory: unsupported or unknown board type: '"
            << target_type << "' in config: " << config_path << std::endl;
  return nullptr;
}

} // namespace calibration_utils
