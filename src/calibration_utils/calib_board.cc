#include "calib_board.h"

#include <fstream>
#include <cereal/archives/binary.hpp>
#include <cereal/types/polymorphic.hpp>
#include "serialization_registration.h"

namespace calibration_utils {

CalibBoard::CalibBoard(CalibBoard::ID id, BoardType type)
    : id_(std::move(id)), type_(type) {}

const CalibBoard::ID &CalibBoard::id() const { return id_; }
BoardType CalibBoard::type() const { return type_; }

cv::Mat CalibDetection::showInImage(const cv::Mat &img,
                                    const std::vector<uint8_t> &color) const {
  if (color.size() != 3) {
    std::cerr << "[ERROR] Color must be a 3-element vector [R, G, B]."
              << std::endl;
    return cv::Mat();
  }

  if (img.empty()) {
    std::cerr << "[ERROR] Input image is empty." << std::endl;
    return cv::Mat();
  }

  cv::Mat show_img = img.clone();
  if (img.channels() == 1) {
    cv::cvtColor(show_img, show_img, cv::COLOR_GRAY2BGR);
  } else if (img.channels() != 3) {
    std::cerr
        << "[ERROR] Unsupported image format. Only 1 or 3 channels supported."
        << std::endl;
    return cv::Mat();
  }

  return _showInImage(show_img, color);
}

CalibDetection::Ptr loadDetection(const std::string& path) {
  std::ifstream is(path, std::ios::binary);
  if (!is) return nullptr;
  cereal::BinaryInputArchive ar(is);
  std::shared_ptr<CalibDetection> det;
  ar(det);
  return det;
}

bool saveDetection(const CalibDetection::Ptr &det, const std::string &path) {
  if (!det)
    return false;
  std::ofstream os(path, std::ios::binary);
  if (!os)
    return false;
  cereal::BinaryOutputArchive ar(os);
  ar(det);
  return true;
}


} // namespace calibration_utils