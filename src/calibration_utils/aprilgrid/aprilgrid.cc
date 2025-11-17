#include "aprilgrid.h"
#include "apriltags.h"
#include <opencv2/opencv.hpp>

namespace calibration_utils {

cv::Mat
AprilGridBoardDetection::_showInImage(cv::Mat &img,
                                      const std::vector<uint8_t> &color) const {
  for (size_t i = 0; i < corners2d_.size(); ++i) {
    const auto &pt = corners2d_[i];
    cv::Point2f cv_pt(static_cast<float>(pt.x()), static_cast<float>(pt.y()));
    cv::circle(img, cv_pt, static_cast<int>(corner2d_radii_[i]),
               cv::Scalar(0, 255, 0), 1);
    cv::putText(img, std::to_string(corner2d_ids_[i]), cv_pt,
                cv::FONT_HERSHEY_SIMPLEX, 0.4,
                cv::Scalar(color[0], color[1], color[2]), 1);
  }
  return img;
}

AprilGridBoard::AprilGridBoard(const CalibBoard::ID &id, int cols, int rows,
                               double tag_size, double tag_spacing_ratio,
                               int start_tag_id)
    : CalibBoard(id, BoardType::AprilGrid), cols_(cols), rows_(rows),
      tagSize_(tag_size), spacing_ratio_(tag_spacing_ratio),
      start_id_(start_tag_id) {
  buildObjectPoints();
  detector_ = std::make_shared<ApriltagDetector>(start_id_, cols_ * rows_);
}

AprilGridBoard::AprilGridBoard(const CalibBoard::ID &id,
                               const std::string &config_path)
    : CalibBoard(id, BoardType::AprilGrid) {
  cv::FileStorage config_node(config_path, cv::FileStorage::READ);

  if (!config_node["tagCols"].empty())
    config_node["tagCols"] >> cols_;
  if (!config_node["tagRows"].empty())
    config_node["tagRows"] >> rows_;
  if (!config_node["tagSize"].empty())
    config_node["tagSize"] >> tagSize_;
  if (!config_node["tagSpacing"].empty())
    config_node["tagSpacing"] >> spacing_ratio_;
  if (!config_node["startID"].empty())
    config_node["startID"] >> start_id_;

  buildObjectPoints();
  detector_ = std::make_shared<ApriltagDetector>(start_id_, cols_ * rows_);
}

int AprilGridBoard::getTagCols() const { return cols_; }
int AprilGridBoard::getTagRows() const { return rows_; }
double AprilGridBoard::getTagSize() const { return tagSize_; }
double AprilGridBoard::getTagSpacing() const { return spacing_ratio_; }
int AprilGridBoard::getStartID() const { return start_id_; }

void AprilGridBoard::buildObjectPoints() {
  double x_corner_offsets[4] = {0, tagSize_, tagSize_, 0};
  double y_corner_offsets[4] = {0, 0, tagSize_, tagSize_};

  object_points_.resize(cols_ * rows_ * 4);

  for (int y = 0; y < rows_; y++) {
    for (int x = 0; x < cols_; x++) {
      int tag_id = cols_ * y + x;
      double x_offset = x * tagSize_ * (1 + spacing_ratio_);
      double y_offset = y * tagSize_ * (1 + spacing_ratio_);

      for (int i = 0; i < 4; i++) {
        int corner_id = (tag_id << 2) + i;
        Eigen::Vector4d &pos_3d = object_points_[corner_id];
        pos_3d[0] = x_offset + x_corner_offsets[i];
        pos_3d[2] = 0;
        pos_3d[1] = y_offset + y_corner_offsets[i];
        pos_3d[3] = 1;
      }
    }
  }
}

CalibDetection::Ptr AprilGridBoard::detect(const cv::Mat &image) const {
  if (image.empty()) {
    std::cerr << "[ERROR] Input image is empty." << std::endl;
    return nullptr;
  }

  std::vector<Eigen::Vector2d> corners, corners_rejected;
  std::vector<int> ids, ids_rejected;
  std::vector<double> radii, radii_rejected;
  detector_->detectTags(image, corners, ids, radii, corners_rejected,
                        ids_rejected, radii_rejected);

  if (corners.empty() || ids.empty()) {
    std::cerr << "[ERROR] No corners or IDs detected." << std::endl;
    return nullptr;
  }

  const int min_tags = 3;
  int tag_count = static_cast<int>(ids.size()) / 4;
  if (tag_count < min_tags) {
    std::cerr << "[ERROR] Not enough tags detected. Minimum required: " << min_tags << std::endl;
    return nullptr;
  }
  
  std::vector<Eigen::Vector3d> corners3d;
  for (size_t i = 0; i < ids.size(); ++i) {
    int corner_id = ids[i];
    if (corner_id < 0 || corner_id >= static_cast<int>(object_points_.size())) {
      continue;
    }
    const Eigen::Vector4d &pos_3d_h = object_points_[corner_id];
    corners3d.emplace_back(Eigen::Vector3d(pos_3d_h[0], pos_3d_h[1], pos_3d_h[2]));
  }

  auto detection = std::make_shared<AprilGridBoardDetection>();
  detection->board_ = std::dynamic_pointer_cast<const CalibBoard>(shared_from_this());
  detection->corners2d_ = std::move(corners);
  detection->corners3d_ = std::move(corners3d);
  detection->corner2d_ids_ = ids;
  detection->corner2d_radii_ = radii;

  std::cout << "[INFO] Detected " << tag_count << " tags with " << detection->corners2d_ .size() << " corners." << std::endl;

  return std::dynamic_pointer_cast<CalibDetection>(detection);
}

} // namespace calibration_utils
