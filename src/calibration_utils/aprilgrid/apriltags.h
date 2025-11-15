#pragma once

#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <vector>

namespace calibration_utils {

struct ApriltagDetectorData;

class ApriltagDetector {
public:
  // Accepts number of tags and optional starting tag id (default 0)
  ApriltagDetector(int numTags);
  ApriltagDetector(int startId, int numTags);

  ~ApriltagDetector();

  void detectTags(const cv::Mat &img_raw, std::vector<Eigen::Vector2d> &corners,
                  std::vector<int> &ids, std::vector<double> &radii,
                  std::vector<Eigen::Vector2d> &corners_rejected,
                  std::vector<int> &ids_rejected,
                  std::vector<double> &radii_rejected);

private:
  ApriltagDetectorData *data;
};

} // namespace calibration_utils
