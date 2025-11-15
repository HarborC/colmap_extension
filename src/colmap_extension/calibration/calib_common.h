#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace calibration_utils {

// Camera model for pose estimation
struct CameraModel {
  cv::Mat K;    // 3x3, CV_64F
  cv::Mat dist; // 1xN, CV_64F

  bool valid() const {
    return !K.empty() && K.rows == 3 && K.cols == 3 && K.type() == CV_64F;
  }
};

// Rigid pose (T_cam_obj)
struct Pose {
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t = Eigen::Vector3d::Zero();

  Eigen::Matrix4d matrix() const {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = R;
    T.block<3, 1>(0, 3) = t;
    return T;
  }

  static Pose FromRvecTvec(const cv::Mat &rvec, const cv::Mat &tvec) {
    Pose p;
    cv::Mat R_cv;
    cv::Rodrigues(rvec, R_cv);
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c)
        p.R(r, c) = R_cv.at<double>(r, c);
      p.t(r) = tvec.at<double>(r, 0);
    }
    return p;
  }
};

using CalibFrameID = std::string;

struct CalibFrame {
  using Ptr = std::shared_ptr<CalibFrame>;
  CalibFrameID id;
  double timestamp = 0.0;
  cv::Mat image;
  std::vector<Detection> detections;
};

} // namespace calibration_utils
