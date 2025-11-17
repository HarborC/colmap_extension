// SPDX: internal
#pragma once

#include <memory>
#include <string>
#include <utility>

#include <Eigen/Core>
#include <opencv2/opencv.hpp>

// cereal
#include <cereal/cereal.hpp>
#include <cereal/access.hpp>
#include <cereal/types/string.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/memory.hpp>

#include "serialization_registration.h"

namespace calibration_utils {

// Only AprilGrid board type for now.
enum class BoardType { AprilGrid };

class CalibBoard;

class CalibDetection {
public:
  typedef std::shared_ptr<CalibDetection> Ptr;
  CalibDetection() = default;
  virtual ~CalibDetection() = default;

public:
  std::shared_ptr<const CalibBoard> board_; // board type
  std::vector<Eigen::Vector2d> corners2d_;
  std::vector<Eigen::Vector3d> corners3d_;
  std::shared_ptr<Eigen::Matrix4d> cam_from_world_ = nullptr; // estimated pose (cam_from_board)

  cv::Mat showInImage(const cv::Mat &img,
                      const std::vector<unsigned char> &color = {0, 255,
                                                                 0}) const;

  virtual cv::Mat
  _showInImage(cv::Mat &img, const std::vector<unsigned char> &color) const = 0;

private:
  friend class cereal::access;
  template <class Archive>
  void save(Archive &ar) const {
    ar(board_, corners2d_, corners3d_);
  }
  template <class Archive>
  void load(Archive &ar) {
    ar(board_, corners2d_, corners3d_);
  }
};

// Abstract calibration board interface
class CalibBoard {
public:
  using ID = std::string;
  using Ptr = std::shared_ptr<CalibBoard>;
  CalibBoard() = default;
  CalibBoard(ID id, BoardType type);
  virtual ~CalibBoard() = default;

  const ID &id() const;
  BoardType type() const;

  // Try to detect this board in an image and estimate pose.
  // Returns true if detected and pose estimated.
  virtual CalibDetection::Ptr detect(const cv::Mat &image) const = 0;

protected:
  ID id_;
  BoardType type_;

private:
  friend class cereal::access;
  template <class Archive>
  void save(Archive &ar) const {
    int type_int = static_cast<int>(type_);
    ar(id_, type_int);
  }
  template <class Archive>
  void load(Archive &ar) {
    int type_int;
    ar(id_, type_int);
    type_ = static_cast<BoardType>(type_int);
  }
};

bool saveDetection(const CalibDetection::Ptr& det, const std::string& path);
CalibDetection::Ptr loadDetection(const std::string& path);

} // namespace calibration_utils

// ------- cereal helpers for Eigen types (Vector2d/3d/4d) -------
namespace cereal {
template <class Archive>
inline void save(Archive &ar, const Eigen::Vector2d &v) {
  double x = v.x(), y = v.y();
  ar(x, y);
}
template <class Archive>
inline void load(Archive &ar, Eigen::Vector2d &v) {
  double x, y;
  ar(x, y);
  v.x() = x; v.y() = y;
}

template <class Archive>
inline void save(Archive &ar, const Eigen::Vector3d &v) {
  double x = v.x(), y = v.y(), z = v.z();
  ar(x, y, z);
}
template <class Archive>
inline void load(Archive &ar, Eigen::Vector3d &v) {
  double x, y, z;
  ar(x, y, z);
  v.x() = x; v.y() = y; v.z() = z;
}

template <class Archive>
inline void save(Archive &ar, const Eigen::Vector4d &v) {
  double x = v.x(), y = v.y(), z = v.z(), w = v.w();
  ar(x, y, z, w);
}
template <class Archive>
inline void load(Archive &ar, Eigen::Vector4d &v) {
  double x, y, z, w;
  ar(x, y, z, w);
  v.x() = x; v.y() = y; v.z() = z; v.w() = w;
}
} // namespace cereal

