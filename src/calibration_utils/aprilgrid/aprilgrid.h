#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <vector>

#include "calib_board.h"
#include "apriltags.h"

// cereal
#include <cereal/cereal.hpp>
#include <cereal/access.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/memory.hpp>

namespace calibration_utils {

class ApriltagDetector;

class AprilGridBoardDetection : public CalibDetection {
public:
  typedef std::shared_ptr<AprilGridBoardDetection> Ptr;
  AprilGridBoardDetection() = default;
  virtual ~AprilGridBoardDetection() = default;

  virtual cv::Mat
  _showInImage(cv::Mat &img,
               const std::vector<unsigned char> &color) const override;

public:
  std::vector<int> corner2d_ids_;
  std::vector<double> corner2d_radii_;

private:
  friend class cereal::access;
  template <class Archive>
  void save(Archive &ar) const {
    ar(cereal::base_class<CalibDetection>(this), corner2d_ids_, corner2d_radii_);
  }
  template <class Archive>
  void load(Archive &ar) {
    ar(cereal::base_class<CalibDetection>(this), corner2d_ids_, corner2d_radii_);
  }
};

class AprilGridBoard final : public CalibBoard,
                             public std::enable_shared_from_this<AprilGridBoard> {
public:
  AprilGridBoard() = default;
  AprilGridBoard(const CalibBoard::ID &id, int cols, int rows, double tag_size,
                 double tag_spacing_ratio, int start_tag_id = 0);
  AprilGridBoard(const CalibBoard::ID &id, const std::string &config_path);

  CalibDetection::Ptr detect(const cv::Mat &image) const override;

  int getTagCols() const;
  int getTagRows() const;
  double getTagSize() const;
  double getTagSpacing() const;
  int getStartID() const;

private:
  int cols_ = 0;             // number of apriltags (cols)
  int rows_ = 0;             // number of apriltags (rows)
  double tagSize_ = 0;       // size of apriltag, edge to edge [m]
  double spacing_ratio_ = 0; // ratio of space between tags to tagSize
  int start_id_ = 0;

  // 4 corners per tag in board frame
  std::vector<Eigen::Vector4d> object_points_;
  std::shared_ptr<ApriltagDetector> detector_;

  // Build (or rebuild) tag corner 3D positions (4 corners per tag) in board
  // frame (Z=0) Corner order: TL, TR, BR, BL
  void buildObjectPoints();

private:
  friend class cereal::access;
  template <class Archive>
  void save(Archive &ar) const {
    ar(cereal::base_class<CalibBoard>(this), cols_, rows_, tagSize_, spacing_ratio_, start_id_);
    // object_points_ and detector_ are re-built on load
  }
  template <class Archive>
  void load(Archive &ar) {
    ar(cereal::base_class<CalibBoard>(this), cols_, rows_, tagSize_, spacing_ratio_, start_id_);
    buildObjectPoints();
    // rebuild detector
    detector_ = std::make_shared<ApriltagDetector>(start_id_, cols_ * rows_);
  }
};

} // namespace calibration_utils