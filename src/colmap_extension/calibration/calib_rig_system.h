// A lightweight, extensible multi-board calibration rig interface.
// Supports multiple board types (Chessboard, ArUco) and returns per-image
// detections.

#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "aprilgrid.h"
#include "apriltags.h"
#include "calib_board.h"
#include "calib_common.h"
#include <opencv2/opencv.hpp>

namespace calibration_utils {

// The rig aggregates multiple board types and runs detection per image.
class CalibRigSystem {
public:
  using Ptr = std::shared_ptr<CalibRigSystem>;

  void addBoard(const CalibBoard::Ptr &board) { boards_.push_back(board); }

  // Detect all configured boards in an image (stateless helper)
  std::vector<CalibDetection::Ptr> detectBoards(const cv::Mat &image) const {
    std::vector<CalibDetection::Ptr> results;
    results.reserve(boards_.size());
    for (const auto &b : boards_) {
      CalibDetection::Ptr d = b->detect(image);
      if (d) {
        results.emplace_back(d);
      }
    }
    return results;
  }

  // Ingest a frame: runs detection, stores frame + detections
  CalibFrame::Ptr ingestFrame(const CalibFrameID &id, double ts,
                              const cv::Mat &image) {
    auto frame = std::make_shared<CalibFrame>();
    frame->id = id;
    frame->timestamp = ts;
    frame->image = image.clone();
    frame->detections = detectBoards(image);
    frames_.push_back(frame);
    return frame;
  }

  const std::vector<CalibFrame::Ptr> &frames() const { return frames_; }
  const std::vector<CalibBoard::Ptr> &boards() const { return boards_; }

  // Global optimization (Bundle Adjustment) entry (implemented elsewhere)
  // Optimizes: camera poses per frame, board poses, shared intrinsics.
  // Returns true if convergence succeeded.
  bool solveGlobalBA();

private:
  std::vector<CalibBoard::Ptr> boards_;
  std::vector<CalibFrame::Ptr> frames_;
};

} // namespace calibration_utils