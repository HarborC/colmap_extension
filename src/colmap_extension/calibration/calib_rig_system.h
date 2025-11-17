#pragma once

#include "colmap/scene/reconstruction.h"
#include "colmap/estimators/pose.h"

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/opencv.hpp>

#include "calibration_utils/calib_board_factory.h"

namespace calibration_utils {

struct CalibFrame {
  using Ptr = std::shared_ptr<CalibFrame>;
  colmap::image_t id;
  std::string image_path;
  std::vector<CalibDetection::Ptr> detections;
};

// The rig aggregates multiple board types and runs detection per image.
class CalibRigSystem {
public:
  using Ptr = std::shared_ptr<CalibRigSystem>;
  CalibRigSystem() = default;
  CalibRigSystem(const std::string &json_path);

  // Detect all configured boards in an image (stateless helper)
  bool addFrame(const cv::Mat &image, const colmap::Image &colmap_image) {
    CalibFrame::Ptr frame(new CalibFrame());
    frame->id = colmap_image.ImageId();
    frame->image_path = colmap_image.Name();
    for (const auto &bd : boards_) {
      CalibBoard::Ptr b = bd.second;
      CalibDetection::Ptr d = b->detect(image);
      if (d) {
        frame->detections.emplace_back(d);
        colmap::Rigid3d cam_from_world;
        colmap::AbsolutePoseEstimationOptions options;
        colmap::Camera &camera = *colmap_image.CameraPtr();
        size_t num_inliers;
        std::vector<char> inlier_mask;
        if (colmap::EstimateAbsolutePose(options, d->corners2d_, d->corners3d_, &cam_from_world, &camera, &num_inliers, &inlier_mask)) {
          Eigen::Matrix4d pose_matrix = Eigen::Matrix4d::Identity();
          pose_matrix.block<3,3>(0,0) = cam_from_world.rotation.toRotationMatrix();
          pose_matrix.block<3,1>(0,3) = cam_from_world.translation;
          d->cam_from_world_ = std::make_shared<Eigen::Matrix4d>(pose_matrix);
          std::cout << "[INFO] Estimated pose for board " << b->id() << " with " << num_inliers << " inliers." << std::endl;  
          
          if (rig_origin_board_id_ == "") {
            rig_origin_board_id_ = b->id();
            board_poses_[b->id()] = colmap::Rigid3d(Eigen::Quaterniond::Identity(),
                                                    Eigen::Vector3d(0, 0, 0));
            std::cout << "[INFO] Set rig origin to board: " << rig_origin_board_id_ << std::endl;
          } 

          // TODO: Compute board pose in rig frame if rig origin is set.
        }
      }
    }
    if (!frame->detections.empty()) {
      frames_.emplace(frame->id, frame);
      return true;
    }
    return false;
  }

  const std::unordered_map<colmap::image_t, CalibFrame::Ptr> &frames() const {
    return frames_;
  }
  
  const std::unordered_map<CalibBoard::ID, CalibBoard::Ptr> &boards() const {
    return boards_;
  }

  // Global optimization (Bundle Adjustment) entry (implemented elsewhere)
  // Optimizes: camera poses per frame, board poses, shared intrinsics.
  // Returns true if convergence succeeded.
  // bool solveGlobalBA();

  std::unordered_map<CalibBoard::ID, colmap::Rigid3d> board_poses_;
  CalibBoard::ID rig_origin_board_id_ = "";

private:
  std::unordered_map<CalibBoard::ID, CalibBoard::Ptr> boards_;
  std::unordered_map<colmap::image_t, CalibFrame::Ptr> frames_;
  
};

} // namespace calibration_utils