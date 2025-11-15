#pragma once
#include "calib_rig_system.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ceres/ceres.h>
#include <vector>

namespace calibration_utils {

// Parameter blocks layout suggestions:
//  - Camera pose per frame: 7 params (qw,qx,qy,qz, tx,ty,tz) with quaternion
//  normalized via local parameterization.
//  - Board pose per board: 7 params.
//  - Intrinsics: fx, fy, cx, cy, k1, k2, p1, p2 (extendable). Distortion model:
//  Brown.
// Gauge fixing: fix first camera pose or one board pose.

struct IntrinsicsParams {
  double fx = 0, fy = 0, cx = 0, cy = 0;
  double k1 = 0, k2 = 0, p1 = 0, p2 = 0; // extendable
  double *data() { return &fx; }
  static constexpr int size() { return 8; }
};

// Reprojection residual for a single AprilGrid corner.
struct ReprojectionError {
  ReprojectionError(const Eigen::Vector3d &p_board, const Eigen::Vector2d &obs)
      : p_board_(p_board), obs_(obs) {}

  template <typename T>
  bool operator()(const T *const cam_pose, const T *const board_pose,
                  const T *const intr, T *residuals) const {
    // cam_pose: [qw qx qy qz tx ty tz] world->camera pose? We store
    // T_world_cam. board_pose: [qw qx qy qz tx ty tz] T_world_board. Transform
    // p_board (board frame) to world. Actually use separate to avoid confusion;
    // here we assume cam_pose and board_pose both as T_world_*.
    Eigen::Quaternion<T> q_w_cam(cam_pose[0], cam_pose[1], cam_pose[2],
                                 cam_pose[3]);
    Eigen::Matrix<T, 3, 1> t_w_cam(cam_pose[4], cam_pose[5], cam_pose[6]);
    Eigen::Quaternion<T> q_w_board(board_pose[0], board_pose[1], board_pose[2],
                                   board_pose[3]);
    Eigen::Matrix<T, 3, 1> t_w_board(board_pose[4], board_pose[5],
                                     board_pose[6]);

    Eigen::Matrix<T, 3, 1> p_b(T(p_board_.x()), T(p_board_.y()),
                               T(p_board_.z()));
    Eigen::Matrix<T, 3, 1> p_w = q_w_board * p_b + t_w_board;
    // Camera coordinates: p_c = R_cw * (p_w - t_w_cam) with R_cw =
    // q_w_cam.conjugate()
    Eigen::Quaternion<T> q_c_w = q_w_cam.conjugate();
    Eigen::Matrix<T, 3, 1> p_c = q_c_w * (p_w - t_w_cam);

    T x = p_c.x() / p_c.z();
    T y = p_c.y() / p_c.z();

    // Distortion
    T r2 = x * x + y * y;
    T k1 = intr[4];
    T k2 = intr[5];
    T p1 = intr[6];
    T p2 = intr[7];
    T radial = T(1) + k1 * r2 + k2 * r2 * r2;
    T x_dist = x * radial + T(2) * p1 * x * y + p2 * (r2 + T(2) * x * x);
    T y_dist = y * radial + p1 * (r2 + T(2) * y * y) + T(2) * p2 * x * y;

    T fx = intr[0];
    T fy = intr[1];
    T cx = intr[2];
    T cy = intr[3];

    T u = fx * x_dist + cx;
    T v = fy * y_dist + cy;

    residuals[0] = u - T(obs_.x());
    residuals[1] = v - T(obs_.y());
    return true;
  }

  static ceres::CostFunction *Create(const Eigen::Vector3d &p_board,
                                     const Eigen::Vector2d &obs) {
    return new ceres::AutoDiffCostFunction<ReprojectionError, 2, 7, 7,
                                           IntrinsicsParams::size()>(
        new ReprojectionError(p_board, obs));
  }

  Eigen::Vector3d p_board_;
  Eigen::Vector2d obs_;
};

class GlobalCalibrator {
public:
  GlobalCalibrator(CalibRigSystem &rig) : rig_(rig) {}

  struct Result {
    bool success = false;
    int iterations = 0;
    double final_cost = 0.0;
  };

  Result
  run(IntrinsicsParams &intrinsics,
      std::vector<std::array<double, 7>> &camera_poses, // T_world_cam per frame
      std::vector<std::array<double, 7>>
          &board_poses); // T_world_board per board

private:
  CalibRigSystem &rig_;
};

} // namespace calibration_utils
