#include "calib_global_solver.h"
#include <ceres/ceres.h>
#include <iostream>

namespace calibration_utils {

namespace {
struct QuaternionParameterization : public ceres::LocalParameterization {
  bool Plus(const double *x, const double *delta,
            double *x_plus_delta) const override {
    Eigen::Map<const Eigen::Quaterniond> q(x);
    Eigen::Vector3d omega(delta[0], delta[1], delta[2]);
    double theta = omega.norm();
    Eigen::Quaterniond dq;
    if (theta < 1e-12) {
      dq = Eigen::Quaterniond(1, 0, 0, 0);
    } else {
      double half = 0.5 * theta;
      double sin_half = sin(half);
      dq = Eigen::Quaterniond(cos(half), sin_half * omega.x() / theta,
                              sin_half * omega.y() / theta,
                              sin_half * omega.z() / theta);
    }
    Eigen::Quaterniond q_new = dq * q;
    q_new.normalize();
    x_plus_delta[0] = q_new.w();
    x_plus_delta[1] = q_new.x();
    x_plus_delta[2] = q_new.y();
    x_plus_delta[3] = q_new.z();
    // Translation part passes through (stored right after quaternion in our
    // 7-array)
    x_plus_delta[4] = x[4] + delta[3];
    x_plus_delta[5] = x[5] + delta[4];
    x_plus_delta[6] = x[6] + delta[5];
    return true;
  }
  bool ComputeJacobian(const double *x, double *jacobian) const override {
    // 7x6 matrix: d(q,t)/d(delta). For simplicity use identity approximation
    // for translation, leave quaternion part as identity-like.
    Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> J(jacobian);
    J.setZero();
    // Derivative wrt rotation increment approximated
    J(0, 0) = 1;
    J(1, 1) = 1;
    J(2, 2) = 1; // simplistic; for small-angle ok
    J(4, 3) = 1;
    J(5, 4) = 1;
    J(6, 5) = 1;
    return true;
  }
  int GlobalSize() const override { return 7; }
  int LocalSize() const override { return 6; }
};
} // namespace

GlobalCalibrator::Result
GlobalCalibrator::run(IntrinsicsParams &intr,
                      std::vector<std::array<double, 7>> &cam_poses,
                      std::vector<std::array<double, 7>> &board_poses) {
  Result res;
  ceres::Problem problem;

  // Local parameterizations for poses
  auto *quat_param = new QuaternionParameterization();
  for (auto &cp : cam_poses) {
    problem.AddParameterBlock(cp.data(), 7, quat_param);
  }
  for (auto &bp : board_poses) {
    problem.AddParameterBlock(bp.data(), 7, quat_param);
  }

  // Fix gauge: lock first camera pose (or alternatively first board pose) to
  // prevent drift.
  if (!cam_poses.empty()) {
    problem.SetParameterBlockConstant(cam_poses.front().data());
  }

  // Intrinsics parameter block (no special parameterization yet)
  problem.AddParameterBlock(intr.data(), IntrinsicsParams::size());

  // Build residuals
  const auto &frames = rig_.frames();
  const auto &boards = rig_.boards();
  // Build mapping from board id to index
  std::unordered_map<std::string, size_t> board_index;
  for (size_t i = 0; i < boards.size(); ++i)
    board_index[boards[i]->id()] = i;

  for (size_t f = 0; f < frames.size(); ++f) {
    for (const auto &det : frames[f]->detections) {
      auto it = board_index.find(det.board_id);
      if (it == board_index.end())
        continue;
      size_t bidx = it->second;
      // Each corner residual
      for (size_t k = 0; k < det.corners3d.size(); ++k) {
        const auto &p3 = det.corners3d[k];
        const auto &p2 = det.corners2d[k];
        Eigen::Vector3d p_board(p3.x, p3.y, p3.z);
        Eigen::Vector2d obs(p2.x, p2.y);
        ceres::CostFunction *cost = ReprojectionError::Create(p_board, obs);
        ceres::LossFunction *loss = new ceres::HuberLoss(1.0);
        problem.AddResidualBlock(cost, loss, cam_poses[f].data(),
                                 board_poses[bidx].data(), intr.data());
      }
    }
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::SPARSE_SCHUR;
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 50;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  res.success = summary.IsSolutionUsable();
  res.iterations = summary.iterations.size();
  res.final_cost = summary.final_cost;
  std::cout << summary.BriefReport() << std::endl;
  return res;
}

bool CalibRigSystem::solveGlobalBA() {
  // Prepare initial parameter vectors.
  if (frames_.empty() || boards_.empty())
    return false;

  // Initialize camera poses: identity quaternion + zero translation
  std::vector<std::array<double, 7>> cam_poses(frames_.size());
  for (auto &p : cam_poses) {
    p = {1, 0, 0, 0, 0, 0, 0};
  }

  // Initialize board poses
  std::vector<std::array<double, 7>> board_poses(boards_.size());
  for (auto &p : board_poses) {
    p = {1, 0, 0, 0, 0, 0, 0};
  }

  IntrinsicsParams
      intr; // Should set from current CameraModel prior if available.
  // For now we assume first frame's camera model is representative (not passed
  // here yet).

  GlobalCalibrator calibrator(*this);
  auto result = calibrator.run(intr, cam_poses, board_poses);
  return result.success;
}

} // namespace calibration_utils
