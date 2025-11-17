#pragma once

#include "colmap/estimators/cost_functions.h"

namespace colmap_extension {

// Standard bundle adjustment cost function for variable
// camera pose, calibration, and point parameters.
template <typename CameraModel>
class CalibReprojErrorCostFunctor
    : public colmap::AutoDiffCostFunctor<
          CalibReprojErrorCostFunctor<CameraModel>, 2, 4, 3, 4, 3,
          CameraModel::num_params> {
public:
  explicit CalibReprojErrorCostFunctor(const Eigen::Vector2d &point2D,
                                       const Eigen::Vector3d &board3D)
      : observed_x_(point2D(0)), observed_y_(point2D(1)), board3D_(board3D) {}

  template <typename T>
  bool operator()(const T *const cam_from_world_rotation,
                  const T *const cam_from_world_translation,
                  const T *const board_rotation,
                  const T *const board_translation,
                  const T *const camera_params, T *residuals) const {
    Eigen::Matrix<T, 3, 1> board3D = board3D_.cast<T>();
    Eigen::Matrix<T, 3, 1> point3D =
        colmap::EigenQuaternionMap<T>(board_rotation) * board3D +
        colmap::EigenVector3Map<T>(board_translation);
    const Eigen::Matrix<T, 3, 1> point3D_in_cam =
        colmap::EigenQuaternionMap<T>(cam_from_world_rotation) * point3D +
        colmap::EigenVector3Map<T>(cam_from_world_translation);
    if (CameraModel::ImgFromCam(camera_params, point3D_in_cam[0],
                                point3D_in_cam[1], point3D_in_cam[2],
                                &residuals[0], &residuals[1])) {
      residuals[0] -= T(observed_x_);
      residuals[1] -= T(observed_y_);
    } else {
      residuals[0] = T(0);
      residuals[1] = T(0);
    }
    return true;
  }

private:
  const double observed_x_;
  const double observed_y_;
  Eigen::Vector3d board3D_;
};

// Bundle adjustment cost function for variable
// camera calibration and point parameters, and fixed camera pose.
template <typename CameraModel>
class CalibReprojErrorConstantPoseCostFunctor
    : public colmap::AutoDiffCostFunctor<
          CalibReprojErrorConstantPoseCostFunctor<CameraModel>, 2, 4, 3,
          CameraModel::num_params> {
public:
  CalibReprojErrorConstantPoseCostFunctor(const Eigen::Vector2d &point2D,
                                          const colmap::Rigid3d &cam_from_world,
                                          const Eigen::Vector3d &board3D)
      : cam_from_world_(cam_from_world), reproj_cost_(point2D, board3D) {}

  template <typename T>
  bool operator()(const T *const board_rotation,
                  const T *const board_translation,
                  const T *const camera_params, T *residuals) const {
    const Eigen::Quaternion<T> cam_from_world_rotation =
        cam_from_world_.rotation.cast<T>();
    const Eigen::Matrix<T, 3, 1> cam_from_world_translation =
        cam_from_world_.translation.cast<T>();
    return reproj_cost_(cam_from_world_rotation.coeffs().data(),
                        cam_from_world_translation.data(), board_rotation,
                        board_translation, camera_params, residuals);
  }

private:
  const colmap::Rigid3d cam_from_world_;
  const CalibReprojErrorCostFunctor<CameraModel> reproj_cost_;
};

// Rig bundle adjustment cost function for variable camera pose and calibration
// and point parameters. Different from the standard bundle adjustment function,
// this cost function is suitable for camera rigs with consistent relative poses
// of the cameras within the rig. The cost function first projects points into
// the local system of the camera rig and then into the local system of the
// camera within the rig.
template <typename CameraModel>
class CalibRigReprojErrorCostFunctor
    : public colmap::AutoDiffCostFunctor<
          CalibRigReprojErrorCostFunctor<CameraModel>, 2, 4, 3, 4, 3, 4, 3,
          CameraModel::num_params> {
public:
  explicit CalibRigReprojErrorCostFunctor(const Eigen::Vector2d &point2D,
                                          const Eigen::Vector3d &board3D)
      : observed_x_(point2D(0)), observed_y_(point2D(1)), board3D_(board3D) {}

  template <typename T>
  bool operator()(const T *const cam_from_rig_rotation,
                  const T *const cam_from_rig_translation,
                  const T *const rig_from_world_rotation,
                  const T *const rig_from_world_translation,
                  const T *const board_rotation,
                  const T *const board_translation,
                  const T *const camera_params, T *residuals) const {
    Eigen::Matrix<T, 3, 1> board3D = board3D_.cast<T>();
    Eigen::Matrix<T, 3, 1> point3D =
        colmap::EigenQuaternionMap<T>(board_rotation) * board3D +
        colmap::EigenVector3Map<T>(board_translation);
    const Eigen::Matrix<T, 3, 1> point3D_in_cam =
        colmap::EigenQuaternionMap<T>(cam_from_rig_rotation) *
            (colmap::EigenQuaternionMap<T>(rig_from_world_rotation) * point3D +
             colmap::EigenVector3Map<T>(rig_from_world_translation)) +
        colmap::EigenVector3Map<T>(cam_from_rig_translation);
    if (CameraModel::ImgFromCam(camera_params, point3D_in_cam[0],
                                point3D_in_cam[1], point3D_in_cam[2],
                                &residuals[0], &residuals[1])) {
      residuals[0] -= T(observed_x_);
      residuals[1] -= T(observed_y_);
    } else {
      residuals[0] = T(0);
      residuals[1] = T(0);
    }
    return true;
  }

private:
  const double observed_x_;
  const double observed_y_;
  Eigen::Vector3d board3D_;
};

// Rig bundle adjustment cost function for variable camera pose and camera
// calibration and point parameters but fixed rig extrinsic poses.
template <typename CameraModel>
class CalibRigReprojErrorConstantRigCostFunctor
    : public colmap::AutoDiffCostFunctor<
          CalibRigReprojErrorConstantRigCostFunctor<CameraModel>, 2, 4, 3, 4, 3,
          CameraModel::num_params> {
public:
  CalibRigReprojErrorConstantRigCostFunctor(const Eigen::Vector2d &point2D,
                                            const colmap::Rigid3d &cam_from_rig,
                                            const Eigen::Vector3d &board3D)
      : cam_from_rig_(cam_from_rig), reproj_cost_(point2D, board3D) {}

  template <typename T>
  bool operator()(const T *const rig_from_world_rotation,
                  const T *const rig_from_world_translation,
                  const T *const board_rotation,
                  const T *const board_translation,
                  const T *const camera_params, T *residuals) const {
    const Eigen::Quaternion<T> cam_from_rig_rotation =
        cam_from_rig_.rotation.cast<T>();
    const Eigen::Matrix<T, 3, 1> cam_from_rig_translation =
        cam_from_rig_.translation.cast<T>();
    return reproj_cost_(
        cam_from_rig_rotation.coeffs().data(), cam_from_rig_translation.data(),
        rig_from_world_rotation, rig_from_world_translation, board_rotation,
        board_translation, camera_params, residuals);
  }

private:
  const colmap::Rigid3d cam_from_rig_;
  const CalibRigReprojErrorCostFunctor<CameraModel> reproj_cost_;
};

} // namespace colmap_extension
