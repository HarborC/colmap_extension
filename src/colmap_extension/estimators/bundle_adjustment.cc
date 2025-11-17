#include "bundle_adjustment.h"

#include "colmap/estimators/alignment.h"
#include "colmap/estimators/cost_functions.h"
#include "colmap/estimators/manifold.h"
#include "colmap/scene/projection.h"
#include "colmap/sensor/models.h"
#include "colmap/util/cuda.h"
#include "colmap/util/misc.h"
#include "colmap/util/threading.h"
#include "colmap/util/timer.h"
#include "colmap_extension/estimators/cost_functions.h"

namespace colmap_extension {

using namespace colmap;

void ParameterizeCameras(const BundleAdjustmentOptions &options,
                         const BundleAdjustmentConfig &config,
                         const std::set<camera_t> &camera_ids,
                         Reconstruction &reconstruction,
                         ceres::Problem &problem) {
  const bool constant_camera = !options.refine_focal_length &&
                               !options.refine_principal_point &&
                               !options.refine_extra_params;
  for (const camera_t camera_id : camera_ids) {
    Camera &camera = reconstruction.Camera(camera_id);

    if (constant_camera || config.HasConstantCamIntrinsics(camera_id)) {
      problem.SetParameterBlockConstant(camera.params.data());
    } else {
      std::vector<int> const_camera_params;

      if (!options.refine_focal_length) {
        const span<const size_t> params_idxs = camera.FocalLengthIdxs();
        const_camera_params.insert(const_camera_params.end(),
                                   params_idxs.begin(), params_idxs.end());
      }
      if (!options.refine_principal_point) {
        const span<const size_t> params_idxs = camera.PrincipalPointIdxs();
        const_camera_params.insert(const_camera_params.end(),
                                   params_idxs.begin(), params_idxs.end());
      }
      if (!options.refine_extra_params) {
        const span<const size_t> params_idxs = camera.ExtraParamsIdxs();
        const_camera_params.insert(const_camera_params.end(),
                                   params_idxs.begin(), params_idxs.end());
      }

      if (const_camera_params.size() > 0) {
        if (problem.GetParameterization(camera.params.data()) == nullptr) {
          SetSubsetManifold(static_cast<int>(camera.params.size()),
                            const_camera_params, &problem, camera.params.data());
        } 
      }
    }
  }
}

void ParameterizeImages(const BundleAdjustmentOptions &options,
                        const BundleAdjustmentConfig &config,
                        const std::set<image_t> &image_ids,
                        Reconstruction &reconstruction,
                        ceres::Problem &problem) {
  std::unordered_set<rig_t> parameterized_rig_ids;
  std::unordered_set<sensor_t> parameterized_sensor_ids;
  std::unordered_set<frame_t> parameterized_frame_ids;
  for (const image_t image_id : image_ids) {
    Image &image = reconstruction.Image(image_id);
    parameterized_rig_ids.insert(image.FramePtr()->RigId());

    // Parameterize sensor_from_rig.
    const sensor_t sensor_id = image.CameraPtr()->SensorId();
    const bool not_parameterized_before =
        parameterized_sensor_ids.insert(sensor_id).second;
    if (not_parameterized_before && !image.HasTrivialFrame()) {
      Rigid3d &sensor_from_rig =
          image.FramePtr()->RigPtr()->SensorFromRig(sensor_id);
      // CostFunction assumes unit quaternions.
      sensor_from_rig.rotation.normalize();
      if (problem.HasParameterBlock(sensor_from_rig.rotation.coeffs().data())) {
        if (problem.GetParameterization(sensor_from_rig.rotation.coeffs().data()) == nullptr) {
          SetQuaternionManifold(&problem, sensor_from_rig.rotation.coeffs().data());
        }
        if (!options.refine_sensor_from_rig ||
            config.HasConstantSensorFromRigPose(sensor_id)) {
          problem.SetParameterBlockConstant(
              sensor_from_rig.rotation.coeffs().data());
          problem.SetParameterBlockConstant(sensor_from_rig.translation.data());
        }
      }
    }

    // Parameterize rig_from_world.
    if (parameterized_frame_ids.insert(image.FrameId()).second) {
      Rigid3d &rig_from_world = image.FramePtr()->RigFromWorld();
      // CostFunction assumes unit quaternions.
      rig_from_world.rotation.normalize();
      if (problem.HasParameterBlock(rig_from_world.rotation.coeffs().data())) {
        if (problem.GetParameterization(rig_from_world.rotation.coeffs().data()) == nullptr) {
          SetQuaternionManifold(&problem, rig_from_world.rotation.coeffs().data());
        }
        if (!options.refine_rig_from_world ||
            config.HasConstantRigFromWorldPose(image.FrameId())) {
          problem.SetParameterBlockConstant(
              rig_from_world.rotation.coeffs().data());
          problem.SetParameterBlockConstant(rig_from_world.translation.data());
        }
      }
    }
  }

  // Set the rig poses as constant, if the reference sensor is not part of the
  // problem. Otherwise, the relative pose between the sensors is not well
  // constrained. Notice that this does not handle degenerate configurations and
  // assumes the observations in the problem constrain the relative poses
  // sufficiently.
  for (const rig_t rig_id : parameterized_rig_ids) {
    Rig &rig = reconstruction.Rig(rig_id);
    if (parameterized_sensor_ids.count(rig.RefSensorId()) != 0) {
      continue;
    }
    for (auto &[_, sensor_from_rig] : rig.NonRefSensors()) {
      if (sensor_from_rig.has_value() &&
          problem.HasParameterBlock(sensor_from_rig->translation.data())) {
        problem.SetParameterBlockConstant(
            sensor_from_rig->rotation.coeffs().data());
        problem.SetParameterBlockConstant(sensor_from_rig->translation.data());
      }
    }
  }
}

void ParameterizeBoards(
    const BundleAdjustmentOptions &options,
    const BundleAdjustmentConfig &config,
    const std::set<calibration_utils::CalibBoard::ID> &board_ids,
    calibration_utils::CalibRigSystem::Ptr &rig_system,
    ceres::Problem &problem) {
  // Parameterize rig_from_world.
  for (const auto &board_id : board_ids) {
    Rigid3d &world_from_board = rig_system->board_poses_.at(board_id);
    // CostFunction assumes unit quaternions.
    world_from_board.rotation.normalize();
    if (problem.HasParameterBlock(world_from_board.rotation.coeffs().data())) {
      if (problem.GetParameterization(world_from_board.rotation.coeffs().data()) == nullptr) {
        SetQuaternionManifold(&problem, world_from_board.rotation.coeffs().data());
      }
      if (board_id == rig_system->rig_origin_board_id_) {
        problem.SetParameterBlockConstant(
            world_from_board.rotation.coeffs().data());
        problem.SetParameterBlockConstant(world_from_board.translation.data());
      }
    }
  }
}

class CalibBundleAdjuster : public BundleAdjuster {
public:
  CalibBundleAdjuster(BundleAdjustmentOptions options,
                      BundleAdjustmentConfig config,
                      Reconstruction &reconstruction,
                      calibration_utils::CalibRigSystem::Ptr &rig_system)
      : BundleAdjuster(std::move(options), std::move(config)),
        reconstruction_(reconstruction), rig_system_(rig_system),
        loss_function_(std::unique_ptr<ceres::LossFunction>(
            options_.CreateLossFunction())) {
    AlignReconstruction();
    normalized_from_metric_ = reconstruction_.Normalize(/*fixed_scale=*/true);

    bundle_adjuster_ =
        CreateDefaultBundleAdjuster(options_, config_, reconstruction);

    for (const image_t image_id : config_.Images()) {
      auto frame_it = rig_system_->frames().find(image_id);
      if (frame_it == rig_system_->frames().end()) {
        continue;
      }
      const auto &frame = frame_it->second;
      AddImageToProblem(image_id, reconstruction_, frame);
    }

    std::shared_ptr<ceres::Problem> problem = Problem();

    ParameterizeCameras(options_, config_, parameterized_camera_ids_,
                        reconstruction, *problem);
    ParameterizeImages(options_, config_, parameterized_image_ids_,
                       reconstruction, *problem);
    ParameterizeBoards(options_, config_, parameterized_board_ids_, rig_system_,
                       *problem);
  }

  ceres::Solver::Summary Solve() override {
    ceres::Solver::Summary summary;
    std::shared_ptr<ceres::Problem> problem = bundle_adjuster_->Problem();
    if (problem->NumResiduals() == 0) {
      return summary;
    }

    options_.solver_options.max_num_iterations = 10000;

    const ceres::Solver::Options solver_options =
        options_.CreateSolverOptions(config_, *problem);

    ceres::Solve(solver_options, problem.get(), &summary);

    reconstruction_.Transform(Inverse(normalized_from_metric_));

    if (options_.print_summary || VLOG_IS_ON(1)) {
      PrintSolverSummary(summary, "Calib Bundle adjustment report");
    }

    return summary;
  }

  std::shared_ptr<ceres::Problem> &Problem() override {
    return bundle_adjuster_->Problem();
  }

  void AddImageToProblem(const image_t image_id, Reconstruction &reconstruction,
                         const calibration_utils::CalibFrame::Ptr &frame) {
    Image &image = reconstruction.Image(image_id);

    if (image.HasTrivialFrame()) {
      AddImageWithTrivialFrame(image, reconstruction, frame);
    } else {
      AddImageWithNonTrivialFrame(image, reconstruction, frame);
    }
  }

  void
  AddImageWithTrivialFrame(Image &image, Reconstruction &reconstruction,
                           const calibration_utils::CalibFrame::Ptr &frame) {
    Camera &camera = *image.CameraPtr();

    THROW_CHECK(image.HasTrivialFrame());
    Rigid3d &cam_from_world = image.FramePtr()->RigFromWorld();

    const bool constant_cam_from_world =
        !options_.refine_rig_from_world ||
        config_.HasConstantRigFromWorldPose(image.FrameId());

    std::shared_ptr<ceres::Problem> problem = Problem();

    // Add residuals to bundle adjustment problem.
    size_t num_observations = 0;
    for (auto &det : frame->detections) {
      auto &board = det->board_;
      Rigid3d &world_from_board = rig_system_->board_poses_.at(board->id());
      for (size_t i = 0; i < det->corners2d_.size(); ++i) {
        num_observations += 1;
        const Eigen::Vector2d &point2D = det->corners2d_[i];
        const Eigen::Vector3d &point3D = det->corners3d_[i];

        if (constant_cam_from_world) {
          problem->AddResidualBlock(
              CreateCameraCostFunction<CalibReprojErrorConstantPoseCostFunctor>(
                  camera.model_id, point2D, cam_from_world, point3D),
              loss_function_.get(), world_from_board.rotation.coeffs().data(),
              world_from_board.translation.data(), camera.params.data());
        } else {
          problem->AddResidualBlock(
              CreateCameraCostFunction<CalibReprojErrorCostFunctor>(
                  camera.model_id, point2D, point3D),
              loss_function_.get(), cam_from_world.rotation.coeffs().data(),
              cam_from_world.translation.data(),
              world_from_board.rotation.coeffs().data(),
              world_from_board.translation.data(), camera.params.data());
        }
      }
      parameterized_board_ids_.insert(board->id());
    }

    if (num_observations > 0) {
      parameterized_camera_ids_.insert(image.CameraId());
      parameterized_image_ids_.insert(image.ImageId());
    }
  }

  void
  AddImageWithNonTrivialFrame(Image &image, Reconstruction &reconstruction,
                              const calibration_utils::CalibFrame::Ptr &frame) {
    Camera &camera = *image.CameraPtr();
    const sensor_t sensor_id = camera.SensorId();

    THROW_CHECK(!image.HasTrivialFrame());
    Rigid3d &cam_from_rig =
        image.FramePtr()->RigPtr()->SensorFromRig(sensor_id);
    Rigid3d &rig_from_world = image.FramePtr()->RigFromWorld();

    const bool constant_sensor_from_rig =
        !options_.refine_sensor_from_rig ||
        config_.HasConstantSensorFromRigPose(sensor_id);
    const bool constant_rig_from_world =
        !options_.refine_rig_from_world ||
        config_.HasConstantRigFromWorldPose(image.FrameId());

    std::shared_ptr<ceres::Problem> problem = Problem();

    // Add residuals to bundle adjustment problem.
    size_t num_observations = 0;
    for (auto &det : frame->detections) {
      auto &board = det->board_;
      Rigid3d &world_from_board = rig_system_->board_poses_.at(board->id());
      for (size_t i = 0; i < det->corners2d_.size(); ++i) {
        num_observations += 1;
        const Eigen::Vector2d &point2D = det->corners2d_[i];
        const Eigen::Vector3d &point3D = det->corners3d_[i];

        if (constant_sensor_from_rig && constant_rig_from_world) {
          problem->AddResidualBlock(
              CreateCameraCostFunction<CalibReprojErrorConstantPoseCostFunctor>(
                  camera.model_id, point2D, cam_from_rig * rig_from_world,
                  point3D),
              loss_function_.get(), world_from_board.rotation.coeffs().data(),
              world_from_board.translation.data(), camera.params.data());
        } else if (!constant_rig_from_world && constant_sensor_from_rig) {
          problem->AddResidualBlock(
              CreateCameraCostFunction<
                  CalibRigReprojErrorConstantRigCostFunctor>(
                  camera.model_id, point2D, cam_from_rig, point3D),
              loss_function_.get(), rig_from_world.rotation.coeffs().data(),
              rig_from_world.translation.data(),
              world_from_board.rotation.coeffs().data(),
              world_from_board.translation.data(), camera.params.data());
        } else {
          problem->AddResidualBlock(
              CreateCameraCostFunction<CalibRigReprojErrorCostFunctor>(
                  camera.model_id, point2D, point3D),
              loss_function_.get(), cam_from_rig.rotation.coeffs().data(),
              cam_from_rig.translation.data(),
              rig_from_world.rotation.coeffs().data(),
              rig_from_world.translation.data(),
              world_from_board.rotation.coeffs().data(),
              world_from_board.translation.data(), camera.params.data());
        }
      }
      parameterized_board_ids_.insert(board->id());
    }

    if (num_observations > 0) {
      parameterized_camera_ids_.insert(image.CameraId());
      parameterized_image_ids_.insert(image.ImageId());
    }
  }

  bool AlignReconstruction() {
    RANSACOptions ransac_options;

    // VLOG(2) << "Robustly aligning reconstruction with max_error="
    //         << ransac_options.max_error;

    std::unordered_map<image_t, colmap::PosePrior> pose_priors;
    for (const auto &frame_pair : rig_system_->frames()) {
      const image_t image_id = frame_pair.first;
      const auto &frame = frame_pair.second;
      if (frame->detections.empty()) {
        continue;
      }
      // Use the first detection's pose as prior.
      for (const auto &detection : frame->detections) {
        if (detection->board_->id() == rig_system_->rig_origin_board_id_) {
          if (detection->cam_from_world_ == nullptr) {
            continue;
          }
          Eigen::Matrix4d cam_from_world_mat = *(detection->cam_from_world_);
          Eigen::Matrix4d world_from_cam_mat = cam_from_world_mat.inverse();
          Rigid3d world_from_cam;
          world_from_cam.rotation =
              Eigen::Quaterniond(world_from_cam_mat.block<3, 3>(0, 0));
          world_from_cam.translation = world_from_cam_mat.block<3, 1>(0, 3);
          PosePrior pose_prior(world_from_cam.translation);
          pose_priors.emplace(image_id, pose_prior);
          break;
        }
      }
    }

    Sim3d metric_from_orig;
    if (!AlignReconstructionToPosePriors(reconstruction_, pose_priors, ransac_options,
            &metric_from_orig)) {
      LOG(WARNING) << "Alignment w.r.t. prior positions failed";
      return false;
    }
    reconstruction_.Transform(metric_from_orig);

    return true;
  }

private:
  // const PosePriorBundleAdjustmentOptions prior_options_;
  Reconstruction &reconstruction_;
  calibration_utils::CalibRigSystem::Ptr &rig_system_;

  std::unique_ptr<BundleAdjuster> bundle_adjuster_;
  std::unique_ptr<ceres::LossFunction> loss_function_;

  Sim3d normalized_from_metric_;
  std::set<camera_t> parameterized_camera_ids_;
  std::set<image_t> parameterized_image_ids_;
  std::set<calibration_utils::CalibBoard::ID> parameterized_board_ids_;
};

std::unique_ptr<BundleAdjuster>
CreateCalibBundleAdjuster(BundleAdjustmentOptions options,
                          BundleAdjustmentConfig config,
                          Reconstruction &reconstruction,
                          calibration_utils::CalibRigSystem::Ptr &rig_system) {
  return std::make_unique<CalibBundleAdjuster>(
      std::move(options), std::move(config), reconstruction, rig_system);
}

} // namespace colmap_extension