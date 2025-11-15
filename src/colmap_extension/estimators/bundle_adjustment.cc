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

namespace colmap_extension {

using namespace colmap;

void ParameterizeCameras(const BundleAdjustmentOptions& options,
                         const BundleAdjustmentConfig& config,
                         const std::set<camera_t>& camera_ids,
                         Reconstruction& reconstruction,
                         ceres::Problem& problem) {
  const bool constant_camera = !options.refine_focal_length &&
                               !options.refine_principal_point &&
                               !options.refine_extra_params;
  for (const camera_t camera_id : camera_ids) {
    Camera& camera = reconstruction.Camera(camera_id);

    if (constant_camera || config.HasConstantCamIntrinsics(camera_id)) {
      problem.SetParameterBlockConstant(camera.params.data());
    } else {
      std::vector<int> const_camera_params;

      if (!options.refine_focal_length) {
        const span<const size_t> params_idxs = camera.FocalLengthIdxs();
        const_camera_params.insert(
            const_camera_params.end(), params_idxs.begin(), params_idxs.end());
      }
      if (!options.refine_principal_point) {
        const span<const size_t> params_idxs = camera.PrincipalPointIdxs();
        const_camera_params.insert(
            const_camera_params.end(), params_idxs.begin(), params_idxs.end());
      }
      if (!options.refine_extra_params) {
        const span<const size_t> params_idxs = camera.ExtraParamsIdxs();
        const_camera_params.insert(
            const_camera_params.end(), params_idxs.begin(), params_idxs.end());
      }

      if (const_camera_params.size() > 0) {
        SetSubsetManifold(static_cast<int>(camera.params.size()),
                          const_camera_params,
                          &problem,
                          camera.params.data());
      }
    }
  }
}

struct FixedGaugeWithThreePoints {
  // The number of fixed points for the Gauge.
  Eigen::Index num_fixed_points = 0;
  // The coordinates of the fixed points as columns.
  Eigen::Matrix3d fixed_points = Eigen::Matrix3d::Zero();
  bool MaybeAddFixedPoint(const Eigen::Vector3d& point) {
    if (num_fixed_points >= 3) {
      return false;
    }
    fixed_points.col(num_fixed_points) = point;
    if (fixed_points.colPivHouseholderQr().rank() > num_fixed_points) {
      ++num_fixed_points;
      return true;
    } else {
      fixed_points.col(num_fixed_points).setZero();
      return false;
    }
  }
};

void FixGaugeWithThreePoints(
    const std::unordered_map<point3D_t, size_t>& point3D_num_observations,
    Reconstruction& reconstruction,
    ceres::Problem& problem) {
  FixedGaugeWithThreePoints fixed_gauge;

  // First check if we already fixed enough points in the problem.
  for (const auto& [point3D_id, num_observations] : point3D_num_observations) {
    Point3D& point3D = reconstruction.Point3D(point3D_id);
    if (problem.IsParameterBlockConstant(point3D.xyz.data()) &&
        fixed_gauge.MaybeAddFixedPoint(point3D.xyz) &&
        fixed_gauge.num_fixed_points >= 3) {
      return;
    }
  }

  // Otherwise, fix sufficient points in the problem.
  for (const auto& [point3D_id, num_observations] : point3D_num_observations) {
    Point3D& point3D = reconstruction.Point3D(point3D_id);
    if (!problem.IsParameterBlockConstant(point3D.xyz.data()) &&
        fixed_gauge.MaybeAddFixedPoint(point3D.xyz)) {
      problem.SetParameterBlockConstant(point3D.xyz.data());
      if (fixed_gauge.num_fixed_points >= 3) {
        return;
      }
    }
  }

  LOG(WARNING)
      << "Failed to fix Gauge due to insufficient number of fixed points: "
      << fixed_gauge.num_fixed_points;
}

// Note that the following implementation does not handle all degenerate edge
// cases well, e.g., where the selected two cameras are not well constrained
// with respect to each other with shared observations. Furthermore, the
// implementation could be more sophisticated for multi-camera rigs by selecting
// camera pairs within a rig, etc.
void FixGaugeWithTwoCamsFromWorld(
    const BundleAdjustmentOptions& options,
    const BundleAdjustmentConfig& config,
    const std::set<image_t>& image_ids,
    const std::unordered_map<point3D_t, size_t>& point3D_num_observations,
    Reconstruction& reconstruction,
    ceres::Problem& problem) {
  // No need to fix the Gauge if all frames are constant.
  if (!options.refine_rig_from_world) {
    return;
  }

  Image* image1 = nullptr;
  Image* image2 = nullptr;

  // Check if a sensor is either a reference sensor, or a non-reference sensor
  // with sensor_from_rig fixed.
  auto IsParameterizedConstSensor = [&problem, &config, &options](
                                        const Image& image) {
    if (image.FramePtr()->RigPtr()->IsRefSensor(
            image.CameraPtr()->SensorId())) {
      return true;
    }
    Rigid3d& sensor_from_rig = image.FramePtr()->RigPtr()->SensorFromRig(
        image.CameraPtr()->SensorId());
    if (problem.HasParameterBlock(sensor_from_rig.rotation.coeffs().data()) &&
        problem.IsParameterBlockConstant(
            sensor_from_rig.rotation.coeffs().data()) &&
        problem.HasParameterBlock(sensor_from_rig.translation.data()) &&
        problem.IsParameterBlockConstant(sensor_from_rig.translation.data())) {
      return true;
    }
    // Cover corner case when ReprojErrorConstantPoseCostFunctor is used
    if (config.HasConstantSensorFromRigPose(image.CameraPtr()->SensorId()) ||
        !options.refine_sensor_from_rig) {
      return true;
    }
    return false;
  };

  // First, search through the already fixed cameras in the problem.
  for (const image_t image_id : image_ids) {
    Image& image = reconstruction.Image(image_id);
    if (config.HasConstantRigFromWorldPose(image.FrameId()) &&
        IsParameterizedConstSensor(image)) {
      if (image1 == nullptr) {
        image1 = &image;
      } else if (image1 != nullptr && image1->FrameId() != image.FrameId()) {
        // No need to fix the Gauge if two frames are already fixed.
        return;
      }
    }
  }

  // Otherwise, search through the variable cameras in the problem.
  Eigen::Index frame2_from_world_fixed_dim = 0;
  for (const image_t image_id : image_ids) {
    Image& image = reconstruction.Image(image_id);
    if (image1 == nullptr && IsParameterizedConstSensor(image)) {
      image1 = &image;
    } else if (image1 != nullptr && image1->FrameId() != image.FrameId() &&
               IsParameterizedConstSensor(image) &&
               problem.HasParameterBlock(
                   image.FramePtr()->RigFromWorld().translation.data())) {
      // Check if one of the baseline dimensions is large enough and
      // choose it as the fixed coordinate. If there is no such pair of
      // frames, then the scale is not constrained well.
      const Eigen::Vector3d baseline =
          (image1->FramePtr()->RigFromWorld() *
           Inverse(image.FramePtr()->RigFromWorld()))
              .translation;
      if (baseline.cwiseAbs().maxCoeff(&frame2_from_world_fixed_dim) > 1e-9) {
        image2 = &image;
        break;
      }
    }
  }

  // TODO(jsch): Notice that we could alternatively fall back to fixing the
  // Gauge between two cameras in the same frame or in different frames. Since
  // there are many different combinations to iterate through, we instead fall
  // back to fixing the Gauge with three points for simplicity. Furthermore,
  // once we support IMUs or other sensors, we should fix the Gauge differently.
  if (image1 == nullptr || image2 == nullptr) {
    LOG(WARNING) << "Failed to fix Gauge with two cameras. "
                    "Falling back to fixing Gauge with three points.";
    FixGaugeWithThreePoints(point3D_num_observations, reconstruction, problem);
    return;
  }

  Rigid3d& frame1_from_world = image1->FramePtr()->RigFromWorld();
  if (!config.HasConstantRigFromWorldPose(image1->FrameId())) {
    problem.SetParameterBlockConstant(
        frame1_from_world.rotation.coeffs().data());
    problem.SetParameterBlockConstant(frame1_from_world.translation.data());
  }

  Rigid3d& frame2_from_world = image2->FramePtr()->RigFromWorld();
  if (!config.HasConstantRigFromWorldPose(image2->FrameId())) {
    SetSubsetManifold(3,
                      {static_cast<int>(frame2_from_world_fixed_dim)},
                      &problem,
                      frame2_from_world.translation.data());
  }
}

void ParameterizeImages(const BundleAdjustmentOptions& options,
                        const BundleAdjustmentConfig& config,
                        const std::set<image_t>& image_ids,
                        Reconstruction& reconstruction,
                        ceres::Problem& problem) {
  std::unordered_set<rig_t> parameterized_rig_ids;
  std::unordered_set<sensor_t> parameterized_sensor_ids;
  std::unordered_set<frame_t> parameterized_frame_ids;
  for (const image_t image_id : image_ids) {
    Image& image = reconstruction.Image(image_id);
    parameterized_rig_ids.insert(image.FramePtr()->RigId());

    // Parameterize sensor_from_rig.
    const sensor_t sensor_id = image.CameraPtr()->SensorId();
    const bool not_parameterized_before =
        parameterized_sensor_ids.insert(sensor_id).second;
    if (not_parameterized_before && !image.HasTrivialFrame()) {
      Rigid3d& sensor_from_rig =
          image.FramePtr()->RigPtr()->SensorFromRig(sensor_id);
      // CostFunction assumes unit quaternions.
      sensor_from_rig.rotation.normalize();
      if (problem.HasParameterBlock(sensor_from_rig.rotation.coeffs().data())) {
        SetQuaternionManifold(&problem,
                              sensor_from_rig.rotation.coeffs().data());
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
      Rigid3d& rig_from_world = image.FramePtr()->RigFromWorld();
      // CostFunction assumes unit quaternions.
      rig_from_world.rotation.normalize();
      if (problem.HasParameterBlock(rig_from_world.rotation.coeffs().data())) {
        SetQuaternionManifold(&problem,
                              rig_from_world.rotation.coeffs().data());
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
    Rig& rig = reconstruction.Rig(rig_id);
    if (parameterized_sensor_ids.count(rig.RefSensorId()) != 0) {
      continue;
    }
    for (auto& [_, sensor_from_rig] : rig.NonRefSensors()) {
      if (sensor_from_rig.has_value() &&
          problem.HasParameterBlock(sensor_from_rig->translation.data())) {
        problem.SetParameterBlockConstant(
            sensor_from_rig->rotation.coeffs().data());
        problem.SetParameterBlockConstant(sensor_from_rig->translation.data());
      }
    }
  }
}

void ParameterizePoints(
    const BundleAdjustmentConfig& config,
    const std::unordered_map<point3D_t, size_t>& point3D_num_observations,
    Reconstruction& reconstruction,
    ceres::Problem& problem) {
  for (const auto& [point3D_id, num_observations] : point3D_num_observations) {
    Point3D& point3D = reconstruction.Point3D(point3D_id);
    if (point3D.track.Length() > num_observations) {
      problem.SetParameterBlockConstant(point3D.xyz.data());
    }
  }

  for (const point3D_t point3D_id : config.ConstantPoints()) {
    Point3D& point3D = reconstruction.Point3D(point3D_id);
    problem.SetParameterBlockConstant(point3D.xyz.data());
  }
}

class CalibBundleAdjuster : public BundleAdjuster {
 public:
  CalibBundleAdjuster(BundleAdjustmentOptions options,
                      BundleAdjustmentConfig config,
                      Reconstruction& reconstruction)
      : BundleAdjuster(std::move(options), std::move(config)),
        loss_function_(std::unique_ptr<ceres::LossFunction>(
            options_.CreateLossFunction())) {
    ceres::Problem::Options problem_options;
    problem_options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    problem_ = std::make_shared<ceres::Problem>(problem_options);

    // Set up problem
    // Warning: AddPointsToProblem assumes that AddImageToProblem is called
    // first. Do not change order of instructions!
    for (const image_t image_id : config_.Images()) {
      AddImageToProblem(image_id, reconstruction);
    }
    for (const auto point3D_id : config_.VariablePoints()) {
      AddPointToProblem(point3D_id, reconstruction);
    }
    for (const auto point3D_id : config_.ConstantPoints()) {
      AddPointToProblem(point3D_id, reconstruction);
    }

    ParameterizeCameras(options_,
                        config_,
                        parameterized_camera_ids_,
                        reconstruction,
                        *problem_);
    ParameterizeImages(
        options_, config_, parameterized_image_ids_, reconstruction, *problem_);
    ParameterizePoints(
        config_, point3D_num_observations_, reconstruction, *problem_);

    switch (config_.FixedGauge()) {
      case BundleAdjustmentGauge::UNSPECIFIED:
        break;
      case BundleAdjustmentGauge::TWO_CAMS_FROM_WORLD:
        FixGaugeWithTwoCamsFromWorld(options_,
                                     config_,
                                     parameterized_image_ids_,
                                     point3D_num_observations_,
                                     reconstruction,
                                     *problem_);
        break;
      case BundleAdjustmentGauge::THREE_POINTS:
        FixGaugeWithThreePoints(
            point3D_num_observations_, reconstruction, *problem_);
        break;
      default:
        LOG(FATAL) << "Unknown BundleAdjustmentGauge";
    }
  }

  ceres::Solver::Summary Solve() override {
    ceres::Solver::Summary summary;
    if (problem_->NumResiduals() == 0) {
      return summary;
    }

    const ceres::Solver::Options solver_options =
        options_.CreateSolverOptions(config_, *problem_);

    ceres::Solve(solver_options, problem_.get(), &summary);

    if (options_.print_summary || VLOG_IS_ON(1)) {
      PrintSolverSummary(summary, "Bundle adjustment report");
    }

    return summary;
  }

  std::shared_ptr<ceres::Problem>& Problem() override { return problem_; }

  void AddImageToProblem(const image_t image_id,
                         Reconstruction& reconstruction) {
    Image& image = reconstruction.Image(image_id);

    if (image.HasTrivialFrame()) {
      AddImageWithTrivialFrame(image, reconstruction);
    } else {
      AddImageWithNonTrivialFrame(image, reconstruction);
    }
  }

  void AddImageWithTrivialFrame(Image& image, Reconstruction& reconstruction) {
    Camera& camera = *image.CameraPtr();

    THROW_CHECK(image.HasTrivialFrame());
    Rigid3d& cam_from_world = image.FramePtr()->RigFromWorld();

    const bool constant_cam_from_world =
        !options_.refine_rig_from_world ||
        config_.HasConstantRigFromWorldPose(image.FrameId());

    // Add residuals to bundle adjustment problem.
    size_t num_observations = 0;
    for (const Point2D& point2D : image.Points2D()) {
      if (!point2D.HasPoint3D() || config_.IsIgnoredPoint(point2D.point3D_id)) {
        continue;
      }

      num_observations += 1;
      point3D_num_observations_[point2D.point3D_id] += 1;

      Point3D& point3D = reconstruction.Point3D(point2D.point3D_id);
      THROW_CHECK_GT(point3D.track.Length(), 1);

      if (constant_cam_from_world) {
        problem_->AddResidualBlock(
            CreateCameraCostFunction<ReprojErrorConstantPoseCostFunctor>(
                camera.model_id, point2D.xy, cam_from_world),
            loss_function_.get(),
            point3D.xyz.data(),
            camera.params.data());
      } else {
        problem_->AddResidualBlock(
            CreateCameraCostFunction<ReprojErrorCostFunctor>(camera.model_id,
                                                             point2D.xy),
            loss_function_.get(),
            cam_from_world.rotation.coeffs().data(),
            cam_from_world.translation.data(),
            point3D.xyz.data(),
            camera.params.data());
      }
    }

    if (num_observations > 0) {
      parameterized_camera_ids_.insert(image.CameraId());
      parameterized_image_ids_.insert(image.ImageId());
    }
  }

  void AddImageWithNonTrivialFrame(Image& image,
                                   Reconstruction& reconstruction) {
    Camera& camera = *image.CameraPtr();
    const sensor_t sensor_id = camera.SensorId();

    THROW_CHECK(!image.HasTrivialFrame());
    Rigid3d& cam_from_rig =
        image.FramePtr()->RigPtr()->SensorFromRig(sensor_id);
    Rigid3d& rig_from_world = image.FramePtr()->RigFromWorld();

    const bool constant_sensor_from_rig =
        !options_.refine_sensor_from_rig ||
        config_.HasConstantSensorFromRigPose(sensor_id);
    const bool constant_rig_from_world =
        !options_.refine_rig_from_world ||
        config_.HasConstantRigFromWorldPose(image.FrameId());

    // Add residuals to bundle adjustment problem.
    size_t num_observations = 0;
    for (const Point2D& point2D : image.Points2D()) {
      if (!point2D.HasPoint3D() || config_.IsIgnoredPoint(point2D.point3D_id)) {
        continue;
      }

      num_observations += 1;
      point3D_num_observations_[point2D.point3D_id] += 1;

      Point3D& point3D = reconstruction.Point3D(point2D.point3D_id);
      THROW_CHECK_GT(point3D.track.Length(), 1);

      // The !constant_sensor_from_rig && constant_rig_from_world is
      // rare enough that we do not have a specialized cost function for it.
      if (constant_sensor_from_rig && constant_rig_from_world) {
        problem_->AddResidualBlock(
            CreateCameraCostFunction<ReprojErrorConstantPoseCostFunctor>(
                camera.model_id, point2D.xy, cam_from_rig * rig_from_world),
            loss_function_.get(),
            point3D.xyz.data(),
            camera.params.data());
      } else if (!constant_rig_from_world && constant_sensor_from_rig) {
        problem_->AddResidualBlock(
            CreateCameraCostFunction<RigReprojErrorConstantRigCostFunctor>(
                camera.model_id, point2D.xy, cam_from_rig),
            loss_function_.get(),
            rig_from_world.rotation.coeffs().data(),
            rig_from_world.translation.data(),
            point3D.xyz.data(),
            camera.params.data());
      } else {
        problem_->AddResidualBlock(
            CreateCameraCostFunction<RigReprojErrorCostFunctor>(camera.model_id,
                                                                point2D.xy),
            loss_function_.get(),
            cam_from_rig.rotation.coeffs().data(),
            cam_from_rig.translation.data(),
            rig_from_world.rotation.coeffs().data(),
            rig_from_world.translation.data(),
            point3D.xyz.data(),
            camera.params.data());
      }
    }

    if (num_observations > 0) {
      parameterized_camera_ids_.insert(image.CameraId());
      parameterized_image_ids_.insert(image.ImageId());
    }
  }

  void AddPointToProblem(const point3D_t point3D_id,
                         Reconstruction& reconstruction) {
    THROW_CHECK(!config_.IsIgnoredPoint(point3D_id));
    Point3D& point3D = reconstruction.Point3D(point3D_id);

    size_t& num_observations = point3D_num_observations_[point3D_id];

    // Is 3D point already fully contained in the problem? I.e. its entire
    // track is contained in `variable_image_ids`, `constant_image_ids`,
    // `constant_x_image_ids`.
    if (num_observations == point3D.track.Length()) {
      return;
    }

    for (const auto& track_el : point3D.track.Elements()) {
      // Skip observations that were already added in `FillImages`.
      if (config_.HasImage(track_el.image_id)) {
        continue;
      }

      num_observations += 1;

      Image& image = reconstruction.Image(track_el.image_id);
      Camera& camera = *image.CameraPtr();
      const Point2D& point2D = image.Point2D(track_el.point2D_idx);

      if (image.HasTrivialFrame()) {
        Rigid3d& cam_from_world = image.FramePtr()->RigFromWorld();

        problem_->AddResidualBlock(
            CreateCameraCostFunction<ReprojErrorConstantPoseCostFunctor>(
                camera.model_id, point2D.xy, cam_from_world),
            loss_function_.get(),
            point3D.xyz.data(),
            camera.params.data());
      } else {
        Rigid3d& cam_from_rig = image.FramePtr()->RigPtr()->SensorFromRig(
            image.CameraPtr()->SensorId());
        Rigid3d& rig_from_world = image.FramePtr()->RigFromWorld();

        problem_->AddResidualBlock(
            CreateCameraCostFunction<ReprojErrorConstantPoseCostFunctor>(
                camera.model_id, point2D.xy, cam_from_rig * rig_from_world),
            loss_function_.get(),
            point3D.xyz.data(),
            camera.params.data());
      }

      // Do not optimize intrinsics if th corresponding images
      // were not included explicitly in the config.
      if (parameterized_camera_ids_.insert(image.CameraId()).second) {
        config_.SetConstantCamIntrinsics(image.CameraId());
      }
    }
  }

 private:
  std::shared_ptr<ceres::Problem> problem_;
  std::unique_ptr<ceres::LossFunction> loss_function_;

  std::set<camera_t> parameterized_camera_ids_;
  std::set<image_t> parameterized_image_ids_;
  std::unordered_map<point3D_t, size_t> point3D_num_observations_;
};


std::unique_ptr<BundleAdjuster> CreateCalibBundleAdjuster(
    BundleAdjustmentOptions options,
    BundleAdjustmentConfig config,
    Reconstruction& reconstruction) {
  return std::make_unique<CalibBundleAdjuster>(
      std::move(options), std::move(config), reconstruction);
}

}  // namespace colmap_extension