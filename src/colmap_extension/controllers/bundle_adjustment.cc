#include "colmap_extension/controllers/bundle_adjustment.h"

#include <colmap/estimators/bundle_adjustment.h>
#include <colmap/sfm/observation_manager.h>
#include <colmap/util/misc.h>
#include <colmap/util/timer.h>
#include <colmap/scene/database_cache.h>
#include <colmap/controllers/incremental_pipeline.h>

#include <ceres/ceres.h>

using namespace colmap;

namespace colmap_extension {
namespace {

// Callback functor called after each bundle adjustment iteration.
class BundleAdjustmentIterationCallback : public ceres::IterationCallback {
 public:
  explicit BundleAdjustmentIterationCallback(BaseController* controller)
      : controller_(controller) {}

  virtual ceres::CallbackReturnType operator()(
      const ceres::IterationSummary& summary) {
    THROW_CHECK_NOTNULL(controller_);
    if (controller_->CheckIfStopped()) {
      return ceres::SOLVER_TERMINATE_SUCCESSFULLY;
    } else {
      return ceres::SOLVER_CONTINUE;
    }
  }

 private:
  BaseController* controller_;
};

}  // namespace

BundleAdjustmentController::BundleAdjustmentController(
    const OptionManager& options,
    std::shared_ptr<Reconstruction> reconstruction)
    : options_(options), reconstruction_(std::move(reconstruction)) {}

void BundleAdjustmentController::Run(bool _use_prior_position) {
  THROW_CHECK_NOTNULL(reconstruction_);

  std::unordered_set<std::string> image_names;
  for (const image_t image_id : reconstruction_->RegImageIds()) {
    const auto& image = reconstruction_->Image(image_id);
    image_names.insert(image.Name());
  }

  Database database(*(options_.database_path));
  Timer timer;
  timer.Start();
  const size_t min_num_matches = 0;
  auto database_cache = DatabaseCache::Create(database, min_num_matches, false, image_names);
  timer.PrintMinutes();

  if (database_cache->NumImages() == 0) {
    LOG(WARNING) << "No images with matches found in the database";
    return;
  }

  // If prior positions are to be used and setup from the database, convert
  // geographic coords. to cartesian ones
  if (_use_prior_position) {
    database_cache->SetupPosePriors();
  }

  PrintHeading1("Global bundle adjustment");
  Timer run_timer;
  run_timer.Start();

  if (reconstruction_->NumRegFrames() == 0) {
    LOG(ERROR) << "Need at least one registered frame.";
    return;
  }

  // Avoid degeneracies in bundle adjustment.
  ObservationManager(*reconstruction_).FilterObservationsWithNegativeDepth();

  BundleAdjustmentOptions ba_options = *options_.bundle_adjustment;

  BundleAdjustmentIterationCallback iteration_callback(this);
  ba_options.solver_options.callbacks.push_back(&iteration_callback);

  // Configure bundle adjustment.
  BundleAdjustmentConfig ba_config;
  for (const image_t image_id : reconstruction_->RegImageIds()) {
    ba_config.AddImage(image_id);
  }

  std::unique_ptr<BundleAdjuster> bundle_adjuster;
  if (!_use_prior_position) {
    ba_config.FixGauge(BundleAdjustmentGauge::TWO_CAMS_FROM_WORLD);

    // Run bundle adjustment.
    std::unique_ptr<BundleAdjuster> bundle_adjuster = CreateDefaultBundleAdjuster(
        std::move(ba_options), std::move(ba_config), *reconstruction_);
  } else {
    LOG(INFO) << "start bundle adjustment with prior positions";
    PosePriorBundleAdjustmentOptions prior_options;
    prior_options.use_robust_loss_on_prior_position = false;
    prior_options.prior_position_loss_scale = 7.815;
    bundle_adjuster = CreatePosePriorBundleAdjuster(std::move(ba_options),
                                                    prior_options,
                                                    std::move(ba_config),
                                                    database_cache->PosePriors(),
                                                    *reconstruction_);
  }
  bundle_adjuster->Solve();

  run_timer.PrintMinutes();
}

}  // namespace colmap_extension
