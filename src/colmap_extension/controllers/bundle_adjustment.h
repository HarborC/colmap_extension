#pragma once

#include "colmap/controllers/option_manager.h"
#include "colmap/scene/reconstruction.h"
#include "colmap/util/base_controller.h"

namespace colmap_extension {

// Class that controls the global bundle adjustment procedure.
class BundleAdjustmentController : public colmap::BaseController {
 public:
  BundleAdjustmentController(const colmap::OptionManager& options,
                             std::shared_ptr<colmap::Reconstruction> reconstruction);

  void Run(bool _use_prior_position = false);

 private:
  const colmap::OptionManager options_;
  std::shared_ptr<colmap::Reconstruction> reconstruction_;
};

}  // namespace colmap_extension
