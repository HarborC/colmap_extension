#pragma once
#include "colmap/estimators/bundle_adjustment.h"
#include "colmap_extension/calibration/calib_rig_system.h"

namespace colmap_extension {

std::unique_ptr<colmap::BundleAdjuster>
CreateCalibBundleAdjuster(colmap::BundleAdjustmentOptions options,
                          colmap::BundleAdjustmentConfig config,
                          colmap::Reconstruction &reconstruction,
                          calibration_utils::CalibRigSystem::Ptr &rig_system);
}