#pragma once
#include "colmap/estimators/bundle_adjustment.h"

namespace colmap_extension {

std::unique_ptr<colmap::BundleAdjuster> CreateCalibBundleAdjuster(
    colmap::BundleAdjustmentOptions options,
    colmap::BundleAdjustmentConfig config,
    colmap::Reconstruction& reconstruction);

}