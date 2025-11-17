#include "calibration.h"
#include "colmap/controllers/option_manager.h"
#include "colmap/scene/reconstruction.h"
#include "colmap/util/file.h"
#include "colmap/util/misc.h"
#include "colmap/sfm/observation_manager.h"
#include "colmap/util/base_controller.h"

#include "colmap_extension/calibration/calib_rig_system.h"
#include "colmap_extension/estimators/bundle_adjustment.h"

using namespace colmap;

namespace colmap_extension {

int RunCalibBoardDetection(int argc, char **argv) {
  std::string image_path;
  std::string detection_path;
  std::string board_config_path;
  std::string debug_image_path = "";

  OptionManager options;
  options.AddRequiredOption("image_path", &image_path);
  options.AddRequiredOption("board_config_path", &board_config_path);
  options.AddRequiredOption("detection_path", &detection_path);
  options.AddDefaultOption("debug_image_path", &debug_image_path);
  if (!options.Parse(argc, argv)) {
    return EXIT_FAILURE;
  }

  auto board =
      calibration_utils::CalibBoardFactory::FromConfig(board_config_path);

  if (!board) {
    LOG(ERROR) << "Failed to create calibration board from config: "
               << board_config_path;
    return EXIT_FAILURE;
  }

  cv::Mat image = cv::imread(image_path);
  if (image.empty()) {
    LOG(ERROR) << "Failed to read image: " << image_path;
    return EXIT_FAILURE;
  }

  auto detection = board->detect(image);
  if (!detection) {
    LOG(INFO) << "No detection found in image: " << image_path;
    return EXIT_FAILURE;
  }

  if (debug_image_path != "") {
    cv::Mat debug_image = detection->showInImage(image);
    cv::imwrite(debug_image_path, debug_image);
    LOG(INFO) << "Wrote debug image to: " << debug_image_path;
  }

  calibration_utils::saveDetection(detection, detection_path);
  LOG(INFO) << "Detection successful in image: " << image_path;

  return EXIT_SUCCESS;
}

int RunCalibBundleAdjuster(int argc, char **argv) {
  std::string image_dir;
  std::string detection_path;
  std::string input_path;
  std::string output_path;

  OptionManager options;
  options.AddRequiredOption("image_dir", &image_dir);
  options.AddRequiredOption("detection_path", &detection_path);
  options.AddRequiredOption("input_path", &input_path);
  options.AddRequiredOption("output_path", &output_path);
  options.AddBundleAdjustmentOptions();
  if (!options.Parse(argc, argv)) {
    return EXIT_FAILURE;
  }

  if (!ExistsDir(image_dir)) {
    LOG(ERROR) << image_dir << " image_dir is not a directory";
    return EXIT_FAILURE;
  }

  if (!ExistsDir(input_path)) {
    LOG(ERROR) << input_path << " input_path is not a directory";
    return EXIT_FAILURE;
  }

  auto reconstruction = std::make_shared<Reconstruction>();
  reconstruction->Read(input_path);

  calibration_utils::CalibRigSystem::Ptr rig_system(
      new calibration_utils::CalibRigSystem(detection_path));

  for (const image_t image_id : reconstruction->RegImageIds()) {
    Image &image = reconstruction->Image(image_id);
    std::string rel_name = image.Name();
    std::string abs_name = JoinPaths(image_dir, rel_name);
    cv::Mat img = cv::imread(abs_name, cv::IMREAD_GRAYSCALE);
    if (img.empty()) {
      LOG(WARNING) << "Could not read image: " << abs_name;
      continue;
    }

    rig_system->addFrame(img, image);
  }

  PrintHeading1("Global bundle adjustment");

  if (reconstruction->NumRegFrames() == 0) {
    LOG(ERROR) << "Need at least one registered frame.";
    return EXIT_FAILURE;
  }

  // Avoid degeneracies in bundle adjustment.
  ObservationManager(*reconstruction).FilterObservationsWithNegativeDepth();

  BundleAdjustmentOptions ba_options = *options.bundle_adjustment;

  // Configure bundle adjustment.
  colmap::BundleAdjustmentConfig ba_config;
  for (const image_t image_id : reconstruction->RegImageIds()) {
    ba_config.AddImage(image_id);
  }

  // Run bundle adjustment.
  auto bundle_adjuster = CreateCalibBundleAdjuster(
      std::move(ba_options), std::move(ba_config), *reconstruction, rig_system);
  bundle_adjuster->Solve();
  reconstruction->UpdatePoint3DErrors();

  reconstruction->Write(output_path);

  return EXIT_SUCCESS;
}

} // namespace colmap_extension