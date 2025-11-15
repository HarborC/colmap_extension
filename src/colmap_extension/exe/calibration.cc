#include "calibration.h"
#include "colmap/controllers/option_manager.h"
#include "colmap/util/file.h"
#include "colmap/util/misc.h"
#include "colmap/scene/reconstruction.h"

#include "calibration_utils/calib_board_factory.h"

using namespace colmap;

namespace colmap_extension {

int RunCalibBoardDetection(int argc, char** argv) {
    std::string image_path;
    std::string detection_path;
    std::string board_config_path;
    std::string debug_image_path = "";

    OptionManager options;
    options.AddRequiredOption("image_path", &image_path);
    options.AddRequiredOption("board_config_path", &board_config_path);
    options.AddRequiredOption("detection_path", &detection_path);
    options.AddDefaultOption("debug_image_path", &debug_image_path);
    options.Parse(argc, argv);

    auto board = calibration_utils::CalibBoardFactory::FromConfig(board_config_path);

    if (!board) {
        LOG(ERROR) << "Failed to create calibration board from config: " << board_config_path;
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

int RunCalibBundleAdjuster(int argc, char** argv) {
    std::string input_path;
    std::string output_path;

    OptionManager options;
    options.AddDatabaseOptions();
    options.AddRequiredOption("input_path", &input_path);
    options.AddRequiredOption("output_path", &output_path);
    options.AddBundleAdjustmentOptions();
    options.Parse(argc, argv);

    if (!ExistsDir(input_path)) {
        LOG(ERROR) << "`input_path` is not a directory";
        return EXIT_FAILURE;
    }

    auto reconstruction = std::make_shared<Reconstruction>();
    reconstruction->Read(input_path);

    return EXIT_SUCCESS;
}

}  // namespace colmap_extension