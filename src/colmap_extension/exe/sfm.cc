#include "colmap_extension/exe/sfm.h"

#include "colmap/controllers/automatic_reconstruction.h"
#include "colmap/controllers/bundle_adjustment.h"
#include "colmap/controllers/hierarchical_pipeline.h"
#include "colmap/controllers/option_manager.h"
#include "colmap/estimators/similarity_transform.h"
#include "colmap/exe/gui.h"
#include "colmap/scene/reconstruction.h"
#include "colmap/sfm/observation_manager.h"
#include "colmap/util/file.h"
#include "colmap/util/misc.h"
#include "colmap/util/opengl_utils.h"

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

using namespace colmap;

namespace colmap_extension {

void UpdateDatabasePosePriorsCovariance(const std::string& database_path,
                                        const Eigen::Matrix3d& covariance) {
  Database database(database_path);
  DatabaseTransaction database_transaction(&database);

  LOG(INFO)
      << "Setting up database pose priors with the same covariance matrix: \n"
      << covariance << "\n";

  for (const auto& image : database.ReadAllImages()) {
    if (database.ExistsPosePrior(image.ImageId())) {
      PosePrior prior = database.ReadPosePrior(image.ImageId());
      prior.position_covariance = covariance;
      database.UpdatePosePrior(image.ImageId(), prior);
    }
  }
}

int RunBundleAdjusterWithDB(int argc, char** argv) {
  std::string input_path;
  std::string output_path;
  bool use_prior_position = false;

  OptionManager options;
  options.AddDatabaseOptions();
  options.AddRequiredOption("input_path", &input_path);
  options.AddRequiredOption("output_path", &output_path);
  options.AddDefaultOption("use_prior_position", &use_prior_position);
  options.AddBundleAdjustmentOptions();
  options.Parse(argc, argv);

  if (!ExistsDir(input_path)) {
    LOG(ERROR) << "`input_path` is not a directory";
    return EXIT_FAILURE;
  }

  if (!ExistsDir(output_path)) {
    LOG(ERROR) << "`output_path` is not a directory";
    return EXIT_FAILURE;
  }

  auto reconstruction = std::make_shared<Reconstruction>();
  reconstruction->Read(input_path);

  BundleAdjustmentController ba_controller(options, reconstruction);
//   ba_controller.RunWithDB(use_prior_position);

  reconstruction->Write(output_path);

  return EXIT_SUCCESS;
}

}  // namespace colmap
