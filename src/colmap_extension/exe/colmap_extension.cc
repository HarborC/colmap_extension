#include "colmap_extension/exe/sfm.h"
#include "colmap_extension/exe/calibration.h"
#include "colmap/exe/database.h"
#include "colmap/util/version.h"

namespace {

typedef std::function<int(int, char**)> command_func_t;

int ShowHelp(
    const std::vector<std::pair<std::string, command_func_t>>& commands) {
  std::cout << colmap::GetVersionInfo()
            << " -- COLMAP Extension — extra SfM/MVS utilities\n("
            << colmap::GetBuildInfo() << ")\n\n";

  std::cout << "Usage:\n";
  std::cout << "  colmap_extension [command] [options]\n";

  std::cout << "Example usage:\n";
  std::cout << "  colmap_extension help [ -h, --help ]\n";
  std::cout << "  ...\n";

  std::cout << "Available commands:\n";
  std::cout << "  help\n";
  for (const auto& command : commands) {
    std::cout << "  " << command.first << '\n';
  }
  std::cout << '\n';

  return EXIT_SUCCESS;
}

}  // namespace

int main(int argc, char** argv) {
  colmap::InitializeGlog(argv);

  std::vector<std::pair<std::string, command_func_t>> commands;
  commands.emplace_back("calibration_bundle_adjuster", &colmap_extension::RunCalibBundleAdjuster);
  commands.emplace_back("calibration_detection", &colmap_extension::RunCalibBoardDetection);

  if (argc == 1) {
    return ShowHelp(commands);
  }

  const std::string command = argv[1];
  if (command == "help" || command == "-h" || command == "--help") {
    return ShowHelp(commands);
  } else {
    command_func_t matched_command_func = nullptr;
    for (const auto& command_func : commands) {
      if (command == command_func.first) {
        matched_command_func = command_func.second;
        break;
      }
    }
    if (matched_command_func == nullptr) {
      LOG(ERROR) << colmap::StringPrintf(
          "Command `%s` not recognized. To list the "
          "available commands, run `colmap_extension help`.",
          command.c_str());
      return EXIT_FAILURE;
    } else {
      int command_argc = argc - 1;
      char** command_argv = &argv[1];
      command_argv[0] = argv[0];
      return matched_command_func(command_argc, command_argv);
    }
  }

  return ShowHelp(commands);
}
