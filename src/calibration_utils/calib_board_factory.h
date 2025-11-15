#pragma once

#include <memory>
#include <string>

#include "calib_board.h"

namespace calibration_utils {

// Factory for creating CalibBoard instances from a config file.
// Currently supports AprilGrid via AprilGridBoard.
class CalibBoardFactory {
public:
  // Create a board from a config file. If id is empty, the factory will try to
  // read it from the config or derive it from the filename.
  static CalibBoard::Ptr FromConfig(const std::string &config_path,
                                    const CalibBoard::ID &id = "");
};

} // namespace calibration_utils
