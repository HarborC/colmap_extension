#include "calib_board.h"
#include "aprilgrid/aprilgrid.h"
#include "serialization_registration.h"

#include <cereal/archives/binary.hpp> 
#include <cereal/types/polymorphic.hpp>

// Register derived types for polymorphic serialization
CEREAL_REGISTER_TYPE(calibration_utils::AprilGridBoard)
CEREAL_REGISTER_POLYMORPHIC_RELATION(calibration_utils::CalibBoard, calibration_utils::AprilGridBoard)

CEREAL_REGISTER_TYPE(calibration_utils::AprilGridBoardDetection)
CEREAL_REGISTER_POLYMORPHIC_RELATION(calibration_utils::CalibDetection, calibration_utils::AprilGridBoardDetection)

namespace calibration_utils {
void EnsureCerealRegistration() {}
} // namespace calibration_utils
