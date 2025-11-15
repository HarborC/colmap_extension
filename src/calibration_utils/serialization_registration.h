#pragma once

namespace calibration_utils {

// Ensures the translation unit with CEREAL registration is linked
// and its static initializers have a chance to run.
void EnsureCerealRegistration();

} // namespace calibration_utils
