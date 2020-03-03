#pragma once

#include <string>

#include "drake/examples/hsr/parameters/robot_parameters.h"

namespace drake {
namespace examples {
namespace hsr {
namespace parameters {

bool ReadParametersFromFile(
    const std::string& filename,
    const std::string& filepath_prefix = "drake/examples/hsr/models/config/",
    RobotParameters<double>* parameters = nullptr);

}  // namespace parameters
}  // namespace hsr
}  // namespace examples
}  // namespace drake
