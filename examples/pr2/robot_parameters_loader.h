#pragma once

#include <string>

#include "drake/examples/pr2/robot_parameters.h"

namespace drake {
namespace examples {
namespace pr2 {

bool ReadParametersFromFile(const std::string& filename,
                            const std::string& filepath_prefix,
                            RobotParameters* parameters = nullptr);

}  // namespace pr2
}  // namespace examples
}  // namespace drake
