#pragma once

#include <string>

#include "drake/common/drake_deprecated.h"
#include "drake/systems/framework/system.h"

namespace drake {
namespace systems {

DRAKE_DEPRECATED("2024-01-01", "This feature has been removed from Drake.")
std::string GenerateHtml(const System<double>& system, int initial_depth = 1);

}  // namespace systems
}  // namespace drake
