#pragma once

#include <string>

#include "drake/systems/framework/system.h"

namespace drake {
namespace systems {

/** Generates an html string to "render" the @p system, with collapsible
diagrams.  Use @p initial_depth to set the depth to which the subdiagrams are
expanded by default (0 for all collapsed, +∞ for all expanded). 

@ingroup systems
*/
std::string GenerateHtml(const System<double>& system, int initial_depth = 1);

}  // namespace systems
}  // namespace drake
