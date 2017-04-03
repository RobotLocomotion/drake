# pragma once

# include <vector>

#include "drake/common/drake_path.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace pick_and_place {

// These constants are used in several locations in the Monolithic
// pick and place demo.
const std::string kIiwaUrdf =
    "/manipulation/models/iiwa_description/urdf/"
        "iiwa14_primitive_collision.urdf";
const std::string kIiwaEndEffectorName = "iiwa_link_ee";

}  // namespace pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
