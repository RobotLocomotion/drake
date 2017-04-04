#pragma once

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace pick_and_place {

// These constants are used in several locations in the Monolithic
// pick and place demo.
const char kIiwaUrdf[] =
    "/manipulation/models/iiwa_description/urdf/"
    "iiwa14_polytope_collision.urdf";
const char kIiwaEndEffectorName[] = "iiwa_link_ee";

}  // namespace pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
