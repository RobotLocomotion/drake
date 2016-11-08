#pragma once

#include "drake/examples/QPInverseDynamicsForHumanoids/qp_controller.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

/**
 * Helper function to make an example QPInput.
 */
QPInput MakeExampleQPInput(const RigidBodyTree<double>& robot);

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
