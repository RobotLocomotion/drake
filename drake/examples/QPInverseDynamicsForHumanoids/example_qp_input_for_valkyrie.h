#pragma once

#include "drake/examples/QPInverseDynamicsForHumanoids/qp_controller.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

QPInput MakeExampleQPInput(const RigidBodyTree& robot);

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
