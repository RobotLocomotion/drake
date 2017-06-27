#pragma once

#include "drake/multibody/rigid_body_tree.h"

#include "sdf/sdf.hh"

namespace drake {
namespace examples {
namespace double_pendulum {

void ParseModel(const sdf::ElementPtr& model,
                RigidBodyTree<double>* tree);

}  // namespace double_pendulum
}  // namespace examples
}  // namespace drake

