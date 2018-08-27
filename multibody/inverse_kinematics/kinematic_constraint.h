#pragma once

#include "drake/multibody/multibody_tree/multibody_tree.h"
#include "drake/solvers/constraint.h"

namespace drake {
namespace multibody {
namespace inverse_kinematics {
// Constrains the position of a point Q, rigidly attached to a frame A, to be within a bounding box measured and expressed in frame B. Namely lb <= p_BQ <= ub.
class PositionConstraint : solvers::Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PositionConstraint)

  PositionConstraint(const multibody_tree::MultibodyTree& tree, int frameA_idx
};
}  // namespace inverse_kinematics
}  // namespace multibody
}  // namespace drake
