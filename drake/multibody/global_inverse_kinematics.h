#pragma once
#include <vector>

#include "drake/multibody/rigid_body_tree.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace multibody {
class GlobalInverseKinematics : public solvers::MathematicalProgram {
 public:
  GlobalInverseKinematics(std::unique_ptr<RigidBodyTreed> robot);

 private:
  const RigidBodyTree<double> *robot_;

  // body_rotmat_[i] is the rotation matrix for body i.
  std::vector<solvers::MatrixDecisionVariable<3, 3>> body_rotmat_;

  // body_pos_[i] is the position for body i.
  std::vector<solvers::VectorDecisionVariable<3>> body_pos_;
};
}  // namespace multibody
}  // namespace drake
