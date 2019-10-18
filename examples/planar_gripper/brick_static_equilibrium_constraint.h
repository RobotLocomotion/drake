#pragma once

#include <utility>
#include <vector>

#include "drake/examples/planar_gripper/gripper_brick.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace examples {
namespace planar_gripper {
/** Given the set of contacts between the fingertips and the brick, impose the
 * static equilibrium as a nonlinear constraint, that the total force/torque
 * applied on the brick is 0. The decision variables are (q, f_Cb_B) where q is
 * the generalized position, and f_Cb_B are the contact force (y, z) component
 * applied to the point Cb on the brick, expressed in the brick frame B.
 * The constraint are
 *
 *     0 = R_BW * m * g_W +  ∑ᵢ f_Cbi_B
 *     0 = ∑ᵢp_Cbi_B.cross(f_Cbi_B)
 * Notice that since the gripper/brick system is planar, the first equation is
 * only on the (y, z) component of the total force, and the second equation is
 * only on the x component of the torque.
 *
 * The constraint evaluates the right-hand side of the two equations above.
 */
class BrickStaticEquilibriumNonlinearConstraint : public solvers::Constraint {
 public:
  BrickStaticEquilibriumNonlinearConstraint(
      const GripperBrickHelper<double>& gripper_brick_system,
      std::vector<std::pair<Finger, BrickFace>> finger_face_contacts,
      systems::Context<double>* plant_mutable_context);

 private:
  template<typename T>
  void DoEvalGeneric(const Eigen::Ref<const VectorX<T>> &x,
                     VectorX<T> *y) const;

  void DoEval(const Eigen::Ref<const Eigen::VectorXd> &x,
              Eigen::VectorXd *y) const;

  void DoEval(const Eigen::Ref<const AutoDiffVecXd> &x, AutoDiffVecXd *y) const;

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>> &,
              VectorX<symbolic::Expression> *) const;

  const GripperBrickHelper<double>& gripper_brick_system_;
  double brick_mass_;
  std::vector<std::pair<Finger, BrickFace>> finger_face_contacts_;
  systems::Context<double> *plant_mutable_context_;
};

/**
 * Given the assignment of contacts between fingers and faces, add the following
 * constraints/variables to the program:
 * 1. Decision variables representing the finger contact force on the brick,
 *    expressed in the brick frame.
 * 2. The static equilibrium constraint ensuring that the total wrench on the
 *    brick is 0.
 * 3. The constraint that the contact force is within a friction cone.
 */
Matrix2X<symbolic::Variable> AddBrickStaticEquilibriumConstraint(
    const GripperBrickHelper<double>& gripper_brick_system,
    const std::vector<std::pair<Finger, BrickFace>>& finger_face_contacts,
    const Eigen::Ref<const VectorX<symbolic::Variable>>& q_vars,
    systems::Context<double>* plant_mutable_context,
    solvers::MathematicalProgram* prog);
}  // namespace planar_gripper
}  // namespace examples
}  // namespace drake
