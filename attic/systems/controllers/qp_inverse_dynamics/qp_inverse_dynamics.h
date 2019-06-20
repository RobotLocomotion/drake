#pragma once

#include <list>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/systems/controllers/qp_inverse_dynamics/deprecated.h"
#include "drake/systems/controllers/qp_inverse_dynamics/qp_inverse_dynamics_common.h"
#include "drake/systems/controllers/qp_inverse_dynamics/robot_kinematic_state.h"

namespace drake {
namespace systems {
namespace controllers {
namespace qp_inverse_dynamics {

class DRAKE_DEPRECATED_QPID QpInverseDynamics {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(QpInverseDynamics)

  /**
   * Constructor for QpInverseDynamics.
   * @throws std::runtime_error if Gurobi solver is not available.
   */
  QpInverseDynamics();

  /**
   * Computes the generalized acceleration, joint torque and contact wrenches
   * that best tracks the input given the current robot configuration.
   * @param robot_status Robot configuration
   * @param input Desired motions and objectives specified by a higher level
   * controller
   * @param output Container for outputs
   * @return 0 if successful. < 0 if error.
   */
  int Control(const RobotKinematicState<double>& robot_status,
              const QpInput& input, QpOutput* output);

  static const double kUpperBoundForContactBasis;

 private:
  // These are temporary matrices and vectors used by the controller.
  MatrixX<double> tmp_vd_mat_;
  VectorX<double> tmp_vd_vec_;
  MatrixX<double> basis_reg_mat_;
  VectorX<double> basis_reg_vec_;

  MatrixX<double> stacked_contact_jacobians_;
  VectorX<double> stacked_contact_jacobians_dot_times_v_;
  VectorX<double> stacked_contact_velocities_;
  MatrixX<double> basis_to_force_matrix_;

  MatrixX<double> torque_linear_;
  VectorX<double> torque_constant_;
  MatrixX<double> dynamics_linear_;
  VectorX<double> dynamics_constant_;

  MatrixX<double> inequality_linear_;
  VectorX<double> inequality_upper_bound_;
  VectorX<double> inequality_lower_bound_;

  MatrixX<double> JB_;
  VectorX<double> point_forces_;

  MatrixX<double> mass_matrix_;
  VectorX<double> dynamics_bias_;

  Matrix3X<double> J_com_;
  VectorX<double> J_dot_times_v_com_;
  Matrix6X<double> centroidal_momentum_matrix_;
  VectorX<double> centroidal_momentum_matrix_dot_times_v_;

  VectorX<double> solution_;

  std::vector<Matrix6X<double>> body_J_;
  std::vector<Vector6<double>> body_Jdv_;

  // These determines the size of the QP. These are set in ResizeQP
  bool has_floating_base_{true};
  int num_dynamics_equations_{6};
  int num_contact_body_{0};
  int num_vd_{0};
  int num_point_force_{0};
  int num_basis_{0};
  int num_torque_{0};
  int num_variable_{0};
  // One cost / equality constraint term per body motion.
  // For each dimension (row) of the desired body motion, it can be treated
  // as a cost term (Soft), skipped (SKip) or as an equality constraint (Hard)
  // depending on the given constraint type. If it's a Soft constraint, the
  // corresponding weight needs to be positive.
  // E.g. if pelvis's desired body motion has type
  // (Soft, Soft, ,Skip, Skip, Hard, Hard),
  // num_body_motion_as_cost_ and num_body_motion_as_eq_ are both incremented
  // by 1.
  //
  // For soft constraints, the cost term is:
  // weight * (0.5 * vd^T * J(1:2,:)^T * J(1:2,:) * vd
  //           + vd^T * J(1:2,:)^T * (Jdv(1:2,:) - pelvdd_d(1:2)))
  // The equality constraint term is:
  // J(5:6,:) * vd + Jdv(5:6,:) = pelvdd_d(5:6)
  int num_body_motion_as_cost_{0};
  int num_body_motion_as_eq_{0};
  // Same as for body_motiom, replace J with the identity matrix.
  int num_dof_motion_as_cost_{0};
  int num_dof_motion_as_eq_{0};
  int num_cen_mom_dot_as_cost_{0};
  int num_cen_mom_dot_as_eq_{0};
  int num_contact_as_cost_{0};
  int num_contact_as_eq_{0};

  // prog_ is only allocated in ResizeQP, Control only updates the appropriate
  // matrices / vectors.
  std::unique_ptr<drake::solvers::MathematicalProgram> prog_;
  drake::solvers::GurobiSolver solver_;
  drake::solvers::VectorXDecisionVariable basis_;
  drake::solvers::VectorXDecisionVariable vd_;

  // pointers to different cost / constraint terms inside prog_
  drake::solvers::LinearEqualityConstraint* eq_dynamics_{nullptr};
  // TODO(siyuan.feng): Switch to cost for contact_constraints
  std::vector<drake::solvers::LinearEqualityConstraint*> eq_contacts_;
  std::vector<drake::solvers::LinearEqualityConstraint*> eq_body_motion_;
  drake::solvers::LinearEqualityConstraint* eq_dof_motion_{nullptr};
  drake::solvers::LinearEqualityConstraint* eq_cen_mom_dot_{nullptr};

  drake::solvers::LinearConstraint* ineq_contact_wrench_{nullptr};
  drake::solvers::LinearConstraint* ineq_torque_limit_{nullptr};

  std::vector<drake::solvers::QuadraticCost*> cost_contacts_;
  drake::solvers::QuadraticCost* cost_cen_mom_dot_{nullptr};
  std::vector<drake::solvers::QuadraticCost*> cost_body_motion_;
  drake::solvers::QuadraticCost* cost_dof_motion_{nullptr};

  drake::solvers::QuadraticCost* cost_basis_reg_{nullptr};

  // Resize the QP. This resizes the temporary matrices. It also reinitializes
  // prog_ to the correct size, so that Control only updates the
  // matrices and vectors in prog_ instead of making a new one on every call.
  // Size change typically happens when contact state changes (making / breaking
  // contacts).
  void ResizeQP(const RigidBodyTree<double>& robot, const QpInput& input);

  template <typename DerivedA, typename DerivedB>
  void AddAsConstraints(const Eigen::MatrixBase<DerivedA>& A,
                        const Eigen::MatrixBase<DerivedB>& b,
                        const std::list<int>& idx,
                        drake::solvers::LinearEqualityConstraint* eq);

  template <typename DerivedA, typename DerivedB, typename DerivedW>
  void AddAsCosts(const Eigen::MatrixBase<DerivedA>& A,
                  const Eigen::MatrixBase<DerivedB>& b,
                  const Eigen::MatrixBase<DerivedW>& weights,
                  const std::list<int>& idx,
                  drake::solvers::QuadraticCost* cost);

  bool HasFloatingBase(const RigidBodyTree<double>& robot) const;

  // Zeros out the temporary matrices.
  void SetTempMatricesToZero();
};

}  // namespace qp_inverse_dynamics
}  // namespace controllers
}  // namespace systems
}  // namespace drake
