#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/kinematics_cache.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace manipulation {

struct QuasistaticSystemOptions {
  double period_sec{0.01};
  std::vector<int> idx_unactuated_bodies{};
  int idx_base;  // base body (has a floating joint to the world) of
  // the unactuated rigid body mechanism.
  std::vector<int> fixed_base_positions{};
  std::vector<int> fixed_base_velocities{};
  std::vector<bool> is_contact_2d{};
  double mu{1};  // coefficent of friction for all contacts
  double kBigM{1};
  bool is_analytic{false};
  bool is_using_kinetic_energy_minimizing_QP{false};
};

template <class Scalar>
class QuasistaticSystem : public systems::LeafSystem<Scalar> {
 public:
  explicit QuasistaticSystem(const QuasistaticSystemOptions& options);

  const systems::OutputPort<Scalar>& state_output() const;
  const systems::OutputPort<Scalar>& decision_variables_output() const;
  double get_period_sec() { return period_sec_; }

 protected:
  void Initialize();
  void DoCalcDiscreteVariableUpdates(
      const systems::Context<Scalar>& context,
      const std::vector<const systems::DiscreteUpdateEvent<Scalar>*>&,
      systems::DiscreteValues<Scalar>* discrete_state) const override;
  VectorX<Scalar> GetQuasistaticSystemStatesFromRigidBodyTreePositions(
      const KinematicsCache<Scalar>& cache) const;
  VectorX<Scalar> GetRigidBodyTreePositionsFromQuasistaticSystemStates(
      const Eigen::Ref<const VectorX<Scalar>>& q_quasistatic_system) const;

  // Wn: wrench matrix corresponding to normal forces (lambda_n).
  // Wf: wrench matrix corresponding to friction forces (lambda_f).
  // Jn: Jacobian corresponding to normal velocities.
  // Jf: Jacobian corresponding to tangent velocities.
  // phi: vector of signed distance functions for all contact pairs.

  void CalcJf(const Eigen::Ref<const MatrixX<Scalar>>& Jf_half,
              MatrixX<Scalar>* const Jf_ptr) const;
  void CalcWnWfJnJfPhi(const KinematicsCache<Scalar>& cache,
                       MatrixX<Scalar>* const Wn_ptr,
                       MatrixX<Scalar>* const Wf_ptr,
                       MatrixX<Scalar>* const Jn_ptr,
                       MatrixX<Scalar>* const Jf_ptr,
                       VectorX<Scalar>* const phi_ptr) const;
  void DoCalcWnWfJnJfPhi(const KinematicsCache<Scalar>& cache,
                         MatrixX<Scalar>* const Wn_ptr,
                         MatrixX<Scalar>* const Wf_ptr,
                         MatrixX<Scalar>* const Jn_ptr,
                         MatrixX<Scalar>* const Jf_ptr,
                         VectorX<Scalar>* const phi_ptr) const;
  Scalar CalcBigM(Scalar max_impulse, Scalar max_delta_q, Scalar max_gamma,
                  const Eigen::Ref<const MatrixX<Scalar>>& Jn,
                  const Eigen::Ref<const MatrixX<Scalar>>& Jf,
                  const Eigen::Ref<const VectorX<Scalar>>& phi,
                  const Eigen::Ref<const VectorX<Scalar>>& qa_dot_d) const;
  virtual void DoCalcWnWfJnJfPhiAnalytic(const KinematicsCache<Scalar>& cache,
                                         MatrixX<Scalar>* const Wn_ptr,
                                         MatrixX<Scalar>* const Wf_ptr,
                                         MatrixX<Scalar>* const Jn_ptr,
                                         MatrixX<Scalar>* const Jf_ptr,
                                         VectorX<Scalar>* const phi_ptr) const;
  MatrixX<Scalar> CalcE() const;
  VectorX<Scalar> CalcExternalGeneralizedForce(
      KinematicsCache<Scalar>* const cache) const;

  void CopyStateOut(const systems::Context<Scalar>& context,
                    systems::BasicVector<Scalar>* output) const {
    output->SetFromVector(GetRigidBodyTreePositionsFromQuasistaticSystemStates(
        context.get_discrete_state(0).CopyToVector().head(n1_)));
  }

  void CopyDecisionVariablesOut(const systems::Context<Scalar>& context,
                                systems::BasicVector<Scalar>* output) const {
    output->SetFromVector(
        context.get_discrete_state(0).CopyToVector().segment(n1_, n_));
  }

  const double period_sec_;
  // indices of unactuated bodies into the RigidBodyTree. The force balance of
  // these bodies are constraints in the MIQP.
  std::vector<int> idx_unactuated_bodies_;
  // index (into rigidbodytree) of the base link of the unactuated mechanism.
  int idx_base_;
  // DoFs of the floating joint of the base body of the unactuated rigid body
  // mechanisms that are fixed.
  const std::vector<int> fixed_base_positions_;
  // When base roatation is parametrized by a quaternion, fixing components of
  // the
  // quaternion doesn't make much sense. Angular velocities in body frame are
  // fixed instead.
  std::vector<int> fixed_base_velocities_;

  // Each entry indicates if a contact is 2-dimensional, i.e. friction force
  // spanned only by 2 vectors.
  std::vector<bool> is_contact_2d_;

  const double mu_;
  // Gives users the option to specify a big M.
  // Currently this value is overwritten by a value calculated with interval
  // arithmetic based on bounds of all decision variables.
  const double kBigM_;
  // true if analytic expressions are available for Jn and Jf.
  const bool is_analytic_;
  const bool is_using_kinetic_energy_minimizing_QP_;

  int nu_{0};           // number of underactuated DOFs, dim(qu).
  int n_vu_{0};         // number of underactuated velocities.
  int na_{0};           // number of actuated DOFs (inputs), dim(qa)
  int nc_;              // number of contacts.
  int nq_tree_;         // number of positions in RBT
  Eigen::VectorXi nf_;  // nubmer of vectors spanning each friction cone

  // indices of actuated states in increasing order.
  std::vector<int> idx_qa_;
  // columns of "contactJacobian" corresponding to qa.
  std::vector<int> idx_qa_in_q_;

  // indices of unactuated states of RigidBodyTree in increasing order
  // also the columns of "contactJacobian" corresponding to qu.
  std::vector<int> idx_qu_;
  std::vector<int> idx_qu_in_q_;

  std::vector<int> idx_vu_;
  bool is_base_quaternion_;

  // indices of all states in increasing order
  // columns of "contactJacobian" that are used in the MIQP.
  std::vector<int> idx_q_;

  int n1_;  // = nu_ + na_  dimension of discrete state q = [qu; qa]
  int nd_;  // defined in QuasistaticSystem::UpdateNs()
  int n2_;  // defined in QuasistaticSystem::UpdateNs()
  int n_;   // number of decision variables in the MIQP

  // rigid body tree for model
  std::unique_ptr<RigidBodyTree<Scalar>> tree_ =
      std::make_unique<RigidBodyTree<Scalar>>();

 private:
  void UpdateNs() {
    // creates nf_ from is_contact_2d_
    nf_.resize(nc_);
    for (int i = 0; i < nc_; i++) {
      if (is_contact_2d_[i]) {
        nf_(i) = 2;
      } else {
        nf_(i) = 4;
      }
    }
    nu_ = idx_qu_.size();
    n_vu_ = idx_vu_.size();
    na_ = idx_qa_.size();
    n1_ = nu_ + na_;
    nd_ = nf_.sum();
    n2_ = nc_ * 2 + nd_;
    n_ = n1_ + 2 * n2_;
  }

  // initial guesses
  std::unique_ptr<drake::VectorX<Scalar>> lambda_n_start_ =
      std::make_unique<drake::VectorX<Scalar>>();
  std::unique_ptr<drake::VectorX<Scalar>> lambda_f_start_ =
      std::make_unique<drake::VectorX<Scalar>>();
  std::unique_ptr<drake::VectorX<Scalar>> gamma_start_ =
      std::make_unique<drake::VectorX<Scalar>>();
  std::unique_ptr<drake::VectorX<Scalar>> z_n_start_ =
      std::make_unique<drake::VectorX<Scalar>>();
  std::unique_ptr<drake::VectorX<Scalar>> z_f_start_ =
      std::make_unique<drake::VectorX<Scalar>>();
  std::unique_ptr<drake::VectorX<Scalar>> z_gamma_start_ =
      std::make_unique<drake::VectorX<Scalar>>();

  // "hypothetical penetration"
  std::unique_ptr<drake::VectorX<Scalar>> phi_bar_ =
      std::make_unique<drake::VectorX<Scalar>>();

  // MIQP program
  std::unique_ptr<solvers::MathematicalProgram> prog_;
  solvers::GurobiSolver solver_;

  solvers::VectorXDecisionVariable delta_q_;
  solvers::VectorXDecisionVariable lambda_n_;
  solvers::VectorXDecisionVariable lambda_f_;
  solvers::VectorXDecisionVariable gamma_;
  solvers::VectorXDecisionVariable z_n_;
  solvers::VectorXDecisionVariable z_f_;
  solvers::VectorXDecisionVariable z_gamma_;

  solvers::LinearConstraint* bounds_delta_q_{nullptr};
  solvers::LinearConstraint* bounds_gamma_{nullptr};
  solvers::LinearConstraint* bounds_lambda_n_{nullptr};
  solvers::LinearConstraint* bounds_lambda_f_{nullptr};
  solvers::LinearEqualityConstraint* force_balance_{nullptr};
  solvers::LinearConstraint* non_penetration_{nullptr};
  solvers::LinearConstraint* coulomb_friction1_{nullptr};
  solvers::LinearConstraint* coulomb_friction2_{nullptr};
  solvers::LinearConstraint* non_penetration_complementary_{nullptr};
  solvers::LinearConstraint* coulomb_friction1_complementary_{nullptr};
  solvers::LinearConstraint* coulomb_friction2_complementary_{nullptr};
  solvers::LinearConstraint* decision_variables_complementary_{nullptr};

  solvers::QuadraticCost* objective_{nullptr};

  void StepForward(
      const drake::MatrixX<Scalar>& Wn, const drake::MatrixX<Scalar>& Wf,
      const drake::MatrixX<Scalar>& Jn, const drake::MatrixX<Scalar>& Jf,
      const drake::MatrixX<Scalar>& U, const drake::MatrixX<Scalar>& E,
      const drake::VectorX<Scalar>& phi, const drake::VectorX<Scalar>& f,
      const drake::VectorX<Scalar>& qa_dot_d) const;
};

}  // namespace manipulation
}  // namespace drake
