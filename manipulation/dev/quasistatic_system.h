#pragma once

#include <memory>
#include <vector>
#include "drake/common/drake_assert.h"
#include "drake/multibody/kinematics_cache.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace manipulation {

using Eigen::Matrix;
using Eigen::Matrix3Xd;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::VectorXi;
using drake::solvers::LinearConstraint;
using drake::solvers::LinearEqualityConstraint;
using drake::solvers::MathematicalProgram;
using drake::solvers::QuadraticCost;
using drake::solvers::VectorXDecisionVariable;
using systems::BasicVector;

class QuasistaticSystem : public drake::systems::LeafSystem<double> {
 public:
  QuasistaticSystem(
      const double period_sec, const std::vector<int>& idx_unactuated_bodies,
      const int idx_base,  // base body (has a floating joint to the world) of
      // the unactuated rigid body mechanism.
      const std::vector<int>& fixed_base_positions = {},
      const std::vector<int>& fixed_base_velocities = {},
      const std::vector<bool>& is_contact_2d = {},
      const double mu = 1,  // coefficent of friction for all contacts
      const double kBigM = 1, const bool is_analytic = false,
      const bool is_using_kinetic_energy_minimizing_QP = true);

  void Initialize();
  const systems::OutputPort<double>& state_output() const;
  const systems::OutputPort<double>& decision_variables_output() const;

 protected:
  void DoCalcDiscreteVariableUpdates(
      const drake::systems::Context<double>& context,
      const std::vector<const drake::systems::DiscreteUpdateEvent<double>*>&,
      drake::systems::DiscreteValues<double>* discrete_state) const override;

  VectorXd getQuasistaticSystemStatesFromRigidBodyTreePositions(
      const KinematicsCache<double>& cache) const;
  VectorXd getRigidBodyTreePositionsFromQuasistaticSystemStates(
      const VectorXd q_quasistatic_system) const;
  void CalcJf(const KinematicsCache<double>& cache, const MatrixXd& Jf_half,
              MatrixXd& Jf) const;
  void CalcWnWfJnJfPhi(const KinematicsCache<double>& cache, MatrixXd& Wn,
                       MatrixXd& Wf, MatrixXd& Jn, MatrixXd& Jf,
                       VectorXd& phi) const;
  void DoCalcWnWfJnJfPhi(const KinematicsCache<double>& cache, MatrixXd& Wn,
                         MatrixXd& Wf, MatrixXd& Jn, MatrixXd& Jf,
                         VectorXd& phi) const;
  double CalcBigM(const double max_impulse, const double max_delta_q,
                  const double max_delta_gamma, const MatrixXd& Jn,
                  const MatrixXd& Jf, const VectorXd& phi, const MatrixXd& E,
                  const MatrixXd& U, const VectorXd qa_dot_d) const;
  virtual void DoCalcWnWfJnJfPhiAnalytic(const KinematicsCache<double>& cache,
                                         MatrixXd& Wn, MatrixXd& Wf,
                                         MatrixXd& Jn, MatrixXd& Jf,
                                         VectorXd& phi) const;
  virtual MatrixXd CalcE() const;
  virtual VectorXd CalcExternalGeneralizedForce(
      KinematicsCache<double>& cache) const;

  void CopyStateOut(const drake::systems::Context<double>& context,
                    drake::systems::BasicVector<double>* output) const {
    output->SetFromVector(getRigidBodyTreePositionsFromQuasistaticSystemStates(
        context.get_discrete_state(0).CopyToVector().head(n1_)));
  }

  void CopyDecisionVariablesOut(
      const drake::systems::Context<double>& context,
      drake::systems::BasicVector<double>* output) const {
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

  int nu_{0};    // number of underactuated DOFs, dim(qu).
  int n_vu_{0};  // number of underactuated velocities.
  int na_{0};    // number of actuated DOFs (inputs), dim(qa)
  int nc_;       // number of contacts.
  int nq_tree_;  // number of positions in RBT
  VectorXi nf_;  // nubmer of vectors spanning each friction cone

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
  std::unique_ptr<RigidBodyTreed> tree_ = std::make_unique<RigidBodyTreed>();

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
  std::unique_ptr<VectorXd> lambda_n_start_ = std::make_unique<VectorXd>();
  std::unique_ptr<VectorXd> lambda_f_start_ = std::make_unique<VectorXd>();
  std::unique_ptr<VectorXd> gamma_start_ = std::make_unique<VectorXd>();
  std::unique_ptr<VectorXd> z_n_start_ = std::make_unique<VectorXd>();
  std::unique_ptr<VectorXd> z_f_start_ = std::make_unique<VectorXd>();
  std::unique_ptr<VectorXd> z_gamma_start_ = std::make_unique<VectorXd>();

  // "hypothetical penetration"
  std::unique_ptr<VectorXd> phi_bar_ = std::make_unique<VectorXd>();

  // MIQP program
  std::unique_ptr<MathematicalProgram> prog_;
  drake::solvers::GurobiSolver solver_;

  VectorXDecisionVariable delta_q_;
  VectorXDecisionVariable lambda_n_;
  VectorXDecisionVariable lambda_f_;
  VectorXDecisionVariable gamma_;
  VectorXDecisionVariable z_n_;
  VectorXDecisionVariable z_f_;
  VectorXDecisionVariable z_gamma_;

  LinearConstraint* bounds_delta_q_{nullptr};
  LinearConstraint* bounds_gamma_{nullptr};
  LinearConstraint* bounds_lambda_n_{nullptr};
  LinearConstraint* bounds_lambda_f_{nullptr};

  LinearEqualityConstraint* force_balance_{nullptr};

  LinearConstraint* non_penetration_{nullptr};
  LinearConstraint* coulomb_friction1_{nullptr};
  LinearConstraint* coulomb_friction2_{nullptr};
  LinearConstraint* non_penetration_complementary_{nullptr};
  LinearConstraint* coulomb_friction1_complementary_{nullptr};
  LinearConstraint* coulomb_friction2_complementary_{nullptr};
  LinearConstraint* decision_variables_complementary_{nullptr};

  solvers::QuadraticCost* objective_{nullptr};

  void StepForward(const MatrixXd& Wn, const MatrixXd& Wf, const MatrixXd& Jn,
                   const MatrixXd& Jf, const MatrixXd& U, const MatrixXd& E,
                   const VectorXd& phi, const VectorXd& f,
                   const VectorXd& qa_dot_d) const;
};

}  // namespace manipulation
}  // namespace drake
