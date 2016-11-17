#include <string>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/multibody/inverse_kinematics_backend.h"
#include "drake/multibody/rigid_body_ik.h"
#include "drake/multibody/rigid_body_tree.h"

using Eigen::Map;
using Eigen::MatrixBase;
using Eigen::MatrixXd;

using drake::systems::plants::inverseKinBackend;

template <typename DerivedA, typename DerivedB, typename DerivedC>
void inverseKinPointwise(
    RigidBodyTree<double>* model, const int nT, const double* t,
    const MatrixBase<DerivedA>& q_seed, const MatrixBase<DerivedB>& q_nom,
    const int num_constraints,
    const RigidBodyConstraint* const* constraint_array,
    const IKoptions& ikoptions, MatrixBase<DerivedC>* q_sol, int* info,
    std::vector<std::string>* infeasible_constraint) {
  inverseKinBackend(model, nT, t, q_seed, q_nom, num_constraints,
                    constraint_array, ikoptions, q_sol,
                    info, infeasible_constraint);
}

template void inverseKinPointwise(
    RigidBodyTree<double>* model, const int nT, const double* t,
    const MatrixBase<Map<MatrixXd>>& q_seed,
    const MatrixBase<Map<MatrixXd>>& q_nom, const int num_constraints,
    const RigidBodyConstraint* const* constraint_array,
    const IKoptions& ikoptions, MatrixBase<Map<MatrixXd>>* q_sol, int *info,
    std::vector<std::string>* infeasible_constraint);
template void inverseKinPointwise(
    RigidBodyTree<double>* model, const int nT, const double* t,
    const MatrixBase<MatrixXd>& q_seed, const MatrixBase<MatrixXd>& q_nom,
    const int num_constraints,
    const RigidBodyConstraint* const* constraint_array,
    const IKoptions& ikoptions, MatrixBase<MatrixXd>* q_sol, int* info,
    std::vector<std::string>* infeasible_constraint);

IKResults inverseKinPointwiseSimple(
    RigidBodyTree<double>* model,
    const Eigen::VectorXd& t,
    const Eigen::MatrixXd& q_seed,
    const Eigen::MatrixXd& q_nom,
    const std::vector<RigidBodyConstraint*>& constraint_array,
    const IKoptions& ikoptions) {

  Eigen::MatrixXd q_sol_mat(model->get_num_positions(), t.size());
  q_sol_mat.fill(0);
  IKResults results;
  results.info.resize(t.size(), 0);
  inverseKinBackend(
      model, t.size(), t.data(), q_seed, q_nom,
      constraint_array.size(), constraint_array.data(),
      ikoptions, &q_sol_mat, results.info.data(),
      &results.infeasible_constraints);
  results.q_sol.resize(t.size());
  for (int i = 0; i < t.size(); i++) {
    results.q_sol[i] = q_sol_mat.col(i);
  }
  return results;
}
