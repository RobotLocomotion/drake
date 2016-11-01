#include <string>
#include <vector>

#include "drake/systems/plants/RigidBodyIK.h"
#include "drake/systems/plants/RigidBodyTree.h"
#include "drake/systems/plants/inverseKinBackend.h"

using Eigen::Map;
using Eigen::MatrixBase;
using Eigen::MatrixXd;
using Eigen::VectorXd;

using drake::systems::plants::inverseKinTrajBackend;

template <typename DerivedA, typename DerivedB, typename DerivedC,
          typename DerivedD, typename DerivedE, typename DerivedF>
DRAKE_EXPORT void inverseKinTraj(
    RigidBodyTree* model, const int nT, const double* t,
    const MatrixBase<DerivedA>& qdot0_seed, const MatrixBase<DerivedB>& q_seed,
    const MatrixBase<DerivedC>& q_nom, const int num_constraints,
    const RigidBodyConstraint* const* constraint_array,
    const IKoptions& ikoptions_in,
    MatrixBase<DerivedD>* q_sol, MatrixBase<DerivedE>* qdot_sol,
    MatrixBase<DerivedF>* qddot_sol, int* info,
    std::vector<std::string>* infeasible_constraint) {
  IKoptions ikoptions = ikoptions_in;
  if (ikoptions.getFixInitialState()) {
    ikoptions.setqd0(qdot0_seed, qdot0_seed);
  }
  inverseKinTrajBackend(model, nT, t, q_seed, q_nom, num_constraints,
                        constraint_array, ikoptions, q_sol, qdot_sol, qddot_sol,
                        info, infeasible_constraint);
}

template DRAKE_EXPORT void inverseKinTraj(
    RigidBodyTree* model, const int nT, const double* t,
    const MatrixBase<Map<VectorXd>>& qdot0_seed,
    const MatrixBase<Map<MatrixXd>>& q_seed,
    const MatrixBase<Map<MatrixXd>>& q_nom, const int num_constraints,
    const RigidBodyConstraint* const* constraint_array,
    const IKoptions& ikoptions,
    MatrixBase<Map<MatrixXd>>* q_sol, MatrixBase<Map<MatrixXd>>* qdot_sol,
    MatrixBase<Map<MatrixXd>>* qddot_sol, int* info,
    std::vector<std::string>* infeasible_constraint);
template DRAKE_EXPORT void inverseKinTraj(
    RigidBodyTree* model, const int nT, const double* t,
    const MatrixBase<VectorXd>& qdot0_seed, const MatrixBase<MatrixXd>& q_seed,
    const MatrixBase<MatrixXd>& q_nom, const int num_constraints,
    const RigidBodyConstraint* const* constraint_array,
    const IKoptions& ikoptions, MatrixBase<MatrixXd>* q_sol,
    MatrixBase<MatrixXd>* qdot_sol, MatrixBase<MatrixXd>* qddot_sol,
    int* info, std::vector<std::string>* infeasible_constraint);

DRAKE_EXPORT IKResults inverseKinTrajSimple(
    RigidBodyTree* model,
    const Eigen::VectorXd& t,
    const Eigen::MatrixXd& q_seed,
    const Eigen::MatrixXd& q_nom,
    const std::vector<RigidBodyConstraint*>& constraint_array,
    const IKoptions& ikoptions) {

  Eigen::MatrixXd q_sol_mat(model->get_num_positions(), t.size());
  q_sol_mat.fill(0);
  IKResults results;
  results.info.resize(1, 0);

  Eigen::MatrixXd qdot_sol_dummy(model->get_num_positions(), t.size());
  Eigen::MatrixXd qddot_sol_dummy(model->get_num_positions(), t.size());

  inverseKinTrajBackend(
      model, t.size(), t.data(), q_seed, q_nom,
      constraint_array.size(), constraint_array.data(),
      ikoptions, &q_sol_mat, &qdot_sol_dummy, &qddot_sol_dummy,
      results.info.data(), &results.infeasible_constraints);
  results.q_sol.resize(t.size());
  for (int i = 0; i < t.size(); i++) {
    results.q_sol[i] = q_sol_mat.col(i);
  }
  return results;
}
