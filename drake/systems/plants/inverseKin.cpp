#include <string>
#include <vector>

#include "drake/systems/plants/RigidBodyIK.h"
#include "drake/systems/plants/RigidBodyTree.h"
#include "drake/systems/plants/inverseKinBackend.h"

using Eigen::Map;
using Eigen::MatrixBase;
using Eigen::VectorXd;

using drake::systems::plants::inverseKinBackend;

template <typename DerivedA, typename DerivedB, typename DerivedC>
DRAKE_EXPORT void inverseKin(
    RigidBodyTree *model,
    const MatrixBase<DerivedA>& q_seed,
    const MatrixBase<DerivedB>& q_nom,
    const int num_constraints,
    const RigidBodyConstraint* const* constraint_array,
    const IKoptions& ikoptions,
    MatrixBase<DerivedC>* q_sol, int* info,
    std::vector<std::string>* infeasible_constraint) {
  double *t = nullptr;
  inverseKinBackend(model, 1, t, q_seed, q_nom, num_constraints,
                    constraint_array, ikoptions, q_sol, info,
                    infeasible_constraint);
}

template DRAKE_EXPORT void inverseKin(
    RigidBodyTree *model, const MatrixBase<VectorXd>& q_seed,
    const MatrixBase<VectorXd>& q_nom, const int num_constraints,
    const RigidBodyConstraint* const* constraint_array,
    const IKoptions& ikoptions,
    MatrixBase<VectorXd>* q_sol,
    int* info, std::vector<std::string>* infeasible_constraint);

template DRAKE_EXPORT void inverseKin(
    RigidBodyTree *model, const MatrixBase<Map<VectorXd>>& q_seed,
    const MatrixBase<Map<VectorXd>>& q_nom, const int num_constraints,
    const RigidBodyConstraint* const* constraint_array,
    const IKoptions& ikoptions,
    MatrixBase<Map<VectorXd>>* q_sol, int* info,
    std::vector<std::string>* infeasible_constraint);

IKResults inverseKinSimple(
    RigidBodyTree *model, const Eigen::VectorXd& q_seed,
    const Eigen::VectorXd& q_nom,
    const std::vector<RigidBodyConstraint *>& constraint_array,
    const IKoptions& ikoptions) {
  auto results = IKResults();
  results.q_sol.push_back(Eigen::VectorXd(q_nom.size()));
  results.info.resize(1, 0);
  int num_constraints = static_cast<int>(constraint_array.size());
  inverseKin<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd>(
      model, q_seed, q_nom, num_constraints, constraint_array.data(), ikoptions,
      &results.q_sol[0], &results.info[0], &results.infeasible_constraints);

  return results;
}
