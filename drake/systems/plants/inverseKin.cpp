#include "drake/systems/plants/RigidBodyIK.h"
#include "drake/systems/plants/RigidBodyTree.h"
#include "inverseKinBackend.h"

using namespace Eigen;
using namespace std;

using Drake::systems::plants::inverseKinBackend;

template <typename DerivedA, typename DerivedB, typename DerivedC>
DRAKEIK_EXPORT void inverseKin(RigidBodyTree *model,
                               const MatrixBase<DerivedA>& q_seed,
                               const MatrixBase<DerivedB>& q_nom,
                               const int num_constraints,
                               RigidBodyConstraint **const constraint_array,
                               const IKoptions& ikoptions,
                               MatrixBase<DerivedC>* q_sol, int* INFO,
                               vector<string>* infeasible_constraint) {
  VectorXd qdot_dummy(model->number_of_velocities());
  VectorXd qddot_dummy(model->number_of_velocities());
  double *t = nullptr;
  inverseKinBackend(model, 1, 1, t, q_seed, q_nom, num_constraints,
                    constraint_array, ikoptions,
                    q_sol, &qdot_dummy, &qddot_dummy, INFO,
                    infeasible_constraint);
}

template DRAKEIK_EXPORT void inverseKin(
    RigidBodyTree *model, const MatrixBase<VectorXd>& q_seed,
    const MatrixBase<VectorXd>& q_nom, const int num_constraints,
    RigidBodyConstraint **const constraint_array,
    const IKoptions& ikoptions,
    MatrixBase<VectorXd>* q_sol,
    int* INFO, vector<string>* infeasible_constraint);

template DRAKEIK_EXPORT void inverseKin(
    RigidBodyTree *model, const MatrixBase<Map<VectorXd>>& q_seed,
    const MatrixBase<Map<VectorXd>>& q_nom, const int num_constraints,
    RigidBodyConstraint **const constraint_array,
    const IKoptions& ikoptions,
    MatrixBase<Map<VectorXd>>* q_sol, int* INFO,
    vector<string>* infeasible_constraint);

IKResults inverseKinSimple(
    RigidBodyTree *model, const Eigen::VectorXd& q_seed,
    const Eigen::VectorXd& q_nom,
    const std::vector<RigidBodyConstraint *>& constraint_array,
    const IKoptions& ikoptions) {
  auto results = IKResults();
  results.q_sol.resize(q_nom.size());
  int num_constraints = constraint_array.size();
  RigidBodyConstraint **const constraint_array_ptr =
      (RigidBodyConstraint * *const)constraint_array.data();
  inverseKin<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd>(
      model, q_seed, q_nom, num_constraints, constraint_array_ptr, ikoptions,
      &results.q_sol, &results.INFO, &results.infeasible_constraints);

  return results;
}
