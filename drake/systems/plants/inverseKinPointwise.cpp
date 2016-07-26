#include "drake/systems/plants/RigidBodyIK.h"
#include "drake/systems/plants/RigidBodyTree.h"
#include "inverseKinBackend.h"

using namespace std;
using namespace Eigen;

using drake::systems::plants::inverseKinBackend;

template <typename DerivedA, typename DerivedB, typename DerivedC>
DRAKEIK_EXPORT void inverseKinPointwise(
    RigidBodyTree* model, const int nT, const double* t,
    const MatrixBase<DerivedA>& q_seed, const MatrixBase<DerivedB>& q_nom,
    const int num_constraints, RigidBodyConstraint** const constraint_array,
    const IKoptions& ikoptions, MatrixBase<DerivedC>* q_sol, int* INFO,
    vector<string>* infeasible_constraint) {
  inverseKinBackend(model, nT, t, q_seed, q_nom, num_constraints,
                    constraint_array, ikoptions, q_sol,
                    INFO, infeasible_constraint);
}

template DRAKEIK_EXPORT void inverseKinPointwise(
    RigidBodyTree* model, const int nT, const double* t,
    const MatrixBase<Map<MatrixXd>>& q_seed,
    const MatrixBase<Map<MatrixXd>>& q_nom, const int num_constraints,
    RigidBodyConstraint** const constraint_array,
    const IKoptions& ikoptions, MatrixBase<Map<MatrixXd>>* q_sol, int *INFO,
    vector<string>* infeasible_constraint);
template DRAKEIK_EXPORT void inverseKinPointwise(
    RigidBodyTree* model, const int nT, const double* t,
    const MatrixBase<MatrixXd>& q_seed, const MatrixBase<MatrixXd>& q_nom,
    const int num_constraints, RigidBodyConstraint** const constraint_array,
    const IKoptions& ikoptions, MatrixBase<MatrixXd>* q_sol, int* INFO,
    vector<string>* infeasible_constraint);
