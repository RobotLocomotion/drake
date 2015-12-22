#include "RigidBodyIK.h"
#include "RigidBodyTree.h"
#include "constraint/RigidBodyConstraint.h"
#include "IKoptions.h"
#include "inverseKinBackend.h"

using namespace Eigen;
using namespace std;

template <typename DerivedA, typename DerivedB, typename DerivedC>
drakeIK_DLLEXPORT void inverseKin(RigidBodyTree * model, const MatrixBase<DerivedA> &q_seed, const MatrixBase<DerivedB> &q_nom, const int num_constraints, RigidBodyConstraint** const constraint_array, MatrixBase<DerivedC> &q_sol, int &INFO, vector<string> &infeasible_constraint, const IKoptions &ikoptions)
{
  VectorXd qdot_dummy(model->num_velocities);
  VectorXd qddot_dummy(model->num_velocities);
  double* t = nullptr;
  inverseKinBackend(model, 1,1,t,q_seed,q_nom,num_constraints,constraint_array,q_sol,qdot_dummy,qddot_dummy,&INFO, infeasible_constraint, ikoptions);
}

template drakeIK_DLLEXPORT void inverseKin(RigidBodyTree * model, const MatrixBase<VectorXd> &q_seed, const MatrixBase<VectorXd> &q_nom, const int num_constraints, RigidBodyConstraint** const constraint_array, MatrixBase<VectorXd> &q_sol, int &INFO, vector<string> &infeasible_constraint, const IKoptions &ikoptions);
template drakeIK_DLLEXPORT void inverseKin(RigidBodyTree * model, const MatrixBase<Map<VectorXd>> &q_seed, const MatrixBase<Map<VectorXd>> &q_nom, const int num_constraints, RigidBodyConstraint** const constraint_array, MatrixBase<Map<VectorXd>> &q_sol, int &INFO, vector<string> &infeasible_constraint, const IKoptions &ikoptions);

IKResults inverseKinSimple(RigidBodyTree* model, 
  const Eigen::VectorXd &q_seed,
  const Eigen::VectorXd &q_nom,
  const std::vector<RigidBodyConstraint*> &constraint_array,
  const IKoptions &ikoptions) {
  auto results = IKResults();
  results.q_sol.resize(q_nom.size());
  int num_constraints = constraint_array.size();
  RigidBodyConstraint** const constraint_array_ptr = (RigidBodyConstraint** const) &constraint_array[0];
  inverseKin<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd>(model,
    q_seed,
    q_nom,
    num_constraints,
    constraint_array_ptr,
    results.q_sol,
    results.INFO,
    results.infeasible_constraint,
    ikoptions);

  return results;
}
