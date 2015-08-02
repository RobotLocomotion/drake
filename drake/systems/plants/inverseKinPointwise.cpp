#include "RigidBodyIK.h"
#include "RigidBodyManipulator.h"
#include "constraint/RigidBodyConstraint.h"
#include "IKoptions.h"
#include "inverseKinBackend.h"

using namespace std;
using namespace Eigen;

template <typename DerivedA, typename DerivedB, typename DerivedC>
drakeIK_DLLEXPORT void inverseKinPointwise(RigidBodyManipulator* model, const int nT, const double* t, const MatrixBase<DerivedA> &q_seed, const MatrixBase<DerivedB> &q_nom, const int num_constraints, RigidBodyConstraint** const constraint_array, MatrixBase<DerivedC> &q_sol, int* INFO, vector<string> &infeasible_constraint, const IKoptions &ikoptions)
{
  int nq = model->num_positions;
  MatrixXd qdot_dummy(nq,nT);
  MatrixXd qddot_dummy(nq,nT);
  inverseKinBackend(model,1,nT,t,q_seed,q_nom,num_constraints,constraint_array,q_sol,qdot_dummy,qddot_dummy,INFO,infeasible_constraint,ikoptions);
}

template drakeIK_DLLEXPORT void inverseKinPointwise(RigidBodyManipulator* model, const int nT, const double* t, const MatrixBase<Map<MatrixXd>> &q_seed, const MatrixBase<Map<MatrixXd>> &q_nom, const int num_constraints, RigidBodyConstraint** const constraint_array, MatrixBase<Map<MatrixXd>> &q_sol, int* INFO, vector<string> &infeasible_constraint, const IKoptions &ikoptions);
template drakeIK_DLLEXPORT void inverseKinPointwise(RigidBodyManipulator* model, const int nT, const double* t, const MatrixBase<MatrixXd> &q_seed, const MatrixBase<MatrixXd> &q_nom, const int num_constraints, RigidBodyConstraint** const constraint_array, MatrixBase<MatrixXd> &q_sol, int* INFO, vector<string> &infeasible_constraint, const IKoptions &ikoptions);
