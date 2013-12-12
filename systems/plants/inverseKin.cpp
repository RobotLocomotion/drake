#include "RigidBodyIK.h"
#include "RigidBodyManipulator.h"
#include "constraint/RigidBodyConstraint.h"
#include "IKoptions.h"
#include "inverseKinBackend.h"

using namespace Eigen;
using namespace std;

template <typename DerivedA, typename DerivedB, typename DerivedC>
void inverseKin(RigidBodyManipulator* model, const MatrixBase<DerivedA> &q_seed, const MatrixBase<DerivedB> &q_nom, const int num_constraints, RigidBodyConstraint** const constraint_array, MatrixBase<DerivedC> &q_sol, int &INFO, vector<string> &infeasible_constraint, const IKoptions &ikoptions) 
{
  VectorXd qdot_dummy(model->num_dof);
  VectorXd qddot_dummy(model->num_dof);
  double* t = NULL;
  inverseKinBackend(model, 1,0,t,q_seed,q_nom,num_constraints,constraint_array,q_sol,qdot_dummy,qddot_dummy,&INFO, infeasible_constraint, ikoptions);
}

template void inverseKin(RigidBodyManipulator*, const MatrixBase<Map<VectorXd>> &, const MatrixBase<Map<VectorXd>> &, const int, RigidBodyConstraint** const, MatrixBase<Map<VectorXd>> &, int &, vector<string> &, const IKoptions&);
template void inverseKin(RigidBodyManipulator*, const MatrixBase<VectorXd> &, const MatrixBase<VectorXd> &, const int, RigidBodyConstraint** const, MatrixBase<VectorXd> &, int &, vector<string> &, const IKoptions&);
