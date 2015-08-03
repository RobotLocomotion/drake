#include "RigidBodyIK.h"
#include "RigidBodyManipulator.h"
#include "constraint/RigidBodyConstraint.h"
#include "IKoptions.h"
#include "inverseKinBackend.h"

using namespace std;
using namespace Eigen;


template <typename DerivedA, typename DerivedB, typename DerivedC, typename DerivedD, typename DerivedE, typename DerivedF>
drakeIK_DLLEXPORT void inverseKinTraj(RigidBodyManipulator* model, const int nT, const double* t, const MatrixBase<DerivedA> &qdot0_seed, const MatrixBase<DerivedB> &q_seed, const MatrixBase<DerivedC> &q_nom, const int num_constraints, RigidBodyConstraint** const constraint_array, MatrixBase<DerivedD> &q_sol, MatrixBase<DerivedE> &qdot_sol, MatrixBase<DerivedF> &qddot_sol, int &INFO, vector<string> &infeasible_constraint, IKoptions ikoptions)
{
  if(ikoptions.getFixInitialState())
  {
    ikoptions.setqd0(qdot0_seed,qdot0_seed);
  }
  inverseKinBackend(model,2,nT,t,q_seed,q_nom,num_constraints,constraint_array,q_sol,qdot_sol,qddot_sol,&INFO, infeasible_constraint,ikoptions); 
}

template drakeIK_DLLEXPORT void inverseKinTraj(RigidBodyManipulator* model, const int nT, const double* t, const MatrixBase<Map<VectorXd>> &qdot0_seed, const MatrixBase<Map<MatrixXd>> &q_seed, const MatrixBase<Map<MatrixXd>> &q_nom, const int num_constraints, RigidBodyConstraint** const constraint_array, MatrixBase<Map<MatrixXd>> &q_sol, MatrixBase<Map<MatrixXd>> &qdot_sol, MatrixBase<Map<MatrixXd>> &qddot_sol, int &INFO, vector<string> &infeasible_constraint, IKoptions ikoptions);
template drakeIK_DLLEXPORT void inverseKinTraj(RigidBodyManipulator* model, const int nT, const double* t, const MatrixBase<VectorXd> &qdot0_seed, const MatrixBase<MatrixXd> &q_seed, const MatrixBase<MatrixXd> &q_nom, const int num_constraints, RigidBodyConstraint** const constraint_array, MatrixBase<MatrixXd> &q_sol, MatrixBase<MatrixXd> &qdot_sol, MatrixBase<MatrixXd> &qddot_sol, int &INFO, vector<string> &infeasible_constraint, IKoptions ikoptions);
