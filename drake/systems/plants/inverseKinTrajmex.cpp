#include "mex.h"
#include "RigidBodyManipulator.h"
#include "constraint/RigidBodyConstraint.h"
#include "IKoptions.h"
#include "RigidBodyIK.h"
#include <Eigen/Dense>
#include "drakeMexUtil.h"

using namespace std;
using namespace Eigen;

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  if(nrhs < 7)
  {
    mexErrMsgIdAndTxt("Drake:inverseKinTrajmex:NotEnoughInputs","Usage inverseKinPointwisemex(model_ptr,t,qdot0_seed,q_seed,q_nom,constraint1,constraint2,...,ikoptions");
  }
  RigidBodyManipulator* model = (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);
  int nq = model->num_positions;
  int nT = static_cast<int>(mxGetNumberOfElements(prhs[1]));
  double* t = mxGetPrSafe(prhs[1]);
  Map<VectorXd> qdot0_seed(mxGetPrSafe(prhs[2]),nq);
  Map<MatrixXd> q_seed(mxGetPrSafe(prhs[3]),nq,nT);
  Map<MatrixXd> q_nom(mxGetPrSafe(prhs[4]),nq,nT);
  int num_constraints = nrhs-6;
  RigidBodyConstraint** constraint_array = new RigidBodyConstraint*[num_constraints];
  for(int i = 0;i<num_constraints;i++)
  {
    constraint_array[i] = (RigidBodyConstraint*) getDrakeMexPointer(prhs[5+i]);
  }
  IKoptions* ikoptions = (IKoptions*) getDrakeMexPointer(prhs[nrhs-1]);
  plhs[0] = mxCreateDoubleMatrix(nq,nT,mxREAL);
  Map<MatrixXd> q_sol(mxGetPrSafe(plhs[0]),nq,nT);
  plhs[1] = mxCreateDoubleMatrix(nq,nT,mxREAL);
  Map<MatrixXd> qdot_sol(mxGetPrSafe(plhs[1]),nq,nT);
  plhs[2] = mxCreateDoubleMatrix(nq,nT,mxREAL);
  Map<MatrixXd> qddot_sol(mxGetPrSafe(plhs[2]),nq,nT);
  int info = 0; 
  vector<string> infeasible_constraint;
  inverseKinTraj(model,nT,t,qdot0_seed,q_seed,q_nom,num_constraints,constraint_array,q_sol,qdot_sol,qddot_sol,info,infeasible_constraint,*ikoptions);
  plhs[3] = mxCreateDoubleScalar((double) info);
  mwSize name_dim[1] = {static_cast<mwSize>(infeasible_constraint.size())};
  plhs[4] = mxCreateCellArray(1,name_dim);
  for(int i = 0;i<infeasible_constraint.size();i++)
  {
    mxArray* name_ptr = mxCreateString(infeasible_constraint[i].c_str());
    mxSetCell(plhs[4],i,name_ptr);
  }
  delete[] constraint_array;
}
