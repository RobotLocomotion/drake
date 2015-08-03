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
  if(nrhs < 5)
  {
    mexErrMsgIdAndTxt("Drake:inverseKinmex:NotEnoughInputs","Usage inverseKinmex(model_ptr,q_seed,q_nom,constraint1,constraint2,...,ikoptions");
  }
  RigidBodyManipulator* model = (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);
  int nq = model->num_positions;
  Map<VectorXd> q_seed(mxGetPrSafe(prhs[1]),nq);
  Map<VectorXd> q_nom(mxGetPrSafe(prhs[2]),nq);
  int num_constraints = nrhs-4;
  RigidBodyConstraint** constraint_array = new RigidBodyConstraint*[num_constraints];
  for(int i = 0;i<num_constraints;i++)
  {
    constraint_array[i] = (RigidBodyConstraint*) getDrakeMexPointer(prhs[3+i]);
  }
  IKoptions* ikoptions = (IKoptions*) getDrakeMexPointer(prhs[nrhs-1]);
  plhs[0] = mxCreateDoubleMatrix(nq,1,mxREAL);
  Map<VectorXd> q_sol(mxGetPrSafe(plhs[0]),nq);
  int info;
  vector<string> infeasible_constraint;
  inverseKin(model,q_seed,q_nom,num_constraints,constraint_array,q_sol,info,infeasible_constraint,*ikoptions);
  plhs[1] = mxCreateDoubleScalar((double) info);
  mwSize name_dim[1] = {static_cast<mwSize>(infeasible_constraint.size())};
  plhs[2] = mxCreateCellArray(1,name_dim);
  for(int i = 0;i<infeasible_constraint.size();i++)
  {
    mxArray* name_ptr = mxCreateString(infeasible_constraint[i].c_str());
    mxSetCell(plhs[2],i,name_ptr);
  }
  delete[] constraint_array;
}
