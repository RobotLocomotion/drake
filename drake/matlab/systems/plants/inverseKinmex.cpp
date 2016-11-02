#include <mex.h>

#include "drake/systems/plants/RigidBodyTree.h"
#include "drake/systems/plants/constraint/RigidBodyConstraint.h"
#include "drake/systems/plants/IKoptions.h"
#include "drake/systems/plants/RigidBodyIK.h"
#include <Eigen/Dense>
#include "drake/matlab/util/drakeMexUtil.h"

using namespace std;
using namespace Eigen;

DLL_EXPORT_SYM
void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
  if (nrhs < 5) {
    mexErrMsgIdAndTxt("Drake:inverseKinmex:NotEnoughInputs",
                      "Usage "
                      "inverseKinmex(model_ptr, q_seed, q_nom, constraint1,"
                      "constraint2,..., ikoptions");
  }
  RigidBodyTree* model = (RigidBodyTree*)getDrakeMexPointer(prhs[0]);
  int nq = model->get_num_positions();
  Map<VectorXd> q_seed(mxGetPrSafe(prhs[1]), nq);
  Map<VectorXd> q_nom(mxGetPrSafe(prhs[2]), nq);
  int num_constraints = nrhs - 4;
  RigidBodyConstraint** constraint_array =
      new RigidBodyConstraint* [num_constraints];
  for (int i = 0; i < num_constraints; i++) {
    constraint_array[i] = (RigidBodyConstraint*)getDrakeMexPointer(prhs[3 + i]);
  }
  IKoptions* ikoptions = (IKoptions*)getDrakeMexPointer(prhs[nrhs - 1]);
  plhs[0] = mxCreateDoubleMatrix(nq, 1, mxREAL);
  Map<VectorXd> q_sol(mxGetPrSafe(plhs[0]), nq);
  int info;
  vector<string> infeasible_constraint;
  inverseKin(model, q_seed, q_nom, num_constraints, constraint_array,
             *ikoptions, &q_sol, &info, &infeasible_constraint);
  plhs[1] = mxCreateDoubleScalar((double)info);
  mwSize name_dim[1] = {static_cast<mwSize>(infeasible_constraint.size())};
  plhs[2] = mxCreateCellArray(1, name_dim);
  for (int i = 0; i < infeasible_constraint.size(); i++) {
    mxArray* name_ptr = mxCreateString(infeasible_constraint[i].c_str());
    mxSetCell(plhs[2], i, name_ptr);
  }
  delete[] constraint_array;
}
