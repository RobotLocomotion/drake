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
  if (nrhs < 6) {
    mexErrMsgIdAndTxt("Drake:inverseKinPointwisemex:NotEnoughInputs",
                      "Usage "
                      "inverseKinPointwisemex(model_ptr, t, q_seed, q_nom,"
                      "constraint1, constraint2,..., ikoptions");
  }
  RigidBodyTree* model = (RigidBodyTree*)getDrakeMexPointer(prhs[0]);
  int nq = model->get_num_positions();
  int nT = static_cast<int>(mxGetNumberOfElements(prhs[1]));
  double* t = mxGetPrSafe(prhs[1]);
  Map<MatrixXd> q_seed(mxGetPrSafe(prhs[2]), nq, nT);
  Map<MatrixXd> q_nom(mxGetPrSafe(prhs[3]), nq, nT);
  int num_constraints = nrhs - 5;
  RigidBodyConstraint** constraint_array =
      new RigidBodyConstraint* [num_constraints];
  for (int i = 0; i < num_constraints; i++) {
    constraint_array[i] = (RigidBodyConstraint*)getDrakeMexPointer(prhs[4 + i]);
  }
  IKoptions* ikoptions = (IKoptions*)getDrakeMexPointer(prhs[nrhs - 1]);
  plhs[0] = mxCreateDoubleMatrix(nq, nT, mxREAL);
  Map<MatrixXd> q_sol(mxGetPrSafe(plhs[0]), nq, nT);
  int* info = new int[nT];
  vector<string> infeasible_constraint;
  inverseKinPointwise(model, nT, t, q_seed, q_nom, num_constraints,
                      constraint_array, *ikoptions,
                      &q_sol, info, &infeasible_constraint);

  plhs[1] = mxCreateDoubleMatrix(1, nT, mxREAL);
  for (int i = 0; i < nT; i++) {
    *(mxGetPrSafe(plhs[1]) + i) = (double)info[i];
  }
  mwSize name_dim[1] = {static_cast<mwSize>(infeasible_constraint.size())};
  plhs[2] = mxCreateCellArray(1, name_dim);
  for (int i = 0; i < infeasible_constraint.size(); i++) {
    mxArray* name_ptr = mxCreateString(infeasible_constraint[i].c_str());
    mxSetCell(plhs[2], i, name_ptr);
  }
  delete[] info;
  delete[] constraint_array;
}
