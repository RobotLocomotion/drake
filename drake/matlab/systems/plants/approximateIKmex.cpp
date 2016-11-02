#include <mex.h>

#include "drake/systems/plants/RigidBodyIK.h"
#include "drake/systems/plants/RigidBodyTree.h"
#include "drake/systems/plants/constraint/RigidBodyConstraint.h"
#include "drake/systems/plants/IKoptions.h"
#include "drake/matlab/util/drakeMexUtil.h"

using namespace std;
using namespace Eigen;

DLL_EXPORT_SYM
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  if (nrhs < 5) {
    mexErrMsgIdAndTxt("Drake:approximateIKmex:NotEnoughInputs",
                      "Usage "
                      "approximateIKmex(model_ptr, q_seed, q_nom, constraint1,"
                      "constraint2,..., ikoptions)");
  }
  RigidBodyTree *model = (RigidBodyTree *)getDrakeMexPointer(prhs[0]);
  int nq = model->get_num_positions();
  Map<VectorXd> q_seed(mxGetPrSafe(prhs[1]), nq);
  Map<VectorXd> q_nom(mxGetPrSafe(prhs[2]), nq);
  // VectorXd q_seed(nq);
  // memcpy(q_seed.data(), mxGetPrSafe(prhs[1]), sizeof(double)*nq);
  // VectorXd q_nom(nq);
  // memcpy(q_nom.data(), mxGetPrSafe(prhs[2]), sizeof(double)*nq);
  int num_constraints = nrhs - 4;
  RigidBodyConstraint** constraint_array =
      new RigidBodyConstraint *[num_constraints];
  for (int i = 0; i < num_constraints; i++) {
    constraint_array[i] =
        (RigidBodyConstraint *)getDrakeMexPointer(prhs[3 + i]);
  }
  IKoptions *ikoptions = (IKoptions *)getDrakeMexPointer(prhs[nrhs - 1]);
  plhs[0] = mxCreateDoubleMatrix(nq, 1, mxREAL);
  Map<VectorXd> q_sol(mxGetPrSafe(plhs[0]), nq);
  int info;
  // VectorXd q_sol(nq);
  approximateIK(model, q_seed, q_nom, num_constraints, constraint_array,
                *ikoptions, &q_sol, &info);
  // plhs[0] = mxCreateDoubleMatrix(nq, 1, mxREAL);
  // memcpy(mxGetPrSafe(plhs[0]), q_sol.data(), sizeof(double)*nq);
  if (nlhs > 1) {
    plhs[1] = mxCreateDoubleScalar((double)info);
  }
  delete[] constraint_array;
}
