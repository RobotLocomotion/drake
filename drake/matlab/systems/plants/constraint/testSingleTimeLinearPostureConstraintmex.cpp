#include <mex.h>

#include "drake/systems/plants/constraint/RigidBodyConstraint.h"
#include "drake/matlab/util/drakeMexUtil.h"
#include "drake/systems/plants/RigidBodyTree.h"
#include <cstring>
/*
 * [type,
 * num_constraint, constraint_val, iAfun, jAvar, A, constraint_name, lower_bound, upper_bound]
 * = testSingleTimeLinearPostureConstraintmex(stlpc_ptr, q, t)
 * @param stlpc_ptr             A pointer to a SingleTimeLinearPostureConstraint
 * object
 * @param q                     A nqx1 double vector
 * @param t                     A double array, the time moments to evaluate
 * constraint value, bounds and name.
 * @retval type                 The type of the constraint
 * @retval num_constraint       The number of constraint active at time t
 * @retval iAfun                The row index of non-zero element in the sparse
 * linear constraint
 * @retval jAvar                The column index of the non-zero element in the
 * sparse linear constraint
 * @retval A                    The value of the non-zeroelement in the sparse
 * linear constraint
 * @retval constraint_name      The name of the constraint at time t
 * @retval lower_bound          The lower bound of the constraint at time t
 * @retval upper_bound          The upper bound of the constraint at time t
 * */

using namespace std;
using namespace Eigen;

DLL_EXPORT_SYM
void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
  if (nrhs != 3 || nlhs != 9) {
    mexErrMsgIdAndTxt(
        "Drake:testSingleTimeLinearPostureConstraintmex:BadInputs",
        "Usage "
        "[type, num_cnst, cnst_val, iAfun, jAvar, A, cnst_name, lb, ub] = "
        "testSingleTimeLinearPostureConstraintmex(stlpc_ptr, q, t)");
  }
  SingleTimeLinearPostureConstraint* stlpc =
      (SingleTimeLinearPostureConstraint*)getDrakeMexPointer(prhs[0]);
  int nq = stlpc->getRobotPointer()->get_num_positions();
  if (!mxIsNumeric(prhs[1]) || mxGetN(prhs[1]) != 1 || mxGetM(prhs[1]) != nq) {
    mexErrMsgIdAndTxt(
        "Drake:testSingleTimeLinearPostureConstraintmex:BadInputs",
        "q must a numeric column vector with size nq");
  }
  VectorXd q(nq);
  memcpy(q.data(), mxGetPrSafe(prhs[1]), sizeof(double) * nq);
  double* t_ptr = nullptr;
  if (mxGetNumberOfElements(prhs[2]) == 0) {
    t_ptr = nullptr;
  } else if (mxGetNumberOfElements(prhs[2]) == 1) {
    t_ptr = mxGetPrSafe(prhs[2]);
  }
  int type = stlpc->getType();
  int num_cnst = stlpc->getNumConstraint(t_ptr);
  VectorXd c(num_cnst);
  stlpc->feval(t_ptr, q, c);
  VectorXi iAfun, jAvar;
  VectorXd A;
  stlpc->geval(t_ptr, iAfun, jAvar, A);
  vector<string> cnst_names;
  stlpc->name(t_ptr, cnst_names);
  VectorXd lb, ub;
  stlpc->bounds(t_ptr, lb, ub);
  plhs[0] = mxCreateDoubleScalar((double)type);
  plhs[1] = mxCreateDoubleScalar((double)num_cnst);
  int retvec_size;
  if (num_cnst == 0) {
    retvec_size = 0;
  } else {
    retvec_size = 1;
  }
  plhs[2] = mxCreateDoubleMatrix(num_cnst, retvec_size, mxREAL);
  memcpy(mxGetPrSafe(plhs[2]), c.data(), sizeof(double) * num_cnst);
  plhs[3] =
      mxCreateDoubleMatrix(static_cast<int>(iAfun.size()), retvec_size, mxREAL);
  plhs[4] =
      mxCreateDoubleMatrix(static_cast<int>(jAvar.size()), retvec_size, mxREAL);
  plhs[5] =
      mxCreateDoubleMatrix(static_cast<int>(A.size()), retvec_size, mxREAL);
  for (int i = 0; i < iAfun.size(); i++) {
    *(mxGetPrSafe(plhs[3]) + i) = (double)iAfun(i) + 1;
    *(mxGetPrSafe(plhs[4]) + i) = (double)jAvar(i) + 1;
    *(mxGetPrSafe(plhs[5]) + i) = A(i);
  }
  int name_ndim = 1;
  mwSize name_dims[] = {(mwSize)num_cnst};
  plhs[6] = mxCreateCellArray(name_ndim, name_dims);
  mxArray* name_ptr;
  for (int i = 0; i < num_cnst; i++) {
    name_ptr = mxCreateString(cnst_names[i].c_str());
    mxSetCell(plhs[6], i, name_ptr);
  }
  plhs[7] = mxCreateDoubleMatrix(num_cnst, retvec_size, mxREAL);
  plhs[8] = mxCreateDoubleMatrix(num_cnst, retvec_size, mxREAL);
  memcpy(mxGetPrSafe(plhs[7]), lb.data(), sizeof(double) * num_cnst);
  memcpy(mxGetPrSafe(plhs[8]), ub.data(), sizeof(double) * num_cnst);
}
