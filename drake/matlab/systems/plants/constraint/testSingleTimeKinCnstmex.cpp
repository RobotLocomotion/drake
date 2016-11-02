#include <mex.h>

#include "drake/systems/plants/constraint/RigidBodyConstraint.h"
#include "drake/matlab/util/drakeMexUtil.h"
#include "drake/systems/plants/RigidBodyTree.h"
#include <cstring>
/*
 * [type, num_constraint, constraint_val, dconstraint_val, constraint_name, lower_bound, upper_bound]
 * = testSingleTimeKinCnstmex(kinCnst_ptr, q, t)
 * @param kinCnst_ptr           A pointer to a SingleTimeKinematicConstraint
 * object
 * @param q                     A nqx1 double vector
 * @param t                     A double scalar, the time to evaluate constraint
 * value, bounds and name. This is optional.
 * @retval type                 The type of the constraint
 * @retval num_constraint       The number of constraint active at time t
 * @retval constraint_val       The value of the constraint at time t
 * @retval dconstraint_val      The gradient of the constraint w.r.t q at time t
 * @retval constraint_name      The name of the constraint at time t
 * @retval lower_bound          The lower bound of the constraint at time t
 * @retval upper_bound          The upper bound of the constraint at time t
 * */
DLL_EXPORT_SYM
void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
  if ((nrhs != 3 && nrhs != 2) || nlhs != 7) {
    mexErrMsgIdAndTxt("Drake:testSingleTimeKinCnstmex:BadInputs",
                      "Usage [type, "
                      "num_cnst, cnst_val, dcnst_val, cnst_name, lb, ub] = "
                      "testSingleTimeKinKinCnstmex(kinCnst, q, t)");
  }
  SingleTimeKinematicConstraint* cnst =
      (SingleTimeKinematicConstraint*)getDrakeMexPointer(prhs[0]);
  double* t_ptr;
  if (nrhs == 2) {
    t_ptr = nullptr;
  } else {
    size_t num_t = mxGetNumberOfElements(prhs[2]);
    if (num_t == 0) {
      t_ptr = nullptr;
    }
    if (num_t == 1) {
      t_ptr = mxGetPrSafe(prhs[2]);
    }
    if (num_t > 1) {
      mexErrMsgIdAndTxt("Drake:testSingleTimeKinCnstmex:BadInputs",
                        "t must be either empty or a single number");
    }
  }
  int type = cnst->getType();
  int num_cnst = cnst->getNumConstraint(t_ptr);
  // mexPrintf("num_cnst = %d\n", num_cnst);
  int nq = cnst->getRobotPointer()->get_num_positions();
  Eigen::Map<Eigen::VectorXd> q(mxGetPrSafe(prhs[1]), nq);
  KinematicsCache<double> cache = cnst->getRobotPointer()->doKinematics(q);
  Eigen::VectorXd c(num_cnst);
  Eigen::MatrixXd dc(num_cnst, nq);
  cnst->eval(t_ptr, cache, c, dc);
  // mexPrintf("get c, dc\n");
  Eigen::VectorXd lb(num_cnst);
  Eigen::VectorXd ub(num_cnst);
  cnst->bounds(t_ptr, lb, ub);
  // mexPrintf("get lb, ub\n");
  std::vector<std::string> cnst_names;
  cnst->name(t_ptr, cnst_names);
  // mexPrintf("get name\n");
  int retvec_size;
  if (num_cnst == 0) {
    retvec_size = 0;
  } else {
    retvec_size = 1;
  }
  plhs[0] = mxCreateDoubleScalar((double)type);
  plhs[1] = mxCreateDoubleScalar((double)num_cnst);
  plhs[2] = mxCreateDoubleMatrix(num_cnst, retvec_size, mxREAL);
  memcpy(mxGetPrSafe(plhs[2]), c.data(), sizeof(double) * num_cnst);
  plhs[3] = mxCreateDoubleMatrix(num_cnst, nq, mxREAL);
  memcpy(mxGetPrSafe(plhs[3]), dc.data(), sizeof(double) * num_cnst * nq);
  int name_ndim = 1;
  mwSize name_dims[] = {(mwSize)num_cnst};
  plhs[4] = mxCreateCellArray(name_ndim, name_dims);
  mxArray* name_ptr;
  for (int i = 0; i < num_cnst; i++) {
    name_ptr = mxCreateString(cnst_names[i].c_str());
    mxSetCell(plhs[4], i, name_ptr);
  }
  plhs[5] = mxCreateDoubleMatrix(num_cnst, retvec_size, mxREAL);
  plhs[6] = mxCreateDoubleMatrix(num_cnst, retvec_size, mxREAL);
  memcpy(mxGetPrSafe(plhs[5]), lb.data(), sizeof(double) * num_cnst);
  memcpy(mxGetPrSafe(plhs[6]), ub.data(), sizeof(double) * num_cnst);
}
