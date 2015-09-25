#include "mex.h"
#include "RigidBodyConstraint.h"
#include "drakeMexUtil.h"
#include "../RigidBodyManipulator.h"
#include <cstring>
/* 
 * [num_constraint,constraint_val,iAfun,jAvar,A ,constraint_name,lower_bound,upper_bound] = testMultipleTimeLinearPostureConstraintmex(kinCnst_ptr,q,t)
 * @param kinCnst_ptr           A pointer to a MultipleTimeLinearPostureConstraint object
 * @param q                     A nqxnT double vector
 * @param t                     A double array, the time moments to evaluate constraint value, bounds and name. 
 * @retval num_constraint       The number of constraint active at time t
 * @retval constraint_val       The value of the constraint at time t
 * @retval iAfun,jAvar,A        The sparse matrix sparse(iAfun,jAvar,A,num_constraint,numel(q)) is the gradient of constraint_val w.r.t q
 * @retval constraint_name      The name of the constraint at time t
 * @retval lower_bound          The lower bound of the constraint at time t
 * @retval upper_bound          The upper bound of the constraint at time t
 * */

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray *prhs[])
{
  if(nrhs!=3 || nlhs != 8)
  {
    mexErrMsgIdAndTxt("Drake:testMultipleTimeLinearPostureConstrainttmex:BadInputs","Usage [num_cnst,cnst_val,iAfun,jAvar,A,cnst_name,lb,ub] = testMultipleTimeLinearPostureConstraintmex(kinCnst,q,t)");
  }
  MultipleTimeLinearPostureConstraint* cnst = (MultipleTimeLinearPostureConstraint*) getDrakeMexPointer(prhs[0]);
  int n_breaks = static_cast<int>(mxGetNumberOfElements(prhs[2]));
  double* t_ptr = new double[n_breaks];
  memcpy(t_ptr,mxGetPrSafe(prhs[2]),sizeof(double)*n_breaks);
  int nq = cnst->getRobotPointer()->num_positions;
  Eigen::MatrixXd q(nq,n_breaks);
  if(mxGetM(prhs[1]) != nq || mxGetN(prhs[1]) != n_breaks)
  {
    mexErrMsgIdAndTxt("Drake:testMultipleTimeLinearPostureConstraintmex:BadInputs","Argument 2 must be of size nq*n_breaks");
  }
  memcpy(q.data(),mxGetPrSafe(prhs[1]),sizeof(double)*nq*n_breaks); 
  int num_cnst = cnst->getNumConstraint(t_ptr,n_breaks); 
  Eigen::VectorXd c(num_cnst);
  cnst->feval(t_ptr,n_breaks,q,c);
  Eigen::VectorXi iAfun;
  Eigen::VectorXi jAvar;
  Eigen::VectorXd A;
  cnst->geval(t_ptr,n_breaks,iAfun,jAvar,A);
  std::vector<std::string> cnst_names;
  cnst->name(t_ptr,n_breaks,cnst_names);
  Eigen::VectorXd lb(num_cnst);
  Eigen::VectorXd ub(num_cnst);
  cnst->bounds(t_ptr,n_breaks,lb,ub);
  Eigen::VectorXd iAfun_tmp(iAfun.size());
  Eigen::VectorXd jAvar_tmp(jAvar.size());
  for(int i = 0;i<iAfun.size();i++)
  {
    iAfun_tmp(i) = (double) iAfun(i)+1;
    jAvar_tmp(i) = (double) jAvar(i)+1;
  }
  plhs[0] = mxCreateDoubleScalar((double) num_cnst);
  plhs[1] = mxCreateDoubleMatrix(num_cnst,1,mxREAL);
  memcpy(mxGetPrSafe(plhs[1]),c.data(),sizeof(double)*num_cnst);
  plhs[2] = mxCreateDoubleMatrix(iAfun_tmp.size(),1,mxREAL);
  memcpy(mxGetPrSafe(plhs[2]),iAfun_tmp.data(),sizeof(double)*iAfun_tmp.size());
  plhs[3] = mxCreateDoubleMatrix(jAvar_tmp.size(),1,mxREAL);
  memcpy(mxGetPrSafe(plhs[3]),jAvar_tmp.data(),sizeof(double)*jAvar_tmp.size());
  plhs[4] = mxCreateDoubleMatrix(A.size(),1,mxREAL);
  memcpy(mxGetPrSafe(plhs[4]),A.data(),sizeof(double)*A.size());
  int name_ndim = 1;
  mwSize name_dims[] = {(mwSize) num_cnst};
  plhs[5] = mxCreateCellArray(name_ndim,name_dims);
  mxArray *name_ptr;
  for(int i = 0;i<num_cnst;i++)
  {
    name_ptr = mxCreateString(cnst_names[i].c_str());
    mxSetCell(plhs[5],i,name_ptr);
  }
  plhs[6] = mxCreateDoubleMatrix(num_cnst,1,mxREAL);
  plhs[7] = mxCreateDoubleMatrix(num_cnst,1,mxREAL);
  memcpy(mxGetPrSafe(plhs[6]),lb.data(),sizeof(double)*num_cnst);
  memcpy(mxGetPrSafe(plhs[7]),ub.data(),sizeof(double)*num_cnst);
  delete[] t_ptr;
}

