#include "mex.h"
#include "RigidBodyConstraint.h"
#include "drakeUtil.h"
#include "../RigidBodyManipulator.h"
#include <cstring>
/* 
 * [num_constraint,constraint_val,dconstraint_val,constraint_name,lower_bound,upper_bound] = testSingleKinCnstmex(kinCnst_ptr,q,t)
 * @param kinCnst_ptr           A pointer to a KinematicConstraint object
 * @param q                     A nqx1 double vector
 * @param t                     A double array, the time moments to evaluate constraint value, bounds and name. 
 * @retval num_constraint       The number of constraint active at time t
 * @retval constraint_val       The value of the constraint at time t
 * @retval dconstraint_val      The gradient of the constraint w.r.t q at time t
 * @retval constraint_name      The name of the constraint at time t
 * @retval lower_bound          The lower bound of the constraint at time t
 * @retval upper_bound          The upper bound of the constraint at time t
 * */

void mexFunction(int nlhs,mxArray* plhs[], int nrhs, const mxArray * prhs[])
{
  if(nrhs!=3 || nlhs != 6)
  {
    mexErrMsgIdAndTxt("Drake:testMultipleTimeKinCnstmex:BadInputs","Usage [num_cnst,cnst_val,dcnst_val,cnst_name,lb,ub] = testMultipleTimeKinCnstmex(kinCnst,q,t)");
  }
  MultipleTimeKinematicConstraint* cnst = (MultipleTimeKinematicConstraint*) getDrakeMexPointer(prhs[0]);
  int n_breaks = mxGetNumberOfElements(prhs[2]);
  double* t_ptr = new double[n_breaks];
  memcpy(t_ptr,mxGetPr(prhs[2]),sizeof(double)*n_breaks);
  int nq = cnst->getRobotPointer()->num_dof;
  MatrixXd q(nq,n_breaks);
  if(mxGetM(prhs[1]) != nq || mxGetN(prhs[1]) != n_breaks)
  {
    mexErrMsgIdAndTxt("Drake:testMultipleTimeKinCnstmex:BadInputs","Argument 2 must be of size nq*n_breaks");
  }
  memcpy(q.data(),mxGetPr(prhs[1]),sizeof(double)*nq*n_breaks); 
  int num_cnst = cnst->getNumConstraint(t_ptr,n_breaks); 
  VectorXd c(num_cnst);
  MatrixXd dc(num_cnst,nq*n_breaks);
  cnst->eval(t_ptr,n_breaks,q,c,dc);
  VectorXd lb(num_cnst);
  VectorXd ub(num_cnst);
  cnst->bounds(t_ptr,n_breaks,lb,ub);
  std::vector<std::string> cnst_names;
  cnst->name(t_ptr,n_breaks,cnst_names);
  plhs[0] = mxCreateDoubleScalar((double) num_cnst);
  plhs[1] = mxCreateDoubleMatrix(num_cnst,1,mxREAL);
  memcpy(mxGetPr(plhs[1]),c.data(),sizeof(double)*num_cnst);
  plhs[2] = mxCreateDoubleMatrix(num_cnst,nq*n_breaks,mxREAL);
  memcpy(mxGetPr(plhs[2]),dc.data(),sizeof(double)*num_cnst*nq*n_breaks);
  int name_ndim = 1;
  mwSize name_dims[] = {(mwSize) num_cnst};
  plhs[3] = mxCreateCellArray(name_ndim,name_dims);
  mxArray *name_ptr;
  for(int i = 0;i<num_cnst;i++)
  {
    name_ptr = mxCreateString(cnst_names[i].c_str());
    mxSetCell(plhs[3],i,name_ptr);
  }
  plhs[4] = mxCreateDoubleMatrix(num_cnst,1,mxREAL);
  plhs[5] = mxCreateDoubleMatrix(num_cnst,1,mxREAL);
  memcpy(mxGetPr(plhs[4]),lb.data(),sizeof(double)*num_cnst);
  memcpy(mxGetPr(plhs[5]),ub.data(),sizeof(double)*num_cnst);
  delete[] t_ptr;
}
