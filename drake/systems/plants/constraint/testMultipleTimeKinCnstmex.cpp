#include "mex.h"
#include "RigidBodyConstraint.h"
#include "drakeMexUtil.h"
#include "../RigidBodyManipulator.h"
#include <cstring>
/* 
 * [type,num_constraint,constraint_val,dconstraint_val,constraint_name,lower_bound,upper_bound] = testSingleKinCnstmex(kinCnst_ptr,q,t)
 * @param kinCnst_ptr           A pointer to a SingleTimeKinematicConstraint object
 * @param q                     A nqx1 double vector
 * @param t                     A double array, the time moments to evaluate constraint value, bounds and name. 
 * @retval type                 The type of the constraint
 * @retval num_constraint       The number of constraint active at time t
 * @retval constraint_val       The value of the constraint at time t
 * @retval dconstraint_val      The gradient of the constraint w.r.t q at time t
 * @retval constraint_name      The name of the constraint at time t
 * @retval lower_bound          The lower bound of the constraint at time t
 * @retval upper_bound          The upper bound of the constraint at time t
 * */

void mexFunction(int nlhs,mxArray* plhs[], int nrhs, const mxArray * prhs[])
{
  if(nrhs!=3 || nlhs != 7)
  {
    mexErrMsgIdAndTxt("Drake:testMultipleTimeKinCnstmex:BadInputs","Usage [type,num_cnst,cnst_val,dcnst_val,cnst_name,lb,ub] = testMultipleTimeKinCnstmex(kinCnst,q,t)");
  }
  MultipleTimeKinematicConstraint* cnst = (MultipleTimeKinematicConstraint*) getDrakeMexPointer(prhs[0]);
  int n_breaks = static_cast<int>(mxGetNumberOfElements(prhs[2]));
  double* t_ptr = new double[n_breaks];
  memcpy(t_ptr,mxGetPrSafe(prhs[2]),sizeof(double)*n_breaks);
  int nq = cnst->getRobotPointer()->num_positions;
  Eigen::MatrixXd q(nq,n_breaks);
  if(mxGetM(prhs[1]) != nq || mxGetN(prhs[1]) != n_breaks)
  {
    mexErrMsgIdAndTxt("Drake:testMultipleTimeKinCnstmex:BadInputs","Argument 2 must be of size nq*n_breaks");
  }
  memcpy(q.data(),mxGetPrSafe(prhs[1]),sizeof(double)*nq*n_breaks); 
  int type = cnst->getType();
  int num_cnst = cnst->getNumConstraint(t_ptr,n_breaks); 
  Eigen::VectorXd c(num_cnst);
  Eigen::MatrixXd dc(num_cnst,nq*n_breaks);
  cnst->eval(t_ptr,n_breaks,q,c,dc);
  Eigen::VectorXd lb(num_cnst);
  Eigen::VectorXd ub(num_cnst);
  cnst->bounds(t_ptr,n_breaks,lb,ub);
  std::vector<std::string> cnst_names;
  cnst->name(t_ptr,n_breaks,cnst_names);
  int retvec_size;
  if(num_cnst == 0)
  {
    retvec_size = 0;
  }
  else
  {
    retvec_size = 1;
  }
  plhs[0] = mxCreateDoubleScalar((double) type);
  plhs[1] = mxCreateDoubleScalar((double) num_cnst);
  plhs[2] = mxCreateDoubleMatrix(num_cnst,retvec_size,mxREAL);
  memcpy(mxGetPrSafe(plhs[2]),c.data(),sizeof(double)*num_cnst);
  plhs[3] = mxCreateDoubleMatrix(num_cnst,nq*n_breaks,mxREAL);
  memcpy(mxGetPrSafe(plhs[3]),dc.data(),sizeof(double)*num_cnst*nq*n_breaks);
  int name_ndim = 1;
  mwSize name_dims[] = {(mwSize) num_cnst};
  plhs[4] = mxCreateCellArray(name_ndim,name_dims);
  mxArray *name_ptr;
  for(int i = 0;i<num_cnst;i++)
  {
    name_ptr = mxCreateString(cnst_names[i].c_str());
    mxSetCell(plhs[4],i,name_ptr);
  }
  plhs[5] = mxCreateDoubleMatrix(num_cnst,retvec_size,mxREAL);
  plhs[6] = mxCreateDoubleMatrix(num_cnst,retvec_size,mxREAL);
  memcpy(mxGetPrSafe(plhs[5]),lb.data(),sizeof(double)*num_cnst);
  memcpy(mxGetPrSafe(plhs[6]),ub.data(),sizeof(double)*num_cnst);
  delete[] t_ptr;
}
