#include "mex.h"
#include "Constraint.h"
#include "drakeUtil.h"
#include "../RigidBodyManipulator.h"
#include <cstring>
/* 
 * [num_constraint,constraint_val,dconstraint_val,constraint_name,lower_bound,upper_bound] = testSingleTimeKinCnstmex(kinCnst_ptr,q,t)
 * @param kinCnst_ptr           A pointer to a SingleTimeKinematicConstraint object
 * @param q                     A nqx1 double vector
 * @param t                     A double scalar, the time to evaluate constraint value, bounds and name. This is optional.
 * @retval num_constraint       The number of constraint active at time t
 * @retval constraint_val       The value of the constraint at time t
 * @retval dconstraint_val      The gradient of the constraint w.r.t q at time t
 * @retval constraint_name      The name of the constraint at time t
 * @retval lower_bound          The lower bound of the constraint at time t
 * @retval upper_bound          The upper bound of the constraint at time t
 * */
void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
  if((nrhs!=3&&nrhs != 2)||nlhs != 6)
  {
    mexErrMsgIdAndTxt("Drake:testSingleTimeKinCnstmex:BadInputs","Usage [num_cnst,cnst_val,dcnst_val,cnst_name,lb,ub] = testSingleTimeKinKinCnstmex(kinCnst,q,t)");
  }
  SingleTimeKinematicConstraint* cnst = (SingleTimeKinematicConstraint*) getDrakeMexPointer(prhs[0]);
  double* t_ptr;
  if(nrhs == 2)
  {
    t_ptr = NULL;
  }
  else
  {
    int num_t = mxGetNumberOfElements(prhs[2]);
    if(num_t == 0)
    {
      t_ptr = NULL;
    }
    if(num_t == 1)
    {
      t_ptr = mxGetPr(prhs[2]);
    }
    if(num_t>1)
    {
      mexErrMsgIdAndTxt("Drake:testSingleTimeKinCnstmex:BadInputs","t must be either empty or a single number");
    }
  }
  int num_cnst = cnst->getNumConstraint(t_ptr);
  //mexPrintf("num_cnst = %d\n",num_cnst);
  double num_cnst_double = (double) num_cnst;
  int nq = cnst->getRobotPointer()->num_dof;
  double* q = new double[nq];
  memcpy(q,mxGetPr(prhs[1]),sizeof(double)*nq);
  cnst->getRobotPointer()->doKinematics(q);
  VectorXd c(num_cnst);
  MatrixXd dc(num_cnst,nq);
  cnst->eval(t_ptr,c,dc);
  //mexPrintf("get c,dc\n");
  VectorXd lb(num_cnst);
  VectorXd ub(num_cnst);
  cnst->bounds(t_ptr,lb,ub);
  //mexPrintf("get lb, ub\n");
  std::vector<std::string> cnst_names;
  cnst->name(t_ptr,cnst_names);
  //mexPrintf("get name\n");
  plhs[0] = mxCreateDoubleMatrix(1,1,mxREAL);
  memcpy(mxGetPr(plhs[0]),&num_cnst_double,sizeof(double));
  plhs[1] = mxCreateDoubleMatrix(num_cnst,1,mxREAL);
  memcpy(mxGetPr(plhs[1]),c.data(),sizeof(double)*num_cnst);
  plhs[2] = mxCreateDoubleMatrix(num_cnst,nq,mxREAL);
  memcpy(mxGetPr(plhs[2]),dc.data(),sizeof(double)*num_cnst*nq);
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
  delete[] q;
}
