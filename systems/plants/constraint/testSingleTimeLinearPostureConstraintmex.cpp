#include "mex.h"
#include "RigidBodyConstraint.h"
#include "drakeUtil.h"
#include "../RigidBodyManipulator.h"
#include <cstring>
/* 
 * [num_constraint,constraint_val,iAfun,jAvar,A,constraint_name,lower_bound,upper_bound] = testSingleTimeLinearPostureConstraintmex(stlpc_ptr,q,t)
 * @param stlpc_ptr             A pointer to a SingleTimeLinearPostureConstraint object
 * @param q                     A nqx1 double vector
 * @param t                     A double array, the time moments to evaluate constraint value, bounds and name. 
 * @retval num_constraint       The number of constraint active at time t
 * @retval iAfun                The row index of non-zero element in the sparse linear constraint 
 * @retval jAvar                The column index of the non-zero element in the sparse linear constraint
 * @retval A                    The value of the non-zeroelement in the sparse linear constraint
 * @retval constraint_name      The name of the constraint at time t
 * @retval lower_bound          The lower bound of the constraint at time t
 * @retval upper_bound          The upper bound of the constraint at time t
 * */

using namespace std;
using namespace Eigen;

void mexFunction(int nlhs,mxArray* plhs[], int nrhs, const mxArray * prhs[])
{
  if(nrhs!=3 || nlhs != 8)
  {
    mexErrMsgIdAndTxt("Drake:testSingleTimeLinearPostureConstraintmex:BadInputs","Usage [num_cnst,cnst_val,iAfun,jAvar,A,cnst_name,lb,ub] = testSingleTimeLinearPostureConstraintmex(stlpc_ptr,q,t)");
  }
  SingleTimeLinearPostureConstraint* stlpc = (SingleTimeLinearPostureConstraint*) getDrakeMexPointer(prhs[0]);
  int nq = stlpc->getRobotPointer()->num_dof;
  if(!mxIsNumeric(prhs[1]) || mxGetN(prhs[1]) != 1 || mxGetM(prhs[1]) != nq)
  {
    mexErrMsgIdAndTxt("Drake:testSingleTimeLinearPostureConstraintmex:BadInputs","q must a numeric column vector with size nq");
  }
  VectorXd q(nq);
  memcpy(q.data(),mxGetPr(prhs[1]),sizeof(double)*nq);
  double* t_ptr = NULL;
  if(mxGetNumberOfElements(prhs[2]) == 0)
  {
    t_ptr = NULL;
  }
  else if(mxGetNumberOfElements(prhs[2]) == 1)
  {
    t_ptr = mxGetPr(prhs[2]);
  }
  int num_cnst = stlpc->getNumConstraint(t_ptr);
  VectorXd c(num_cnst);
  stlpc->feval(t_ptr,q,c);
  VectorXi iAfun, jAvar;
  VectorXd A;
  stlpc->geval(t_ptr,iAfun,jAvar,A);
  vector<string> cnst_names;
  stlpc->name(t_ptr,cnst_names);
  VectorXd lb,ub;
  stlpc->bounds(t_ptr,lb,ub);
  plhs[0] = mxCreateDoubleScalar((double) num_cnst);
  int retvec_size;
  if(num_cnst == 0)
  {
    retvec_size = 0;
  }
  else
  {
    retvec_size = 1;
  }
  plhs[1] = mxCreateDoubleMatrix(num_cnst,retvec_size,mxREAL);
  memcpy(mxGetPr(plhs[1]),c.data(),sizeof(double)*num_cnst);
  plhs[2] = mxCreateDoubleMatrix(iAfun.size(),retvec_size,mxREAL);
  plhs[3] = mxCreateDoubleMatrix(jAvar.size(),retvec_size,mxREAL);
  plhs[4] = mxCreateDoubleMatrix(A.size(),retvec_size,mxREAL);
  for(int i = 0;i<iAfun.size();i++)
  {
    *(mxGetPr(plhs[2])+i) = (double) iAfun(i)+1;
    *(mxGetPr(plhs[3])+i) = (double) jAvar(i)+1;
    *(mxGetPr(plhs[4])+i) = A(i);
  }
  int name_ndim = 1;
  mwSize name_dims[] = {(mwSize) num_cnst};
  plhs[5] = mxCreateCellArray(name_ndim,name_dims);
  mxArray* name_ptr;
  for(int i = 0;i<num_cnst;i++)
  {
    name_ptr = mxCreateString(cnst_names[i].c_str());
    mxSetCell(plhs[5],i,name_ptr);
  }
  plhs[6] = mxCreateDoubleMatrix(num_cnst,retvec_size,mxREAL);
  plhs[7] = mxCreateDoubleMatrix(num_cnst,retvec_size,mxREAL);
  memcpy(mxGetPr(plhs[6]),lb.data(),sizeof(double)*num_cnst);
  memcpy(mxGetPr(plhs[7]),ub.data(),sizeof(double)*num_cnst);
}
