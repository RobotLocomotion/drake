#include "mex.h"
#include "Constraint.h"
#include "drakeUtil.h"
#include "../RigidBodyManipulator.h"
#include <cstring>
/*
 * [active_flag, num_weights,constraint,dconstraint,lower_bound,upper_bound] = testQuasiStaticConstraintmex(quasiStaticConstraint_ptr,q,weights)
 * @param quasiStaticConstraint_ptr       A pointer to a QuasiStaticConstraint object
 * @param q                               An nqx1 double vector, the joint angles (floating base)
 * @param weights                         A num_weightsx1 double vector, the weight at each contact point
 * @retval active_flag                    A boolean flag, true if the QuasiStaticConstraint is active
 * @retval num_weights                    The number of contact points
 * @retval constraint                     The value of the constraint
 * @retval dconstraint                    The gradient of the constraint w.r.t [q;weights]
 * @retval lower_bound                    The lower bound of the constraint
 * @retval upper_bound                    The upper bound of the constraint
 * */
using namespace Eigen;
using namespace std;

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  if(nrhs!= 3)
  {
    mexErrMsgIdAndTxt("Drake:testQuasiStaticConstraintmex:BadInputs","Usage [active,num_weights,c,dc] = testQuasiStaticConstraintmex(qsc_ptr,q,weights)");
  }
  QuasiStaticConstraint* qsc = (QuasiStaticConstraint*) getDrakeMexPointer(prhs[0]);
  bool active = qsc->isActive();
  RigidBodyManipulator* model = qsc->getRobotPointer();
  int nq = model->num_dof;
  double *q = new double[nq];
  memcpy(q,mxGetPr(prhs[1]),sizeof(double)*nq);
  int num_weights = qsc->getNumWeights();
  double* weights = new double[num_weights];
  memcpy(weights,mxGetPr(prhs[2]),sizeof(double)*num_weights);
  Vector2d c;
  MatrixXd dc(2,nq+num_weights);
  model->doKinematics(q);
  qsc->eval(weights,c,dc);
  Vector2d lb,ub;
  qsc->bounds(lb,ub);
  plhs[0] = mxCreateLogicalScalar(active);
  plhs[1] = mxCreateDoubleScalar((double) num_weights);
  plhs[2] = mxCreateDoubleMatrix(2,1,mxREAL);
  memcpy(mxGetPr(plhs[2]),c.data(),sizeof(double)*2);
  plhs[3] = mxCreateDoubleMatrix(2,nq+num_weights,mxREAL);
  memcpy(mxGetPr(plhs[3]),dc.data(),sizeof(double)*2*(nq+num_weights));
  plhs[4] = mxCreateDoubleMatrix(2,1,mxREAL);
  memcpy(mxGetPr(plhs[4]),lb.data(),sizeof(double)*2);
  plhs[5] = mxCreateDoubleMatrix(2,1,mxREAL);
  memcpy(mxGetPr(plhs[5]),ub.data(),sizeof(double)*2);
  delete[] q;
  delete[] weights;
}
