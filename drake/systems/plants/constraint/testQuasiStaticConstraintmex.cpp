#include "mex.h"
#include "RigidBodyConstraint.h"
#include "drakeMexUtil.h"
#include "../RigidBodyManipulator.h"
#include <cstring>
/*
 * [active_flag, num_weights,constraint,dconstraint,lower_bound,upper_bound] = testQuasiStaticConstraintmex(quasiStaticConstraint_ptr,q,weights,t)
 * @param quasiStaticConstraint_ptr       A pointer to a QuasiStaticConstraint object
 * @param q                               An nqx1 double vector, the joint angles (floating base)
 * @param weights                         A num_weightsx1 double vector, the weight at each contact point
 * @param t                               An optional argument. The time to evaluate the constraint.
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
  if(nrhs!= 3 && nrhs != 4)
  {
    mexErrMsgIdAndTxt("Drake:testQuasiStaticConstraintmex:BadInputs","Usage [active,num_weights,c,dc] = testQuasiStaticConstraintmex(qsc_ptr,q,weights,t)");
  }
  double t;
  double* t_ptr;
  if(nrhs == 4&&mxGetNumberOfElements(prhs[3])== 1)
  {
    t = mxGetScalar(prhs[3]);
    t_ptr = &t;
  }
  else
  {
    t_ptr = nullptr;
  }
  QuasiStaticConstraint* qsc = (QuasiStaticConstraint*) getDrakeMexPointer(prhs[0]);
  bool active = qsc->isActive();
  RigidBodyManipulator* model = qsc->getRobotPointer();
  int nq = model->num_positions;
  Map<VectorXd> q(mxGetPrSafe(prhs[1]), nq);
  int num_weights = qsc->getNumWeights();
  double* weights = new double[num_weights];
  memcpy(weights,mxGetPrSafe(prhs[2]),sizeof(double)*num_weights);
  int num_qsc_cnst = qsc->getNumConstraint(t_ptr);
  VectorXd c(num_qsc_cnst-1);
  MatrixXd dc = MatrixXd::Zero(num_qsc_cnst-1,nq+num_weights);
  KinematicsCache<double> cache = model->doKinematics(q, 0);
  qsc->eval(t_ptr, cache, weights, c, dc);
  VectorXd lb,ub;
  lb.resize(num_qsc_cnst-1);
  ub.resize(num_qsc_cnst-1);
  qsc->bounds(t_ptr,lb,ub);
  plhs[0] = mxCreateLogicalScalar(active);
  plhs[1] = mxCreateDoubleScalar((double) num_weights);
  plhs[2] = mxCreateDoubleMatrix(num_qsc_cnst-1,1,mxREAL);
  memcpy(mxGetPrSafe(plhs[2]),c.data(),sizeof(double)*(num_qsc_cnst-1));
  plhs[3] = mxCreateDoubleMatrix(num_qsc_cnst-1,nq+num_weights,mxREAL);
  memcpy(mxGetPrSafe(plhs[3]),dc.data(),sizeof(double)*dc.size());
  plhs[4] = mxCreateDoubleMatrix(num_qsc_cnst-1,1,mxREAL);
  memcpy(mxGetPrSafe(plhs[4]),lb.data(),sizeof(double)*(num_qsc_cnst-1));
  plhs[5] = mxCreateDoubleMatrix(num_qsc_cnst-1,1,mxREAL);
  memcpy(mxGetPrSafe(plhs[5]),ub.data(),sizeof(double)*(num_qsc_cnst-1));
  delete[] weights;
}
