#include "QPCommon.h"
#include <Eigen/StdVector>

using namespace std;


void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  if (nrhs<1) mexErrMsgTxt("usage: ptr = constructQPStatePointerMex(robot_ptr);");
  if (nlhs<1) mexErrMsgTxt("take at least one output... please.");

  int narg = 0;
  struct QPControllerState* pdata;
  pdata = new struct QPControllerState;
  RigidBodyManipulator *r;

  if (!mxIsNumeric(prhs[narg]) || mxGetNumberOfElements(prhs[narg])!=1)
      mexErrMsgIdAndTxt("Drake:constructQPDataPointer:BadInputs","the first argument should be the robot mex ptr");

  memcpy(&(r), mxGetData(prhs[narg]), sizeof(r));
  narg++;

  pdata->t_prev = 0;
  pdata->foot_contact_prev[0] = true;
  pdata->foot_contact_prev[1] = true;
  pdata->vref_integrator_state = VectorXd::Zero(r->num_velocities);
  pdata->q_integrator_state = VectorXd::Zero(r->num_dof);

  mxClassID cid;
  if (sizeof(pdata)==4) cid = mxUINT32_CLASS;
  else if (sizeof(pdata)==8) cid = mxUINT64_CLASS;
  else mexErrMsgIdAndTxt("Drake:constructModelmex:PointerSize","Are you on a 32-bit machine or 64-bit machine??");
  
  plhs[0] = mxCreateNumericMatrix(1,1,cid,mxREAL);
  memcpy(mxGetData(plhs[0]),&pdata,sizeof(pdata));

  return;
}












