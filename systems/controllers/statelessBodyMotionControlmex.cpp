/* 
 * A simple PD control block for regulating a body pose given a desired position, velocity, and acceleration.   
 */
#include "controlUtil.h"
#include "drakeUtil.h"


void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  if (nrhs<1) mexErrMsgTxt("usage: y=statelessBodyMotionControlmex(robot,x,body_ind,body_pose_des,body_v_des,body_vdot_des,params)");
  if (nlhs<1) mexErrMsgTxt("take at least one output... please.");
  
  // first get the ptr back from matlab
  if (mxGetNumberOfElements(prhs[0])!=1)
    mexErrMsgIdAndTxt("DRC:bodyMotionControlmex:BadInputs","the first argument should be the robot");

  RigidBodyManipulator* r;
  mxArray* pm;
  int body_index;

  memcpy(&r,mxGetData(myGetProperty(myGetProperty(prhs[0],"mex_model_ptr"),"ptr")),sizeof(r));

  int narg = 1;
  int nq = r->num_dof;
  double *q = mxGetPr(prhs[narg++]);
  double *qd = &q[nq];

  body_index = (int) mxGetScalar(prhs[narg++]) - 1;

  assert(mxGetM(prhs[narg])==6); assert(mxGetN(prhs[narg])==1);
  Map< Vector6d > body_pose_des(mxGetPr(prhs[narg++]));

  assert(mxGetM(prhs[narg])==6); assert(mxGetN(prhs[narg])==1);
  Map< Vector6d > body_v_des(mxGetPr(prhs[narg++]));

  assert(mxGetM(prhs[narg])==6); assert(mxGetN(prhs[narg])==1);
  Map< Vector6d > body_vdot_des(mxGetPr(prhs[narg++]));

  pm = mxGetField(prhs[narg], 0, "Kp");
  assert(mxGetM(pm)==6); assert(mxGetN(pm) == 1);
  Map< Vector6d > Kp(mxGetPr(pm));
  // assert(mxGetM(mxGetField(prhs[narg]),0,"Kp"))==6); assert(mxGetN(prgs[narg])==1);
  // Map< Vector6d > Kp(mxGetPr(prhs[narg++]));

  pm = mxGetField(prhs[narg], 0, "Kd");
  assert(mxGetM(pm)==6); assert(mxGetN(pm) == 1);
  Map< Vector6d > Kd(mxGetPr(pm));
  // assert(mxGetM(prhs[narg])==6); assert(mxGetN(prgs[narg])==1);
  // Map< Vector6d > Kd(mxGetPr(prhs[narg++]));

  Vector6d body_vdot = bodyMotionPD(r, q, qd, body_index, body_pose_des, body_v_des, body_vdot_des, Kp, Kd);
  
  plhs[0] = eigenToMatlab(body_vdot);
}


