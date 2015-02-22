/* 
 * A simple PD control block for regulating a body pose given a desired position, velocity, and acceleration.   
 */
#include "controlUtil.h"
#include "drakeUtil.h"

RigidBodyManipulator* r;
int body_index;


void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  if (nrhs<1) mexErrMsgTxt("usage: y=statelessBodyMotionControlmex(ptr,x,body_ind,body_pose_des,body_v_des,body_vdot_des,Kp,Kd)");
  if (nlhs<1) mexErrMsgTxt("take at least one output... please.");
  
  // first get the ptr back from matlab
  if (!mxIsNumeric(prhs[0]) || mxGetNumberOfElements(prhs[0])!=1)
    mexErrMsgIdAndTxt("DRC:bodyMotionControlmex:BadInputs","the first argument should be the ptr");

  memcpy(&r,mxGetData(prhs[0]),sizeof(r));


  int nq = r->num_dof;

  int narg = 1;
  double *q = mxGetPr(prhs[narg++]);
  double *qd = &q[nq];
  Map< VectorXd > qdvec(qd,nq);

  body_index = (int) mxGetScalar(prhs[narg++]) - 1;

  assert(mxGetM(prhs[narg])==6); assert(mxGetN(prhs[narg])==1);
  Map< Vector6d > body_pose_des(mxGetPr(prhs[narg++]));

  assert(mxGetM(prhs[narg])==6); assert(mxGetN(prhs[narg])==1);
  Map< Vector6d > body_v_des(mxGetPr(prhs[narg++]));

  assert(mxGetM(prhs[narg])==6); assert(mxGetN(prhs[narg])==1);
  Map< Vector6d > body_vdot_des(mxGetPr(prhs[narg++]));

  assert(mxGetM(prhs[narg])==6); assert(mxGetN(prgs[narg])==1);
  Map< Vector6d > Kp(mxGetPr(prhs[narg++]));

  assert(mxGetM(prhs[narg])==6); assert(mxGetN(prgs[narg])==1);
  Map< Vector6d > Kd(mxGetPr(prhs[narg++]));

  r->doKinematics(q,false,qd);

  // TODO: this must be updated to use quaternions/spatial velocity
  Vector6d body_pose;
  MatrixXd J = MatrixXd::Zero(6,r->num_dof);
  Vector4d zero = Vector4d::Zero();
  zero(3) = 1.0;
  r->forwardKin(body_index,zero,1,body_pose);
  r->forwardJac(body_index,zero,1,J);
  
  Vector6d body_error;
  body_error.head<3>()= body_pose_des.head<3>()-body_pose.head<3>();

  Vector3d error_rpy,pose_rpy,des_rpy;
  pose_rpy = body_pose.tail<3>();
  des_rpy = body_pose_des.tail<3>();
  angleDiff(pose_rpy,des_rpy,error_rpy);
  body_error.tail(3) = error_rpy;

  Vector6d body_vdot = (Kp.array()*body_error.array()).matrix() + (Kd.array()*(body_v_des-J*qdvec).array()).matrix() + body_vdot_des;
  
  plhs[0] = eigenToMatlab(body_vdot);
}


