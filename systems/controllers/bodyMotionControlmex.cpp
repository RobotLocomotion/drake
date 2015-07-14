/* 
 * A simple PD control block for regulating a body pose given a desired position, velocity, and acceleration.   
 */
#include "controlUtil.h"
#include "drakeMexUtil.h"

struct BodyMotionControlData {
  RigidBodyManipulator* r;
  Vector6d Kp;
  Vector6d Kd;
  int body_index;
};

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  if (nrhs<1) mexErrMsgTxt("usage: ptr = bodyMotionControlmex(0,robot_obj,Kp,Kd,body_index); y=bodyMotionControlmex(ptr,x,body_pose_des,body_v_des,body_vdot_des)");
  if (nlhs<1) mexErrMsgTxt("take at least one output... please.");
  
  struct BodyMotionControlData* pdata;

  if (mxGetScalar(prhs[0])==0) { // then construct the data object and return
    pdata = new struct BodyMotionControlData;
    
    // get robot mex model ptr
    if (!mxIsNumeric(prhs[1]) || mxGetNumberOfElements(prhs[1])!=1)
      mexErrMsgIdAndTxt("DRC:bodyMotionControlmex:BadInputs","the second argument should be the robot mex ptr");
    memcpy(&(pdata->r),mxGetData(prhs[1]),sizeof(pdata->r));
        
    if (!mxIsNumeric(prhs[2]) || mxGetM(prhs[2])!=6 || mxGetN(prhs[2])!=1)
    mexErrMsgIdAndTxt("DRC:bodyMotionControlmex:BadInputs","the third argument should be Kp");
    memcpy(&(pdata->Kp),mxGetPrSafe(prhs[2]),sizeof(pdata->Kp));

    if (!mxIsNumeric(prhs[3]) || mxGetM(prhs[3])!=6 || mxGetN(prhs[3])!=1)
    mexErrMsgIdAndTxt("DRC:bodyMotionControlmex:BadInputs","the fourth argument should be Kd");
    memcpy(&(pdata->Kd),mxGetPrSafe(prhs[3]),sizeof(pdata->Kd));

    pdata->body_index = (int) mxGetScalar(prhs[4]) -1;

    mxClassID cid;
    if (sizeof(pdata)==4) cid = mxUINT32_CLASS;
    else if (sizeof(pdata)==8) cid = mxUINT64_CLASS;
    else mexErrMsgIdAndTxt("Drake:bodyMotionControlmex:PointerSize","Are you on a 32-bit machine or 64-bit machine??");
     
    plhs[0] = mxCreateNumericMatrix(1,1,cid,mxREAL);
    memcpy(mxGetData(plhs[0]),&pdata,sizeof(pdata));
     
    return;
  }
  
  // first get the ptr back from matlab
  if (!mxIsNumeric(prhs[0]) || mxGetNumberOfElements(prhs[0])!=1)
    mexErrMsgIdAndTxt("DRC:bodyMotionControlmex:BadInputs","the first argument should be the ptr");

  memcpy(&pdata,mxGetData(prhs[0]),sizeof(pdata));

  int nq = pdata->r->num_positions;
  int nv = pdata->r->num_velocities;

  int narg = 1;
  
  Map<VectorXd> q(mxGetPrSafe(prhs[narg]), nq);
  Map<VectorXd> qd(mxGetPrSafe(prhs[narg++]) + nq, nv);

  assert(mxGetM(prhs[narg])==6); assert(mxGetN(prhs[narg])==1);
  Map< Vector6d > body_pose_des(mxGetPrSafe(prhs[narg++]));

  assert(mxGetM(prhs[narg])==6); assert(mxGetN(prhs[narg])==1);
  Map< Vector6d > body_v_des(mxGetPrSafe(prhs[narg++]));

  assert(mxGetM(prhs[narg])==6); assert(mxGetN(prhs[narg])==1);
  Map< Vector6d > body_vdot_des(mxGetPrSafe(prhs[narg++]));

  pdata->r->doKinematics(q,false,qd);

  // TODO: this must be updated to use quaternions/spatial velocity
  Vector6d body_pose;
  MatrixXd J = MatrixXd::Zero(6,pdata->r->num_positions);
  Vector3d zero = Vector3d::Zero();
  pdata->r->forwardKin(pdata->body_index,zero,1,body_pose);
  pdata->r->forwardJac(pdata->body_index,zero,1,J);
  
  Vector6d body_error;
  body_error.head<3>()= body_pose_des.head<3>()-body_pose.head<3>();

  Vector3d error_rpy,pose_rpy,des_rpy;
  pose_rpy = body_pose.tail<3>();
  des_rpy = body_pose_des.tail<3>();
  angleDiff(pose_rpy,des_rpy,error_rpy);
  body_error.tail(3) = error_rpy;

  Vector6d body_vdot = (pdata->Kp.array()*body_error.array()).matrix() + (pdata->Kd.array()*(body_v_des-J*qd).array()).matrix() + body_vdot_des;
  
  plhs[0] = eigenToMatlab(body_vdot);
}


