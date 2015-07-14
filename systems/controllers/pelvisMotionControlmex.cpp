/*
 * A simple pelvis motion control block for use with bipeds. Uses PD control to regulate the pelvis
 * to a fixed height above the feet and drives the yaw to match the average foot yaw.
 *
 */

#include "controlUtil.h"
#include "drakeUtil.h"
#include "drakeMexUtil.h"

struct PelvisMotionControlData {
  RigidBodyManipulator* r;
  double alpha;
  double pelvis_height_previous;
  double nominal_pelvis_height;
  Vector6d Kp;
  Vector6d Kd;

  int pelvis_body_index;
  int rfoot_body_index;
  int lfoot_body_index;
};

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  if (nrhs<1) mexErrMsgTxt("usage: ptr = pelvisMotionControlmex(0,robot_obj,alpha,nominal_pelvis_height,Kp,Kd); y=pelvisMotionControlmex(ptr,x)");
  if (nlhs<1) mexErrMsgTxt("take at least one output... please.");

  struct PelvisMotionControlData* pdata;

  if (mxGetScalar(prhs[0])==0) { // then construct the data object and return
    pdata = new struct PelvisMotionControlData;

    // get robot mex model ptr
    if (!mxIsNumeric(prhs[1]) || mxGetNumberOfElements(prhs[1])!=1)
      mexErrMsgIdAndTxt("DRC:pelvisMotionControlmex:BadInputs","the second argument should be the robot mex ptr");
    memcpy(&(pdata->r),mxGetData(prhs[1]),sizeof(pdata->r));

    if (!mxIsNumeric(prhs[2]) || mxGetNumberOfElements(prhs[2])!=1)
    mexErrMsgIdAndTxt("DRC:pelvisMotionControlmex:BadInputs","the third argument should be alpha");
    memcpy(&(pdata->alpha),mxGetPrSafe(prhs[2]),sizeof(pdata->alpha));

    if (!mxIsNumeric(prhs[3]) || mxGetNumberOfElements(prhs[3])!=1)
    mexErrMsgIdAndTxt("DRC:pelvisMotionControlmex:BadInputs","the fourth argument should be nominal_pelvis_height");
    memcpy(&(pdata->nominal_pelvis_height),mxGetPrSafe(prhs[3]),sizeof(pdata->nominal_pelvis_height));

    if (!mxIsNumeric(prhs[4]) || mxGetM(prhs[4])!=6 || mxGetN(prhs[4])!=1)
    mexErrMsgIdAndTxt("DRC:pelvisMotionControlmex:BadInputs","the fifth argument should be Kp");
    memcpy(&(pdata->Kp),mxGetPrSafe(prhs[4]),sizeof(pdata->Kp));

    if (!mxIsNumeric(prhs[5]) || mxGetM(prhs[5])!=6 || mxGetN(prhs[5])!=1)
    mexErrMsgIdAndTxt("DRC:pelvisMotionControlmex:BadInputs","the sixth argument should be Kd");
    memcpy(&(pdata->Kd),mxGetPrSafe(prhs[5]),sizeof(pdata->Kd));


    mxClassID cid;
    if (sizeof(pdata)==4) cid = mxUINT32_CLASS;
    else if (sizeof(pdata)==8) cid = mxUINT64_CLASS;
    else mexErrMsgIdAndTxt("Drake:pelvisMotionControlmex:PointerSize","Are you on a 32-bit machine or 64-bit machine??");

    pdata->pelvis_height_previous = -10001;

    pdata->pelvis_body_index = pdata->r->findLinkId("pelvis", 0);
    pdata->rfoot_body_index = pdata->r->findLinkId("r_foot", 0);
    pdata->lfoot_body_index = pdata->r->findLinkId("l_foot", 0);

    plhs[0] = mxCreateNumericMatrix(1,1,cid,mxREAL);
    memcpy(mxGetData(plhs[0]),&pdata,sizeof(pdata));

    return;
  }

  // first get the ptr back from matlab
  if (!mxIsNumeric(prhs[0]) || mxGetNumberOfElements(prhs[0])!=1)
    mexErrMsgIdAndTxt("DRC:pelvisMotionControlmex:BadInputs","the first argument should be the ptr");
  memcpy(&pdata,mxGetData(prhs[0]),sizeof(pdata));

  int nq = pdata->r->num_positions;
  int nv = pdata->r->num_velocities;

  int narg = 1;
  Map<VectorXd> q(mxGetPrSafe(prhs[narg]), nq);
  Map<VectorXd> qd(mxGetPrSafe(prhs[narg++]) + nq, nv);
  double lfoot_yaw = mxGetScalar(prhs[narg++]);
  double rfoot_yaw = mxGetScalar(prhs[narg++]);
  double foot_z = mxGetScalar(prhs[narg++]);

  pdata->r->doKinematics(q,false,qd);

  // TODO: this must be updated to use quaternions/spatial velocity
  Vector6d pelvis_pose;
  MatrixXd Jpelvis = MatrixXd::Zero(6,pdata->r->num_positions);
  Vector3d zero = Vector3d::Zero();
  pdata->r->forwardKin(pdata->pelvis_body_index,zero,1,pelvis_pose);
  pdata->r->forwardJac(pdata->pelvis_body_index,zero,1,Jpelvis);

  if (pdata->pelvis_height_previous<-10000) {
    pdata->pelvis_height_previous = pelvis_pose(2);
  }

  double mean_foot_yaw = angleAverage(lfoot_yaw,rfoot_yaw);

  double pelvis_height_desired = pdata->alpha*pdata->pelvis_height_previous + (1.0-pdata->alpha)*(foot_z + pdata->nominal_pelvis_height);
  pdata->pelvis_height_previous = pelvis_height_desired;

  Vector6d body_des;
  double nan = std::numeric_limits<double>::quiet_NaN();
  body_des << nan,nan,pelvis_height_desired,0,0,mean_foot_yaw;
  Vector6d error;
  error.head<3>()= body_des.head<3>()-pelvis_pose.head<3>();

  Vector3d error_rpy,pose_rpy,des_rpy;
  pose_rpy = pelvis_pose.tail<3>();
  des_rpy = body_des.tail<3>();
  angleDiff(pose_rpy,des_rpy,error_rpy);
  error.tail(3) = error_rpy;

  Vector6d body_vdot = (pdata->Kp.array()*error.array()).matrix() - (pdata->Kd.array()*(Jpelvis*qd).array()).matrix();

  plhs[0] = eigenToMatlab(body_vdot);
}
