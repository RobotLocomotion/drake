#include "drakeGeometryUtil.h"
#include "mex.h"
using namespace Eigen;

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
  if(nrhs != 5)
  {
    mexErrMsgIdAndTxt("Drake:testQuatmex:BadInputs","Usage [r,dr,e,ed,quat,dquat,q3,dq3,w,dw] = testQuatmex(q1,q2,axis,u,v)");
  }
  Vector4d q1;
  Vector4d q2;
  memcpy(q1.data(),mxGetPr(prhs[0]),sizeof(double)*4);
  memcpy(q2.data(),mxGetPr(prhs[1]),sizeof(double)*4);
  Vector4d r = quatDiff(q1,q2);
  Matrix<double,4,8> dr = dquatDiff(q1,q2);

  plhs[0] = mxCreateDoubleMatrix(4,1,mxREAL);
  plhs[1] = mxCreateDoubleMatrix(4,8,mxREAL);
  memcpy(mxGetPr(plhs[0]),r.data(),sizeof(double)*4);
  memcpy(mxGetPr(plhs[1]),dr.data(),sizeof(double)*4*8);

  Vector3d axis;
  memcpy(axis.data(),mxGetPr(prhs[2]),sizeof(double)*3);
  double e = quatDiffAxisInvar(q1,q2,axis);
  Matrix<double,1,11> de = dquatDiffAxisInvar(q1,q2,axis);
  plhs[2] = mxCreateDoubleScalar(e);
  plhs[3] = mxCreateDoubleMatrix(1,11,mxREAL);
  memcpy(mxGetPr(plhs[3]),de.data(),sizeof(double)*11);

  Vector3d u,v;
  Vector4d quat;
  Matrix<double,4,6> dquat;
  memcpy(u.data(),mxGetPr(prhs[3]),sizeof(double)*3);
  memcpy(v.data(),mxGetPr(prhs[4]),sizeof(double)*3);

  Vector4d q3 = quatProduct(q1,q2);
  Matrix<double,4,8> dq3 = dquatProduct(q1,q2);

  plhs[4] = mxCreateDoubleMatrix(4,1,mxREAL);
  plhs[5] = mxCreateDoubleMatrix(4,8,mxREAL);
  memcpy(mxGetPr(plhs[4]),q3.data(),sizeof(double)*4);
  memcpy(mxGetPr(plhs[5]),dq3.data(),sizeof(double)*4*8);

  Vector3d w = quatRotateVec(q1,u);
  Matrix<double,3,7> dw = dquatRotateVec(q1,u);

  plhs[6] = mxCreateDoubleMatrix(3,1,mxREAL);
  plhs[7] = mxCreateDoubleMatrix(3,7,mxREAL);
  memcpy(mxGetPr(plhs[6]),w.data(),sizeof(double)*3);
  memcpy(mxGetPr(plhs[7]),dw.data(),sizeof(double)*3*7);
}
