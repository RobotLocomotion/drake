#include "../drakeQuatUtil.h"
#include "mex.h"
using namespace Eigen;

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
  if(nrhs != 5)
  {
    mexErrMsgIdAndTxt("Drake:testQuatmex:BadInputs","Usage [r,dr,e,ed,quat,dquat] = testQuatmex(q1,q2,axis,u,v)");
  }
  Vector4d q1;
  Vector4d q2;
  memcpy(q1.data(),mxGetPr(prhs[0]),sizeof(double)*4);
  memcpy(q2.data(),mxGetPr(prhs[1]),sizeof(double)*4);
  Vector4d r;
  Matrix<double,4,8> dr;
  quatDiff(q1,q2,r,dr);
  plhs[0] = mxCreateDoubleMatrix(4,1,mxREAL);
  plhs[1] = mxCreateDoubleMatrix(4,8,mxREAL);
  memcpy(mxGetPr(plhs[0]),r.data(),sizeof(double)*4);
  memcpy(mxGetPr(plhs[1]),dr.data(),sizeof(double)*4*8);

  Vector3d axis;
  memcpy(axis.data(),mxGetPr(prhs[2]),sizeof(double)*3);
  double e;
  Matrix<double,1,11> de;
  quatDiffAxisInvar(q1,q2,axis,e,de);
  plhs[2] = mxCreateDoubleScalar(e);
  plhs[3] = mxCreateDoubleMatrix(1,11,mxREAL);
  memcpy(mxGetPr(plhs[3]),de.data(),sizeof(double)*11);

  Vector3d u,v;
  Vector4d quat;
  Matrix<double,4,6> dquat;
  memcpy(u.data(),mxGetPr(prhs[3]),sizeof(double)*3);
  memcpy(v.data(),mxGetPr(prhs[4]),sizeof(double)*3);
  quatTransform(u,v,quat,dquat);
  plhs[4] = mxCreateDoubleMatrix(4,1,mxREAL);
  plhs[5] = mxCreateDoubleMatrix(4,6,mxREAL);
  memcpy(mxGetPr(plhs[4]),quat.data(),sizeof(double)*4);
  memcpy(mxGetPr(plhs[5]),dquat.data(),sizeof(double)*4*6);
}
