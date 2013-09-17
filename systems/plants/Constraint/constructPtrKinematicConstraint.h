#ifndef  __CONSTRUCTPTRKINEMATICCONSTRAINT_H__
#define  __CONSTRUCTPTRKINEMATICCONSTRAINT_H__
#include "mex.h"
#include "RigidBodyManipulator.h"
#include <iostream>
#include <Eigen/Dense>
#include <cstdio>
#include "Constraint.h"

void drakeKinCnstParseTspan(const mxArray* pm,Eigen::Vector2d &tspan)
{
  if(!mxIsNumeric(pm))
  {
    mexErrMsgIdAndTxt("Drake:drakeKinCnstParseTspan:BadInputs","tspan is a 1x2 vector");
  }
  int num_tspan = mxGetNumberOfElements(pm);
  if(num_tspan == 0)
  {
    tspan<<-mxGetInf(),mxGetInf();
  }
  else
  {
    double* tspan_tmp = new double[num_tspan];
    memcpy(tspan_tmp,mxGetPr(pm),sizeof(double)*num_tspan);
    tspan<<tspan_tmp[0],tspan_tmp[num_tspan-1];
    if(tspan(0)>tspan(1))
    {
      mexErrMsgIdAndTxt("Drake:drakeKinCnstParseTspan:BadInputs","tspan[0] should be no larger than tspan[1]");
    }
    delete[] tspan_tmp;
  }
};

void drakeKinCnstParse3dUnitVector(const mxArray* pm, Vector3d &unit_vec)
{
  if(!mxIsNumeric(pm))
  {
    mexErrMsgIdAndTxt("Drake:drakeKinCnstParse3dUnitVector:BadInputs","vector should be a 3x1 double vector");
  }
  if(!(mxGetM(pm) == 3 && mxGetN(pm) == 1))
  {
    mexErrMsgIdAndTxt("Drake:drakeKinCnstParse3dUnitVector:BadInputs","vector should be of size 3x1");
  }
  memcpy(unit_vec.data(),mxGetPr(pm),sizeof(double)*3);
  double vec_norm = unit_vec.norm();
  if(vec_norm==0.0)
  {
    mexErrMsgIdAndTxt("Drake:drakeKinCnstParse3dUnitVector:BadInputs","The input cannot be a zero vector");
  }
  unit_vec = unit_vec/vec_norm;
};

void drakeKinCnstParseQuat(const mxArray* pm, Vector4d &quat)
{
  if(!mxIsNumeric(pm))
  {
    mexErrMsgIdAndTxt("Drake:drakeKinCnstParseQuat:BadInputs","The input argument 1 should be a 4x1 double vector");
  }
  if(!(mxGetM(pm) == 4 && mxGetN(pm) == 1))
  {
    mexErrMsgIdAndTxt("Drake:drakeKinCnstParseQuat:BadInputs","The input argument 1 should be of size 4x1");
  }
  memcpy(quat.data(),mxGetPr(pm),sizeof(double)*4);
  for(int i = 0;i<4;i++)
  {
    if((mxIsInf(quat(i)))||(mxIsNaN(quat(i))))
    {
      mexErrMsgIdAndTxt("Drake:drakeKinCnstParseQuat:BadInputs","The input argument 1 cannot have entry equal to NaN or Inf");
    }
  }
  double quat_norm = quat.norm();
  if(quat_norm==0.0)
  {
    mexErrMsgIdAndTxt("Drake:drakeKinCnstParseQuat:BadInputs","The Input argument 1 must be a nonzero vector");
  }
  quat = quat/quat_norm;
}

double drakeKinCnstParseGazeConethreshold(const mxArray* pm)
{
  if(mxGetNumberOfElements(pm) == 0)
  {
    return 0.0;
  }
  else
  {
    if(!mxIsNumeric(pm)||mxGetNumberOfElements(pm) != 1)
    {
      mexErrMsgIdAndTxt("Drake:drakeKinCnstParseGazeConethreshold:BadInputs","conethreshold must be a double scalar");
    }
    double conethreshold = mxGetScalar(pm);
    if(conethreshold<0.0 || conethreshold >M_PI)
    {
      mexErrMsgIdAndTxt("Drake:drakeKinCnstParseGazeConethreshold:BadInputs","conethreshold must be within [0 pi]");
    }
    return conethreshold;
  }
};

double drakeKinCnstParseGazeThreshold(const mxArray* pm)
{
  if(mxGetNumberOfElements(pm) == 0)
  {
    return M_PI;
  }
  else
  {
    if(!mxIsNumeric(pm)||mxGetNumberOfElements(pm) != 1)
    {
      mexErrMsgIdAndTxt("Drake:drakeKinCnstParseGazeThreshold:BadInputs","threshold must be a double scalar");
    }
    double threshold = mxGetScalar(pm);
    if(threshold<0.0 || threshold >M_PI)
    {
      mexErrMsgIdAndTxt("Drake:drakeKinCnstParseGazeThreshold:BadInputs","threshold must be within [0 pi]");
    }
    return threshold;
  }
}
#endif
