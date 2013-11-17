#include "mex.h"
#include <iostream>
#include "drakeUtil.h"
#include <Eigen/Dense>
#include "RigidBodyConstraint.h"
#include "RigidBodyManipulator.h"
#include "constructPtrDrakeConstraint.h"
#include <cstdio>

using namespace Eigen;
using namespace std;

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  if(nrhs != 7 && nrhs != 8)
  {
    mexErrMsgIdAndTxt("Drake:constructPtrPoint2LineSegDistConstraintmex:BadInputs","Usage ptr = constructPtrPoint2LineSegDistConstraintmex(obj.robot.mex_model_ptr,pt_body,pt,line_body,line_ends,lb,ub,tspan");
  }
  RigidBodyManipulator *model = (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);
  Vector2d tspan;
  if(nrhs == 7)
  {
    tspan<< -mxGetInf(), mxGetInf();
  }
  else
  {
    drakeKinCnstParseTspan(prhs[7],tspan);
  }
  if(!mxIsNumeric(prhs[1])||mxGetNumberOfElements(prhs[1]) != 1)
  {
    mexErrMsgIdAndTxt("Drake:constructPtrPoint2LineSegDistConstraintmex:BadInputs","pt_body should be a numeric scalar");
  }
  int pt_body = (int) mxGetScalar(prhs[1])-1;
  if(pt_body>=model->num_bodies || pt_body<0)
  {
    mexErrMsgIdAndTxt("Drake:constructPtrPoint2LineSegDistConstraintmex:BadInputs","pt_body is invalid");
  }
  if(!mxIsNumeric(prhs[2]) || mxGetM(prhs[2]) != 3 || mxGetN(prhs[2]) != 1)
  {
    mexErrMsgIdAndTxt("Drake:constructPtrPoint2LineSegDistConstraintmex:BadInputs","pt should be a 3x1 vector");
  }
  Vector4d pt;
  memcpy(pt.data(),mxGetPr(prhs[2]),sizeof(double)*3);
  pt(3) = 1.0;
  if(!mxIsNumeric(prhs[3])||mxGetNumberOfElements(prhs[3]) != 1)
  {
    mexErrMsgIdAndTxt("Drake:constructPtrPoint2LineSegDistConstraintmex:BadInputs","line_body should be a numeric scalar");
  }
  int line_body = (int) mxGetScalar(prhs[3])-1;
  if(line_body>=model->num_bodies || line_body<0)
  {
    mexErrMsgIdAndTxt("Drake:constructPtrPoint2LineSegDistConstraintmex:BadInputs","line_body is invalid");
  }
  if(!mxIsNumeric(prhs[4]) || mxGetM(prhs[4]) != 3 || mxGetN(prhs[4]) != 2)
  {
    mexErrMsgIdAndTxt("Drake:constructPtrPoint2LineSegDistConstraintmex:BadInputs","line_ends should be a 3x2 vector");
  }
  Matrix<double,4,2> line_ends;
  Matrix<double,3,2> line_ends_tmp;
  memcpy(line_ends_tmp.data(),mxGetPr(prhs[4]),sizeof(double)*6);
  line_ends.block(0,0,3,2) = line_ends_tmp;
  line_ends.row(3) = MatrixXd::Ones(1,2);
  if(!mxIsNumeric(prhs[5]) || !mxIsNumeric(prhs[6]) || mxGetNumberOfElements(prhs[5]) != 1 || mxGetNumberOfElements(prhs[6]) != 1)
  {
    mexErrMsgIdAndTxt("Drake:constructPtrPoint2LineSegDistConstraintmex:BadInputs","dist_lb, dist_ub should be scalars");
  }
  double dist_lb = mxGetScalar(prhs[5]);
  double dist_ub = mxGetScalar(prhs[6]);
  if(dist_lb<0 || dist_lb>dist_ub)
  {
    mexErrMsgIdAndTxt("Drake:constructPtrPoint2LineSegDistConstraintmex:BadInputs","dist_lb should be nonnegative, and dist_ub should be no less than dist_lb");
  }
  Point2LineSegDistConstraint* cnst = new Point2LineSegDistConstraint(model,pt_body,pt,line_body,line_ends,dist_lb,dist_ub,tspan);
  plhs[0] = createDrakeConstraintMexPointer((void*) cnst,"deleteRigidBodyConstraintmex","Point2LineSegDistConstraint");
}
    


  

