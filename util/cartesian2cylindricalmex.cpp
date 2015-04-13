#include "mex.h"
#include "drakeGeometryUtil.h"

using namespace Eigen;
using namespace std;

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  if(nrhs != 5){
    mexErrMsgIdAndTxt("Drake:cartesian2cylindricalmex:IncorrectInputs", "Usage [x_cartesian,v_cartesian,J,Jdotv] = cartesian2cylindrical(cylinder_axis,cylinder_x_dir,cylinder_origin,x_cylinder,v_cylinder)");
  }
  if(!mxIsNumeric(prhs[0]) || mxGetNumberOfElements(prhs[0]) != 3)
  {
    mexErrMsgIdAndTxt("Drake:cartesian2cylindricalmex:InvalidInput","cylinder_axis should be a 3 x 1 vector");
  }
  Vector3d cylinder_axis;
  memcpy(cylinder_axis.data(), mxGetPr(prhs[0]),sizeof(double)*3);
  if(!mxIsNumeric(prhs[1]) || mxGetNumberOfElements(prhs[1]) != 3)
  {
    mexErrMsgIdAndTxt("Drake:cartesian2cylindricalmex:InvalidInput","cylinder_x_dir should be a 3 x 1 vector");
  }
  Vector3d cylinder_x_dir;
  memcpy(cylinder_x_dir.data(),mxGetPr(prhs[1]),sizeof(double)*3);
  if(!mxIsNumeric(prhs[2]) || mxGetNumberOfElements(prhs[2]) != 3)
  {
    mexErrMsgIdAndTxt("Drake:cartesian2cylindricalmex:InvalidInput","cylinder_origin should be a 3 x 1 vector");
  }
  Vector3d cylinder_origin;
  memcpy(cylinder_origin.data(),mxGetPr(prhs[2]),sizeof(double)*3);
  if(!mxIsNumeric(prhs[3]) || mxGetM(prhs[3]) != 6 || mxGetN(prhs[3]) != 1)
  {
    mexErrMsgIdAndTxt("Drake:cartesian2cylindricalmex:InvalidInput","x should be a 6 x 1 vector");
  }
  Matrix<double,6,1> x_cartesian;
  memcpy(x_cartesian.data(),mxGetPr(prhs[3]),sizeof(double)*6);
  if(!mxIsNumeric(prhs[4]) || mxGetM(prhs[4]) != 6 || mxGetN(prhs[3]) != 1)
  {
    mexErrMsgIdAndTxt("Drake:cartesian2cylindricalmex:InvalidInput","v should be a 6 x 1 vector");
  }
  Matrix<double,6,1> v_cartesian;
  memcpy(v_cartesian.data(),mxGetPr(prhs[4]),sizeof(double)*6);
  cylinder_axis = cylinder_axis/cylinder_axis.norm();
  cylinder_x_dir = cylinder_x_dir/cylinder_x_dir.norm();
  if(abs(cylinder_axis.transpose()*cylinder_x_dir)>1e-10)
  {
    mexErrMsgIdAndTxt("Drake:cartesian2cylindricalmex:InvalidInput","cylinder_x_dir and cylinder_axis should be perpendicular to each othter");
  }
  Matrix<double,6,1> x_cylinder;
  Matrix<double,6,1> v_cylinder;
  Matrix<double,6,6> J;
  Matrix<double,6,1> Jdotv;
  cartesian2cylindrical(cylinder_axis,cylinder_x_dir,cylinder_origin,x_cartesian,v_cartesian,x_cylinder,v_cylinder,J,Jdotv);
  plhs[0] = mxCreateDoubleMatrix(6,1,mxREAL);
  memcpy(mxGetPr(plhs[0]),x_cylinder.data(),sizeof(double)*6);
  plhs[1] = mxCreateDoubleMatrix(6,1,mxREAL);
  memcpy(mxGetPr(plhs[1]),v_cylinder.data(),sizeof(double)*6);
  plhs[2] = mxCreateDoubleMatrix(6,6,mxREAL);
  memcpy(mxGetPr(plhs[2]),J.data(),sizeof(double)*36);
  plhs[3] = mxCreateDoubleMatrix(6,1,mxREAL);
  memcpy(mxGetPr(plhs[3]),Jdotv.data(),sizeof(double)*6);
}
