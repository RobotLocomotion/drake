#include "mex.h"
#include "drakeGeometryUtil.h"

using namespace Eigen;
using namespace std;

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  if(nrhs != 4){
    mexErrMsgIdAndTxt("Drake:cartesian2cylindricalmex:IncorrectInputs", "Usage [x,dx] = cartesian2cylindrical(cylinder_axis,cylinder_x_dir,cylinder_origin,x_cartesian)");
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
  Matrix<double,6,1> x_input;
  memcpy(x_input.data(),mxGetPr(prhs[3]),sizeof(double)*6);
  cylinder_axis = cylinder_axis/cylinder_axis.norm();
  cylinder_x_dir = cylinder_x_dir/cylinder_x_dir.norm();
  if(abs(cylinder_axis.transpose()*cylinder_x_dir)>1e-10)
  {
    mexErrMsgIdAndTxt("Drake:cartesian2cylindricalmex:InvalidInput","cylinder_x_dir and cylinder_axis should be perpendicular to each othter");
  }
  Matrix3d R;
  R.col(0) = cylinder_x_dir;
  R.col(1) = cylinder_axis.cross(cylinder_x_dir);
  R.col(2) = cylinder_axis;
  Transform<double,3,Isometry,0> T;
  T.setIdentity();
  T = T.rotate(R);  
  T = T.translate(cylinder_origin);
  auto x_output = cartesian2cylindrical(T,x_input);
  plhs[0] = mxCreateDoubleMatrix(6,1,mxREAL);
  memcpy(mxGetPr(plhs[0]),x_output.value().data(),sizeof(double)*6);
  plhs[1] = mxCreateDoubleMatrix(6,6,mxREAL);
  memcpy(mxGetPr(plhs[1]),x_output.gradient().value().data(),sizeof(double)*36);
}
