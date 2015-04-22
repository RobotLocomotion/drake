/*
 * drakeUtil.cpp
 *
 *  Created on: Jun 19, 2013
 *      Author: russt
 */

#include "drakeUtil.h"
#include <string.h>
#include <string>
#include <math.h>
#include <limits>
#include <Eigen/Dense>
#include <stdexcept>

using namespace std;
using namespace Eigen;

bool isa(const mxArray* mxa, const char* class_str)
// mxIsClass seems to not be able to handle derived classes. so i'll implement what I need by calling back to matlab
{
  mxArray* plhs;
  mxArray* prhs[2];
  prhs[0] = const_cast<mxArray*>(mxa);
  prhs[1] = mxCreateString(class_str);
  mexCallMATLAB(1, &plhs, 2, prhs, "isa");
  bool tf = *mxGetLogicals(plhs);
  mxDestroyArray(plhs);
  mxDestroyArray(prhs[1]);
  return tf;
}

bool mexCallMATLABsafe(int nlhs, mxArray* plhs[], int nrhs, mxArray* prhs[], const char* filename)
{
  int i;
  mxArray* ex = mexCallMATLABWithTrap(nlhs, plhs, nrhs, prhs, filename);
  if (ex) {
    mexPrintf("DrakeSystem S-Function: error when calling ''%s'' with the following arguments:\n", filename);
    for (i = 0; i < nrhs; i++)
      mexCallMATLAB(0, NULL, 1, &prhs[i], "disp");
    mxArray *report;
    mexCallMATLAB(1, &report, 1, &ex, "getReport");
    char *errmsg = mxArrayToString(report);
    mexPrintf(errmsg);
    mxFree(errmsg);
    mxDestroyArray(report);
    mexErrMsgIdAndTxt("Drake:mexCallMATLABsafe:CallbackError", "Error in MATLAB callback.\nSee additional debugging information above");
    mxDestroyArray(ex);
    return true;
  }
  for (i = 0; i < nlhs; i++)
    if (!plhs[i]) {
      mexPrintf("Drake mexCallMATLABsafe: error when calling ''%s'' with the following arguments:\n", filename);
      for (i = 0; i < nrhs; i++)
        mexCallMATLAB(0, NULL, 1, &prhs[i], "disp");
      mexErrMsgIdAndTxt("Drake:mexCallMATLABsafe:NotEnoughOutputs", "Asked for %d outputs, but function only returned %d\n", nrhs, i);
      return true;
    }
  return false;
}


/*
 * @param subclass_name (optional) if you want to call a class that derives from
 * DrakeMexPointer (e.g. so that you can refer to it as something more specific in
 * your matlab code), then you can pass in the alternative name here.  The constructor
 * for this class must take the same inputs as the DrakeMexPointer constructor.
 */
mxArray* createDrakeMexPointer(void* ptr, const char* name, int num_additional_inputs, mxArray* delete_fcn_additional_inputs[], const char* subclass_name)
{
  mxClassID cid;
  if (sizeof(ptr) == 4) cid = mxUINT32_CLASS;
  else if (sizeof(ptr) == 8) cid = mxUINT64_CLASS;
  else mexErrMsgIdAndTxt("Drake:constructDrakeMexPointer:PointerSize", "Are you on a 32-bit machine or 64-bit machine??");

  int nrhs = 3 + num_additional_inputs;
  mxArray *plhs[1];
  mxArray **prhs;  
  prhs = new mxArray*[nrhs];

  prhs[0] = mxCreateNumericMatrix(1, 1, cid, mxREAL);
  memcpy(mxGetData(prhs[0]), &ptr, sizeof(ptr));

  prhs[1] = mxCreateString(mexFunctionName());

  prhs[2] = mxCreateString(name);

  for (int i = 0; i < num_additional_inputs; i++)
    prhs[3+i] = delete_fcn_additional_inputs[i];

//  mexPrintf("deleteMethod = %s\n name =%s\n", deleteMethod,name);

  // call matlab to construct mex pointer object
  if (subclass_name) {
    mexCallMATLABsafe(1, plhs, nrhs, prhs, subclass_name);
    if (!isa(plhs[0], "DrakeMexPointer")) {
      mxDestroyArray(plhs[0]);
      mexErrMsgIdAndTxt("Drake:createDrakeMexPointer:InvalidSubclass", "subclass_name is not a valid subclass of DrakeMexPointer");
    }
  }
  else
    mexCallMATLABsafe(1, plhs, nrhs, prhs, "DrakeMexPointer");

  mexLock();

//  mexPrintf("incrementing lock count\n");

  delete[] prhs;
  return plhs[0];
}

void* getDrakeMexPointer(const mxArray* mx)
{
  void* ptr = NULL;

  // todo: optimize this by caching the pointer values, as described in
  // http://groups.csail.mit.edu/locomotion/bugs/show_bug.cgi?id=1590
  mxArray* ptrArray = mxGetProperty(mx, 0, "ptr");

  if (!ptrArray)
    mexErrMsgIdAndTxt("Drake:getDrakeMexPointer:BadInputs", "cannot retrieve 'ptr' field from this mxArray.  are you sure it's a valid DrakeMexPointer object?");

  switch (sizeof(void*)) { 
    case 4:
      if (!mxIsUint32(ptrArray))
        mexErrMsgIdAndTxt("Drake:getDrakeMexPointer:BadPointerSize", "DrakeMexPointer expected a 32-bit ptr field but got something else");        
      break;
    case 8:
      if (!mxIsUint64(ptrArray))
        mexErrMsgIdAndTxt("Drake:getDrakeMexPointer:BadPointerSize", "DrakeMexPointer expected a 64-bit ptr field but got something else");        
      break;
    default:
      mexErrMsgIdAndTxt("Drake:getDrakeMexPointer:BadPointerSize", "DrakeMexPointer got a pointer that was neither 32-bit nor 64-bit.");
  }

  if (!mxIsNumeric(ptrArray) || mxGetNumberOfElements(ptrArray) != 1)
    mexErrMsgIdAndTxt("Drake:getDrakeMexPointer:BadInputs","the ptr property of this DrakeMexPointer does not appear to contain a valid pointer");
  memcpy(&ptr, mxGetData(ptrArray), sizeof(ptr));     // note: could use a reinterpret_cast here instead

  return ptr;
}

double angleAverage(double theta1, double theta2) {
  // Computes the average between two angles by averaging points on the unit
  // circle and taking the arctan of the result.
  //   see: http://en.wikipedia.org/wiki/Mean_of_circular_quantities
  // theta1 is a scalar or column vector of angles (rad)
  // theta2 is a scalar or column vector of angles (rad)

  double x_mean = 0.5 * (cos(theta1) + cos(theta2));
  double y_mean = 0.5 * (sin(theta1) + sin(theta2));

  double angle_mean = atan2(y_mean, x_mean);

  return angle_mean;
}

std::pair<Eigen::Vector3d, double> resolveCenterOfPressure(Eigen::Vector3d torque, Eigen::Vector3d force, Eigen::Vector3d normal, Eigen::Vector3d point_on_contact_plane)
{
  // TODO: implement multi-column version
  using namespace Eigen;

  if (abs(normal.squaredNorm() - 1.0) > 1e-12) {
    mexErrMsgIdAndTxt("Drake:resolveCenterOfPressure:BadInputs", "normal should be a unit vector");
  }

  Vector3d cop;
  double normal_torque_at_cop;

  double fz = normal.dot(force);
  bool cop_exists = abs(fz) > 1e-12;

  if (cop_exists) {
    auto torque_at_point_on_contact_plane = torque - point_on_contact_plane.cross(force);
    double normal_torque_at_point_on_contact_plane = normal.dot(torque_at_point_on_contact_plane);
    auto tangential_torque = torque_at_point_on_contact_plane - normal * normal_torque_at_point_on_contact_plane;
    cop = normal.cross(tangential_torque) / fz + point_on_contact_plane;
    auto torque_at_cop = torque - cop.cross(force);
    normal_torque_at_cop = normal.dot(torque_at_cop);
  }
  else {
    cop.setConstant(std::numeric_limits<double>::quiet_NaN());
    normal_torque_at_cop = std::numeric_limits<double>::quiet_NaN();
  }
  return std::pair<Vector3d, double>(cop, normal_torque_at_cop);
}

double * mxGetPrSafe(const mxArray *pobj) {
  if (!mxIsDouble(pobj)) mexErrMsgIdAndTxt("Drake:mxGetPrSafe:BadInputs", "mxGetPr can only be called on arguments which correspond to Matlab doubles");
  return mxGetPr(pobj);
}

mxArray* mxGetPropertySafe(const mxArray* array, std::string const& field_name) {
  return mxGetPropertySafe(array, 0, field_name);
}

mxArray* mxGetPropertySafe(const mxArray* array, size_t index, std::string const& field_name)
{
  mxArray* ret = mxGetProperty(array, index, field_name.c_str());
  if (!ret)
  {
    mexErrMsgIdAndTxt("Drake:mxGetPropertySafe", ("Field not found: " + field_name).c_str());
  }
  return ret;
}

mxArray* mxGetFieldSafe(const mxArray* array, std::string const& field_name) {
  return mxGetFieldSafe(array, 0, field_name);
}

mxArray* mxGetFieldSafe(const mxArray* array, size_t index, std::string const& field_name)
{
  mxArray* ret = mxGetField(array, index, field_name.c_str());
  if (!ret)
  {
    mexErrMsgIdAndTxt("Drake:mxGetFieldSafe", ("Field not found: " + field_name).c_str());
  }
  return ret;
}

void mxSetFieldSafe(mxArray* array, size_t index, std::string const & fieldname, mxArray* data)
{
  int fieldnum;
  fieldnum = mxGetFieldNumber(array, fieldname.c_str());
  if (fieldnum < 0) {
    fieldnum = mxAddField(array, fieldname.c_str());
  }
  mxSetFieldByNumber(array, index, fieldnum, data);
}

const std::vector<double> matlabToStdVector(const mxArray* in) {
  // works for both row vectors and column vectors
  if (mxGetM(in) != 1 && mxGetN(in) != 1)
    throw runtime_error("Not a vector");
  double* data = mxGetPrSafe(in);
  return std::vector<double>(data, data + mxGetNumberOfElements(in));
}

int sub2ind(mwSize ndims, const mwSize* dims, const mwSize* sub) {
  int stride = 1;
  int ret = 0;
  for (int i = 0; i < ndims; i++) {
    ret += sub[i] * stride;
    stride *= dims[i];
  }
  return ret;
}

void sizecheck(const mxArray* mat, int M, int N) {
  if (mxGetM(mat) != M) {
    mexErrMsgIdAndTxt("Drake:WrongSize", "wrong number of rows. Expected: %d but got: %d", M, mxGetM(mat));
  }
  if (mxGetN(mat) != N) {
    mexErrMsgIdAndTxt("Drake:WrongSize", "wrong number of columns. Expected: %d but got: %d", N, mxGetN(mat));
  }
  return;
}

//builds a matlab sparse matrix in mex from a given eigen matrix
//the caller is responsible for destroying the resulting array
template <typename Derived>
mxArray* eigenToMatlabSparse(MatrixBase<Derived> const & M, int & num_non_zero) 
{
  const mwSize rows = M.rows();
  const mwSize cols = M.cols();

  vector<mwIndex> ir;
  vector<mwIndex> jc;
  vector<double> pr;

  mwSize cumulative_nonzero = 0;
  jc.push_back(cumulative_nonzero);
  double eps = std::numeric_limits<double>::epsilon();
  for (mwIndex j = 0; j < cols; j++) {
    for (mwIndex i = 0; i < rows; i++)  {
      double value = M(i, j);
      
      if (value > eps || value < -eps) {
        ir.push_back(i);
        pr.push_back(value);
        cumulative_nonzero++;
      }
    }
    jc.push_back(cumulative_nonzero);
  }

  mxArray* sparse_mex = mxCreateSparse(rows, cols, cumulative_nonzero, mxREAL);
  
  memcpy((double*)mxGetPr(sparse_mex), pr.data(), cumulative_nonzero * sizeof(double));
  memcpy((int*)mxGetIr(sparse_mex), ir.data(), cumulative_nonzero * sizeof(mwIndex));
  memcpy((int *)mxGetJc(sparse_mex), jc.data(), (cols + 1) * sizeof(mwIndex));

  num_non_zero = cumulative_nonzero;
  return sparse_mex;
}

template DLLEXPORT mxArray* eigenToMatlabSparse(MatrixBase< MatrixXd > const &, int &) ;
template DLLEXPORT mxArray* eigenToMatlabSparse(MatrixBase< Map< MatrixXd> > const &, int &) ;
