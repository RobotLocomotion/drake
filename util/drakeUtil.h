/*
 * drakeUtil.h
 *
 *  Created on: Jun 19, 2013
 *      Author: russt
 */

#include "mex.h"
#include <string>
#include <stdexcept>
#include <vector>
#include <utility>
#include <Eigen/Core>

// needed only for valuecheck (but keeping that code in the header avoids explicit instantiations)
#include <sstream> 
#include <cmath>
// end valuecheck includes


#ifndef DRAKE_UTIL_H_
#define DRAKE_UTIL_H_

#undef DLLEXPORT
#if defined(WIN32) || defined(WIN64)
  #if defined(drakeUtil_EXPORTS)
    #define DLLEXPORT __declspec( dllexport )
  #else
    #define DLLEXPORT __declspec( dllimport )
  #endif
#else
    #define DLLEXPORT
#endif

// Helper routines
DLLEXPORT bool isa(const mxArray* mxa, const char* class_str);
DLLEXPORT bool mexCallMATLABsafe(int nlhs, mxArray* plhs[], int nrhs, mxArray* prhs[], const char* filename);

// Mex pointers shared through matlab
DLLEXPORT mxArray* createDrakeMexPointer(void* ptr, const char* name="", int num_additional_inputs=0, mxArray *delete_fcn_additional_inputs[] = NULL, const char* subclass_name=NULL);  // increments lock count
// Note: the same mex function which calls this method will be called with the syntax mexFunction(drake_mex_ptr) as the destructor

DLLEXPORT void* getDrakeMexPointer(const mxArray* mx);

template <typename Derived> inline void destroyDrakeMexPointer(const mxArray* mx)
{
  Derived typed_ptr = (Derived) getDrakeMexPointer(mx);

  //mexPrintf("deleting drake mex pointer\n");
  delete typed_ptr;
  mexUnlock();  // decrement lock count

//  mexPrintf(mexIsLocked() ? "mex file is locked\n" : "mex file is unlocked\n");
}

template <typename Derived>
DLLEXPORT mxArray* eigenToMatlabSparse(Eigen::MatrixBase<Derived> const & M, int & num_non_zero);

template <typename DerivedA>
mxArray* eigenToMatlab(const DerivedA &m)
{
  // this avoids zero initialization that would occur using mxCreateDoubleMatrix with nonzero dimensions.
  // see https://classes.soe.ucsc.edu/ee264/Fall11/cmex.pdf, page 8
  mxArray* pm = mxCreateDoubleMatrix(0, 0, mxREAL);
  int rows = static_cast<int>(m.rows());
  int cols = static_cast<int>(m.cols());
  int numel = rows * cols;
  mxSetM(pm, rows);
  mxSetN(pm, cols);
  if (numel)
    mxSetData(pm, mxMalloc(sizeof(double) * numel));
  memcpy(mxGetPr(pm), m.data(), sizeof(double)* numel);
  return pm;
}

template<int RowsAtCompileTime, int ColsAtCompileTime>
Eigen::Matrix<double, RowsAtCompileTime, ColsAtCompileTime> matlabToEigen(const mxArray* matlab_array)
{
  const mwSize* size_array = mxGetDimensions(matlab_array);
  Eigen::Matrix<double, RowsAtCompileTime, ColsAtCompileTime> ret(size_array[0], size_array[1]);
  memcpy(ret.data(), mxGetPr(matlab_array), sizeof(double) * ret.size());
  return ret;
}

template <typename Scalar>
mxArray* stdVectorToMatlab(const std::vector<Scalar>& vec) {
  mxArray* pm = mxCreateDoubleMatrix(static_cast<int>(vec.size()), 1, mxREAL);
  for (int i = 0; i < static_cast<int>(vec.size()); i++) {
    mxGetPr(pm)[i] = (double) vec[i];
  }
  return pm;
}

DLLEXPORT const std::vector<double> matlabToStdVector(const mxArray* in);

DLLEXPORT int sub2ind(mwSize ndims, const mwSize* dims, const mwSize* sub);

void baseZeroToBaseOne(std::vector<int>& vec)
{
  for (std::vector<int>::iterator iter=vec.begin(); iter!=vec.end(); iter++)
    (*iter)++;
}

DLLEXPORT double angleAverage(double theta1, double theta2);

DLLEXPORT std::pair<Eigen::Vector3d, double> resolveCenterOfPressure(Eigen::Vector3d torque, Eigen::Vector3d force, Eigen::Vector3d normal, Eigen::Vector3d point_on_contact_plane);

DLLEXPORT double *mxGetPrSafe(const mxArray *pobj);

DLLEXPORT mxArray* mxGetPropertySafe(const mxArray* array, std::string const& field_name);
DLLEXPORT mxArray* mxGetFieldSafe(const mxArray* array, std::string const& field_name);
DLLEXPORT mxArray* mxGetPropertySafe(const mxArray* array, size_t index, std::string const& field_name);
DLLEXPORT mxArray* mxGetFieldSafe(const mxArray* array, size_t index, std::string const& field_name);

DLLEXPORT void mxSetFieldSafe(mxArray* array, size_t index, std::string const & fieldname, mxArray* data);

DLLEXPORT void sizecheck(const mxArray* mat, int M, int N);

template<typename Derived>
std::string to_string(const Eigen::MatrixBase<Derived> & a)
{
	std::stringstream ss;
	ss << a;
	return ss.str();
}

inline int my_isnan(double x) {
#ifdef WIN32
  return _isnan(x);
#else
  return std::isnan(x);
#endif
}

template<typename DerivedA, typename DerivedB>
void valuecheck(const Eigen::MatrixBase<DerivedA>& a, const Eigen::MatrixBase<DerivedB>& b, double tol, std::string error_msg)
{
	// note: isApprox uses the L2 norm, so is bad for comparing against zero
	if (a.rows() != b.rows() || a.cols() != b.cols()) {
	  throw std::runtime_error("Drake:ValueCheck ERROR:" + error_msg + "size mismatch: (" + std::to_string(static_cast<unsigned long long>(a.rows())) + " by " + std::to_string(static_cast<unsigned long long>(a.cols())) + ") and (" + std::to_string(static_cast<unsigned long long>(b.rows())) + " by " + std::to_string(static_cast<unsigned long long>(b.cols())) + ")");
	}
	if (!(a-b).isZero(tol)) {
		if (!a.allFinite() && !b.allFinite()) {
			// could be failing because inf-inf = nan
			bool ok=true;
			for (int i=0; i<a.rows(); i++)
				for (int j=0; j<a.cols(); j++) {
					ok = ok && ((a(i,j) == std::numeric_limits<double>::infinity() && b(i,j) == std::numeric_limits<double>::infinity()) ||
					(a(i,j) == -std::numeric_limits<double>::infinity() && b(i,j) == -std::numeric_limits<double>::infinity()) ||
			    (my_isnan(a(i,j)) && my_isnan(b(i,j))) || (std::abs(a(i,j)-b(i,j))<tol));
				}
			if (ok) return;
		}
		error_msg += "A:\n" + to_string(a) + "\nB:\n" + to_string(b) + "\n";
		throw std::runtime_error("Drake:ValueCheck ERROR:" + error_msg);
	}
}

#endif /* DRAKE_UTIL_H_ */
