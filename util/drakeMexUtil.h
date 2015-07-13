#include "mex.h"
#include <vector>
#include <Eigen/Core>
#include "TrigPoly.h"

#ifndef DRAKE_MEX_UTIL_H_
#define DRAKE_MEX_UTIL_H_

#undef DLLEXPORT
#if defined(WIN32) || defined(WIN64)
  #if defined(drakeMexUtil_EXPORTS)
    #define DLLEXPORT __declspec( dllexport )
  #else
    #define DLLEXPORT __declspec( dllimport )
  #endif
#else
    #define DLLEXPORT
#endif

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

template<int RowsAtCompileTime=-1, int ColsAtCompileTime=-1>
Eigen::Map<const Eigen::Matrix<double, RowsAtCompileTime, ColsAtCompileTime>> matlabToEigenMap(const mxArray* matlab_array)
{
  Eigen::Map<const Eigen::Matrix<double, RowsAtCompileTime, ColsAtCompileTime>> ret(mxGetPr(matlab_array), mxGetM(matlab_array), mxGetN(matlab_array));
  return ret;
}

DLLEXPORT std::string mxGetStdString(const mxArray* array);

template <typename Scalar>
mxArray* stdVectorToMatlab(const std::vector<Scalar>& vec) {
  mxArray* pm = mxCreateDoubleMatrix(static_cast<int>(vec.size()), 1, mxREAL);
  for (int i = 0; i < static_cast<int>(vec.size()); i++) {
    mxGetPr(pm)[i] = (double) vec[i];
  }
  return pm;
}

DLLEXPORT void sizecheck(const mxArray* mat, int M, int N);

template <size_t Rows, size_t Cols>
void matlabToCArrayOfArrays(const mxArray *source, double (&destination)[Rows][Cols])  {
  // Matlab arrays come in as column-major data. The format used in e.g. LCM messages is an array of arrays.
  // from http://stackoverflow.com/a/17569578/2228557
  sizecheck(source, static_cast<int>(Rows), static_cast<int>(Cols));
  double* source_data = mxGetPr(source);
  for (size_t row = 0; row < Rows; ++row) {
    for (size_t col = 0; col < Cols; ++col) {
      destination[row][col] = source_data[row + col * Rows];
    }
  }
}

DLLEXPORT mwSize sub2ind(mwSize ndims, const mwSize* dims, const mwSize* sub);

template <typename T>
const std::vector<T> matlabToStdVector(const mxArray* in);

DLLEXPORT Eigen::Matrix<Polynomiald, Eigen::Dynamic, Eigen::Dynamic> msspolyToEigen(const mxArray* msspoly);

template< int Rows, int Cols >
DLLEXPORT mxArray* eigenToMSSPoly(const Eigen::Matrix<Polynomiald,Rows,Cols> & poly)
{
  int num_monomials = 0, max_terms = 0;
  for (int i=0; i<poly.size(); i++) {
    auto monomials = poly(i).getMonomials();
    num_monomials += monomials.size();
    for (std::vector<Polynomiald::Monomial>::const_iterator iter=monomials.begin(); iter!=monomials.end(); iter++) {
      if (iter->terms.size() > max_terms)
        max_terms = iter->terms.size();
    }
  }

  Eigen::Matrix<double,1,2> dim; dim << poly.rows(), poly.cols();
  Eigen::MatrixXd sub(num_monomials,2);
  Eigen::MatrixXd var = Eigen::MatrixXd::Zero(num_monomials,max_terms);
  Eigen::MatrixXd pow = Eigen::MatrixXd::Zero(num_monomials,max_terms);
  Eigen::VectorXd coeff(num_monomials);

  int index=0;
  for (int i=0; i<poly.rows(); i++) {
    for (int j=0; j<poly.cols(); j++) {
      auto monomials = poly(i,j).getMonomials();
      for (std::vector<Polynomiald::Monomial>::const_iterator iter=monomials.begin(); iter!=monomials.end(); iter++) {
        sub(index,0) = i+1;
        sub(index,1) = j+1;
        for (int k=0; k<iter->terms.size(); k++) {
          var(index,k)=(double)iter->terms[k].var;
          pow(index,k)=(double)iter->terms[k].power;
        }
        coeff(index) = iter->coefficient;
        index++;
      }
    }
  }

  mxArray* plhs[1];
  mxArray* prhs[5];
  prhs[0] = eigenToMatlab(dim);
  prhs[1] = eigenToMatlab(sub);
  prhs[2] = eigenToMatlab(var);
  prhs[3] = eigenToMatlab(pow);
  prhs[4] = eigenToMatlab(coeff);
  mexCallMATLABsafe(1,plhs,5,prhs,"msspoly");
  return plhs[0];
}

DLLEXPORT Eigen::Matrix<TrigPolyd, Eigen::Dynamic, Eigen::Dynamic> trigPolyToEigen(const mxArray* trigpoly);

#include <iostream> // for debugging
template< int Rows, int Cols >
DLLEXPORT mxArray* eigenToTrigPoly(const Eigen::Matrix<TrigPolyd,Rows,Cols> & trigpoly_mat)
{
  Eigen::Matrix<Polynomiald,Eigen::Dynamic,Eigen::Dynamic> poly_mat(trigpoly_mat.rows(),trigpoly_mat.cols());
  TrigPolyd::SinCosMap sin_cos_map;
  for (int i=0; i<trigpoly_mat.size(); i++) {
    const TrigPolyd::SinCosMap& sc = trigpoly_mat(i).getSinCosMap();
    sin_cos_map.insert(sc.begin(),sc.end());
    poly_mat(i) = trigpoly_mat(i).getPolynomial();
  }

  if (sin_cos_map.empty()) // then just return the msspoly.  what else can i do?
    return eigenToMSSPoly(poly_mat);

  // construct the equivalent of the sin/cos map
  // (do I need to worry about them possibly being out of order?)
  Eigen::Matrix<Polynomiald,Eigen::Dynamic,1> q(sin_cos_map.size()), s(sin_cos_map.size()), c(sin_cos_map.size());
  int i=0;
  for (TrigPolyd::SinCosMap::iterator iter=sin_cos_map.begin(); iter!=sin_cos_map.end(); iter++) {
    q(i) = Polynomiald(1.0,iter->first);
    s(i) = Polynomiald(1.0,iter->second.s);
    c(i) = Polynomiald(1.0,iter->second.c);
    i++;
  }

  mxArray* plhs[1];
  mxArray* prhs[3];
  prhs[0] = eigenToMSSPoly(q);
  prhs[1] = eigenToMSSPoly(s);
  prhs[2] = eigenToMSSPoly(c);
  mexCallMATLABsafe(1,plhs,3,prhs,"TrigPoly");

  mxSetProperty(plhs[0],0,"p",eigenToMSSPoly(poly_mat));

  return plhs[0];
}


DLLEXPORT double *mxGetPrSafe(const mxArray *pobj);

DLLEXPORT mxArray* mxGetPropertySafe(const mxArray* array, std::string const& field_name);
DLLEXPORT mxArray* mxGetFieldSafe(const mxArray* array, std::string const& field_name);
DLLEXPORT mxArray* mxGetPropertySafe(const mxArray* array, size_t index, std::string const& field_name);
DLLEXPORT mxArray* mxGetFieldSafe(const mxArray* array, size_t index, std::string const& field_name);
DLLEXPORT void mxSetFieldSafe(mxArray* array, size_t index, std::string const & fieldname, mxArray* data);
DLLEXPORT mxArray* mxGetFieldOrPropertySafe(const mxArray* array, std::string const& field_name);
DLLEXPORT mxArray* mxGetFieldOrPropertySafe(const mxArray* array, size_t index, std::string const& field_name);

#endif
