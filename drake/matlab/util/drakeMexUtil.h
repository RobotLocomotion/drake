#pragma once

#include <mex.h>

#include <vector>
#include <Eigen/Core>
#include <Eigen/Sparse>
#include "drake/common/trig_poly.h"
/*
 * NOTE: include AutoDiff AFTER trig_poly.h.
 * trig_poly.h includes LLDT.h via Eigenvalues, PolynomialSolver, and our
 * polynomial.h
 * MSVC versions up to and including 2013 have trouble with the rankUpdate
 * method in LLDT.h
 * For some reason there is a bad interaction with AutoDiff, even though LLDT.h
 * still gets included if trig_poly.h is included before AutoDiff.
 * See http://eigen.tuxfamily.org/bz/show_bug.cgi?id=1057
 */
#include <unsupported/Eigen/AutoDiff>
#include <Eigen/src/SparseCore/SparseMatrix.h>

#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/util/drakeGradientUtil.h"

using drake::math::autoDiffToValueMatrix;
using drake::math::autoDiffToGradientMatrix;
using drake::math::Gradient;

DLL_EXPORT_SYM bool isa(const mxArray* mxa, const char* class_str);
DLL_EXPORT_SYM bool mexCallMATLABsafe(int nlhs, mxArray* plhs[], int nrhs,
                                      mxArray* prhs[], const char* filename);

DLL_EXPORT_SYM double* mxGetPrSafe(const mxArray* pobj);

DLL_EXPORT_SYM mxArray* mxGetPropertySafe(const mxArray* array,
                                          std::string const& field_name);
DLL_EXPORT_SYM mxArray* mxGetFieldSafe(const mxArray* array,
                                       std::string const& field_name);
DLL_EXPORT_SYM mxArray* mxGetPropertySafe(const mxArray* array, size_t index,
                                          std::string const& field_name);
DLL_EXPORT_SYM mxArray* mxGetFieldSafe(const mxArray* array, size_t index,
                                       std::string const& field_name);
DLL_EXPORT_SYM void mxSetFieldSafe(mxArray* array, size_t index,
                                   std::string const& fieldname, mxArray* data);
DLL_EXPORT_SYM mxArray* mxGetFieldOrPropertySafe(const mxArray* array,
                                                 std::string const& field_name);
DLL_EXPORT_SYM mxArray* mxGetFieldOrPropertySafe(const mxArray* array,
                                                 size_t index,
                                                 std::string const& field_name);

// Mex pointers shared through matlab
// Note: the same mex function which calls this method will be called with the
// syntax mexFunction(drake_mex_ptr) as the destructor

DLL_EXPORT_SYM mxArray* createDrakeMexPointer(
    void* ptr, const std::string& name = "", int type_id = -1,
    int num_additional_inputs = 0,
    mxArray* delete_fcn_additional_inputs[] = NULL,
    const std::string& subclass_name = "",
    const std::string& mex_function_name_prefix = "");  // increments lock count
DLL_EXPORT_SYM void* getDrakeMexPointer(const mxArray* mx);

template <typename Derived>
inline void destroyDrakeMexPointer(const mxArray* mx) {
  if (!isa(mx, "DrakeMexPointer"))
    mexErrMsgIdAndTxt("Drake:destroyDrakeMexPointer:BadInputs",
                      "This object is not a DrakeMexPointer.  Delete failed.");

  Derived typed_ptr = (Derived)getDrakeMexPointer(mx);

  // mexPrintf("deleting drake mex pointer\n");
  delete typed_ptr;
  mexUnlock();  // decrement lock count

  //  mexPrintf(mexIsLocked() ? "mex file is locked\n" : "mex file is
  //  unlocked\n");
}

template <typename Derived>
DLL_EXPORT_SYM mxArray* eigenToMatlabSparse(Eigen::MatrixBase<Derived> const& M,
                                            int& num_non_zero);

template <typename DerivedA>
mxArray* eigenToMatlab(const DerivedA& m) {
  // this avoids zero initialization that would occur using mxCreateDoubleMatrix
  // with nonzero dimensions.
  // see https://classes.soe.ucsc.edu/ee264/Fall11/cmex.pdf, page 8
  mxArray* pm = mxCreateDoubleMatrix(0, 0, mxREAL);
  int rows = static_cast<int>(m.rows());
  int cols = static_cast<int>(m.cols());
  int numel = rows * cols;
  mxSetM(pm, rows);
  mxSetN(pm, cols);
  if (numel) mxSetData(pm, mxMalloc(sizeof(double) * numel));
  memcpy(mxGetPr(pm), m.data(), sizeof(double) * numel);
  return pm;
}

template <int RowsAtCompileTime, int ColsAtCompileTime>
Eigen::Matrix<double, RowsAtCompileTime, ColsAtCompileTime> matlabToEigen(
    const mxArray* matlab_array) {
  const mwSize* size_array = mxGetDimensions(matlab_array);
  Eigen::Matrix<double, RowsAtCompileTime, ColsAtCompileTime> ret(
      size_array[0], size_array[1]);
  memcpy(ret.data(), mxGetPr(matlab_array), sizeof(double) * ret.size());
  return ret;
}

template<int Rows, int Cols>
Eigen::Map<const Eigen::Matrix<double, Rows, Cols>> matlabToEigenMap(
    const mxArray *mex) {
  using namespace Eigen;
  using namespace std;

  Index rows, cols;  // at runtime
  if (mxIsEmpty(mex)) {
    // be lenient when it comes to dimensions in the empty input case
    if (Rows == Dynamic && Cols == Dynamic) {
      // if both dimensions are dynamic, then follow the dimensions of
      // the Matlab matrix
      rows = mxGetM(mex);
      cols = mxGetN(mex);
    } else {
      // if only one dimension is dynamic, use the known dimension at
      // compile time and set the other dimension to zero
      rows = Rows == Dynamic ? 0 : Rows;
      cols = Cols == Dynamic ? 0 : Cols;
    }
  } else {
    // the non-empty case
    rows = Rows == Dynamic ? mxGetM(mex) : Rows;
    cols = Cols == Dynamic ? mxGetN(mex) : Cols;
  }

  if (Rows != Dynamic && Rows != rows) {
    ostringstream stream;
    stream << "Error converting Matlab matrix. Expected " << Rows
        << " rows, but got " << mxGetM(mex) << ".";
    throw runtime_error(stream.str().c_str());
  }

  if (Cols != Dynamic && Cols != cols) {
    ostringstream stream;
    stream << "Error converting Matlab matrix. Expected " << Cols
        << " cols, but got " << mxGetN(mex) << ".";
    throw runtime_error(stream.str().c_str());
  }

  double *data = rows * cols == 0 ? nullptr : mxGetPrSafe(mex);
  return Map<const Matrix<double, Rows, Cols>>(data, rows, cols);
}

DLL_EXPORT_SYM Eigen::SparseMatrix<double> matlabToEigenSparse(
    const mxArray* mex);

DLL_EXPORT_SYM std::string mxGetStdString(const mxArray* array);
DLL_EXPORT_SYM std::vector<std::string> mxGetVectorOfStdStrings(
    const mxArray* array);

template <typename Scalar>
mxArray* stdVectorToMatlab(const std::vector<Scalar>& vec) {
  mxArray* pm = mxCreateDoubleMatrix(static_cast<int>(vec.size()), 1, mxREAL);
  for (int i = 0; i < static_cast<int>(vec.size()); i++) {
    mxGetPr(pm)[i] = (double)vec[i];
  }
  return pm;
}
DLL_EXPORT_SYM mxArray* stdStringToMatlab(const std::string& str);
DLL_EXPORT_SYM mxArray* vectorOfStdStringsToMatlab(
    const std::vector<std::string>& strs);

DLL_EXPORT_SYM void sizecheck(const mxArray* mat, mwSize M, mwSize N);

template <size_t Rows, size_t Cols>
void matlabToCArrayOfArrays(const mxArray* source,
                            double(&destination)[Rows][Cols]) {
  // Matlab arrays come in as column-major data. The format used in e.g. LCM
  // messages is an array of arrays.
  // from http://stackoverflow.com/a/17569578/2228557
  sizecheck(source, static_cast<int>(Rows), static_cast<int>(Cols));
  double* source_data = mxGetPr(source);
  for (size_t row = 0; row < Rows; ++row) {
    for (size_t col = 0; col < Cols; ++col) {
      destination[row][col] = source_data[row + col * Rows];
    }
  }
}

DLL_EXPORT_SYM mwSize sub2ind(mwSize ndims, const mwSize* dims,
                              const mwSize* sub);

template <typename T>
const std::vector<T> matlabToStdVector(const mxArray* in);

DLL_EXPORT_SYM Eigen::Matrix<Polynomiald, Eigen::Dynamic, Eigen::Dynamic>
msspolyToEigen(const mxArray* msspoly);

template <int _Rows, int _Cols>
mxArray* eigenToMSSPoly(const Eigen::Matrix<Polynomiald, _Rows, _Cols>& poly) {
  size_t num_monomials = 0, max_terms = 0;
  for (int i = 0; i < poly.size(); i++) {
    auto monomials = poly(i).GetMonomials();
    num_monomials += monomials.size();
    for (std::vector<Polynomiald::Monomial>::const_iterator iter =
             monomials.begin();
         iter != monomials.end(); iter++) {
      if (iter->terms.size() > max_terms) max_terms = iter->terms.size();
    }
  }

  Eigen::Matrix<double, 1, 2> dim;
  dim << static_cast<double>(poly.rows()), static_cast<double>(poly.cols());
  Eigen::MatrixXd sub(num_monomials, 2);
  Eigen::MatrixXd var = Eigen::MatrixXd::Zero(num_monomials, max_terms);
  Eigen::MatrixXd pow = Eigen::MatrixXd::Zero(num_monomials, max_terms);
  Eigen::VectorXd coeff(num_monomials);

  int index = 0;
  for (int i = 0; i < poly.rows(); i++) {
    for (int j = 0; j < poly.cols(); j++) {
      auto monomials = poly(i, j).GetMonomials();
      for (std::vector<Polynomiald::Monomial>::const_iterator iter =
               monomials.begin();
           iter != monomials.end(); iter++) {
        sub(index, 0) = i + 1;
        sub(index, 1) = j + 1;
        for (int k = 0; k < static_cast<int>(iter->terms.size()); k++) {
          var(index, k) = (double)iter->terms[k].var;
          pow(index, k) = (double)iter->terms[k].power;
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
  mexCallMATLABsafe(1, plhs, 5, prhs, "msspoly");
  return plhs[0];
}

DLL_EXPORT_SYM Eigen::Matrix<TrigPolyd, Eigen::Dynamic, Eigen::Dynamic>
trigPolyToEigen(const mxArray* trigpoly);

template <int _Rows, int _Cols>
mxArray* eigenToTrigPoly(
    const Eigen::Matrix<TrigPolyd, _Rows, _Cols>& trigpoly_mat) {
  Eigen::Matrix<Polynomiald, Eigen::Dynamic, Eigen::Dynamic> poly_mat(
      trigpoly_mat.rows(), trigpoly_mat.cols());
  TrigPolyd::SinCosMap sin_cos_map;
  for (int i = 0; i < trigpoly_mat.size(); i++) {
    const TrigPolyd::SinCosMap& sc = trigpoly_mat(i).sin_cos_map();
    sin_cos_map.insert(sc.begin(), sc.end());
    poly_mat(i) = trigpoly_mat(i).poly();
  }

  if (sin_cos_map
          .empty())  // then just return the msspoly.  what else can i do?
    return eigenToMSSPoly<Eigen::Dynamic, Eigen::Dynamic>(poly_mat);

  // construct the equivalent of the sin/cos map
  // (do I need to worry about them possibly being out of order?)
  Eigen::Matrix<Polynomiald, Eigen::Dynamic, 1> q(sin_cos_map.size()),
      s(sin_cos_map.size()), c(sin_cos_map.size());
  int i = 0;
  for (TrigPolyd::SinCosMap::iterator iter = sin_cos_map.begin();
       iter != sin_cos_map.end(); iter++) {
    q(i) = Polynomiald(1.0, iter->first);
    s(i) = Polynomiald(1.0, iter->second.s);
    c(i) = Polynomiald(1.0, iter->second.c);
    i++;
  }

  mxArray* plhs[1];
  mxArray* prhs[3];
  prhs[0] = eigenToMSSPoly<Eigen::Dynamic, 1>(q);
  prhs[1] = eigenToMSSPoly<Eigen::Dynamic, 1>(s);
  prhs[2] = eigenToMSSPoly<Eigen::Dynamic, 1>(c);
  mexCallMATLABsafe(1, plhs, 3, prhs, "TrigPoly");

  mxSetProperty(plhs[0], 0, "p",
                eigenToMSSPoly<Eigen::Dynamic, Eigen::Dynamic>(poly_mat));

  return plhs[0];
}

template <int Rows, int Cols>
Eigen::Matrix<Eigen::AutoDiffScalar<Eigen::VectorXd>, Rows, Cols>
taylorVarToEigen(const mxArray* taylor_var) {
  if (mxIsEmpty(taylor_var)) {
    return Eigen::Matrix<Eigen::AutoDiffScalar<Eigen::VectorXd>, Rows, Cols>(
        mxGetM(taylor_var), mxGetN(taylor_var));
  }
  auto f = mxGetPropertySafe(taylor_var, "f");
  auto df = mxGetPropertySafe(taylor_var, "df");
  if (mxGetNumberOfElements(df) > 1)
    throw std::runtime_error(
        "TaylorVars of order higher than 1 currently not supported");
  auto ret = matlabToEigenMap<Rows, Cols>(f)
                 .template cast<Eigen::AutoDiffScalar<Eigen::VectorXd>>()
                 .eval();
  typedef Gradient<decltype(ret), Eigen::Dynamic> GradientType;
  auto gradient_matrix =
      matlabToEigenMap<GradientType::type::RowsAtCompileTime,
                       GradientType::type::ColsAtCompileTime>(mxGetCell(df, 0));
  gradientMatrixToAutoDiff(gradient_matrix, ret);
  return ret;
}

template <typename Derived>
mxArray* eigenToTaylorVar(const Eigen::MatrixBase<Derived>& m,
                          int num_variables = Eigen::Dynamic) {
  const int nrhs = 2;
  mxArray* prhs[nrhs];
  prhs[0] = eigenToMatlab(autoDiffToValueMatrix(m));
  mwSize dims[] = {1};
  prhs[1] = mxCreateCellArray(1, dims);
  mxArray* plhs[1];
  mxSetCell(prhs[1], 0,
            eigenToMatlab(autoDiffToGradientMatrix(m, num_variables)));
  mexCallMATLABsafe(1, plhs, nrhs, prhs, "TaylorVar");
  return plhs[0];
}

template <int RowsAtCompileTime, int ColsAtCompileTime, typename DerType>
mxArray* eigenToMatlabGeneral(const Eigen::MatrixBase<Eigen::Matrix<
    Eigen::AutoDiffScalar<DerType>, RowsAtCompileTime, ColsAtCompileTime>>&
                                  mat) {
  return eigenToTaylorVar(mat);
}

template <int RowsAtCompileTime, int ColsAtCompileTime>
mxArray* eigenToMatlabGeneral(const Eigen::MatrixBase<
    Eigen::Matrix<TrigPolyd, RowsAtCompileTime, ColsAtCompileTime>>& mat) {
  return eigenToTrigPoly<RowsAtCompileTime, ColsAtCompileTime>(mat);
}

template <int RowsAtCompileTime, int ColsAtCompileTime>
mxArray* eigenToMatlabGeneral(const Eigen::MatrixBase<
    Eigen::Matrix<double, RowsAtCompileTime, ColsAtCompileTime>>& mat) {
  return eigenToMatlab(mat.const_cast_derived());
}
