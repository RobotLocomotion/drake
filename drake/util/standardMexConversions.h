//
// Created by Twan Koolen on 10/10/15.
//

#ifndef DRAKE_STANDARDMEXCONVERSIONS_H
#define DRAKE_STANDARDMEXCONVERSIONS_H

#include "drakeMexUtil.h"
#include <string>

/**
 * fromMex specializations
 */

int fromMex(const mxArray* source, int*) {
  if (mxGetM(source) != 1 || mxGetN(source) != 1)
    throw MexToCppConversionError("Expected scalar.");
  return static_cast<int>(mxGetScalar(source));
}

bool fromMex(const mxArray* source, bool*) {
  if (!mxIsLogicalScalar(source))
    throw MexToCppConversionError("Expected logical.");
  return mxGetLogicals(source)[0];
}

/*
 * note: leaving Options, MaxRows, and MaxCols out as template parameters (leaving them to their defaults)
 * results in an internal compiler error on MSVC. See https://connect.microsoft.com/VisualStudio/feedback/details/1847159
 */
template <int Rows, int Cols, int Options, int MaxRows, int MaxCols>
Eigen::Map<const Eigen::Matrix<double, Rows, Cols, Options, MaxRows, MaxCols>> fromMex(const mxArray *mex, Eigen::MatrixBase<Eigen::Map<const Eigen::Matrix<double, Rows, Cols, Options, MaxRows, MaxCols>>> *) {
  if (!mxIsNumeric(mex)) {
    throw MexToCppConversionError("Expected a numeric array");
  }
  return matlabToEigenMap<Rows, Cols>(mex);
}

template<typename DerType, int Rows, int Cols, int Options, int MaxRows, int MaxCols>
Eigen::Matrix<Eigen::AutoDiffScalar<DerType>, Rows, Cols, Options, MaxRows, MaxCols> fromMex(
    const mxArray *mex, Eigen::MatrixBase<Eigen::Matrix<Eigen::AutoDiffScalar<DerType>, Rows, Cols, Options, MaxRows, MaxCols>> *) {
  if (!mxIsClass(mex, "TaylorVar"))
    throw MexToCppConversionError("Expected an array of TaylorVar");

  if (mxIsEmpty(mex)) {
    throw std::runtime_error("Can't parse empty TaylorVar arrays");
  }

  using namespace Eigen;
  using namespace std;

  typedef AutoDiffScalar<DerType> ADScalar;
  auto f = mxGetPropertySafe(mex, "f");
  auto derivs = mxGetPropertySafe(mex, "df");
  if (mxGetNumberOfElements(derivs) > 1)
    throw std::runtime_error("TaylorVars of order higher than 1 currently not supported");
  auto df = mxGetCell(derivs, 0);

  auto dim = mxGetPrSafe(mxGetPropertySafe(mex, "dim"));
  auto rows = static_cast<int>(dim[0]); // note: apparently not always the same as mxGetM(f)
  auto cols = static_cast<int>(dim[1]); // note: apparently not always the same as mxGetN(f)

  if ((MaxRows != Dynamic && rows > MaxRows) || (Rows != Dynamic && rows != Rows)) {
    throw MexToCppConversionError("Row size mismatch. rows: " + to_string(rows) +  ", rows at compile time: " + to_string(Rows) +  ", max rows at compile time: " + to_string(MaxRows));
  }

  if ((MaxCols != Dynamic && cols > MaxCols) || (Cols != Dynamic && cols != Cols))
    throw MexToCppConversionError("Col size mismatch. cols: " + to_string(cols) +  ", cols at compile time: " + to_string(Cols) +  ", max cols at compile time: " + to_string(MaxCols));

  auto num_derivs = mxGetN(df);
  if ((DerType::MaxRowsAtCompileTime != Dynamic && num_derivs > DerType::MaxRowsAtCompileTime) || (DerType::RowsAtCompileTime != Dynamic && num_derivs != DerType::RowsAtCompileTime))
    throw MexToCppConversionError("Derivative size mismatch. num_derivs: " + to_string(num_derivs) +  ", num_derivs at compile time: " + to_string(DerType::RowsAtCompileTime) +  ", max num_derivs at compile time: " + to_string(DerType::MaxRowsAtCompileTime));

  Matrix<ADScalar, Rows, Cols, Options, MaxRows, MaxCols> ret(rows, cols);
  if (!mxIsEmpty(mex)) {
    typedef typename ADScalar::Scalar NormalScalar;
    typedef Matrix<NormalScalar, Rows, Cols, Options, MaxRows, MaxCols> ValueMatrixType;
    ret = Map<ValueMatrixType>(mxGetPrSafe(f), rows, cols).template cast<ADScalar>();

    typedef typename Gradient<ValueMatrixType, DerType::RowsAtCompileTime>::type GradientType;
    typedef Matrix<NormalScalar, GradientType::RowsAtCompileTime, GradientType::ColsAtCompileTime, 0, GradientType::RowsAtCompileTime, DerType::MaxRowsAtCompileTime> GradientTypeFixedMaxSize;
    auto gradient_matrix = Map<GradientTypeFixedMaxSize>(mxGetPrSafe(df), mxGetM(df), num_derivs);
    gradientMatrixToAutoDiff(gradient_matrix, ret);
  }
  return ret;
}

/**
 * toMex specializations
 */

void toMex(const bool& source, mxArray* dest[], int nlhs) {
  dest[0] = mxCreateLogicalScalar(source);
}

/*
 * note: leaving Options, MaxRows, and MaxCols out as template parameters (leaving them to their defaults)
 * results in an internal compiler error on MSVC. See https://connect.microsoft.com/VisualStudio/feedback/details/1847159
 */
template <typename Scalar, int Rows, int Cols, int Options, int MaxRows, int MaxCols>
void toMex(const Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols> &source, mxArray *dest[], int nlhs) {
  if (nlhs > 0)
    dest[0] = eigenToMatlabGeneral(source);
};

template<typename Scalar, int Rows, int Cols>
void toMex(const GradientVar<Scalar, Rows, Cols> &source, mxArray *dest[], int nlhs, bool top_level = true) {
  if (top_level) {
    // check number of output arguments
    if (nlhs > source.maxOrder() + 1) {
      std::ostringstream buf;
      buf << nlhs << " output arguments desired, which is more than the maximum number: " << source.maxOrder() + 1 << ".";
      mexErrMsgTxt(buf.str().c_str());
    }
  }

  if (nlhs != 0) {
    // set an output argument
    toMex(source.value(), dest, nlhs);

    // recurse
    if (source.hasGradient()) {
      toMex(source.gradient(), &dest[1], nlhs - 1, false);
    }
  }
};

#endif //DRAKE_STANDARDMEXCONVERSIONS_H
