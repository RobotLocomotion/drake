//
// Created by Twan Koolen on 10/10/15.
//

#ifndef DRAKE_STANDARDMEXCONVERSIONS_H
#define DRAKE_STANDARDMEXCONVERSIONS_H

#include <string>
#include "drakeMexUtil.h"
#include "GradientVar.h"

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

std::set<int> fromMex(const mxArray *source, std::set<int> *) {
  std::set<int> ret;
  int num_robot = static_cast<int>(mxGetNumberOfElements(source));
  double *data = mxGetPrSafe(source);
  for (int i = 0; i < num_robot; i++) {
    ret.insert((int) data[i]);
  }
  return ret;
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

// quick version for VectorXi.
// TODO: generalize to arbitrary matrices
Eigen::Map<const Eigen::VectorXi> fromMex(const mxArray *source, Eigen::MatrixBase<Eigen::Map<const Eigen::VectorXi> > *) {
  if (!mxIsInt32(source))
    throw MexToCppConversionError("Expected an int32 array");
  if (mxGetM(source) != 1 && mxGetN(source) != 1)
    throw MexToCppConversionError("Expected a 1 x n or n x 1 int32 array");
  auto num_elements = mxGetNumberOfElements(source);
  return Eigen::Map<const Eigen::VectorXi>(reinterpret_cast<int*>(mxGetData(source)), num_elements);
}

template <typename T>
std::vector<T> fromMex(const mxArray* source, std::vector<T>*) {
  if (!mxIsCell(source))
    throw MexToCppConversionError("Expected a cell array");
  if (mxGetM(source) != 1 && mxGetN(source) != 1)
    throw MexToCppConversionError("Expected a 1 x n or n x 1 cell array");

  auto num_elements = mxGetNumberOfElements(source);
  std::vector<T> ret;
  ret.reserve(num_elements);
  for (size_t i = 0; i < num_elements; i++) {
    ret.push_back(fromMex(mxGetCell(source, i), static_cast<T*>(nullptr)));
  }
  return ret;
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

    if (mxIsSparse(df)) {
      auto gradient_matrix = GradientTypeFixedMaxSize(matlabToEigenSparse(df)); // TODO: inefficient
      gradientMatrixToAutoDiff(gradient_matrix, ret);
    }
    else {
      auto gradient_matrix = Map<GradientTypeFixedMaxSize>(mxGetPrSafe(df), mxGetM(df), num_derivs); // FIXME: handle sparse case
      gradientMatrixToAutoDiff(gradient_matrix, ret);
    }

  }
  return ret;
}

/**
 * toMex specializations
 */

int toMex(const bool& source, mxArray* dest[], int nlhs) {
  dest[0] = mxCreateLogicalScalar(source);
  return 1;
}

/*
 * note: leaving Options, MaxRows, and MaxCols out as template parameters (leaving them to their defaults)
 * results in an internal compiler error on MSVC. See https://connect.microsoft.com/VisualStudio/feedback/details/1847159
 */
template <typename Scalar, int Rows, int Cols, int Options, int MaxRows, int MaxCols>
int toMex(const Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols> &source, mxArray *dest[], int nlhs) {
  if (nlhs > 0)
    dest[0] = eigenToMatlabGeneral<Rows, Cols>(source);
  return 1;
};

template<typename Scalar, int Rows, int Cols>
int toMex(const GradientVar<Scalar, Rows, Cols> &source, mxArray *dest[], int nlhs, bool top_level = true) {
  if (top_level) {
    // check number of output arguments
    if (nlhs > source.maxOrder() + 1) {
      std::ostringstream buf;
      buf << nlhs << " output arguments desired, which is more than the maximum number: " << source.maxOrder() + 1 << ".";
      mexErrMsgTxt(buf.str().c_str());
    }
  }

  int outputs = 0;
  if (nlhs != 0) {
    // set an output argument
    outputs += toMex(source.value(), dest, nlhs);

    // recurse
    if (source.hasGradient()) {
      outputs += toMex(source.gradient(), &dest[1], nlhs - outputs, false);
    }
  }
  return outputs;
};

template <typename A, typename B>
int toMex(const std::pair<A, B>& source, mxArray* dest[], int nlhs) {
  int num_outputs = 0;
  num_outputs += toMex(source.first, dest, nlhs - num_outputs);
  num_outputs += toMex(source.second, &dest[num_outputs], nlhs - num_outputs);
  return num_outputs;
}

template <typename T>
int toMex(const std::vector<T>& source, mxArray* dest[], int nlhs) {
  mwSize dims[] = {source.size()};
  dest[0] = mxCreateCellArray(1, dims);
  for (size_t i = 0; i < source.size(); i++) {
    mxArray *cell[1];
    toMex(source.at(i), cell, 1);
    mxSetCell(dest[0], i, cell[0]);
  }

  return 1;
}

#endif //DRAKE_STANDARDMEXCONVERSIONS_H
