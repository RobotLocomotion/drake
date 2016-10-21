#pragma once

//
// Created by Twan Koolen on 10/10/15.
//

#include <string>
#include "drake/matlab/util/mexify.h"

/**
 * fromMex specializations
 */

bool isConvertibleFromMex(const mxArray* source, int*,
                          std::ostream* log) NOEXCEPT {
  if (mxGetM(source) != 1 || mxGetN(source) != 1) {
    if (log) *log << "Expected scalar.";
    return false;
  }
  return true;
}

int fromMexUnsafe(const mxArray* source, int*) {
  return static_cast<int>(mxGetScalar(source));
}

bool isConvertibleFromMex(const mxArray* source, bool*,
                          std::ostream* log) NOEXCEPT {
  if (!mxIsLogicalScalar(source)) {
    if (log) *log << "Expected logical scalar.";
    return false;
  }
  return true;
}

bool fromMexUnsafe(const mxArray* source, bool*) {
  return mxGetLogicals(source)[0];
}

/*
 * note: leaving Options, MaxRows, and MaxCols out as template parameters
 * (leaving them to their defaults)
 * results in an internal compiler error on MSVC. See
 * https://connect.microsoft.com/VisualStudio/feedback/details/1847159
 */
template <int Rows, int Cols, int Options, int MaxRows, int MaxCols>
bool isConvertibleFromMex(
    const mxArray* mex,
    Eigen::MatrixBase<Eigen::Map<
        const Eigen::Matrix<double, Rows, Cols, Options, MaxRows, MaxCols>>>*,
    std::ostream* log) NOEXCEPT {
  // TODO(tkoolen): size checks?
  if (!mxIsNumeric(mex)) {
    if (log) *log << "Expected a numeric array";
    return false;
  }
  return true;
}

template <int Rows, int Cols, int Options, int MaxRows, int MaxCols>
Eigen::Map<const Eigen::Matrix<double, Rows, Cols, Options, MaxRows, MaxCols>>
fromMexUnsafe(
    const mxArray* mex,
    Eigen::MatrixBase<Eigen::Map<
        const Eigen::Matrix<double, Rows, Cols, Options, MaxRows, MaxCols>>>*) {
  return matlabToEigenMap<Rows, Cols>(mex);
}

// quick version for VectorXi.
// TODO(tkoolen): generalize to arbitrary matrices of integers
bool isConvertibleFromMex(const mxArray* source,
                          Eigen::MatrixBase<Eigen::Map<const Eigen::VectorXi>>*,
                          std::ostream* log) NOEXCEPT {
  if (!mxIsInt32(source)) {
    if (log) *log << "Expected an int32 array";
    return false;
  }
  if (mxGetM(source) != 1 && mxGetN(source) != 1) {
    if (log) *log << "Expected a 1 x n or n x 1 int32 array";
    return false;
  }
  return true;
}

Eigen::Map<const Eigen::VectorXi> fromMexUnsafe(
    const mxArray* source,
    Eigen::MatrixBase<Eigen::Map<const Eigen::VectorXi>>*) {
  auto num_elements = mxGetNumberOfElements(source);
  return Eigen::Map<const Eigen::VectorXi>(
      reinterpret_cast<int*>(mxGetData(source)), num_elements);
}

template <typename T>
bool isConvertibleFromMex(const mxArray* source, std::vector<T>*,
                          std::ostream* log) NOEXCEPT {
  if (!mxIsCell(source)) {
    if (log) *log << "Expected a cell array";
    return false;
  }
  if (mxGetM(source) != 1 && mxGetN(source) != 1) {
    if (log) *log << "Expected a 1 x n or n x 1 cell array";
    return false;
  }
  return true;
}

template <typename T>
std::vector<T> fromMexUnsafe(const mxArray* source, std::vector<T>*) {
  auto num_elements = mxGetNumberOfElements(source);
  std::vector<T> ret;
  ret.reserve(num_elements);
  for (size_t i = 0; i < num_elements; i++) {
    ret.push_back(
        fromMexUnsafe(mxGetCell(source, i), static_cast<T*>(nullptr)));
  }
  return ret;
}

template <typename DerType, int Rows, int Cols, int Options, int MaxRows,
          int MaxCols>
bool isConvertibleFromMex(
    const mxArray* mex,
    Eigen::MatrixBase<Eigen::Matrix<Eigen::AutoDiffScalar<DerType>, Rows, Cols,
                                    Options, MaxRows, MaxCols>>*,
    std::ostream* log) NOEXCEPT {
  using namespace Eigen;
  using namespace std;

  if (!mxIsClass(mex, "TaylorVar")) {
    if (log) *log << "Expected an array of TaylorVar";
    return false;
  }
  if (mxIsEmpty(mex)) {
    if (log) *log << "Can't parse empty TaylorVar arrays";
    return false;
  }
  auto derivs = mxGetPropertySafe(mex, "df");
  if (mxGetNumberOfElements(derivs) > 1) {
    if (log)
      *log << "TaylorVars of order higher than 1 currently not supported";
    return false;
  }
  auto df = mxGetCell(derivs, 0);
  auto dim = mxGetPrSafe(mxGetPropertySafe(mex, "dim"));
  auto rows = static_cast<int>(
      dim[0]);  // note: apparently not always the same as mxGetM(f)
  auto cols = static_cast<int>(
      dim[1]);  // note: apparently not always the same as mxGetN(f)

  if ((MaxRows != Dynamic && rows > MaxRows) ||
      (Rows != Dynamic && rows != Rows)) {
    if (log)
      *log << "Row size mismatch. rows: " << to_string(rows)
           << ", rows at compile time: " << to_string(Rows)
           << ", max rows at compile time: " << to_string(MaxRows);
    return false;
  }

  if ((MaxCols != Dynamic && cols > MaxCols) ||
      (Cols != Dynamic && cols != Cols)) {
    if (log)
      *log << "Col size mismatch. cols: " << to_string(cols)
           << ", cols at compile time: " << to_string(Cols)
           << ", max cols at compile time: " << to_string(MaxCols);
    return false;
  }

  Eigen::Index num_derivs = mxGetN(df);
  if ((DerType::MaxRowsAtCompileTime != Dynamic &&
       num_derivs > DerType::MaxRowsAtCompileTime) ||
      (DerType::RowsAtCompileTime != Dynamic &&
       num_derivs != DerType::RowsAtCompileTime)) {
    if (log)
      *log << "Derivative size mismatch. num_derivs: " << to_string(num_derivs)
           << ", num_derivs at compile time: "
           << to_string(DerType::RowsAtCompileTime)
           << ", max num_derivs at compile time: "
           << to_string(DerType::MaxRowsAtCompileTime);
    return false;
  }

  return true;
}

template <typename DerType, int Rows, int Cols, int Options, int MaxRows,
          int MaxCols>
Eigen::Matrix<Eigen::AutoDiffScalar<DerType>, Rows, Cols, Options, MaxRows,
              MaxCols>
fromMexUnsafe(
    const mxArray* mex,
    Eigen::MatrixBase<Eigen::Matrix<Eigen::AutoDiffScalar<DerType>, Rows, Cols,
                                    Options, MaxRows, MaxCols>>*) {
  using namespace Eigen;

  typedef AutoDiffScalar<DerType> ADScalar;
  auto f = mxGetPropertySafe(mex, "f");
  auto derivs = mxGetPropertySafe(mex, "df");
  auto df = mxGetCell(derivs, 0);

  auto dim = mxGetPrSafe(mxGetPropertySafe(mex, "dim"));
  auto rows = static_cast<int>(
      dim[0]);  // note: apparently not always the same as mxGetM(f)
  auto cols = static_cast<int>(
      dim[1]);  // note: apparently not always the same as mxGetN(f)

  Matrix<ADScalar, Rows, Cols, Options, MaxRows, MaxCols> ret(rows, cols);
  if (!mxIsEmpty(mex)) {
    typedef typename ADScalar::Scalar NormalScalar;
    typedef Matrix<NormalScalar, Rows, Cols, Options, MaxRows, MaxCols>
        ValueMatrixType;
    ret = Map<ValueMatrixType>(mxGetPrSafe(f), rows, cols)
              .template cast<ADScalar>();

    typedef typename Gradient<ValueMatrixType, DerType::RowsAtCompileTime>::type
        GradientType;
    typedef Matrix<NormalScalar, GradientType::RowsAtCompileTime,
                   GradientType::ColsAtCompileTime, 0,
                   GradientType::RowsAtCompileTime,
                   DerType::MaxRowsAtCompileTime> GradientTypeFixedMaxSize;

    if (mxIsSparse(df)) {
      auto gradient_matrix = GradientTypeFixedMaxSize(
          matlabToEigenSparse(df));  // TODO(tkoolen): inefficient
      gradientMatrixToAutoDiff(gradient_matrix, ret);
    } else {
      auto num_derivs = mxGetN(df);
      auto gradient_matrix = Map<GradientTypeFixedMaxSize>(
          mxGetPrSafe(df), mxGetM(df), num_derivs);
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
 * note: leaving Options, MaxRows, and MaxCols out as template parameters
 * (leaving them to their defaults)
 * results in an internal compiler error on MSVC. See
 * https://connect.microsoft.com/VisualStudio/feedback/details/1847159
 */
template <typename Scalar, int Rows, int Cols, int Options, int MaxRows,
          int MaxCols>
int toMex(
    const Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>& source,
    mxArray* dest[], int nlhs) {
  if (nlhs > 0) dest[0] = eigenToMatlabGeneral<Rows, Cols>(source);
  return 1;
}

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
    mxArray* cell[1];
    toMex(source.at(i), cell, 1);
    mxSetCell(dest[0], i, cell[0]);
  }

  return 1;
}

namespace drake {
namespace internal {
template <size_t Index>
struct TupleToMexHelper {
  template <typename... Ts>
  static int run(const std::tuple<Ts...>& source, mxArray* dest[], int nlhs,
                 int num_outputs = 0) {
    constexpr size_t tuple_index = sizeof...(Ts)-Index;
    num_outputs += toMex(std::get<tuple_index>(source), &dest[num_outputs],
                         nlhs - num_outputs);
    return TupleToMexHelper<Index - 1>::run(source, dest, nlhs, num_outputs);
  }
};

template <>
struct TupleToMexHelper<0> {
  template <typename... Ts>
  static int run(const std::tuple<Ts...>& source, mxArray* dest[], int nlhs,
                 int num_outputs = 0) {
    return num_outputs;
  }
};
}
}

template <typename... Ts>
int toMex(const std::tuple<Ts...>& source, mxArray* dest[], int nlhs) {
  return drake::internal::TupleToMexHelper<sizeof...(Ts)>::run(source, dest,
                                                               nlhs);
}
