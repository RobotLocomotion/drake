#ifndef TESTUTIL_H_
#define TESTUTIL_H_

#include <chrono>
#include <Eigen/Core>
#include <iostream>
#include <stdexcept>
#include "mex.h"


template<typename TimeT = std::chrono::milliseconds>
struct measure
{
  template<typename F, typename ...Args>
  static typename TimeT::rep execution(F func, Args&&... args)
  {
    auto start = std::chrono::system_clock::now();

    // Now call the function with all the parameters you need.
    func(std::forward<Args>(args)...);

    auto duration = std::chrono::duration_cast< TimeT>
    (std::chrono::system_clock::now() - start);

    return duration.count();
  }
};

template<typename DerivedA, typename DerivedB>
void valuecheck(const Eigen::DenseBase<DerivedA>& a, const Eigen::DenseBase<DerivedB>& b, typename DerivedA::Scalar tolerance = 1e-8)
{
  if (!a.isApprox(b)) {
    std::ostringstream stream;
    stream << "Expected:\n" << a << "\nbut got:" << b << "\n";
    throw std::runtime_error(stream.str());
  }
}

void valuecheck(double a, double b, double tolerance = 1e-8)
{
  if (abs(a - b) > tolerance) {
    std::ostringstream stream;
    stream << "Expected:\n" << a << "\nbut got:" << b << "\n";
    throw std::runtime_error(stream.str());
  }
}

template<int RowsAtCompileTime = Eigen::Dynamic, int ColsAtCompileTime = Eigen::Dynamic>
Eigen::Matrix<double, RowsAtCompileTime, ColsAtCompileTime> matlabToEigen(const mxArray* matlab_array)
{
  const mwSize* size_array = mxGetDimensions(matlab_array);
  Eigen::Matrix<double, RowsAtCompileTime, ColsAtCompileTime> ret(size_array[0], size_array[1]);
  memcpy(ret.data(), mxGetPr(matlab_array), sizeof(double) * ret.size());
  return ret;
}

template<int RowsAtCompileTime, int ColsAtCompileTime>
mxArray* eigenToMatlab(Eigen::Matrix<double, RowsAtCompileTime, ColsAtCompileTime> &m)
{
  mxArray* pm = mxCreateDoubleMatrix(m.rows(), m.cols(), mxREAL);
  if (m.rows() * m.cols() > 0)
    memcpy(mxGetPr(pm), m.data(), sizeof(double) * m.rows() * m.cols());
  return pm;
}

#endif /* TESTUTIL_H_ */
