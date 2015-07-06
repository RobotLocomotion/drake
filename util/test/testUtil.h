#ifndef TESTUTIL_H_
#define TESTUTIL_H_

#if !defined(WIN32) && !defined(WIN64)
#include <chrono>
#endif

#include <Eigen/Core>
#include <sstream>
#include <string>
#include <stdexcept>
#include <cmath>
#include "../drakeFloatingPointUtil.h"

// requires <chrono>, which isn't available in MSVC2010...
#if !defined(WIN32) && !defined(WIN64)
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
#endif

template<typename Derived>
std::string to_string(const Eigen::MatrixBase<Derived> & a)
{
  std::stringstream ss;
  ss << a;
  return ss.str();
}

template<typename DerivedA, typename DerivedB>
void valuecheck(const Eigen::MatrixBase<DerivedA>& a, const Eigen::MatrixBase<DerivedB>& b, double tol, std::string error_msg = "")
{
  // note: isApprox uses the L2 norm, so is bad for comparing against zero
  if (a.rows() != b.rows() || a.cols() != b.cols()) {
    throw std::runtime_error(
        "Drake:ValueCheck ERROR:" + error_msg + "size mismatch: (" + std::to_string(static_cast<unsigned long long>(a.rows())) + " by " + std::to_string(static_cast<unsigned long long>(a.cols())) + ") and (" + std::to_string(static_cast<unsigned long long>(b.rows())) + " by "
            + std::to_string(static_cast<unsigned long long>(b.cols())) + ")");
  }
  if (!(a - b).isZero(tol)) {
    if (!a.allFinite() && !b.allFinite()) {
      // could be failing because inf-inf = nan
      bool ok = true;
      for (int i = 0; i < a.rows(); i++)
        for (int j = 0; j < a.cols(); j++) {
          bool both_positive_infinity = a(i, j) == std::numeric_limits<double>::infinity() && b(i, j) == std::numeric_limits<double>::infinity();
          bool both_negative_infinity = a(i, j) == -std::numeric_limits<double>::infinity() && b(i, j) == -std::numeric_limits<double>::infinity();
          bool both_nan = isNaN(a(i, j)) && isNaN(b(i, j));
          ok = ok && (both_positive_infinity || both_negative_infinity || (both_nan) || (std::abs(a(i, j) - b(i, j)) < tol));
        }
      if (ok)
        return;
    }
    error_msg += "A:\n" + to_string(a) + "\nB:\n" + to_string(b) + "\n";
    throw std::runtime_error("Drake:ValueCheck ERROR:" + error_msg);
  }
}

void valuecheck(double a, double b, double tolerance)
{
  if (std::abs(a - b) > tolerance) {
    std::ostringstream stream;
    stream << "Expected:\n" << a << "\nbut got:" << b << "\n";
    throw std::runtime_error(stream.str());
  }
}

void valuecheck(int a, int b)
{
  if (a != b) {
    std::ostringstream stream;
    stream << "Expected:\n" << a << "\nbut got:" << b << "\n";
    throw std::runtime_error(stream.str());
  }
}

template <typename T>
void valuecheck(T const & a, T const & b)
{
  if (a != b) {
    std::ostringstream stream;
    stream << "Expected:\n" << a << "\nbut got:" << b << "\n";
    throw std::runtime_error(stream.str());
  }
}

#endif /* TESTUTIL_H_ */
