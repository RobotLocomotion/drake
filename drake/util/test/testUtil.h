#ifndef TESTUTIL_H_
#define TESTUTIL_H_

#include <chrono>
#include <Eigen/Core>
#include <sstream>
#include <string>
#include <stdexcept>
#include <cmath>

// requires <chrono>, which isn't available in MSVC2010...
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

template<typename Derived>
std::string to_string(const Eigen::MatrixBase<Derived> & a)
{
  std::stringstream ss;
  ss << a;
  return ss.str();
}

template <typename T>
void valuecheck(const T& a, const T& b)
{
  if (a != b) {
    std::ostringstream stream;
    stream << "Expected:\n" << a << "\nbut got:" << b << "\n";
    throw std::runtime_error(stream.str());
  }
}

/*
 * +++tk:
 * Note: can't call this function 'valuecheck' because it conflicts with our valuecheck template for general types above on GCC.
 * (yes, even though the number of template arguments and the number of function arguments is different!)
 * It does compile on MSVC 2010 and Clang.
 * What I believe happens is that during argument-dependent name lookup, GCC instantiates Eigen::MatrixBase<int>, which causes errors deep in Eigen::internal code
 * Possibly related to this discussion:
 * https://stackoverflow.com/questions/25925551/gcc-and-clang-implicitly-instantiate-template-arguments-during-operator-overload
 *
 * Fairly minimal failure example (still involving Eigen):
 * #include <Eigen/Core>
 * #include <iostream>
 *
 * template <typename Derived>
 * void foo(const Eigen::MatrixBase<Derived>& m, double bla) {
 *   std::cout << m << ", " << bla << std::endl;
 * }
 *
 * template <typename T>
 * void foo(const T& t) {
 *   std::cout << t << std::endl;
 * }
 *
 * int main() {
 *   int i = 1;
 *   foo<int>(i); // doesn't compile
 *   foo(i); // compiles
 *   return 0;
 * }
 */
template<typename DerivedA, typename DerivedB>
void valuecheckMatrix(const Eigen::MatrixBase<DerivedA>& a, const Eigen::MatrixBase<DerivedB>& b, double tol, std::string error_msg = "")
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
          bool both_nan = std::isnan(a(i, j)) && std::isnan(b(i, j));
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

#endif /* TESTUTIL_H_ */
