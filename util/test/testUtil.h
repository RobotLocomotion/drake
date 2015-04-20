#ifndef TESTUTIL_H_
#define TESTUTIL_H_

#if !defined(WIN32) && !defined(WIN64)
#include <chrono>
#endif

#include <Eigen/Core>
#include <iostream>
#include <stdexcept>
#include <cmath>

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

template<typename DerivedA, typename DerivedB>
void valuecheck(const Eigen::DenseBase<DerivedA>& a, const Eigen::DenseBase<DerivedB>& b, typename DerivedA::Scalar tolerance = 1e-8)
{
  if (!a.isApprox(b)) {
    std::ostringstream stream;
    stream << "Expected:\n" << a << "\nbut got:\n" << b << "\n";
    throw std::runtime_error(stream.str());
  }
}

void valuecheck(double a, double b, double tolerance = 1e-8)
{
  if (std::abs(a - b) > tolerance) {
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
