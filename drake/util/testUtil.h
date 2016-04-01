#ifndef DRAKE_UTIL_TESTUTIL_H_
#define DRAKE_UTIL_TESTUTIL_H_

#include <chrono>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>

#include <Eigen/Core>

// requires <chrono>, which isn't available in MSVC2010...
template <typename TimeT = std::chrono::milliseconds>
struct measure {
  template <typename F, typename... Args>
  static typename TimeT::rep execution(F func, Args&&... args) {
    auto start = std::chrono::system_clock::now();

    // Now call the function with all the parameters you need.
    func(std::forward<Args>(args)...);

    auto duration = std::chrono::duration_cast<TimeT>(
        std::chrono::system_clock::now() - start);

    return duration.count();
  }
};

template <typename Derived>
std::string to_string(const Eigen::MatrixBase<Derived>& a) {
  std::stringstream ss;
  ss << a;
  return ss.str();
}

template <typename T>
void valuecheck(const T& a, const T& b, std::string error_msg = "") {
  if (a != b) {
    std::ostringstream stream;
    stream << error_msg;
    stream << "Expected:\n" << a << "\nbut got:" << b << "\n";
    throw std::runtime_error(stream.str());
  }
}

void valuecheck(double a, double b, double tolerance) {
  if (std::abs(a - b) > tolerance) {
    std::ostringstream stream;
    stream << "Expected:\n" << a << "\nbut got:" << b << "\n";
    throw std::runtime_error(stream.str());
  }
}

#endif  // DRAKE_UTIL_TESTUTIL_H_
