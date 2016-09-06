#pragma once

#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>

#include <Eigen/Core>

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
    stream << "Expected:\n" << a << "\nbut got:\n" << b << "\n";
    throw std::runtime_error(stream.str());
  }
}

void valuecheck(double a, double b, double tolerance) {
  if (std::abs(a - b) > tolerance) {
    std::ostringstream stream;
    stream << "Expected:\n" << a << "\nbut got:\n" << b << "\n";
    throw std::runtime_error(stream.str());
  }
}
