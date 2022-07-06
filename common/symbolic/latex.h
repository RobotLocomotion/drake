#pragma once

#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>

#include "drake/common/drake_copyable.h"
#include "drake/common/symbolic/expression.h"

namespace drake {
namespace symbolic {

/// Generates a LaTeX string representation of `e` with floating point
/// coefficients displayed using `precision`.
/// @pydrake_mkdoc_identifier{expression}
std::string ToLatex(const Expression& e, int precision = 3);

/// Generates a LaTeX string representation of `f` with floating point
/// coefficients displayed using `precision`.
/// @pydrake_mkdoc_identifier{formula}
std::string ToLatex(const Formula& f, int precision = 3);

/// Generates a Latex string representation of `val` displayed with `precision`,
/// with one exception. If the fractional part of `val` is exactly zero, then
/// `val` is represented perfectly as an integer, and is displayed without the
/// trailing decimal point and zeros (in this case, the `precision` argument is
/// ignored).
std::string ToLatex(double val, int precision = 3);

/// Generates a LaTeX string representation of `M` with floating point
/// coefficients displayed using `precision`.
/// @pydrake_mkdoc_identifier{matrix}
template <typename Derived>
std::string ToLatex(const Eigen::PlainObjectBase<Derived>& M,
                    int precision = 3) {
  std::ostringstream oss;
  oss << "\\begin{bmatrix}";
  for (int i = 0; i < M.rows(); ++i) {
    for (int j = 0; j < M.cols(); ++j) {
      oss << " " << ToLatex(M(i, j), precision);
      if (j < M.cols() - 1) {
        oss << " &";
      }
    }
    if (i < M.rows() - 1) {
      oss << " \\\\";
    }
  }
  oss << " \\end{bmatrix}";
  return oss.str();
}

}  // namespace symbolic
}  // namespace drake
