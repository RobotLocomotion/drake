#pragma once

#include <algorithm>
#include <cmath>
#include <type_traits>

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/common/drake_bool.h"

namespace drake {
namespace pydrake {
namespace internal {

/* Returns X.inverse() but with special casing for symbolic types. */
template <typename T>
MatrixX<T> CalcMatrixInverse(const MatrixX<T>& X) {
  constexpr bool is_symbolic_algebra = !scalar_predicate<T>::is_bool;
  if (is_symbolic_algebra) {
    const int N = X.rows();
    // Note that MatrixX<Expression>.inverse() will fail for symbolic matrices
    // that contain variables. We'll special case the sizes shown to work in
    // `expression_matrix_test.cc`.
    if (N == X.cols()) {
      switch (N) {
        case 1:
          return Eigen::Matrix<T, 1, 1>(X).inverse();
        case 2:
          return Eigen::Matrix<T, 2, 2>(X).inverse();
        case 3:
          return Eigen::Matrix<T, 3, 3>(X).inverse();
        case 4:
          return Eigen::Matrix<T, 4, 4>(X).inverse();
      }
    }
  }
  return X.inverse();
}

// TODO(eric.cousineau): Deprecate these methods once we support proper NumPy
// UFuncs.

/* Adds math function overloads (ADL free functions) for the given class `T`.
This is used for both NumPy methods and `pydrake.math` module functions.
@param obj If this is py::class_, this is intended to register class methods on
to overload NumPy's math methods. If this is py::module_, this is intended to
register the overloads in `pydrake.math`. */
template <typename T, typename PyObject>
void BindMathOperators(PyObject* obj) {
  // Prepare for ADL in case T is `double`.
  using std::abs;
  using std::acos;
  using std::asin;
  using std::atan;
  using std::atan2;
  using std::ceil;
  using std::cos;
  using std::cosh;
  using std::exp;
  using std::floor;
  using std::isnan;
  using std::log;
  using std::max;
  using std::min;
  using std::pow;
  using std::sin;
  using std::sinh;
  using std::sqrt;
  using std::tan;
  using std::tanh;

  // A few functions differ for module vs class bindings.
  constexpr bool is_module = std::is_same_v<PyObject, py::module_>;

  // When binding certain types (e.g., symbolic::Variable), operations return
  // a type other than the argument type.
  using Promoted = decltype(std::declval<T>() + std::declval<T>());

  // Add functions that exist both on the class and in the module.
  (*obj)  // BR
      .def("abs", [](const T& x) { return abs(x); })
      .def("acos", [](const T& x) { return acos(x); })
      .def("arccos", [](const T& x) { return acos(x); })
      .def("asin", [](const T& x) { return asin(x); })
      .def("arcsin", [](const T& x) { return asin(x); })
      .def("atan", [](const T& x) { return atan(x); })
      .def("arctan", [](const T& x) { return atan(x); })
      .def("ceil", [](const T& x) { return ceil(x); })
      .def("cos", [](const T& x) { return cos(x); })
      .def("cosh", [](const T& x) { return cosh(x); })
      .def("exp", [](const T& x) { return exp(x); })
      .def("floor", [](const T& x) { return floor(x); })
      .def("log", [](const T& x) { return log(x); })
      .def("max",
          [](const T& x, const T& y) {
            if constexpr (std::is_same_v<T, Promoted>) {
              return max(x, y);
            } else {
              // For types that use promotion, we need to promote prior to
              // calling the operator, to avoid ADL confusion with std::max.
              return max(Promoted{x}, Promoted{y});
            }
          })
      .def("min",
          [](const T& x, const T& y) {
            if constexpr (std::is_same_v<T, Promoted>) {
              return min(x, y);
            } else {
              // For types that use promotion, we need to promote prior to
              // calling the operator, to avoid ADL confusion with std::max.
              return min(Promoted{x}, Promoted{y});
            }
          })
      .def("pow", [](const T& x, double y) { return pow(x, y); })
      .def("sin", [](const T& x) { return sin(x); })
      .def("sinh", [](const T& x) { return sinh(x); })
      .def("sqrt", [](const T& x) { return sqrt(x); })
      .def("tan", [](const T& x) { return tan(x); })
      .def("tanh", [](const T& x) { return tanh(x); });
  // For symbolic types (only), the `pow` exponent can also be an Expression.
  if constexpr (!scalar_predicate<T>::is_bool) {
    obj->def("pow", [](const T& x, const Promoted& y) { return pow(x, y); });
  }
  // For atan2 the named arguments change for the module function vs the method.
  {
    auto func = [](const T& y, const T& x) { return atan2(y, x); };
    if constexpr (is_module) {
      obj->def("atan2", func, py::arg("y"), py::arg("x"));
      obj->def("arctan2", func, py::arg("y"), py::arg("x"));
    } else {
      obj->def("atan2", func, py::arg("x"),
          "Uses ``self`` for ``y`` in ``atan2(y, x)``.");
      obj->def("arctan2", func, py::arg("x"),
          "Uses ``self`` for ``y`` in ``arctan2(y, x)``.");
    }
  }

  // Add functions specific to either the class or the module.
  if constexpr (is_module) {
    auto& m = *obj;
    m  // BR
        .def("inv",
            [](const MatrixX<T>& X) {
              if constexpr (std::is_same_v<T, Promoted>) {
                return CalcMatrixInverse(X);
              } else {
                return CalcMatrixInverse(X.template cast<Promoted>().eval());
              }
            })
        .def(
            "isnan", [](const T& x) { return isnan(x); }, py::arg("x"));
  } else {
    auto& cls = *obj;
    cls  // BR
        .def("__abs__", [](const T& x) { return abs(x); })
        .def("__ceil__", [](const T& x) { return ceil(x); })
        .def("__floor__", [](const T& x) { return floor(x); });
  }
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
