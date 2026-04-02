#pragma once

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/common/symbolic/expression.h"
#include "drake/common/symbolic/polynomial.h"
#include "drake/common/symbolic/rational_function.h"

// Whenever we want to cast any array / matrix type of `T` in C++ (e.g.,
// `Eigen::MatrixX<T>`) to a NumPy array, we should have it in the following
// list.
DRAKE_PYBIND11_NUMPY_OBJECT_DTYPE(  // NOLINT
    drake::symbolic::Expression)
DRAKE_PYBIND11_NUMPY_OBJECT_DTYPE(  // NOLINT
    drake::symbolic::Formula)
DRAKE_PYBIND11_NUMPY_OBJECT_DTYPE(  // NOLINT
    drake::symbolic::Monomial)
DRAKE_PYBIND11_NUMPY_OBJECT_DTYPE(  // NOLINT
    drake::symbolic::Polynomial)
DRAKE_PYBIND11_NUMPY_OBJECT_DTYPE(  // NOLINT
    drake::symbolic::RationalFunction)
DRAKE_PYBIND11_NUMPY_OBJECT_DTYPE(  // NOLINT
    drake::symbolic::Variable)

namespace drake {
namespace symbolic {
class VariableIdPythonAttorney {
 public:
  VariableIdPythonAttorney() = delete;
  static Variable::Id construct(uint64_t value) {
    Variable::Id result;
    result.value_ = value;
    return result;
  }
};
}  // namespace symbolic
}  // namespace drake

namespace pybind11 {
namespace detail {
template <>
struct type_caster<drake::symbolic::Variable::Id> {
 public:
  using Attorney = drake::symbolic::VariableIdPythonAttorney;

  PYBIND11_TYPE_CASTER(drake::symbolic::Variable::Id, _("int"));

  bool load(handle src, bool /* convert */) {
    if (!src) {
      return false;
    }

    pybind11::int_ integer;
    try {
      integer = pybind11::cast<pybind11::int_>(src);
    } catch (...) {
      return false;
    }

    value = Attorney::construct(integer.cast<uint64_t>());

    return true;
  }

  static handle cast(drake::symbolic::Variable::Id src,
      return_value_policy /* policy */, handle /* parent */) {
    pybind11::int_ value{src.value()};
    return value.release();
  }
};
}  // namespace detail
}  // namespace pybind11
