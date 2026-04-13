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
/* Internal use only. */
class VariableIdPythonAttorney {
 public:
  VariableIdPythonAttorney() = delete;
  static uint64_t hi(const Variable::Id& id) { return id.hi_; }
  static uint64_t lo(const Variable::Id& id) { return id.lo_; }
  static Variable::Id Construct(uint64_t hi, uint64_t lo) {
    // We need to maintain Id's invariant that the low byte of hi_ is the
    // Variable::Type, by rejecting out-of-bounds types.
    const uint8_t var_type = static_cast<uint8_t>(hi);
    if (var_type > static_cast<uint8_t>(Variable::Type::RANDOM_EXPONENTIAL)) {
      throw std::domain_error("Ill-formed Variable::Id");
    }
    Variable::Id result;
    result.hi_ = hi;
    result.lo_ = lo;
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

    pybind11::int_ concat;
    try {
      concat = pybind11::cast<pybind11::int_>(src);
    } catch (...) {
      return false;
    }

    const pybind11::object hi_py = concat >> pybind11::int_(64);
    const pybind11::object lo_py = concat & pybind11::int_(~uint64_t{});
    const uint64_t hi = hi_py.cast<uint64_t>();
    const uint64_t lo = lo_py.cast<uint64_t>();
    // N.B. "value" is a magic variable declared by pybind11 where we're
    // supposed to put the loaded result.
    value = Attorney::Construct(hi, lo);

    return true;
  }

  static handle cast(drake::symbolic::Variable::Id src,
      return_value_policy /* policy */, handle /* parent */) {
    const pybind11::int_ hi_py{Attorney::hi(src)};
    const pybind11::int_ lo_py{Attorney::lo(src)};
    pybind11::object concat = (hi_py << pybind11::int_(64)) + lo_py;
    return concat.release();
  }
};
}  // namespace detail
}  // namespace pybind11
