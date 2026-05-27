#pragma once

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/common/symbolic/expression.h"
#include "drake/common/symbolic/polynomial.h"
#include "drake/common/symbolic/rational_function.h"

// Whenever we want to cast any array / matrix type of `T` in C++ (e.g.,
// `Eigen::MatrixX<T>`) to a NumPy array, we should have it in the following
// list.
DRAKE_NB_NUMPY_OBJECT_DTYPE(  // NOLINT
    drake::symbolic::Expression)
DRAKE_NB_NUMPY_OBJECT_DTYPE(  // NOLINT
    drake::symbolic::Formula)
DRAKE_NB_NUMPY_OBJECT_DTYPE(  // NOLINT
    drake::symbolic::Monomial)
DRAKE_NB_NUMPY_OBJECT_DTYPE(  // NOLINT
    drake::symbolic::Polynomial)
DRAKE_NB_NUMPY_OBJECT_DTYPE(  // NOLINT
    drake::symbolic::RationalFunction)
DRAKE_NB_NUMPY_OBJECT_DTYPE(  // NOLINT
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

namespace PYDRAKE_BINDER_NAMESPACE {
namespace detail {
namespace py = PYDRAKE_BINDER_NAMESPACE;
template <>
struct type_caster<drake::symbolic::Variable::Id> {
 public:
  using Attorney = drake::symbolic::VariableIdPythonAttorney;

#ifdef PYDRAKE_USE_PYBIND11
  PYBIND11_TYPE_CASTER(drake::symbolic::Variable::Id, _("int"));
#else  // PYDRAKE_USE_NANOBIND
  NB_TYPE_CASTER(drake::symbolic::Variable::Id, const_name("int"));
#endif

#ifdef PYDRAKE_USE_PYBIND11
  bool load(handle src, bool /* convert */)
#else  // PYDRAKE_USE_NANOBIND
  bool from_python(handle src, uint8_t, cleanup_list*)
#endif
  {
    if (!src) {
      return false;
    }

    py::int_ concat;
    try {
      concat = py::cast<py::int_>(src);
    } catch (...) {
      return false;
    }

    const py::object hi_py = concat >> py::int_(64);
    const py::object lo_py = concat & py::int_(~uint64_t{});
    const uint64_t hi = py::cast<uint64_t>(hi_py);
    const uint64_t lo = py::cast<uint64_t>(lo_py);
    // N.B. "value" is a magic variable declared by pybind11 where we're
    // supposed to put the loaded result.
    value = Attorney::Construct(hi, lo);

    return true;
  }

#ifdef PYDRAKE_USE_PYBIND11
  static handle cast(drake::symbolic::Variable::Id src, rv_policy /* policy */,
      handle /* parent */)
#else  // PYDRAKE_USE_NANOBIND
  static handle from_cpp(
      const drake::symbolic::Variable::Id& src, rv_policy, cleanup_list*)
#endif
  {
    const py::int_ hi_py{Attorney::hi(src)};
    const py::int_ lo_py{Attorney::lo(src)};
    py::object concat = (hi_py << py::int_(64)) + lo_py;
    return concat.release();
  }
};
}  // namespace detail
}  // namespace PYDRAKE_BINDER_NAMESPACE
