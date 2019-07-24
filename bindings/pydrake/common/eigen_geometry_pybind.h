#pragma once

/// @file
/// Provides pybind11 `type_caster`s for Eigen geometric types.
/// N.B. This uses some of pybind's coding conventions.
///
/// See http://pybind11.readthedocs.io/en/stable/advanced/cast/custom.html for
/// more details on custom type casters.

#include <string>
#include <utility>

#include <Eigen/Dense>
#include "pybind11/eigen.h"

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/common/drake_assert.h"

namespace drake {
namespace pydrake {
namespace internal {

// Implements a `type_caster<>` specialization used to convert types using a
// specific wrapping policy.
// @tparam Wrapper
//  Struct which must provide `Type`, `WrappedType`, `unwrap`, and `wrap`.
// @tparam copy_only
//  This may only pass between C++ and Python as copies, not references.
// See `eigen_wrapper_*` structs below for more details.
template <typename Wrapper>
struct type_caster_wrapped {
  using Type = typename Wrapper::Type;
  using WrappedType = typename Wrapper::WrappedType;
  using WrappedTypeCaster = py::detail::type_caster<WrappedType>;

  // Python to C++.
  bool load(py::handle src, bool converter) {
    WrappedTypeCaster caster;
    if (!caster.load(src, converter)) {
      return false;
    }
    value_ = Wrapper::unwrap(caster.operator WrappedType&());
    loaded_ = true;
    return true;
  }

  // See `pybind11/eigen.h`, `type_caster<>` implementations.
  // N.B. Do not use `PYBIND11_TYPE_CASTER(...)` so we can avoid casting
  // garbage values.
  operator Type&() {
    DRAKE_DEMAND(loaded_);
    return value_;
  }
  template <typename T>
  using cast_op_type = py::detail::movable_cast_op_type<T>;
  static constexpr auto name = WrappedTypeCaster::props::descriptor;

  // C++ to Python.
  template <typename TType>
  static py::handle cast(
      TType&& src, py::return_value_policy policy, py::handle parent) {
    if (policy == py::return_value_policy::reference ||
        policy == py::return_value_policy::reference_internal) {
      // N.B. We must declare a local `static constexpr` here to prevent
      // linking errors. This does not appear achievable with
      // `constexpr char[]`, so we use `py::detail::descr`.
      // See `pybind11/pybind11.h`, `cpp_function::initialize(...)` for an
      // example.
      static constexpr auto original_name = Wrapper::original_name;
      throw py::cast_error(
          std::string("Can only pass ") + original_name.text + " by value.");
    }
    return WrappedTypeCaster::cast(
        Wrapper::wrap(std::forward<TType>(src)), policy, parent);
  }

 private:
  bool loaded_{false};
  Type value_;
};

// Wrapper for Eigen::Translation<>, to be used as first parameter to
// `type_caster_wrapped`.
// Since there are not many special operations for a translation vector, we
// shall return it as a nominal vector.
template <typename T, int Dim>
struct wrapper_eigen_translation {
  using Type = Eigen::Translation<T, Dim>;
  using WrappedType = Eigen::Matrix<T, Dim, 1>;
  static constexpr auto original_name = py::detail::_("Eigen::Translation<>");
  static Type unwrap(const WrappedType& arg_wrapped) {
    return Type(arg_wrapped);
  }
  static WrappedType wrap(const Type& arg) { return arg.vector(); }
};

// N.B. Since `Isometry3<>` and `Eigen::Quaternion<>` have more
// complicated structures, they are registered as types in `eigen_geometry_py`.

}  // namespace internal
}  // namespace pydrake
}  // namespace drake

namespace pybind11 {
namespace detail {

template <typename T, int Dim>
struct type_caster<Eigen::Translation<T, Dim>>
    : public drake::pydrake::internal::type_caster_wrapped<
          drake::pydrake::internal::wrapper_eigen_translation<T, Dim>> {};

}  // namespace detail
}  // namespace pybind11
