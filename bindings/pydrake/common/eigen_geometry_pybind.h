#pragma once

/// @file
/// Provides pybind11 `type_caster`s for Eigen geometric types.
/// N.B. This uses some of pybind's coding conventions.
///
/// See http://pybind11.readthedocs.io/en/stable/advanced/cast/custom.html for
/// more details on custom type casters.

#include <string>
#include <utility>

#include "pybind11/eigen.h"
#include <Eigen/Dense>

#include "drake/bindings/pydrake/common/wrap_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"

namespace drake {
namespace pydrake {
namespace internal {

// Wrapper for Eigen::Translation<>, to be used as first parameter to
// `type_caster_wrapped`.
// Since there are not many special operations for a translation vector, we
// shall return it as a nominal vector.
template <typename T, int Dim>
struct wrapper_eigen_translation {
  using Type = Eigen::Translation<T, Dim>;
  static constexpr auto original_name = py::detail::_("Eigen::Translation<>");
  using WrappedType = Eigen::Matrix<T, Dim, 1>;
  static constexpr auto wrapped_name =
      py::detail::type_caster<WrappedType>::props::descriptor;

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
