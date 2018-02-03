#pragma once

/// @file
/// Provides pybind11 type casters for Eigen geometric types.
/// N.B. This uses Drake's coordinate conventions.
/// N.B. This uses some of pybind's coding conventions.

#include <utility>

#include <Eigen/Dense>
#include <pybind11/eigen.h>

namespace pybind11 {
namespace detail {

// Implements a `type_caster<>` specialization used to convert types using a
// specific wrapping policy.
// @tparam Wrapper
//  Struct which must provide `Type`, `WrappedType`, `unwrap`, and `wrap`.
// @tparam copy_only
//  This may only pass between C++ and Python as copies, not references.
// See `eigen_wrapper_*` structs below for more details.
template <typename Wrapper, bool copy_only = true>
struct drake_wrap_caster {
  using Type = typename Wrapper::Type;
  using WrappedType = typename Wrapper::WrappedType;
  using WrappedTypeCaster = type_caster<WrappedType>;

  // Python to C++.
  bool load(handle src, bool converter) {
    WrappedTypeCaster caster;
    if (!caster.load(src, converter)) {
      return false;
    }
    value_ = Wrapper::unwrap(caster.operator WrappedType&());
    return true;
  }

  // See `pybind11/eigen.h`, `type_caster<>` implementations.
  operator Type&() { return value_; }
  template <typename T> using cast_op_type = movable_cast_op_type<T>;
  static constexpr auto name = WrappedTypeCaster::props::descriptor;

  // C++ to Python.
  template <typename TType>
  static handle cast(TType&& src, return_value_policy policy,
      handle parent) {
    if (copy_only &&
        (policy == return_value_policy::reference ||
         policy == return_value_policy::reference_internal)) {
      // @note `_(...)` is pybind secret sauce for constexpr strings.
      throw cast_error(
          (_("Can only pass ") + Wrapper::original_name +
           _(" by value.")).text);
    }
    return WrappedTypeCaster::cast(
        Wrapper::wrap(std::forward<TType>(src)), policy, parent);
  }

 private:
  Type value_;
};

// Wrapper for Transform<>.
template <typename T, int Dim, int Mode, int Options>
struct eigen_wrapper_transform {
  using Type = Eigen::Transform<T, Dim, Mode, Options>;
  using WrappedType = Eigen::Matrix<T, Dim + 1, Dim + 1>;
  static constexpr auto original_name = _("Eigen::Transform<>");
  static Type unwrap(const WrappedType& arg_wrapped) {
    return arg_wrapped;
  }
  static WrappedType wrap(const Type& src) {
    return src.matrix();
  }
  Type value;
};

template <typename T, int Dim, int Mode, int Options>
struct type_caster<Eigen::Transform<T, Dim, Mode, Options>>
    : public drake_wrap_caster<
        eigen_wrapper_transform<T, Dim, Mode, Options>> {};

// Wrapper for Translation<>.
template <typename T, int Dim>
struct eigen_wrapper_translation {
  using Type = Eigen::Translation<T, Dim>;
  using WrappedType = Eigen::Matrix<T, Dim, 1>;
  static constexpr auto original_name = _("Eigen::Translation<>");
  static Type unwrap(const WrappedType& arg_wrapped) {
    return Type(arg_wrapped);
  }
  static WrappedType wrap(const Type& arg) {
    return arg.vector();
  }
};

template <typename T, int Dim>
struct type_caster<Eigen::Translation<T, Dim>>
    : public drake_wrap_caster<eigen_wrapper_translation<T, Dim>> {};

// Wrapper for Quaternion<>.
// Eigen stores the quaternion in `(xyz, w)`, but Drake wants `(w, xyz)`.
template <typename T, int Options>
struct eigen_wrapper_quaternion {
  using Type = Eigen::Quaternion<T, Options>;
  using WrappedType = Eigen::Matrix<T, 4, 1>;
  static constexpr auto original_name = _("Eigen::Quaternion<>");
  static Type unwrap(const WrappedType& wxyz) {
    Type out;
    out.coeffs() << wxyz.tail(3), wxyz(0);
    return out;
  }
  static WrappedType wrap(const Type& arg) {
    WrappedType wxyz;
    wxyz << arg.w(), arg.vec();
    return wxyz;
  }
  Type value;
};

template <typename T, int Options>
struct type_caster<Eigen::Quaternion<T, Options>>
    : public drake_wrap_caster<eigen_wrapper_quaternion<T, Options>> {};

}  // namespace detail
}  // namespace pybind11
