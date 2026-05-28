#pragma once

#include <tuple>

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/common/sorted_pair.h"

namespace PYDRAKE_BINDER_NAMESPACE {
namespace detail {

// Casts `SortedPair<T>` as `Tuple[T]` comprised of `(first, second)`.
#ifdef PYDRAKE_USE_PYBIND11
template <typename T>
struct type_caster<drake::SortedPair<T>> {
  using Type = drake::SortedPair<T>;
  using InnerCaster = make_caster<T>;

  // N.B. This macro assumes placement in `pybind11::detail`.
  PYBIND11_TYPE_CASTER(
      Type, const_name("Tuple[") + type_caster<T>::name + const_name("]"));

  bool load(handle src, bool convert) {
    if (!convert && !tuple::check_(src)) {
      return false;
    }
    tuple t = borrow<tuple>(src);
    if (t.size() != 2) return false;
    InnerCaster first, second;
    if (!first.load(t[0], convert) || !second.load(t[1], convert)) {
      return false;
    }
    value = Type(static_cast<T>(first), static_cast<T>(second));
    return true;
  }

  static handle cast(Type src, rv_policy policy, handle parent) {
    object out = make_tuple(InnerCaster::cast(src.first(), policy, parent),
        InnerCaster::cast(src.second(), policy, parent));
    return out.release();
  }
};
#else   // PYDRAKE_USE_NANOBIND
template <typename T>
struct type_caster<drake::SortedPair<T>> {
  using Value = drake::SortedPair<T>;
  using InnerCaster = make_caster<T>;

  template <typename U>
  using Cast = Value;

  static constexpr auto Name =
      const_name("Tuple[") + type_caster<T>::Name + const_name("]");

  bool from_python(handle src, uint8_t flags, cleanup_list* cleanup) noexcept {
    nanobind::tuple t(src);
    if (t.size() != 2) return false;
    if (!first.from_python(t[0], flags, cleanup) ||
        !second.from_python(t[1], flags, cleanup)) {
      return false;
    }
    return true;
  }

  template <typename U>
  static handle from_cpp(
      U&& value, rv_policy policy, cleanup_list* cleanup) noexcept {
    object out =
        make_tuple(steal(InnerCaster::from_cpp(value.first(), policy, cleanup)),
            steal(InnerCaster::from_cpp(value.second(), policy, cleanup)));
    return out.release();
  }

  template <typename U>
  bool can_cast() const noexcept {
    return first.template can_cast<U>() && second.template can_cast<U>();
  }

  explicit operator Value() {
    return Value(first.operator cast_t<T>(), second.operator cast_t<T>());
  }

  InnerCaster first;
  InnerCaster second;
};
#endif  // PYDRAKE_USE_PYBIND11

}  // namespace detail
}  // namespace PYDRAKE_BINDER_NAMESPACE
