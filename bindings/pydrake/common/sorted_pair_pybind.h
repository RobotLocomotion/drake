#pragma once

#include <utility>

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
struct type_caster<drake::SortedPair<T>> : public type_caster<std::pair<T, T>> {
  using Value = drake::SortedPair<T>;

  template <typename U>
  using Cast = Value;

  template <typename U>
  static handle from_cpp(
      U&& value, rv_policy policy, cleanup_list* cleanup) noexcept {
    using Pair = std::pair<T, T>;
    return type_caster<Pair>::from_cpp(
        Pair(value.first(), value.second()), policy, cleanup);
  }

  explicit operator Value() {
    return Value(
        this->caster1.operator cast_t<T>(), this->caster2.operator cast_t<T>());
  }
};
#endif  // PYDRAKE_USE_PYBIND11

}  // namespace detail
}  // namespace PYDRAKE_BINDER_NAMESPACE
