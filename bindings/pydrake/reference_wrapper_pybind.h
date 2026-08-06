#pragma once

#include <functional>

#ifdef PYDRAKE_USE_NANOBIND
namespace nanobind {
namespace detail {

/* This allow us to use std::ref and std::cref to force a PYDRAKE_OVERRIDE
argument to be passed by (short-lived) reference instead of copying. We should
upstream this eventually. See wjakob/nanobind#384. */
template <typename T>
struct type_caster<std::reference_wrapper<T>> {
  using Caster = make_caster<T>;
  NB_TYPE_CASTER(std::reference_wrapper<T>, Caster::Name)

  static handle from_cpp(std::reference_wrapper<T> value,
      rv_policy /* policy */, cleanup_list* cleanup) noexcept {
    return Caster::from_cpp(&value.get(), rv_policy::reference, cleanup);
  }
};

}  // namespace detail
}  // namespace nanobind
#endif
