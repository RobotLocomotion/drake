#pragma once

#include "pybind11/stl.h"

// TODO(eric.cousineau): Merge this with `pydrake_pybind` when it is available
// (#7829).
#include "drake/common/drake_optional.h"

#ifdef STX_NO_STD_OPTIONAL

namespace pybind11 {
namespace detail {

// Ensure that we expose a type_caster for `stx::optional`.
// @see pybind11/stl.h, `optional_caster`.

template <typename T>
struct type_caster<stx::optional<T>>
    : public optional_caster<stx::optional<T>> {};

template <>
struct type_caster<stx::nullopt_t>  // BR
    : public void_caster<stx::nullopt_t> {};

}  // namespace detail
}  // namespace pybind11

#endif  // STX_NO_STD_OPTIONAL
