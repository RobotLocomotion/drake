#pragma once

#include "pybind11/stl.h"

#include "drake/common/drake_variant.h"

namespace pybind11 {
namespace detail {

// Ensure that we expose a type_caster for `stx::variant`.
// @see pybind11/stl.h, `variant_caster`.

#ifdef STX_NO_STD_VARIANT
template <typename... Types>
struct type_caster<stx::variant<Types...>>
    : public variant_caster<stx::variant<Types...>> {};
#endif

}  // namespace detail
}  // namespace pybind11
