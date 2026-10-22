#pragma once

#include <tuple>

#include "drake/geometry/proximity/fcl_backend.h"

/* @file
 Enumerates the backend policies that Drake's proximity code is built against,
 and provides the explicit-instantiation macros that span them. See
 fcl_backend.h for what a backend policy is.

 This is the single place that knows the full set of backends; adding one means
 adding a line to each macro below and nothing else. The macros parallel the
 scalar-spanning macros in drake/common/default_scalars.h, which cannot be
 reused here because they append exactly one template argument.

 Note that none of the proximity targets set `compile_once_per_scalar`, so --
 unlike default_scalars.h -- these macros need no DRAKE_ONCE_PER_SCALAR_PHASE
 branch. If a target ever opts in, these will need one too.

 There are no DECLARE counterparts to these macros. Every entity instantiated
 this way defines its members in a .cc alongside the explicit instantiation,
 so a caller cannot implicitly instantiate one anyway, and a declaration would
 pull a concrete backend -- and its collision library -- into a header. */

// clang-format off

/* Defines instantiations for a class template whose sole parameter is the
 backend policy. For use in .cc files only. */
#define DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_BACKENDS( \
    SomeType) \
template SomeType<::drake::geometry::internal::FclBackend>;

/* Defines instantiations for free function templates whose sole parameter is
 the backend policy. The argument is a parenthesized, comma-separated list of
 function pointers written in terms of a template parameter named `Backend`,
 e.g.

   DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_BACKENDS(
       (&Callback<Backend>));

 This follows the same used-constexpr-tuple idiom as the scalar-spanning
 function macro in drake/common/default_scalars.h. For use in .cc files only,
 inside the namespace that declares the functions. */
#define DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_BACKENDS( \
    FunctionPointersTuple) \
template <typename Backend> \
constexpr auto Make_Backend_Function_Pointers() { \
  return std::make_tuple FunctionPointersTuple ; \
} \
static constexpr auto Backend_Function_Templates __attribute__((used)) = \
    std::tuple_cat( \
        Make_Backend_Function_Pointers< \
            ::drake::geometry::internal::FclBackend>());

// clang-format on
