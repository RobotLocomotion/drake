#pragma once

#include "drake/common/autodiff.h"
#include "drake/common/symbolic/expression.h"
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
 branch. If a target ever opts in, these will need one too. */

// clang-format off

/* Declares that instantiations exist for a class template whose sole parameter
 is the backend policy. For use in .h files only. */
#define DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_BACKENDS( \
    SomeType) \
extern template SomeType<::drake::geometry::internal::FclBackend>;

/* Defines instantiations for a class template whose sole parameter is the
 backend policy. For use in .cc files only. */
#define DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_BACKENDS( \
    SomeType) \
template SomeType<::drake::geometry::internal::FclBackend>;

/* Declares that instantiations exist for a class template parameterized on a
 nonsymbolic scalar and then the backend policy, in that order. For use in .h
 files only.

 Note the deliberate asymmetry with the DEFINE macro below, which spans all
 three default scalars: CharacterizeResultTest is declared over the
 nonsymbolic scalars but defined over all of them. That predates the backend
 parameter; this pair preserves it rather than changing it. */
#define \
  DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_NONSYMBOLIC_AND_BACKENDS( \
      SomeType) \
extern template SomeType<double, \
    ::drake::geometry::internal::FclBackend>; \
extern template SomeType<::drake::AutoDiffXd, \
    ::drake::geometry::internal::FclBackend>;

/* Defines instantiations for a class template parameterized on a scalar and
 then the backend policy, in that order. For use in .cc files only. */
#define DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_SCALARS_AND_BACKENDS( \
    SomeType) \
template SomeType<double, \
    ::drake::geometry::internal::FclBackend>; \
template SomeType<::drake::AutoDiffXd, \
    ::drake::geometry::internal::FclBackend>; \
template SomeType<::drake::symbolic::Expression, \
    ::drake::geometry::internal::FclBackend>;

// clang-format on
