#pragma once

#include "drake/common/autodiff.h"
#include "drake/common/symbolic.h"

/// A macro that defines explicit class template instantiations for Drake's
/// default set of supported scalar types.  This macro should only be used in
/// .cc files, never in .h files.
///
/// @param SomeType the template typename to instantiate, *including* the
/// leading `class` or `struct` keyword.
///
/// Currently the supported types are:
/// - double
/// - drake::AutoDiffXd
/// - drake::symbolic::Expression
///
/// Example `my_system.h`:
/// @code
/// namespace sample {
/// template <typename T>
/// class MySystem final : public LeafSystem<T> {
///   ...
/// @endcode
///
/// Example `my_system.cc`:
/// @code
/// #include "my_system.h"
///
/// #include "drake/common/default_scalars.h"
///
/// DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
///     class ::sample::MySystem)
/// @endcode
///
/// See also @ref system_scalar_conversion.
#define DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS( \
    SomeType) \
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS( \
    SomeType) \
template SomeType<::drake::symbolic::Expression>;

/// A macro to that defines explicit class template instantiations for Drake's
/// default set of supported scalar types, excluding all symbolic types.  This
/// macro should only be used in .cc files, never in .h files.  This is
/// identical to DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS
/// except that it does not define support for any drake::symbolic types.
#define \
  DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS( \
      SomeType) \
template SomeType<double>; \
template SomeType<::drake::AutoDiffXd>;
