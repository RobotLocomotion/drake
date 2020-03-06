#pragma once

#include "drake/common/autodiff.h"
#include "drake/common/symbolic.h"

// N.B. `CommonScalarPack` in `systems_pybind.h` should be kept in sync
// with this file.

/// @defgroup default_scalars Default Scalars
/// @ingroup technical_notes
/// @{
/// Similar to the Eigen library, many classes in Drake use a template argument
/// to specify the numeric scalar type to use for computation.  We typically
/// name that template argument <b>`<T>`</b>.  For an example, see the class
/// drake::math::RigidTransform.
///
/// Most scalar-templated classes in Drake only support a small, fixed set of
/// scalar types:
/// - `double` (always)
/// - drake::AutoDiffXd (almost always)
/// - drake::symbolic::Expression (sometimes)
///
/// When Drake documentation refers to "default scalars", it means all three
/// of the types above.
///
/// Alternatively, reference to "default nonsymbolic scalars" means all except
/// `drake::symbolic::Expression`.

/// A macro that defines explicit class template instantiations for Drake's
/// default set of supported scalar types.  This macro should only be used in
/// .cc files, never in .h files.
///
/// @param SomeType the template typename to instantiate, *including* the
/// leading `class` or `struct` keyword.
///
/// Currently the supported types are:
///
/// - double
/// - drake::AutoDiffXd
/// - drake::symbolic::Expression
///
/// Example `my_system.h`:
/// @code
/// #include "drake/common/default_scalars.h"
///
/// namespace sample {
/// template <typename T>
/// class MySystem final : public LeafSystem<T> {
///   ...
/// };
/// }  // namespace sample
///
/// DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
///     class ::sample::MySystem)
/// @endcode
///
/// Example `my_system.cc`:
/// @code
/// #include "my_system.h"
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

// N.B. `NonSymbolicScalarPack` in `systems_pybind.h` should be kept in sync
// with this.
/// A macro that defines explicit class template instantiations for Drake's
/// default set of supported scalar types, excluding all symbolic types.  This
/// macro should only be used in .cc files, never in .h files.  This is
/// identical to DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS
/// except that it does not define support for any drake::symbolic types.
#define \
  DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS( \
      SomeType) \
template SomeType<double>; \
template SomeType<::drake::AutoDiffXd>;

/// A macro that declares that an explicit class instantiation exists in the
/// same library for Drake's default set of supported scalar types (having
/// been defined by
/// DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS in a .cc
/// file) . This macro should only be used in .h files, never in .cc files.
#define DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(  \
    SomeType) \
DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS( \
    SomeType) \
extern template SomeType<::drake::symbolic::Expression>;

/// A macro that declares that an explicit class instantiation exists in the
/// same library for Drake's default set of supported scalar types, excluding
/// all symbolic types (having been defined by
/// DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS
/// in a .cc file) . This macro should only be used in .h files, never in .cc
/// files.
#define \
  DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS( \
      SomeType) \
extern template SomeType<double>; \
extern template SomeType<::drake::AutoDiffXd>;

/// @}
