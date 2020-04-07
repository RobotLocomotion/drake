#pragma once

#include "drake/common/autodiff.h"
#include "drake/common/symbolic.h"

// N.B. `CommonScalarPack` and `NonSymbolicScalarPack` in `systems_pybind.h`
// should be kept in sync with this file.

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
///
/// For code within Drake, we offer Doxygen custom commands to document the
/// `<T>` template parameter in the conventional cases:
///
/// - @c \@tparam_default_scalar for the default scalars.
/// - @c \@tparam_nonsymbolic_scalar for the default nonsymbolic scalars.
/// - @c \@tparam_double_only for only the `double` scalar and nothing else.
///
/// All three commands assume that the template parameter is named `T`.  When
/// possible, prefer to use these commands instead of writing out a @c \@tparam
/// line manually.

/// @name Template instantiation macros
/// These macros either declare or define class template instantiations for
/// Drake's supported scalar types (see @ref default_scalars), either "default
/// scalars" or "default nonsymbolic scalars".  Use the `DECLARE` macros only
/// in .h files; use the `DEFINE` macros only in .cc files.
///
/// @param SomeType the template typename to instantiate, *including* the
/// leading `class` or `struct` keyword.
///
/// Example `my_system.h`:
/// @code
/// #include "drake/common/default_scalars.h"
///
/// namespace sample {
/// template <typename T>
/// class MySystem final : public drake::systems::LeafSystem<T> {
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
/// @{

/// Defines template instantiations for Drake's default scalars.
/// This should only be used in .cc files, never in .h files.
#define DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS( \
    SomeType) \
template SomeType<double>; \
template SomeType<::drake::AutoDiffXd>; \
template SomeType<::drake::symbolic::Expression>;

/// Defines template instantiations for Drake's default nonsymbolic scalars.
/// This should only be used in .cc files, never in .h files.
#define \
  DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS( \
      SomeType) \
template SomeType<double>; \
template SomeType<::drake::AutoDiffXd>;

/// Declares that template instantiations exist for Drake's default scalars.
/// This should only be used in .h files, never in .cc files.
#define DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(  \
    SomeType) \
extern template SomeType<double>; \
extern template SomeType<::drake::AutoDiffXd>; \
extern template SomeType<::drake::symbolic::Expression>;

/// Declares that template instantiations exist for Drake's default nonsymbolic
/// scalars.  This should only be used in .h files, never in .cc files.
#define \
  DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS( \
      SomeType) \
extern template SomeType<double>; \
extern template SomeType<::drake::AutoDiffXd>;

/// @}
