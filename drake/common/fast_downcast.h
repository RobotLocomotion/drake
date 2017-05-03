#pragma once

#include <type_traits>

// Users should NOT set these; only this header should set them.
#ifdef DRAKE_UNSAFE_DOWNCAST_IS_INDEED_SAFE
# error Unexpected DRAKE_UNSAFE_DOWNCAST_IS_INDEED_SAFE defined.
#endif
#ifdef DRAKE_UNSAFE_DOWNCAST_IS_INDEED_UNSAFE
# error Unexpected DRAKE_UNSAFE_DOWNCAST_IS_INDEED_UNSAFE defined.
#endif

// Decide whether Drake unsafe_downcast is safe (meaning it behaves like
// dynamic_cast).
// The default is that unsafe_downcast behaves as dynamic_cast in both Release
// and Debug builds.
#if defined(DRAKE_UNSAFE_DOWNCAST_IS_SAFE) && \
    defined(DRAKE_UNSAFE_DOWNCAST_IS_UNSAFE)
# error Conflicting unsafe downcast toggles.
#elif defined(DRAKE_UNSAFE_DOWNCAST_IS_UNSAFE)
# define DRAKE_UNSAFE_DOWNCAST_IS_INDEED_UNSAFE
#else
# define DRAKE_UNSAFE_DOWNCAST_IS_INDEED_SAFE
#endif

namespace drake {

/// Attempts to donwcast a reference of type `Base` to the requested type
/// `Derived`. `Derived` must be a reference and can be more cv-qualified than
/// `from` (i.e. this method can be used to add constness.)
/// By default `unsafe_downcast` behaves as `dynamic_cast` in both Debug and
/// Release builds (i.e. it IS safe), unless the pre-processor macro
/// `DRAKE_UNSAFE_DOWNCAST_IS_UNSAFE` is defined which makes it behave as a
/// `static_cast`.
/// It is meant to be used in places where perfomance is paramount and we wish
/// to be able to control the cost of dynamic casting (for instance, to be able
/// to profile the cost with and without dynamic casting), and the program can
/// guarantee (through some other logic) that the `from` object is definitely of
/// type `Derived`.
/// If the cast is successful, it returns the orignal `from` object downcasted
/// to type `Derived`. If the cast fails, and `DRAKE_UNSAFE_DOWNCAST_IS_UNSAFE`
/// is not defined, it throws an exception that matches a handler of type
/// std::bad_cast.
///
/// @param from The object of type `Base` to be downcasted to type `Derived`.
/// @returns The `from` object downcasted to type `Derived` when successful.
///
/// @throws A std::bad_cast if `from` is cannot be downcasted to `Derived` when
/// `DRAKE_UNSAFE_DOWNCAST_IS_UNSAFE` is not defined.
///
/// @tparam Base A complete class type where to cast from.
/// @tparam Derived A reference to a complete class type.
template<class Derived, class Base>
// NOLINTNEXTLINE(runtime/references): we allow here to cast from either const
// NOLINTNEXTLINE(runtime/references): or non-const input references.
static Derived unsafe_downcast(Base &from) {
  static_assert(std::is_convertible<Derived, Base&>::value,
                "The requested type is not a derived class of the input "
                "object type.");
#ifdef DRAKE_UNSAFE_DOWNCAST_IS_INDEED_SAFE
  return dynamic_cast<Derived>(from);
#else
  return static_cast<Derived>(from);
#endif
}

/// Attempts to donwcast a pointer of type `Base` to the requested type
/// `Derived`. `Derived` must be a pointer and can be more cv-qualified than
/// `from` (i.e. this method can be used to add constness.)
/// By default `unsafe_downcast` behaves as `dynamic_cast` in both Debug and
/// Release builds (i.e. it IS safe), unless the pre-processor macro
/// `DRAKE_UNSAFE_DOWNCAST_IS_UNSAFE` is defined which makes it behave as a
/// `static_cast`.
/// It is meant to be used in places where perfomance is paramount and we wish
/// to be able to control the cost of dynamic casting (for instance, to be able
/// to profile the cost with and without dynamic casting), and the program can
/// guarantee (through some other logic) that the `from` object is definitely of
/// type `Derived`.
/// If the cast is successful, it returns the orignal `from` pointer downcasted
/// to type `Derived`. If the cast fails, and `DRAKE_UNSAFE_DOWNCAST_IS_UNSAFE`
/// is not defined, it returns `nullptr`.
///
/// @param from Pointer to an object of type `Base` to be downcasted to type
///             `Derived` (which must be a pointer type.)
/// @returns The `from` poiter downcasted to type `Derived`. It returns
/// `nullptr` when unsuccessful and `DRAKE_UNSAFE_DOWNCAST_IS_UNSAFE` is not
/// defined.
///
/// @tparam Base A complete class type where to cast from.
/// @tparam Derived A pointer to a complete class type.
template<class Derived, class Base>
static Derived unsafe_downcast(Base *from) {
  static_assert(std::is_convertible<Derived, Base*>::value,
                "The requested type is not a derived class of the input "
                "object type.");
#ifdef DRAKE_UNSAFE_DOWNCAST_IS_INDEED_SAFE
  return dynamic_cast<Derived>(from);
#else
  return static_cast<Derived>(from);
#endif
}

}  // namespace drake

