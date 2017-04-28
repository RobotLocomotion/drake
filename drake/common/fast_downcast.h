#pragma once

#include <type_traits>

/// @file
/// Provides Drake's assertion implementation.  This is intended to be used
/// both within Drake and by other software.  Drake's asserts can be armed
/// and disarmed independently from the system-wide asserts.


//#ifdef DRAKE_ASSERT_IS_ARMED
//#ifdef DRAKE_ASSERT_IS_DISARMED

namespace drake {
namespace detail {

}  // namespace detail

/// The doc here.
///
// Const type version.
template<class Derived, class Base>
static Derived fast_downcast(Base& from) {
  static_assert(std::is_convertible<Derived, Base&>::value,
                "The requested type is not a derived class of the input "
                "object type.");
#ifndef NDEBUG
  return dynamic_cast<Derived>(from);
#else
  return static_cast<Derived>(from);
#endif
}

// Mutable type version.
template<class Derived, class Base>
static Derived fast_downcast(Base* from) {
  static_assert(std::is_convertible<Derived, Base*>::value,
                "The requested type is not a derived class of the input "
                "object type.");
#ifndef NDEBUG
  return dynamic_cast<Derived>(from);
#else
  return static_cast<Derived>(from);
#endif
}

}  // namespace drake

