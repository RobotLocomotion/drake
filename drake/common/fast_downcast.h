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
template<class ToType, class FromType>
static const ToType& fast_downcast(const FromType& from) {
  static_assert(std::is_convertible<const ToType&, const FromType&>::value,
                "FromType is not convertible to ToType.");
#ifndef NDEBUG
  return dynamic_cast<const ToType&>(from);
#else
  return static_cast<const ToType&>(from);
#endif
}

// Mutable type version.
template<class ToType, class FromType>
static ToType* fast_downcast(FromType* from) {
  static_assert(std::is_convertible<ToType*, FromType*>::value,
                "FromType is not convertible to ToType.");
#ifndef NDEBUG
  return dynamic_cast<ToType*>(from);
#else
  return static_cast<ToType*>(from);
#endif
}

}  // namespace drake

