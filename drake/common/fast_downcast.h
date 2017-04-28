#pragma once

#include <type_traits>

namespace drake {

/// Attempts to donwcast a reference of type `Base` to the requested type
/// `Derived`. `Derived` must be a reference and can be more cv-qualified than
/// `from` (i.e. this method can be used to add constness.)
/// fast_downcast behaves as `dynamic_cast` in Debug builds (i.e. NDEBUG **is
/// not** defined) while it behaves like `static_cast` in Release builds (i.e.
/// NDEBUG **is** defined).
/// It is meant to be used in places where perfomance is paramount and we wish
/// to avoid the cost of the runtime check, but it is only safe if the program
/// can guarantee (through some other logic) that the `from` object is
/// definitely of type `Derived`.
/// If the cast is successful, it returns the orignal `from` object downcasted
/// to type `Derived`. If the cast fails, it throws an exception that matches a
/// handler of type std::bad_cast.
///
/// @param from The object of type `Base` to be downcasted to type `Derived`.
/// @returns The `from` object downcasted to type `Derived` when successful.
///
/// @throws A std::bad_cast if `from` is cannot be downcasted to `Derived`.
///
/// @tparam Base A complete class type where to cast from.
/// @tparam Derived A reference to a complete class type.
template<class Derived, class Base>
// NOLINTNEXTLINE(runtime/references): we allow here to cast from either const
// NOLINTNEXTLINE(runtime/references): or non-const input references.
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

/// Attempts to donwcast a pointer of type `Base` to the requested type
/// `Derived`. `Derived` must be a pointer and can be more cv-qualified than
/// `from` (i.e. this method can be used to add constness.)
/// fast_downcast behaves as `dynamic_cast` in Debug builds (i.e. NDEBUG **is
/// not** defined) while it behaves like `static_cast` in Release builds (i.e.
/// NDEBUG **is** defined).
/// It is meant to be used in places where perfomance is paramount and we wish
/// to avoid the cost of the runtime check, but it is only safe if the program
/// can guarantee (through some other logic) that the `from` object is
/// definitely of type `Derived`.
/// If the cast is successful, it returns the orignal `from` pointer downcasted
/// to type `Derived`. If the cast fails, it returns `nullptr`.
///
/// @param from Pointer to an object of type `Base` to be downcasted to type
///             `Derived` (which must be a pointer type.)
/// @returns The `from` poiter downcasted to type `Derived` when successful and
///          `nullptr` when unsuccessful.
///
/// @tparam Base A complete class type where to cast from.
/// @tparam Derived A pointer to a complete class type.
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

