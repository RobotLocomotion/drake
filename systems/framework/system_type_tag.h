#pragma once

#include <type_traits>

namespace drake {
namespace systems {

template <typename T> class System;

/// A tag object that denotes a System subclass `S` in function signatures.
///
/// For example, `SystemTypeTag<MySystem>{}` will create a dummy object that
/// can be used to call functions that look like:
///
/// @code
/// template <template <typename> class S>
/// const char* get_foo(SystemTypeTag<S>) { return S<double>::get_foo(); }
///
/// int main() {
///    std::cout << get_foo(SystemTypeTag<MySystem>{});
/// }
/// @endcode
///
/// In this case, we could directly call get_foo<MySystem>() by specifying the
/// template argument, but that is not always possible.  In particular, tag
/// objects are acutely useful when calling templated constructors, because
/// there is no other mechanism for the caller to specify the template type.
template <template <typename> class S>
struct SystemTypeTag {
  SystemTypeTag() {
    static_assert(std::is_base_of<System<double>, S<double>>::value,
                  "The type argument to SystemTypeTag must be a System");
  }
};

}  // namespace systems
}  // namespace drake
