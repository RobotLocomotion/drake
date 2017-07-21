#pragma once

#include "drake/systems/framework/system.h"

namespace drake {
namespace systems {

/// A tag object that denotes a System subclass named S in function signatures.
///
/// For example, `SystemTypeTag<MySystem>{}` will create a dummy object which
/// is used to call functions that look like:
/// @code
/// template <template <typename> class S>
/// const char* get_name(SystemTypeTag<S>) { return S<double>::get_name(); }
/// @endcode
///
/// Tag objects are acutely useful when calling templated constructors, because
/// there is no other mechanism for the caller to specify template arguments.
template <template <typename> class S>
struct SystemTypeTag {
  SystemTypeTag() {
    static_assert(std::is_base_of<System<double>, S<double>>::value,
                  "The type argument to SystemTypeTag must be a System");
  }
};

}  // namespace systems
}  // namespace drake
