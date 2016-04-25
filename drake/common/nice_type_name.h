#pragma once

#include <string>
#include <typeinfo>
#include <utility>
#include <vector>

#include "drake/drakeCommon_export.h"

namespace drake {
namespace common {

/** Using the algorithm appropriate to the current compiler, demangles a type
name as returned by typeid(T).name(), with the result hopefully suitable for
meaningful display to a human. The result is compiler-dependent.
@see CanonicalizeTypeName()
@relates drake::common::NiceTypeName **/
/** @cond **/ DRAKECOMMON_EXPORT /** @endcond **/
std::string DemangleTypeName(const char *typeid_name);

/** Given a compiler-dependent demangled type name string as returned by
DemangleTypeName(), attempts to form a canonicalized representation that will be
the same for any compiler. Unnecessary spaces and superfluous keywords like
"class" and "struct" are removed. The `Get()` method of NiceTypeName\<T>
uses this function to produce a human-friendly type name that is the same on any
platform. The input argument is left empty.
@relates drake::common::NiceTypeName **/
/** @cond **/ DRAKECOMMON_EXPORT /** @endcond **/
std::string CanonicalizeTypeName(std::string &&demangled_name);

/** Same, but takes an lvalue reference so has to copy the input.
@relates drake::common::NiceTypeName **/
inline std::string CanonicalizeTypeName(const std::string &demangled_name) {
  return CanonicalizeTypeName(std::string(demangled_name));
}

/** @brief Obtain canonicalized, human-readable names for
arbitrarily-complicated C++ types.

@tparam T   The type to be represented as a string. This may be an
            arbitrarily-complicated, templatized type but should not be in an
            anonymous namespace nor a function-local class.

Usage: @code
std::cout << "Type MyVectorType was: "
          << NiceTypeName<MyVectorType>::Get() << std::endl;
@endcode

We demangle and attempt to canonicalize the compiler-generated type names as
reported by `typeid(T).name()` so that the same string is returned by all
supported compilers and platforms. The output of `Get()` should be used for
error messages and testing; `GetRaw()` returns the compiler-specific demangled
result prior to canonicalization, which is useful for debugging and executes
faster. You can get the mangled name for free using `typeid(T).name()`.

@warning Don't expect usable names for types that are defined in an anonymous
namespace or for function-local types. Names will still be produced but they
won't be unique, pretty, or compiler-independent. **/
template <class T>
class NiceTypeName {
 public:
  /** Attempt to return a nicely demangled and canonicalized type name that is
  the same on all platforms, using `drake::common::CanonicalizeTypeName()`. This
  is an expensive operation but is only done once per instantiation of
  `NiceTypeName<T>` for a given type `T`. **/
  static const std::string &Get() {
    static const std::string canonical = CanonicalizeTypeName(GetRaw());
    return canonical;
  }

  /** Returns a demangled type name that is human readable but is compiler- and
  platform-specific, using `drake::common::DemangleTypeName()`. This may be
  expensive on some platforms but is done only once per instantiation of
  `NiceTypeName<T>` for a given type `T`. Prefer Get() unless you have a good
  reason to use GetRaw(). **/
  static const std::string &GetRaw() {
    static const std::string raw = DemangleTypeName(typeid(T).name());
    return raw;
  }
};

}  // namespace common
}  // namespace drake
