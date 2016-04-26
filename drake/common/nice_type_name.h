#pragma once

#include <string>
#include <typeinfo>
#include <utility>
#include <vector>

#include "drake/drakeCommon_export.h"

namespace drake {
namespace common {

/** @brief Obtains canonicalized, human-readable names for
arbitrarily-complicated C++ types.

Usage: @code
std::cout << "Type MyVectorType was: "
          << NiceTypeName::Get<MyVectorType>() << std::endl;
@endcode

We demangle and attempt to canonicalize the compiler-generated type names as
reported by `typeid(T).name()` so that the same string is returned by all
supported compilers and platforms. The output of NiceTypeName::Get<T>() is
suitable for error messages and testing.

@warning Don't expect usable names for types that are defined in an anonymous
namespace or for function-local types. Names will still be produced but they
won't be unique, pretty, or compiler-independent. **/
class NiceTypeName {
 public:
  /** Attempts to return a nicely demangled and canonicalized type name that is
  the same on all platforms, using Canonicalize(). This is an expensive 
  operation but is only done once per instantiation of NiceTypeName::Get<T>() 
  for a given type `T`. **/
  template <typename T>
  static const std::string& Get() {
    static const std::string canonical = Canonicalize(GetRaw<T>());
    return canonical;
  }

  /** Returns a demangled type name that is human readable but is compiler- and
  platform-specific, using Demangle(). This may be expensive on some platforms
  but is done only once per instantiation of NiceTypeName::GetRaw<T>() for a 
  given type `T`. Prefer Get<T>() unless you have a good reason to use 
  GetRaw<T>(). **/
  template <typename T>
  static const std::string& GetRaw() {
    static const std::string raw = Demangle(typeid(T).name());
    return raw;
  }

  /** Using the algorithm appropriate to the current compiler, demangles a type
  name as returned by `typeid(T).name()`, with the result hopefully suitable for
  meaningful display to a human. The result is compiler-dependent.
  @see Canonicalize() **/
  /** @cond **/ DRAKECOMMON_EXPORT /** @endcond **/
  static std::string Demangle(const char* typeid_name);
  
  /** Given a compiler-dependent demangled type name string as returned by
  Demangle(), attempts to form a canonicalized representation that will be
  the same for any compiler. Unnecessary spaces and superfluous keywords like
  "class" and "struct" are removed. The NiceTypeName::Get<T>() method
  uses this function to produce a human-friendly type name that is the same on
  any platform. The input argument is left empty. **/
  /** @cond **/ DRAKECOMMON_EXPORT /** @endcond **/  
  static std::string Canonicalize(std::string&& demangled_name);
  
  /** Same, but takes an lvalue reference so has to copy the input. **/
  static std::string Canonicalize(const std::string& demangled_name) {
    return Canonicalize(std::string(demangled_name));
  }
};

}  // namespace common
}  // namespace drake
