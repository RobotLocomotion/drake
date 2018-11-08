#pragma once

#include <memory>
#include <string>
#include <typeinfo>
#include <utility>
#include <vector>

#include "drake/common/never_destroyed.h"

namespace drake {

/** @brief Obtains canonicalized, platform-independent, human-readable names for
arbitrarily-complicated C++ types.

Usage: @code
// For types:
using std::pair; using std::string;
using MyVectorType = pair<int,string>;
std::cout << "Type MyVectorType was: "
          << drake::NiceTypeName::Get<MyVectorType>() << std::endl;
// Output: std::pair<int,std::string>

// For expressions:
std::unique_ptr<AbstractThing> thing;  // Assume actual type is ConcreteThing.
std::cout << "Actual type of 'thing' was: "
          << drake::NiceTypeName::Get(*thing) << std::endl;
// Output: ConcreteThing
@endcode

We demangle and attempt to canonicalize the compiler-generated type names as
reported by `typeid(T).name()` so that the same string is returned by all
supported compilers and platforms. The output of NiceTypeName::Get<T>() is
useful in error and log messages and testing. It also provides a
persistent, platform-independent identifier for types; `std::type_info` cannot
provide that.

@warning Don't expect usable names for types that are defined in an anonymous
namespace or for function-local types. Names will still be produced but they
won't be unique, pretty, or compiler-independent.

This class exists only to group type name-related static methods; don't try
to construct an object of this type. */
class NiceTypeName {
 public:
  /** Attempts to return a nicely demangled and canonicalized type name that is
  the same on all platforms, using Canonicalize(). This is an expensive
  operation but is only done once per instantiation of NiceTypeName::Get<T>()
  for a given type `T`. The returned reference will not be deleted even at
  program termination, so feel free to use it in error messages even in
  destructors that may be invoked during program tear-down. */
  template <typename T>
  static const std::string& Get() {
    static const never_destroyed<std::string> canonical(
        NiceTypeName::Get(typeid(T)));
    return canonical.access();
  }

  /** Returns the type name of the most-derived type of an object of type T,
  typically but not necessarily polymorphic. This must be calculated on the fly
  so is expensive whenever called, though very reasonable for use in error
  messages. For non-polymorphic types this produces the same result as would
  `Get<decltype(thing)>()` but for polymorphic types the results will
  differ. */
  template <typename T>
  static std::string Get(const T& thing) {
    return Get(typeid(thing));
  }

  /** Returns the nicely demangled and canonicalized type name of `info`. This
  must be calculated on the fly so is expensive whenever called, though very
  reasonable for use in error messages. */
  static std::string Get(const std::type_info& info) {
    return Canonicalize(Demangle(info.name()));
  }

  /** Using the algorithm appropriate to the current compiler, demangles a type
  name as returned by `typeid(T).name()`, with the result hopefully suitable for
  meaningful display to a human. The result is compiler-dependent.
  @see Canonicalize() */
  static std::string Demangle(const char* typeid_name);

  /** Given a compiler-dependent demangled type name string as returned by
  Demangle(), attempts to form a canonicalized representation that will be
  the same for any compiler. Unnecessary spaces and superfluous keywords like
  "class" and "struct" are removed. The NiceTypeName::Get<T>() method
  uses this function to produce a human-friendly type name that is the same on
  any platform. */
  static std::string Canonicalize(const std::string& demangled_name);

  /** Given a canonical type name that may include leading namespaces, attempts
  to remove those namespaces. For example,
  `drake::systems::MyThing<internal::type>` becomes `MyThing<internal::type>`.
  If the last segment ends in `::`, the original string is returned unprocessed.
  Note that this is just string processing -- a segment that looks like a
  namespace textually will be treated as one, even if it is really a class. So
  `drake::MyClass::Impl` will be reduced to `Impl` while
  `drake::MyClass<T>::Impl` is reduced to `MyClass<T>::Impl`. */
  static std::string RemoveNamespaces(const std::string& canonical_name);

 private:
  // No instances of this class should be created.
  NiceTypeName() = delete;
};

}  // namespace drake
