#pragma once

#include <new>
#include <type_traits>
#include <utility>

#include "drake/common/drake_copyable.h"

namespace drake {

/// Wraps an underlying type T such that its storage is a direct member field
/// of this object (i.e., without any indirection into the heap), but *unlike*
/// most member fields T's destructor is never invoked.
///
/// This is especially useful for function-local static variables that are not
/// trivially destructable.  We shouldn't call their destructor at program exit
/// because of the "indeterminate order of ... destruction" as mentioned in
/// cppguide's
/// <a href="https://drake.mit.edu/styleguide/cppguide.html#Static_and_Global_Variables">Static
/// and Global Variables</a> section, but other solutions to this problem place
///  the objects on the heap through an indirection.
///
/// Compared with other approaches, this mechanism more clearly describes the
/// intent to readers, avoids "possible leak" warnings from memory-checking
/// tools, and is probably slightly faster.
///
/// Example uses:
///
/// The singleton pattern:
/// @code
/// class Singleton {
///  public:
///   DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Singleton)
///   static Singleton& getInstance() {
///     static never_destroyed<Singleton> instance;
///     return instance.access();
///   }
///  private:
///   friend never_destroyed<Singleton>;
///   Singleton() = default;
/// };
/// @endcode
///
/// A lookup table, created on demand the first time its needed, and then
/// reused thereafter:
/// @code
/// enum class Foo { kBar, kBaz };
/// Foo ParseFoo(const std::string& foo_string) {
///   using Dict = std::unordered_map<std::string, Foo>;
///   static const drake::never_destroyed<Dict> string_to_enum{
///     std::initializer_list<Dict::value_type>{
///       {"bar", Foo::kBar},
///       {"baz", Foo::kBaz},
///     }
///   };
///   return string_to_enum.access().at(foo_string);
/// }
/// @endcode
///
/// In cases where computing the static data is more complicated than an
/// initializer_list, you can use a temporary lambda to populate the value:
/// @code
/// const std::vector<double>& GetConstantMagicNumbers() {
///   static const drake::never_destroyed<std::vector<double>> result{[]() {
///     std::vector<double> prototype;
///     std::mt19937 random_generator;
///     for (int i = 0; i < 10; ++i) {
///       double new_value = random_generator();
///       prototype.push_back(new_value);
///     }
///     return prototype;
///   }()};
///   return result.access();
/// }
/// @endcode
///
/// Note in particular the `()` after the lambda. That causes it to be invoked.
//
// The above examples are repeated in the unit test; keep them in sync.
template <typename T>
class never_destroyed {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(never_destroyed)

  /// Passes the constructor arguments along to T using perfect forwarding.
  template <typename... Args>
  explicit never_destroyed(Args&&... args) {
    // Uses "placement new" to construct a `T` in `storage_`.
    new (&storage_) T(std::forward<Args>(args)...);
  }

  /// Does nothing.  Guaranteed!
  ~never_destroyed() = default;

  /// Returns the underlying T reference.
  T& access() { return *reinterpret_cast<T*>(&storage_); }
  const T& access() const { return *reinterpret_cast<const T*>(&storage_); }

 private:
  typename std::aligned_storage<sizeof(T), alignof(T)>::type storage_;
};

}  // namespace drake
