#pragma once

#include <new>
#include <type_traits>
#include <utility>

namespace drake {

/// Wraps an underlying type T such that its storage is a direct member field
/// of this object (i.e., without any indirection into the heap), but *unlike*
/// most member fields T's destructor is never invoked.
///
/// This is especially useful for function-local static variables that are not
/// trivially destructable.  We shouldn't call their destructor at program exit
/// because of the "indeterminate order of ... destruction" as mentioned in
/// cppguide's #Static_and_Global_Variables section, but other solutions to
/// this problem place the objects on the heap through an indirection.
///
/// Compared with other approaches, this mechanism more clearly describes the
/// intent to readers, avoids "possible leak" warnings from memory-checking
/// tools, and is probably slightly faster.
template <typename T>
class never_destroyed {
 public:
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

  // Neither copyable nor moveable.
  never_destroyed(const never_destroyed& e) = delete;
  never_destroyed& operator=(const never_destroyed& e) = delete;
  never_destroyed(never_destroyed&& e) = delete;
  never_destroyed& operator=(never_destroyed&& e) = delete;

 private:
  typename std::aligned_storage<sizeof(T), alignof(T)>::type storage_;
};

}  // namespace drake
