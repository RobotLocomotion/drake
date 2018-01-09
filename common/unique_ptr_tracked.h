#pragma once

#include <functional>
#include <memory>
#include <typeindex>
#include <typeinfo>

#include "drake/common/drake_assert.h"

namespace drake {
namespace internal {

class PtrErased {
 public:
  struct type_hasher {
    using type = const std::type_info*;
    template <typename T>
    static type hash() { return &typeid(T); }
  };

  PtrErased(const PtrErased&) = default;

  template <typename T>
  PtrErased(T* ptr)
    : ptr_(ptr), type_(type_hasher::hash<T>()) {}

  operator bool() const { return ptr_ != nullptr; }
  void* ptr() const { return ptr_; }
  type_hasher::type type_hash() const { return type_; }
 private:
  void* ptr_{};
  type_hasher::type type_{};
};

using DeleteCallback = std::function<bool (PtrErased)>;
extern DeleteCallback on_delete;

/// Yar.
template <typename T>
class TrackedDeleter {
 public:
  TrackedDeleter() = default;
  // Allow interfacting with trivial deleters.
  template <typename Deleter>
  TrackedDeleter(Deleter&&) {}
  template <typename Deleter>
  TrackedDeleter& operator=(Deleter&&) {
    return *this;
  }
  template <typename Deleter>
  operator Deleter() {
    return Deleter{};
  }
  // Only called when `ptr` is non-null.
  void operator()(T* ptr) {
    bool is_deleted = on_delete && on_delete(ptr);
    if (!is_deleted)
      delete ptr;
  }
};

}  // namespace internal

/// Used for C++ and Python.
template <typename T>
using unique_ptr_tracked = unique_ptr<T, internal::TrackedDeleter<T>>;

}  // namespace drake
