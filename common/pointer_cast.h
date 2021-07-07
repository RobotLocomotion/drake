#pragma once

#include <memory>
#include <stdexcept>

#include <fmt/format.h>

#include "drake/common/nice_type_name.h"

namespace drake {

/// Casts the object owned by the std::unique_ptr `other` from type `U` to `T`;
/// no runtime type checking is performed.
///
/// This method is analogous to the built-in std::static_pointer_cast that
/// operates on a std::shared_ptr.
///
/// Note that this function only supports default deleters.
template <class T, class U>
std::unique_ptr<T> static_pointer_cast(std::unique_ptr<U>&& other) noexcept {
  return std::unique_ptr<T>(static_cast<T*>(other.release()));
}

/// Casts the object owned by the std::unique_ptr `other` from type `U` to `T`;
/// if the cast fails, returns nullptr.  Casting is performed using
/// `dynamic_cast` on the managed value (i.e., the result of `other.get()`).
/// On success, `other`'s managed value is transferred to the result and
/// `other` is empty; on failure, `other` will retain its original managed
/// value and the result is empty.  As with `dynamic_cast`, casting nullptr to
/// anything always succeeds, so a nullptr result could indicate either that
/// the argument was nullptr or that the cast failed.
///
/// This method is analogous to the built-in std::dynamic_pointer_cast that
/// operates on a std::shared_ptr.
///
/// Note that this function only supports default deleters.
template <class T, class U>
std::unique_ptr<T> dynamic_pointer_cast(std::unique_ptr<U>&& other) noexcept {
  T* result = dynamic_cast<T*>(other.get());
  if (!result) { return nullptr; }
  other.release();
  return std::unique_ptr<T>(result);
}

/// Casts the object owned by the std::unique_ptr `other` from type `U` to `T`;
/// if `other` is nullptr or the cast fails, throws a std::exception.
/// Casting is performed using `dynamic_cast` on the managed value (i.e., the
/// result of `other.get()`).  On success, `other`'s managed value is
/// transferred to the result and `other` is empty; on failure, `other` will
/// retain its original managed value.
///
/// @throws std::exception if the cast fails.
///
/// Note that this function only supports default deleters.
template <class T, class U>
std::unique_ptr<T> dynamic_pointer_cast_or_throw(std::unique_ptr<U>&& other) {
  if (!other) {
    throw std::logic_error(fmt::format(
        "Cannot cast a unique_ptr<{}> containing nullptr to unique_ptr<{}>.",
        NiceTypeName::Get<U>(),
        NiceTypeName::Get<T>()));
  }
  T* result = dynamic_cast<T*>(other.get());
  if (!result) {
    throw std::logic_error(fmt::format(
        "Cannot cast a unique_ptr<{}> containing an object of type {} to "
        "unique_ptr<{}>.",
        NiceTypeName::Get<U>(),
        NiceTypeName::Get(*other),
        NiceTypeName::Get<T>()));
  }
  other.release();
  return std::unique_ptr<T>(result);
}

}  // namespace drake
