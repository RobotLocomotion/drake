#pragma once

#include <cstdlib>
#include <cstring>
#include <functional>
#include <memory>
#include <type_traits>

namespace drake {
namespace test {
/// Checks if @p value of @p T type is movable via memcpy. That is, it tests
/// memcpy on @p value keeps a given invariant between @p value and a copy of
/// it. It uses a binary function object @p invariant_pred to check for
/// invariance.
///
/// @note Eigen's reallocation mechanisms have an issue. It is moving bytes
/// using `memcpy` without calling a move constructor. As a result, if a
/// `Scalar` type of Eigen matrix/array is not IsMemcpyMovable, we have
/// undefined behavior when `conservativeResize` method is called. This should
/// *always* be used to test a `Scalar` used within an `Eigen::EigenBase<...>`
/// object. Please see https://github.com/RobotLocomotion/drake/issues/5974 for
/// more information.
template <typename T, typename InvariantPred = std::equal_to<T>>
[[nodiscard]] bool IsMemcpyMovable(
    const T& value, const InvariantPred& invariant_pred = InvariantPred()) {
  // 1. Create ptr_to_original via placement-new.
  auto original_storage = std::make_unique<
      typename std::aligned_storage<sizeof(T), alignof(T)>::type>();
  T* const ptr_to_original{new (original_storage.get()) T{value}};

  // 2. Create ptr_to_moved from ptr_to_original via memcpy.
  auto moved_storage = std::make_unique<
      typename std::aligned_storage<sizeof(T), alignof(T)>::type>();
  T* const ptr_to_moved{reinterpret_cast<T* const>(moved_storage.get())};
  memcpy(static_cast<void*>(ptr_to_moved), ptr_to_original, sizeof(T));

  // 3. Free original_storage.
  original_storage.reset();

  // 4. Check if the invariant between value and ptr_to_moved hold.
  const bool out{invariant_pred(value, *ptr_to_moved)};

  ptr_to_moved->T::~T();
  return out;
}
}  // namespace test
}  // namespace drake
