#include <cstdlib>
#include <cstring>
#include <functional>

namespace drake {
namespace test {
/// Checks if @p value of @p T type is movable via memcpy. That is, it tests
/// memcpy on @p value keeps a given invariant between @p value and a copy of
/// it. It uses a binary function object @p invariant_pred to check for
/// invariance.
///
/// @note Eigen's reallocation mechanisms have an issue. It is moving bytes
/// using `memcpy` without calling a move constructor. As a result, if a
/// `Scalar` type of Eigen matrix/array is not IsMemcpyMovable, we can have
/// memory leaks when `conservativeResize` method is called. This should
/// *always* be used to test a `Scalar` used within an `Eigen::EigenBase<...>`
/// object. Please see https://github.com/RobotLocomotion/drake/issues/5974 for
/// more information.
template <typename T, typename InvariantPred = std::equal_to<T>>
bool IsMemcpyMovable(const T& value,
                     const InvariantPred& invariant_pred = InvariantPred()) {
  // 1. Create ptr_to_original via placement-new.
  void* const raw_original{malloc(sizeof(T))};
  T* const ptr_to_original{new (raw_original) T(value)};

  // 2. Create ptr_to_moved from ptr_to_original via memcpy.
  void* const raw_moved{malloc(sizeof(T))};
  T* const ptr_to_moved{static_cast<T*>(raw_moved)};
  memcpy(ptr_to_moved, ptr_to_original, sizeof(T));

  // 3. Destroy ptr_to_original and free raw_original.
  ptr_to_original->T::~T();
  free(raw_original);

  // 4. Check if the invariant between value and ptr_to_moved hold.
  const bool out{invariant_pred(value, *ptr_to_moved)};
  free(raw_moved);
  return out;
}
}  // namespace test
}  // namespace drake
