#include <cstdlib>
#include <cstring>
#include <functional>

namespace drake {
namespace test {
/// Checks if @p value of @p T type is movable via memcpy. That is, it tests
/// memcpy on @p value keeps a given invariant between @p value and a copy of
/// it.
///
/// @note @p invariant_pred is a binary function object.
template <typename T, typename Invariant_Pred = std::equal_to<T>>
bool IsMemcpyMovable(const T& value,
                     const Invariant_Pred& invariant_pred = Invariant_Pred()) {
  // 1. Create ptr_to_new_value1 via placement-new.
  void* const raw1{malloc(sizeof(T))};
  T* const ptr_to_new_value1{new (raw1) T(value)};

  // 2. Create ptr_to_new_value2 from ptr_to_new_value1 via memcpy.
  void* const raw2{malloc(sizeof(T))};
  T* const ptr_to_new_value2{static_cast<T*>(raw2)};
  memcpy(ptr_to_new_value2, ptr_to_new_value1, sizeof(T));

  // 3. Destroy ptr_to_new_value1 and free raw1.
  ptr_to_new_value1->T::~T();
  free(raw1);

  // 4. Check if the invariant between value and ptr_to_new_value2 hold.
  const bool out{invariant_pred(value, *ptr_to_new_value2)};
  free(raw2);
  return out;
}
}  // namespace test
}  // namespace drake
