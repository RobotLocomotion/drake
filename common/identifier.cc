#include "drake/common/identifier.h"

#include <mutex>
#include <typeindex>
#include <unordered_map>

#include "drake/common/never_destroyed.h"

namespace drake {
namespace internal {
int64_t new_id_value_from_type(const std::type_info& t) {
  std::type_index i{t};
  static never_destroyed<std::unordered_map<std::type_index, int64_t>>
      static_ids;
  static never_destroyed<std::mutex> map_mutex;
  std::lock_guard<std::mutex> lock(map_mutex.access());
  std::unordered_map<std::type_index, int64_t>& ids = static_ids.access();
  if (ids.count(i) == 0) {
    ids.emplace(i, 1);
  }
  return ids[i]++;
}

}  // namespace internal
}  // namespace drake
