#pragma once

#include <cstddef>
#include <map>
#include <memory>
#include <set>
#include <tuple>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/value.h"

namespace drake {
namespace systems {

// TODO(sherm1) Will contain CacheEntryValue and Cache.

class Cache {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Cache)

  Cache() = default;
  ~Cache() = default;
};

}  // namespace systems
}  // namespace drake
