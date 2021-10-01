#include "drake/common/identifier.h"

#include <atomic>

namespace drake {
namespace internal {

namespace {

// Even though this is a mutable global variable, it does not generate any
// object code for initialization, so it is safe to use even without being
// wrapped within a never_destroyed<>.
static std::atomic<int64_t> g_prior_identifier;

}  // namespace

int64_t get_new_identifier()  {
  // Note that 0 is reserved for the uninitialized Identifier created by the
  // default constructor, so we have an invariant that get_new_identifier() > 0.
  return ++g_prior_identifier;
}

}  // namespace internal
}  // namespace drake
