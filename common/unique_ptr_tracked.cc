#include "drake/common/unique_ptr_tracked.h"

#include "drake/common/never_destroyed.h"

namespace drake {
namespace internal {

never_destroyed<DeleteCallback> on_delete;

}  // namespace internal
}  // namespace drake
