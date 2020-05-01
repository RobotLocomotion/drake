#include "drake/common/nice_type_name_override.h"

#include "drake/common/never_destroyed.h"

namespace drake {
namespace internal {

namespace {
static never_destroyed<NiceTypeNamePtrOverride> ptr_override;
}  // namespace

void SetNiceTypeNamePtrOverride(NiceTypeNamePtrOverride new_ptr_override) {
  DRAKE_DEMAND(!!new_ptr_override);
  DRAKE_DEMAND(!ptr_override.access());
  ptr_override.access() = new_ptr_override;
}

NiceTypeNamePtrOverride GetNiceTypeNamePtrOverride() {
  return ptr_override.access();
}

}  // namespace internal
}  // namespace drake
