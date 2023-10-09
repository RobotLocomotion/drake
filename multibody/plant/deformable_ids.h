#pragma once

#include "drake/common/identifier.h"
#include "drake/common/type_safe_index.h"

namespace drake {
namespace multibody {

/** Uniquely identifies a deformable body. It is valid before and after
 Finalize(). */
using DeformableBodyId = Identifier<class DeformableBodyTag>;

using ExternalForceId = Identifier<class ExternalForceTag>;

/** (Internal use only) Indexes deformable bodies, only used after Finalize().
 */
using DeformableBodyIndex = TypeSafeIndex<class DeformableBodyTag>;

}  // namespace multibody
}  // namespace drake
