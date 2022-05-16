#pragma once

#include "drake/common/identifier.h"
#include "drake/common/type_safe_index.h"

namespace drake {
namespace multibody {
namespace fem {

/** Type used to index FEM nodes. */
using FemNodeIndex = TypeSafeIndex<class FemNodeTag>;

/** Type used to identify FEM models. */
using FemModelId = Identifier<class FemModelTag>;

}  // namespace fem
}  // namespace multibody
}  // namespace drake
