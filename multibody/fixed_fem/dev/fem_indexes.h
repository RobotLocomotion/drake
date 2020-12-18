#pragma once

#include "drake/common/type_safe_index.h"

namespace drake {
namespace multibody {
namespace fixed_fem {
/** Index used to identify element by index among FEM elements. */
using ElementIndex = TypeSafeIndex<class ElementTag>;

/** Index used to identify node by index among FEM nodes. */
using NodeIndex = TypeSafeIndex<class NodeTag>;
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
