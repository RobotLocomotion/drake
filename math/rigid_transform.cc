#include "drake/math/rigid_transform.h"

#include "drake/common/default_scalars.h"

// Explicitly instantiate on non-symbolic scalar types.
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::math::RigidTransform)
