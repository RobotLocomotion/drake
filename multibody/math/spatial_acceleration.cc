#include "drake/multibody/math/spatial_acceleration.h"

#include "drake/common/default_scalars.h"
// Necessary to instantiate dot(). See #14097 for more details.
#include "drake/multibody/math/spatial_algebra.h"

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::SpatialAcceleration)
