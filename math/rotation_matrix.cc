#include "drake/math/rotation_matrix.h"

#include "drake/common/default_scalars.h"

// Explicitly instantiate on non-symbolic scalar types.
// TODO(Mitiguy) Ensure this class handles RotationMatrix<symbolic::Expression>.
// To enable symbolic expressions, remove _NONSYMBOLIC in next line.
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::math::RotationMatrix)
