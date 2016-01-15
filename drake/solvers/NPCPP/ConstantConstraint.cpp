#include "ConstantConstraint.h"

namespace drake {
ConstantConstraint::ConstantConstraint(snopt::doublereal desired_value,
    snopt::integer xdim,
    snopt::integer jAvar) : BoundingBoxConstraint(
        desired_value, desired_value, xdim, jAvar) {}
} // namespace drake