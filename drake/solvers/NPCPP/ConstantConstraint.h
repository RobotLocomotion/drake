#ifndef CONSTANT_CONSTRAINT_H
#define CONSTANT_CONSTRAINT_H

#include "BoundingBoxConstraint.h"

namespace snopt {
#include "snopt.hh"
}

namespace drake {
/**
 * Enforces an equality constraint: x[jAvar] = desired_value
 * where jAvar in [1,  xdim]
 */
class ConstantConstraint : public BoundingBoxConstraint {
public:
  ConstantConstraint(snopt::doublereal desired_value,
    snopt::integer xdim,
    snopt::integer jAvar);
};
} // namespace drake

#endif