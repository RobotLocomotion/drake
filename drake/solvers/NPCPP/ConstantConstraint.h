#ifndef CONSTANT_CONSTRAINT_H
#define CONSTANT_CONSTRAINT_H

#include "BoundingBoxConstraint.h"

namespace snopt {
#include "snopt.hh"
}

namespace drake {
class ConstantConstraint : public BoundingBoxConstraint {
public:
  ConstantConstraint(snopt::doublereal desired_value,
    snopt::integer xdim,
    snopt::integer jAvar);
};
} // namespace drake

#endif