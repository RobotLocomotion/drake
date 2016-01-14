#ifndef BOUNDING_BOX_CONSTRAINT_H
#define BOUNDING_BOX_CONSTRAINT_H

#include "LinearConstraint.h"

namespace snopt {
#include "snopt.hh"
}

namespace drake {
class BoundingBoxConstraint : public LinearConstraint {
public:
  BoundingBoxConstraint(snopt::doublereal lb,
      snopt::doublereal ub,
      snopt::integer xdim,
      snopt::integer jAvar);
};
} // namespace drake

#endif