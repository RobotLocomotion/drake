#include "BoundingBoxConstraint.h"

#include <utility>
#include <vector>

namespace drake {
BoundingBoxConstraint::BoundingBoxConstraint(snopt::doublereal lb,
    snopt::doublereal ub,
    snopt::integer xdim,
    snopt::integer jAvar) : LinearConstraint(lb, ub, xdim,
      std::vector<std::pair<snopt::integer, snopt::doublereal>>(1, std::make_pair(jAvar, 1.0))) {}
} // namespace drake