#ifndef LINEAR_CONSTRAINT_H
#define LINEAR_CONSTRAINT_H

#include <utility>
#include <vector>

#include "Constraint.h"

namespace snopt {
#include "snopt.hh"
}

namespace drake {
class LinearConstraint : public Constraint {
public:
  LinearConstraint(snopt::doublereal lb,
    snopt::doublereal ub,
    snopt::integer xdim,
    std::vector<std::pair<snopt::integer, snopt::doublereal>> A_row);
  void nonlinearEval(snopt::doublereal x[],
      bool needF,
      bool needG,
      snopt::doublereal *f,
      std::vector<snopt::doublereal> *g) const;
};
} // namespace drake

#endif
