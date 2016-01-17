#include "LinearConstraint.h"

namespace drake {
LinearConstraint::LinearConstraint(snopt::doublereal lb,
    snopt::doublereal ub,
    snopt::integer xdim,
    std::vector<std::pair<snopt::integer, snopt::doublereal>> A_row) : Constraint(lb, ub, xdim) {
  m_A = A_row;
}

void LinearConstraint::nonlinearEval(snopt::doublereal x[],
    bool needF,
    bool needG,
    snopt::doublereal *f,
    std::vector<snopt::doublereal> *g) const {
  // Do Nothing
}
} // namespace drake
