#ifndef QUADRATIC_CONSTRAINT_H
#define QUADRATIC_CONSTRAINT_H

#include <utility>
#include <vector>

#include "Constraint.h"

namespace snopt {
#include "snopt.hh"
}

namespace drake {
/**
 * A quadratic constraint, where: lb <= .5 * Q * x'*eye(xdim)*x + b*x <= ub
 * where:
 *   Q: a vector of index, value pairs (sparse 1 x xdim vector)
 *   b: a vector of index, value pairs (sparse 1 x xdim vector)
 */
class QuadraticConstraint : public Constraint {
private:
  void findIfVarInQAndB(std::vector<bool>* varInQ, std::vector<bool>* varInB) const;
  void findVarQAndBVals(std::vector<snopt::doublereal>* varQval, std::vector<snopt::doublereal>* varBval) const;
protected:
  std::vector<std::pair<snopt::integer, snopt::doublereal>> m_Q;
  std::vector<std::pair<snopt::integer, snopt::doublereal>> m_b;

public:
  QuadraticConstraint(snopt::doublereal lb,
      snopt::doublereal ub,
      snopt::integer xdim,
      std::vector<std::pair<snopt::integer, snopt::doublereal>> Q_row,
      std::vector<std::pair<snopt::integer, snopt::doublereal>> b_row,
      std::vector<std::pair<snopt::integer, snopt::doublereal>> A_row);
  void nonlinearEval(snopt::doublereal x[],
      bool needF,
      bool needG,
      snopt::doublereal *f,
      std::vector<snopt::doublereal> *g) const;
};
} // namespace drake

#endif