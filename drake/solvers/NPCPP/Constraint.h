#ifndef CONSTRAINT_H
#define CONSTRAINT_H

#include <utility>
#include <vector>

namespace snopt {
#include "snopt.hh"
}

namespace drake {
/**
 * Abstract constraint class.
 * Subclasses must implement nonlinearEval and must specify:
 *   m_A: a vector containing index, value pairs denoting the linear part of the constraint.
 *        No decision variable may be included in m_A that also has a nonlinear component,
 *        instead, both linear and nonlinear parts must be calculated in nonlinearEval
 *   m_jGvar: a vector containing the indices of decision variables whose gradients are
 *            calculated in nonlinearEval.
 */
class Constraint {
protected:
  snopt::doublereal m_lb;
  snopt::doublereal m_ub;
  snopt::integer m_xdim;
  std::vector<std::pair<snopt::integer, snopt::doublereal>> m_A;
  std::vector<snopt::integer> m_jGvar;

public:
  Constraint(snopt::doublereal lb, snopt::doublereal ub, snopt::integer xdim);
  virtual void nonlinearEval(snopt::doublereal x[],
      bool needF,
      bool needG,
      snopt::doublereal *f,
      std::vector<snopt::doublereal> *g) const = 0;
  void getBounds(snopt::doublereal* lb, snopt::doublereal* ub);
  void getA(snopt::integer* neA, std::vector<snopt::integer>* jAvar, std::vector<snopt::doublereal>* A) const;
  void getG(snopt::integer* neG, std::vector<snopt::integer>* jGvar) const;
};
} // namespace drake

#endif