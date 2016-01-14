#ifndef CONSTRAINT_H
#define CONSTRAINT_H

#include <utility>
#include <vector>

namespace snopt {
#include "snopt.hh"
}

namespace drake {
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