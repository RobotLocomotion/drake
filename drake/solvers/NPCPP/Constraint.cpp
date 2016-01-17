#include "Constraint.h"

namespace drake {
Constraint::Constraint(snopt::doublereal lb,
    snopt::doublereal ub, snopt::integer xdim) : m_lb(lb), m_ub(ub), m_xdim(xdim) {}

void Constraint::getBounds(snopt::doublereal* lb, snopt::doublereal* ub) {
  *lb = m_lb;
  *ub = m_ub;
}

void Constraint::getA(snopt::integer* neA,
    std::vector<snopt::integer>* jAvar,
    std::vector<snopt::doublereal>* A) const {
  *neA = m_A.size();
  jAvar->clear();
  A->clear();
  for (int i = 0; i < *neA; ++i)  {
    std::pair<snopt::integer, snopt::doublereal> pair = m_A[i];
    jAvar->push_back(pair.first);
    A->push_back(pair.second);
  }
}

void Constraint::getG(snopt::integer* neG,
    std::vector<snopt::integer>* jGvar) const {
  *neG = m_jGvar.size();
  jGvar->clear();
  for (int i = 0; i < *neG; ++i)  {
    jGvar->push_back(m_jGvar[i]);
  }
}
} // namespace drake