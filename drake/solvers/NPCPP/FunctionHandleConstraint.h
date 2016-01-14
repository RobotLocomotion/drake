#ifndef FUNCTION_HANDLE_CONSTRAINT_H
#define FUNCTION_HANDLE_CONSTRAINT_H

#include <functional>
#include <utility>
#include <vector>

#include "Constraint.h"

namespace snopt {
#include "snopt.hh"
}

namespace drake {
class FunctionHandleConstraint : public Constraint {
protected:
  std::function<void(snopt::doublereal[],bool,bool,snopt::doublereal*,std::vector<snopt::doublereal>*)> m_fun;

public:
  FunctionHandleConstraint(snopt::doublereal lb,
      snopt::doublereal ub,
      snopt::integer xdim,
      std::vector<std::pair<snopt::integer, snopt::doublereal>> A_row,
      std::vector<snopt::integer> jGvar_row,
      std::function<void(snopt::doublereal[],bool,bool,snopt::doublereal*,std::vector<snopt::doublereal>*)> nonlinearEval);
  void nonlinearEval(snopt::doublereal x[],
    bool needF,
    bool needG,
    snopt::doublereal *f,
    std::vector<snopt::doublereal> *g) const;
};
}

#endif