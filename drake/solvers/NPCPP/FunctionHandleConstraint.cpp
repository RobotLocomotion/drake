#include "FunctionHandleConstraint.h"

namespace snopt {
#include "snopt.hh"
}

namespace drake {
FunctionHandleConstraint::FunctionHandleConstraint(snopt::integer lb,
    snopt::integer ub,
    snopt::integer xdim,
    std::vector<std::pair<snopt::integer, snopt::doublereal>> A_row,
    std::vector<snopt::integer> jGvar_row,
    std::function<void(snopt::doublereal[],bool,bool,snopt::doublereal*,std::vector<snopt::doublereal>*)> nonlinearEval) : Constraint(lb, ub, xdim) {
  m_A = A_row;
  m_jGvar = jGvar_row;
  m_fun = nonlinearEval;
}

void FunctionHandleConstraint::nonlinearEval(snopt::doublereal x[],
    bool needF,
    bool needG,
    snopt::doublereal *f,
    std::vector<snopt::doublereal> *g) const {
  m_fun(x, needF, needG, f, g);
}

}