#ifndef NONLINEAR_PROGRAM_H
#define NONLINEAR_PROGRAM_H

#include <memory>
#include <vector>

#include "Constraint.h"

namespace snopt {
#include "snopt.hh"
}

namespace drake {
class NonlinearProgram {
private:
  bool m_debug_print;
  void debugPrint(snopt::integer const& n,
      snopt::integer const& nf,
      snopt::integer const& lena,
      snopt::integer const& nea,
      std::vector<snopt::integer> const& iafun,
      std::vector<snopt::integer> const& javar,
      std::vector<snopt::doublereal> const& a,
      snopt::integer const& leng,
      snopt::integer const& neg,
      std::vector<snopt::integer> const& igfun,
      std::vector<snopt::integer> const& jgvar,
      std::vector<snopt::doublereal> const& xlow,
      std::vector<snopt::doublereal> const& xupp,
      std::vector<snopt::doublereal> const& flow,
      std::vector<snopt::doublereal> const& fupp) const;
  
protected:
  snopt::integer m_num_vars;
  std::vector<std::unique_ptr<Constraint>> m_constraints;
  std::unique_ptr<Constraint> m_cost;
  void generateA(snopt::integer* lenA,
      snopt::integer* neA,
      std::vector<snopt::integer>* iAfun,
      std::vector<snopt::integer>* jAvar,
      std::vector<snopt::doublereal>* A) const;
  void generateG(snopt::integer* lenG,
      snopt::integer* neG,
      std::vector<snopt::integer>* iGfun,
      std::vector<snopt::integer>* jGvar) const;
  void generateFBounds(std::vector<snopt::doublereal>* flow,
      std::vector<snopt::doublereal>* fupp) const;
  static int userFun(snopt::integer *status, snopt::integer *n, snopt::doublereal x[],
      snopt::integer *needF, snopt::integer *nF, snopt::doublereal F[],
      snopt::integer *needG, snopt::integer *lenG, snopt::doublereal G[],
      char *cu, snopt::integer *lencu,
      snopt::integer iu[], snopt::integer *leniu,
      snopt::doublereal ru[], snopt::integer *lenru);

public:
  NonlinearProgram(snopt::integer num_vars);
  void addConstraint(std::unique_ptr<Constraint>& cnstr);
  void setCost(std::unique_ptr<Constraint>& cost);
  void solve(std::vector<snopt::doublereal>* x,
      snopt::doublereal* objval,
      snopt::integer* info) const;
  void setDebugPrint(bool debug_print);
};
} // namespace drake

#endif
