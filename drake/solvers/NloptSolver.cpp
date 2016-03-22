
#include <stdexcept>
#include "NloptSolver.h"
#include "Optimization.h"

#include <nlopt.hpp>
#include <iostream>

using namespace std;
using namespace Eigen;
using namespace nlopt;

bool Drake::NloptSolver::available() const {
  return false;
}

// TODO TEMP test code
typedef struct {
    double a, b;
} my_constraint_data;

// TODO TEMP test code
double myfunc(unsigned n, const double *x, double *grad, void *my_func_data)
{
    if (grad) {
        grad[0] = 0.0;
        grad[1] = 0.5 / sqrt(x[1]);
    }
    return sqrt(x[1]);
}

// TODO TEMP test code
double myconstraint(unsigned n, const double *x, double *grad, void *data)
{
    my_constraint_data *d = (my_constraint_data *) data;
    double a = d->a, b = d->b;
    if (grad) {
        grad[0] = 3 * a * (a*x[0] + b) * (a*x[0] + b);
        grad[1] = -1.0;
    }
    return ((a*x[0] + b) * (a*x[0] + b) * (a*x[0] + b) - x[1]);
 }

bool Drake::NloptSolver::solve(OptimizationProblem &prog) const {
    
  int nx = prog.getNumVars();

  // Load the algo to use and the size.  
  nlopt::opt opt(nlopt::LD_SLSQP, nx);
  
  std::vector<double> xlow(nx, -numeric_limits<double>::infinity());
  std::vector<double> xupp(nx, numeric_limits<double>::infinity());
  
  for (auto const& binding : prog.getBoundingBoxConstraints()) {
    auto const& c = binding.getConstraint();
    for (const DecisionVariableView& v : binding.getVariableList()) {
      auto const lb = c->getLowerBound(), ub = c->getUpperBound();
      for (int k = 0; k < v.size(); k++) {
        xlow[v.index() + k] = std::max(lb(k), xlow[v.index() + k]);
        xupp[v.index() + k] = std::min(ub(k), xupp[v.index() + k]);
      }
    }
  }
  opt.set_lower_bounds(xlow);
  opt.set_upper_bounds(xupp);

  // TODO OLD

  opt.set_min_objective(myfunc, NULL);
  my_constraint_data data[2] = { {2,0}, {-1,1} };
  opt.add_inequality_constraint(myconstraint, &data[0], 1e-8);
  opt.add_inequality_constraint(myconstraint, &data[1], 1e-8);

  opt.set_xtol_rel(1e-4);

  std::vector<double> x(2);
  x[0] = 1.234; x[1] = 5.678;
  double minf;
  nlopt::result result = opt.optimize(x, minf);
  cout << "NLOPT result: " << result << endl;

  VectorXd sol(nx);
  for (int i = 0; i < nx; i++) {
    sol(i) = -0.30244;
  }

//  int j = 0;
//  sol(4) = -0.302449;
//  sol(5) = -0.164777;

  prog.setDecisionVariableValues(sol);
  return true;
}
