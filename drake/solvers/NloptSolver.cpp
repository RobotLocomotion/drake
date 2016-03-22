
#include <iostream>
#include <stdexcept>
#include <list>
#include <vector>

#include <nlopt.hpp>

#include "NloptSolver.h"
#include "Optimization.h"

using namespace std;
using namespace Eigen;
using namespace nlopt;

namespace Drake {
namespace {
double evaluateCosts(const std::vector<double>& x,
                     std::vector<double>& grad,
                     void* f_data) {
  const OptimizationProblem* prog =
      reinterpret_cast<const OptimizationProblem*>(f_data);

  double cost = 0;
  Eigen::VectorXd xvec(x.size());
  for (size_t i = 0; i < x.size(); i++) {
    xvec[i] = x[i];
  }
  auto tx = initializeAutoDiff(xvec);
  TaylorVecXd ty(1);
  TaylorVecXd this_x(x.size());

  for (auto const& binding : prog->getGenericObjectives()) {
    size_t index = 0;
    for (const DecisionVariableView& v : binding.getVariableList()) {
      this_x.segment(index, v.size()) = tx.segment(v.index(), v.size());
      index += v.size();
    }

    binding.getConstraint()->eval(tx, ty);

    // TODO sammy is it actually valid to sum these?
    cost += ty(0).value();
    if (!grad.empty()) {
      for (const DecisionVariableView& v : binding.getVariableList()) {
        for (size_t j = v.index(); j < v.index() + v.size(); j++) {
          grad[j] = ty(0).derivatives()(j);
        }
      }
    }
  }

  return cost;
}

struct WrappedConstraint {
  const Constraint* constraint;
  const VariableList* variable_list;
};

void evaluateConstraint(unsigned m, double* result, unsigned n,
                        const double* x, double* grad, void* f_data) {
  const WrappedConstraint* wrapped =
      reinterpret_cast<WrappedConstraint*>(f_data);

  Eigen::VectorXd xvec(n);
  for (size_t i = 0; i < n; i++) {
    xvec[i] = x[i];
  }

  size_t vars = 0;
  for (const DecisionVariableView& v : *(wrapped->variable_list)) {
    vars += v.size();
  }

  auto tx = initializeAutoDiff(xvec);
  TaylorVecXd ty(m);
  TaylorVecXd this_x(vars);

  const Constraint* c = wrapped->constraint;
  const size_t num_constraints = c->getNumConstraints();
  std::cerr << "processing constraint " << c << " size " << n
            << " num constraints " << m
            << " decision vars " << vars
            << std::endl;
  assert(num_constraints == m);
  size_t index = 0;
  for (const DecisionVariableView& v : *(wrapped->variable_list)) {
    std::cerr << "Adding vars, out index: " << index
              << " in index: " << v.index()
              << " size " << v.size()
              << std::endl;
    this_x.segment(index, v.size()) = tx.segment(v.index(), v.size());
    index += v.size();
  }
  c->eval(this_x, ty);

  const Eigen::VectorXd upper_bound = c->getUpperBound();
  for (size_t i = 0; i < num_constraints; i++) {
    result[i] = ty(i).value();
    if (upper_bound[i] != std::numeric_limits<double>::infinity()) {
      result[i] -= upper_bound[i];
    }
  }

  if (grad) {
    for (const DecisionVariableView& v : *(wrapped->variable_list)) {
      for (size_t i = 0; i < num_constraints; i++) {
        for (size_t j = v.index(); j < v.index() + v.size(); j++) {
          grad[j] = ty(i).derivatives()(j);
        }
      }
    }
  }
}

}

bool NloptSolver::available() const {
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

bool NloptSolver::solve(OptimizationProblem &prog) const {

  int nx = prog.getNumVars();

  // Load the algo to use and the size.
  nlopt::opt opt(nlopt::LD_SLSQP, nx);

  const Eigen::VectorXd& initial_guess = prog.getInitialGuess();
  std::vector<double> x(initial_guess.size());
  for (size_t i = 0; i < x.size(); i++) {
    x[i] = initial_guess[i];
  }

  std::vector<double> xlow(nx, -numeric_limits<double>::infinity());
  std::vector<double> xupp(nx, numeric_limits<double>::infinity());

  for (auto const& binding : prog.getBoundingBoxConstraints()) {
    auto const& c = binding.getConstraint();
    for (const DecisionVariableView& v : binding.getVariableList()) {
      auto const lb = c->getLowerBound(), ub = c->getUpperBound();
      for (int k = 0; k < v.size(); k++) {
        const int idx = v.index() + k;
        xlow[idx] = std::max(lb(k), xlow[idx]);
        xupp[idx] = std::min(ub(k), xupp[idx]);
        if (x[idx] < xlow[idx]) { x[idx] = xlow[idx]; }
        if (x[idx] > xupp[idx]) { x[idx] = xupp[idx]; }
      }
    }
  }
  opt.set_lower_bounds(xlow);
  opt.set_upper_bounds(xupp);

  opt.set_min_objective(evaluateCosts, &prog);

  // my_constraint_data data[2] = { {2,0}, {-1,1} };
  // opt.add_inequality_constraint(myconstraint, &data[0], 1e-8);
  // opt.add_inequality_constraint(myconstraint, &data[1], 1e-8);

  std::list<WrappedConstraint> wrapped_list;

  for (auto& c: prog.getGenericConstraints()) {
    WrappedConstraint wrapped = { c.getConstraint().get(),
                                  &c.getVariableList() };
    wrapped_list.push_back(wrapped);
    std::vector<double> tol(c.getConstraint()->getNumConstraints(), 1e-4);
    opt.add_inequality_mconstraint(evaluateConstraint, &wrapped_list.back(), tol);
  }

  for (auto& c: prog.getLinearEqualityConstraints()) {
    WrappedConstraint wrapped = { c.getConstraint().get(),
                                  &c.getVariableList() };
    wrapped_list.push_back(wrapped);
    std::vector<double> tol(c.getConstraint()->getNumConstraints(), 1e-4);
    opt.add_equality_mconstraint(evaluateConstraint, &wrapped_list.back(), tol);
  }

  for (auto& c: prog.getLinearConstraints()) {
    WrappedConstraint wrapped = { c.getConstraint().get(),
                                  &c.getVariableList() };
    wrapped_list.push_back(wrapped);
    std::vector<double> tol(c.getConstraint()->getNumConstraints(), 1e-4);
    opt.add_inequality_mconstraint(evaluateConstraint, &wrapped_list.back(), tol);
  }

  opt.set_xtol_rel(1e-6);

  double minf = 0;
  nlopt::result result;
  try {
    result = opt.optimize(x, minf);
  } catch (nlopt::roundoff_limited& r) {
    cout << "NLOPT roundoff limited" << std::endl;
  }
  cout << "NLOPT result: " << result << endl;

  VectorXd sol(x.size());
  for (int i = 0; i < nx; i++) {
    sol(i) = x[i];
  }

  prog.setDecisionVariableValues(sol);
  return true;
}
}
