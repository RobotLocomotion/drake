#pragma once
#include "drake/solvers/Optimization.h"
#include "drake/solvers/SnoptSolver.h"
#include "EigenQuadProg.h"

using namespace Eigen;
using namespace drake::solvers;

inline VectorXd variableList2VectorXd(VariableList const &vlist)
{
  size_t dim = 0;
  for (auto var : vlist) {
    dim += var.size(); 
  }
  VectorXd X(dim);
  dim = 0;
  for (auto var : vlist) {
    X.segment(dim, var.size()) = var.value();
    dim += var.size();
  }
  return X;
}

class sfQP 
{
protected:
  int _nVar;
  int _nEq;
  int _nInEq;

  MatrixXd _CE, _CI, _H;
  VectorXd _ce0, _ci_l, _ci_u, _h0, _X;

  QP qp;

  virtual void resize()
  {
    _CE.resize(_nEq, _nVar);
    _ce0.resize(_nEq);
    _CI.resize(_nInEq, _nVar);
    _ci_l.resize(_nInEq);
    _ci_u.resize(_nInEq);

    _H.resize(_nVar, _nVar);
    _h0.resize(_nVar);

    _X.resize(_nVar);
  }

  virtual void setZero()
  {
    _CE.setZero();
    _ce0.setZero();
    _CI.setZero();
    _ci_u = VectorXd::Constant(_nInEq, std::numeric_limits<double>::infinity());
    _ci_l = VectorXd::Constant(_nInEq, -std::numeric_limits<double>::infinity());
    _H.setZero();
    _h0.setZero(); 
  }

  virtual void solve()
  {
    drake::solvers::OptimizationProblem prog;
    auto x = prog.AddContinuousVariables(_nVar);
    prog.AddQuadraticCost(_H, _h0);
    prog.AddLinearEqualityConstraint(_CE, -_ce0);
    prog.AddLinearConstraint(_CI, _ci_l, _ci_u);
    prog.SetInitialGuess({x}, VectorXd::Zero(_nVar));
    drake::solvers::SolutionResult result;
    result = _solver.Solve(prog);
    assert(result == drake::solvers::SolutionResult::kSolutionFound);
    _X = x.value();
    
    auto costs = prog.generic_costs();
    auto eqs = prog.linear_equality_constraints();
    auto ineqs = prog.linear_constraints();

    // would be nice to have a variable list -> flat VectorXd method
    for (auto cost_b : costs) {
      VectorXd val;
      std::shared_ptr<Constraint> cost = cost_b.constraint();
      cost->eval(variableList2VectorXd(cost_b.variable_list()), val);
      std::cout << "cost term 0.5 x^T * H * x + h0 * x: " << val.transpose() << std::endl;
    }

    for (auto eq_b : eqs) {
      std::shared_ptr<LinearEqualityConstraint> eq = eq_b.constraint();
      VectorXd X = variableList2VectorXd(eq_b.variable_list());
      assert((eq->A() * X - eq->lower_bound()).isZero());
    }

    for (auto ineq_b : ineqs) {
      std::shared_ptr<LinearConstraint> ineq = ineq_b.constraint();
      VectorXd X = variableList2VectorXd(ineq_b.variable_list());
      X = ineq->A() * X;
      for (size_t i = 0; i < X.size(); i++) {
        assert(X[i] >= ineq->lower_bound()[i] && X[i] <= ineq->upper_bound()[i]);
      }
    }

    /*
    int ctr = 0;
    for (int i = 0; i < _nInEq; i++) {
      if (!isinf(_ci_l[i]))
        ctr++;
      if (!isinf(_ci_u(i)))
        ctr++;
    }

    MatrixXd CI(ctr, _nVar);
    VectorXd ci0(ctr);
    qp.resize(_nVar, _nEq, ctr);
    
    ctr = 0;
    for (int i = 0; i < _nInEq; i++) {
      if (!isinf(_ci_l[i])) {
        CI.row(ctr) = _CI.row(i);
        ci0[ctr] = -_ci_l[i];
        ctr++;
      }
      if (!isinf(_ci_u[i])) {
        CI.row(ctr) = -_CI.row(i);
        ci0[ctr] = _ci_u[i];
        ctr++;
      }
    }
    
    qp.solve_quadprog(_H, _h0, _CE.transpose(), _ce0, CI.transpose(), ci0, _X);
    */
  }

private:
  drake::solvers::SnoptSolver _solver;
};
