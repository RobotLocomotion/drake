#pragma once
#include "drake/solvers/Optimization.h"
#include "drake/solvers/SnoptSolver.h"
#include "EigenQuadProg.h"

using namespace Eigen;

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
    /*
    drake::solvers::OptimizationProblem prog;
    auto x = prog.AddContinuousVariables(_nVar);
    prog.AddQuadraticProgramCost(_H, _h0);
    prog.AddLinearEqualityConstraint(_CE, -_ce0);
    prog.AddLinearConstraint(_CI, _ci_l, _ci_u);
    prog.SetInitialGuess({x}, VectorXd::Zero(_nVar));
    drake::solvers::SolutionResult result;
    result = _solver.Solve(prog);
    assert(result == drake::solvers::SolutionResult::kSolutionFound);
    _X = x.value();
    */

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
  }

private:
  drake::solvers::SnoptSolver _solver;
};
