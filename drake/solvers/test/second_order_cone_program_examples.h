#pragma once

#include <gtest/gtest.h>

#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {
namespace test {

enum class EllipsoidsSeparationProblem {
  kProblem0,
  kProblem1,
  kProblem2,
  kProblem3
};

std::vector<EllipsoidsSeparationProblem> GetEllipsoidsSeparationProblems();

class TestEllipsoidsSeparation
    : public ::testing::TestWithParam<EllipsoidsSeparationProblem> {
 public:
  TestEllipsoidsSeparation();

  void SolveAndCheckSolution(const MathematicalProgramSolverInterface& solver);

 private:
  Eigen::VectorXd x1_;
  Eigen::VectorXd x2_;
  Eigen::MatrixXd R1_;
  Eigen::MatrixXd R2_;
  MathematicalProgram prog_;
  VectorDecisionVariable<2> t_;
  VectorXDecisionVariable a_;
};

enum class QPasSOCPProblem {
  kProblem0,
  kProblem1
};

std::vector<QPasSOCPProblem> GetQPasSOCPProblems();

class TestQPasSOCP : public ::testing::TestWithParam<QPasSOCPProblem> {
 public:
  TestQPasSOCP();

  void SolveAndCheckSolution(const MathematicalProgramSolverInterface& solver);

 private:
  Eigen::MatrixXd Q_;
  Eigen::VectorXd c_;
  Eigen::MatrixXd A_;
  Eigen::VectorXd b_lb_;
  Eigen::VectorXd b_ub_;
  MathematicalProgram prog_socp_;
  MathematicalProgram prog_qp_;
  VectorXDecisionVariable x_socp_;
  symbolic::Variable y_;
  VectorXDecisionVariable x_qp_;
};
}  // namespace test
}  // namespace solvers
}  // namespace drake