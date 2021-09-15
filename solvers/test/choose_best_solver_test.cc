#include "drake/solvers/choose_best_solver.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/solvers/clp_solver.h"
#include "drake/solvers/csdp_solver.h"
#include "drake/solvers/equality_constrained_qp_solver.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/ipopt_solver.h"
#include "drake/solvers/linear_system_solver.h"
#include "drake/solvers/moby_lcp_solver.h"
#include "drake/solvers/mosek_solver.h"
#include "drake/solvers/nlopt_solver.h"
#include "drake/solvers/osqp_solver.h"
#include "drake/solvers/scs_solver.h"
#include "drake/solvers/snopt_solver.h"

namespace drake {
namespace solvers {

class ChooseBestSolverTest : public ::testing::Test {
 public:
  ChooseBestSolverTest()
      : prog_{},
        x_{prog_.NewContinuousVariables<3>()},
        clp_solver_{std::make_unique<ClpSolver>()},
        linear_system_solver_{std::make_unique<LinearSystemSolver>()},
        equality_constrained_qp_solver_{
            std::make_unique<EqualityConstrainedQPSolver>()},
        mosek_solver_{std::make_unique<MosekSolver>()},
        gurobi_solver_{std::make_unique<GurobiSolver>()},
        osqp_solver_{std::make_unique<OsqpSolver>()},
        moby_lcp_solver_{std::make_unique<MobyLCPSolver<double>>()},
        snopt_solver_{std::make_unique<SnoptSolver>()},
        ipopt_solver_{std::make_unique<IpoptSolver>()},
        nlopt_solver_{std::make_unique<NloptSolver>()},
        csdp_solver_{std::make_unique<CsdpSolver>()},
        scs_solver_{std::make_unique<ScsSolver>()} {}

  ~ChooseBestSolverTest() {}

  void CheckBestSolver(const SolverId& expected_solver_id) const {
    const SolverId solver_id = ChooseBestSolver(prog_);
    EXPECT_EQ(solver_id, expected_solver_id);

    // Ensure GetKnownSolvers is comprehensive.
    EXPECT_EQ(GetKnownSolvers().count(solver_id), 1);
  }

  void CheckMakeSolver(const SolverInterface& solver) const {
    auto new_solver = MakeSolver(solver.solver_id());
    EXPECT_EQ(new_solver->solver_id(), solver.solver_id());

    // Ensure GetKnownSolvers is comprehensive.
    EXPECT_EQ(GetKnownSolvers().count(solver.solver_id()), 1);
  }

  void CheckBestSolver(const std::vector<SolverInterface*>& solvers) {
    bool is_any_solver_available = false;
    for (const auto solver : solvers) {
      if (solver->available()) {
        is_any_solver_available = true;
        break;
      }
    }
    if (!is_any_solver_available) {
      DRAKE_EXPECT_THROWS_MESSAGE(
          ChooseBestSolver(prog_), std::invalid_argument,
          "There is no available solver for the optimization program");
    } else {
      const SolverId solver_id = ChooseBestSolver(prog_);
      for (const auto solver : solvers) {
        if (solver->available()) {
          EXPECT_EQ(solver_id, solver->solver_id());
          return;
        }
      }
    }
  }

 protected:
  MathematicalProgram prog_;
  VectorDecisionVariable<3> x_;
  std::unique_ptr<ClpSolver> clp_solver_;
  std::unique_ptr<LinearSystemSolver> linear_system_solver_;
  std::unique_ptr<EqualityConstrainedQPSolver> equality_constrained_qp_solver_;
  std::unique_ptr<MosekSolver> mosek_solver_;
  std::unique_ptr<GurobiSolver> gurobi_solver_;
  std::unique_ptr<OsqpSolver> osqp_solver_;
  std::unique_ptr<MobyLCPSolver<double>> moby_lcp_solver_;
  std::unique_ptr<SnoptSolver> snopt_solver_;
  std::unique_ptr<IpoptSolver> ipopt_solver_;
  std::unique_ptr<NloptSolver> nlopt_solver_;
  std::unique_ptr<CsdpSolver> csdp_solver_;
  std::unique_ptr<ScsSolver> scs_solver_;
};

TEST_F(ChooseBestSolverTest, LinearSystemSolver) {
  prog_.AddLinearEqualityConstraint(x_(0) + x_(1), 1);
  CheckBestSolver(LinearSystemSolver::id());
}

TEST_F(ChooseBestSolverTest, EqualityConstrainedQPSolver) {
  prog_.AddQuadraticCost(x_(0) * x_(0));
  prog_.AddLinearEqualityConstraint(x_(0) + x_(1), 1);
  CheckBestSolver(EqualityConstrainedQPSolver::id());
}

TEST_F(ChooseBestSolverTest, LPsolver) {
  prog_.AddLinearEqualityConstraint(x_(0) + 3 * x_(1) == 3);
  CheckBestSolver(LinearSystemSolver::id());
  prog_.AddLinearConstraint(x_(0) + 2 * x_(1) >= 1);
  prog_.AddLinearCost(x_(0) + x_(1));
  CheckBestSolver({gurobi_solver_.get(), mosek_solver_.get(), clp_solver_.get(),
                   snopt_solver_.get(), ipopt_solver_.get(),
                   nlopt_solver_.get(), csdp_solver_.get(), scs_solver_.get()});
}

TEST_F(ChooseBestSolverTest, QPsolver) {
  prog_.AddLinearConstraint(x_(0) + x_(1) >= 1);
  prog_.AddQuadraticCost(x_(0) * x_(0));
  CheckBestSolver({mosek_solver_.get(), gurobi_solver_.get(),
                   osqp_solver_.get(), snopt_solver_.get(), ipopt_solver_.get(),
                   nlopt_solver_.get(), scs_solver_.get()});
}

TEST_F(ChooseBestSolverTest, LorentzCone) {
  prog_.AddLorentzConeConstraint(x_.cast<symbolic::Expression>());
  CheckBestSolver({mosek_solver_.get(), gurobi_solver_.get(),
                   csdp_solver_.get(), scs_solver_.get(), snopt_solver_.get(),
                   ipopt_solver_.get(), nlopt_solver_.get()});
  prog_.AddRotatedLorentzConeConstraint(x_.cast<symbolic::Expression>());
  CheckBestSolver({mosek_solver_.get(), gurobi_solver_.get(),
                   csdp_solver_.get(), scs_solver_.get(), snopt_solver_.get(),
                   ipopt_solver_.get(), nlopt_solver_.get()});

  prog_.AddPolynomialCost(pow(x_(0), 3));
  CheckBestSolver(
      {snopt_solver_.get(), ipopt_solver_.get(), nlopt_solver_.get()});
}

TEST_F(ChooseBestSolverTest, LinearComplementarityConstraint) {
  prog_.AddLinearComplementarityConstraint(Eigen::Matrix3d::Identity(),
                                           Eigen::Vector3d::Ones(), x_);
  CheckBestSolver({moby_lcp_solver_.get(), snopt_solver_.get()});

  prog_.AddLinearCost(x_(0) + 1);
  CheckBestSolver({snopt_solver_.get()});
}

TEST_F(ChooseBestSolverTest, PositiveSemidefiniteConstraint) {
  prog_.AddPositiveSemidefiniteConstraint(
      (Matrix2<symbolic::Variable>() << x_(0), x_(1), x_(1), x_(2)).finished());
  CheckBestSolver({mosek_solver_.get(), csdp_solver_.get(), scs_solver_.get()});
}

TEST_F(ChooseBestSolverTest, BinaryVariable) {
  prog_.NewBinaryVariables<1>();
  prog_.AddLinearConstraint(x_(0) + x_(1) == 1);
  if (GurobiSolver::is_available()) {
    CheckBestSolver(GurobiSolver::id());
  } else if (MosekSolver::is_available()) {
    CheckBestSolver(MosekSolver::id());
  } else {
    DRAKE_EXPECT_THROWS_MESSAGE(
        ChooseBestSolver(prog_), std::invalid_argument,
        "There is no available solver for the optimization program, please "
        "manually instantiate MixedIntegerBranchAndBound.*");
  }
}

TEST_F(ChooseBestSolverTest, NoAvailableSolver) {
  // We don't have a solver for problem with both linear complementarity
  // constraint and binary variables.
  prog_.AddLinearComplementarityConstraint(
      Eigen::Matrix2d::Identity(), Eigen::Vector2d::Ones(), x_.tail<2>());
  prog_.NewBinaryVariables<2>();
  DRAKE_EXPECT_THROWS_MESSAGE(
      ChooseBestSolver(prog_),
      "There is no available solver for the optimization program");
}

TEST_F(ChooseBestSolverTest, MakeSolver) {
  CheckMakeSolver(*linear_system_solver_);
  CheckMakeSolver(*equality_constrained_qp_solver_);
  CheckMakeSolver(*mosek_solver_);
  CheckMakeSolver(*gurobi_solver_);
  CheckMakeSolver(*osqp_solver_);
  CheckMakeSolver(*moby_lcp_solver_);
  CheckMakeSolver(*snopt_solver_);
  CheckMakeSolver(*ipopt_solver_);
  CheckMakeSolver(*nlopt_solver_);
  CheckMakeSolver(*scs_solver_);
  DRAKE_EXPECT_THROWS_MESSAGE(MakeSolver(SolverId("foo")),
                              std::invalid_argument,
                              "MakeSolver: no matching solver foo");
}

// Checks that all of the known solvers can be instantiated.
TEST_F(ChooseBestSolverTest, KnownSolvers) {
  const std::set<SolverId>& ids = GetKnownSolvers();
  EXPECT_GT(ids.size(), 5);
  for (const auto& id : ids) {
    EXPECT_EQ(MakeSolver(id)->solver_id(), id);
  }
}

GTEST_TEST(MakeFirstAvailableSolver, Test) {
  const bool has_gurobi =
      GurobiSolver::is_available() && GurobiSolver::is_enabled();
  const bool has_scs = ScsSolver::is_available() && ScsSolver::is_enabled();

  if (has_gurobi || has_scs) {
    auto solver =
        MakeFirstAvailableSolver({GurobiSolver::id(), ScsSolver::id()});
    if (has_gurobi) {
      EXPECT_EQ(solver->solver_id(), GurobiSolver::id());
    } else {
      EXPECT_EQ(solver->solver_id(), ScsSolver::id());
    }
  }

  if (!has_gurobi) {
    DRAKE_EXPECT_THROWS_MESSAGE(MakeFirstAvailableSolver({GurobiSolver::id()}),
                                std::runtime_error,
                                ".* is available and enabled.");
  }
}

}  // namespace solvers
}  // namespace drake
