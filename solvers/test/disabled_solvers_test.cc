#include <memory>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/solvers/clarabel_solver.h"
#include "drake/solvers/clp_solver.h"
#include "drake/solvers/csdp_solver.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/ipopt_solver.h"
#include "drake/solvers/mosek_solver.h"
#include "drake/solvers/nlopt_solver.h"
#include "drake/solvers/osqp_solver.h"
#include "drake/solvers/scs_solver.h"
#include "drake/solvers/snopt_solver.h"

namespace drake {
namespace solvers {
namespace {

template <typename SomeSolver>
class DisabledSolverTest : public testing::Test {};

TYPED_TEST_SUITE_P(DisabledSolverTest);

TYPED_TEST_P(DisabledSolverTest, SmokeTest) {
  std::unique_ptr<SolverInterface> dut;
  EXPECT_NO_THROW(dut = std::make_unique<TypeParam>());

  EXPECT_FALSE(dut->available());
  EXPECT_NO_THROW(dut->enabled());
  EXPECT_FALSE(dut->solver_id().name().empty());

  const MathematicalProgram prog;
  EXPECT_NO_THROW(dut->AreProgramAttributesSatisfied(prog));
  EXPECT_NO_THROW(dut->ExplainUnsatisfiedProgramAttributes(prog));

  MathematicalProgramResult result;
  DRAKE_EXPECT_THROWS_MESSAGE(dut->Solve(prog, {}, {}, &result),
                              ".*has not been compiled.*");
}

REGISTER_TYPED_TEST_SUITE_P(DisabledSolverTest, SmokeTest);

using AllSolvers = ::testing::Types<  //
    ClarabelSolver,                   //
    ClpSolver,                        //
    CsdpSolver,                       //
    GurobiSolver,                     //
    IpoptSolver,                      //
    MosekSolver,                      //
    NloptSolver,                      //
    OsqpSolver,                       //
    ScsSolver,                        //
    SnoptSolver>;
INSTANTIATE_TYPED_TEST_SUITE_P(All, DisabledSolverTest, AllSolvers);

}  // namespace
}  // namespace solvers
}  // namespace drake
