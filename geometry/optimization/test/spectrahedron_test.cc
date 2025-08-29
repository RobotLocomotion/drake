#include "drake/geometry/optimization/spectrahedron.h"

#include <limits>
#include <memory>
#include <utility>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace geometry {
namespace optimization {
namespace {

using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using solvers::MathematicalProgram;
using solvers::Solve;

GTEST_TEST(SpectrahedronTest, DefaultCtor) {
  Spectrahedron spect;
  EXPECT_EQ(spect.ambient_dimension(), 0);
  EXPECT_FALSE(spect.IsEmpty());
  ASSERT_TRUE(spect.MaybeGetFeasiblePoint().has_value());
  EXPECT_TRUE(spect.PointInSet(spect.MaybeGetFeasiblePoint().value()));
}

GTEST_TEST(SpectrahedronTest, Attributes) {
  EXPECT_TRUE(Spectrahedron::supported_attributes().contains(
      solvers::ProgramAttribute::kPositiveSemidefiniteConstraint));
}

GTEST_TEST(SpectrahedronTest, UnsupportedProgram) {
  MathematicalProgram prog;
  symbolic::Variable x = prog.NewContinuousVariables<1>()[0];
  prog.AddConstraint(x * x * x + x >= 0);  // Add a generic constraint.
  DRAKE_EXPECT_THROWS_MESSAGE(Spectrahedron(prog), ".*does not support.*");
}

/*
 * A trivial SDP
 * max X1(0, 1) + X1(1, 2),
 * s.t X1 ∈ ℝ³ˣ³ is psd,
 *     X1(0, 0) + X1(1, 1) + X1(2, 2) = 1,
 *     -2 ≤ X1(0, 1) + X1(1, 2) - 2 * X1(0, 2) ≤ 0,
 *     X1(2, 2) ∈ [-1, 1].
 */
GTEST_TEST(SpectrahedronTest, TrivialSdp1) {
  MathematicalProgram prog;
  auto X1 = prog.NewSymmetricContinuousVariables<3>();
  prog.AddLinearCost(-(X1(0, 1) + X1(1, 2)));
  prog.AddPositiveSemidefiniteConstraint(X1);
  prog.AddLinearEqualityConstraint(X1(0, 0) + X1(1, 1) + X1(2, 2), 1);
  prog.AddLinearConstraint(X1(0, 1) + X1(1, 2) - 2 * X1(0, 2), -2, 0);
  prog.AddBoundingBoxConstraint(-1, 1, X1(2, 2));
  auto result = Solve(prog);
  ASSERT_TRUE(result.is_success());
  const Vector6d x_star = result.GetSolution(prog.decision_variables());
  Vector6d x_bad;
  x_bad << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6;

  Spectrahedron spect(prog);
  EXPECT_EQ(spect.ambient_dimension(), 3 * (3 + 1) / 2);
  EXPECT_FALSE(spect.IsEmpty());

  // Cross-check that our BUILD file allows parallelism.
  DRAKE_DEMAND(Parallelism::Max().num_threads() > 1);

  EXPECT_TRUE(spect.IsBounded(Parallelism::None()));
  EXPECT_TRUE(spect.IsBounded(Parallelism::Max()));

  const double kTol{1e-6};
  EXPECT_TRUE(spect.PointInSet(x_star, kTol));
  EXPECT_FALSE(spect.PointInSet(x_bad, kTol));
  EXPECT_FALSE(spect.MaybeGetPoint().has_value());
  ASSERT_TRUE(spect.MaybeGetFeasiblePoint().has_value());
  EXPECT_TRUE(spect.PointInSet(spect.MaybeGetFeasiblePoint().value(), kTol));

  MathematicalProgram prog2;
  auto x2 = prog2.NewContinuousVariables<6>("x");
  const auto [new_vars, new_constraints] =
      spect.AddPointInSetConstraints(&prog2, x2);
  EXPECT_EQ(new_constraints.size(), prog.GetAllConstraints().size());
  EXPECT_EQ(prog2.positive_semidefinite_constraints().size(), 1);
  EXPECT_EQ(prog2.linear_equality_constraints().size(), 1);
  EXPECT_TRUE(CompareMatrices(
      prog.linear_equality_constraints()[0].evaluator()->GetDenseA(),
      prog2.linear_equality_constraints()[0].evaluator()->GetDenseA()));
  EXPECT_TRUE(CompareMatrices(
      prog.linear_equality_constraints()[0].evaluator()->lower_bound(),
      prog2.linear_equality_constraints()[0].evaluator()->lower_bound()));
  EXPECT_EQ(prog2.linear_constraints().size(), 1);
  EXPECT_TRUE(
      CompareMatrices(prog.linear_constraints()[0].evaluator()->GetDenseA(),
                      prog2.linear_constraints()[0].evaluator()->GetDenseA()));
  EXPECT_TRUE(CompareMatrices(
      prog.linear_constraints()[0].evaluator()->lower_bound(),
      prog2.linear_constraints()[0].evaluator()->lower_bound()));
  EXPECT_TRUE(CompareMatrices(
      prog.linear_constraints()[0].evaluator()->upper_bound(),
      prog2.linear_constraints()[0].evaluator()->upper_bound()));
  EXPECT_EQ(prog2.bounding_box_constraints().size(), 1);
  EXPECT_TRUE(CompareMatrices(
      prog.bounding_box_constraints()[0].evaluator()->lower_bound(),
      prog2.bounding_box_constraints()[0].evaluator()->lower_bound()));
  EXPECT_TRUE(CompareMatrices(
      prog.bounding_box_constraints()[0].evaluator()->upper_bound(),
      prog2.bounding_box_constraints()[0].evaluator()->upper_bound()));

  MathematicalProgram prog3;
  auto x3 = prog3.NewContinuousVariables<6>("x");
  auto t3 = prog3.NewContinuousVariables<1>("t");
  spect.AddPointInNonnegativeScalingConstraints(&prog3, x3, t3[0]);
  EXPECT_EQ(prog3.positive_semidefinite_constraints().size(), 1);
  EXPECT_EQ(prog3.linear_equality_constraints().size(), 1);
  MatrixXd A(1, 4);
  A << 1, 1, 1, -1;
  EXPECT_TRUE(CompareMatrices(
      prog3.linear_equality_constraints()[0].evaluator()->GetDenseA(), A));
  EXPECT_EQ(prog3.linear_constraints().size(),
            2 /* from the bounding box constraint */ +
                2 /* from the linear constraint */);
  A.resize(1, 4);
  A << 1, -2, 1, 2;
  EXPECT_TRUE(CompareMatrices(
      prog3.linear_constraints()[0].evaluator()->GetDenseA(), A));
  A << 1, -2, 1, 0;
  EXPECT_TRUE(CompareMatrices(
      prog3.linear_constraints()[1].evaluator()->GetDenseA(), A));
  A.resize(1, 2);
  A << 1, 1;
  EXPECT_TRUE(CompareMatrices(
      prog3.linear_constraints()[2].evaluator()->GetDenseA(), A));
  A << 1, -1;
  EXPECT_TRUE(CompareMatrices(
      prog3.linear_constraints()[3].evaluator()->GetDenseA(), A));
  Vector<double, 7> xt_test;
  xt_test << x_star, 1;
  EXPECT_TRUE(prog3.CheckSatisfied(prog3.GetAllConstraints(), xt_test));
  EXPECT_TRUE(
      prog3.CheckSatisfied(prog3.GetAllConstraints(), VectorXd::Zero(7)));
  xt_test << x_bad, 1;
  EXPECT_FALSE(prog3.CheckSatisfied(prog3.GetAllConstraints(), xt_test));

  // Test the A * x + b ∈ (c' * t + d) S variant.
  MathematicalProgram prog4;
  auto x4 = prog4.NewContinuousVariables<6>("x");
  auto t4 = prog4.NewContinuousVariables<1>("t");
  // Using A = I, b = 0, c = 1, d = 0 should produce the same program as prog3
  // (except that the PSD constraint uses additional slack variables).
  spect.AddPointInNonnegativeScalingConstraints(
      &prog4, MatrixXd::Identity(6, 6), Vector6d::Zero(), Vector1d::Ones(), 0,
      x4, t4);

  // We expect prog4 to have one additional linear constraint (for t >= 0).
  EXPECT_EQ(prog3.linear_constraints().size() + 1,
            prog4.linear_constraints().size());
  VectorXd guess = VectorXd::LinSpaced(7, 0.1, 0.9);
  prog3.SetInitialGuessForAllVariables(guess);
  prog4.SetInitialGuessForAllVariables(VectorXd::Zero(prog4.num_vars()));
  prog4.SetInitialGuess(x4, guess.head<6>());
  prog4.SetInitialGuess(t4, guess.tail<1>());
  const double kCoeffTol = 1e-14;
  for (int i = 0; i < static_cast<int>(prog3.linear_constraints().size());
       ++i) {
    const auto& b3 = prog3.linear_constraints()[i];
    const auto& b4 = prog4.linear_constraints()[i];
    // b4 depends on all of the variables, but b3 just depends on a subset, so
    // we eval instead of checking the coefficients.
    EXPECT_TRUE(CompareMatrices(prog3.EvalBindingAtInitialGuess(b3),
                                prog4.EvalBindingAtInitialGuess(b4), kTol));
    EXPECT_TRUE(CompareMatrices(b3.evaluator()->lower_bound(),
                                b4.evaluator()->lower_bound(), kCoeffTol));
    EXPECT_TRUE(CompareMatrices(b3.evaluator()->upper_bound(),
                                b4.evaluator()->upper_bound(), kCoeffTol));
  }
  // We only expect the first linear equality constraint to match (prog4 has
  // additional equality constraints for the PSD slack variables).
  {
    const auto& b3 = prog3.linear_equality_constraints()[0];
    const auto& b4 = prog4.linear_equality_constraints()[0];
    EXPECT_TRUE(CompareMatrices(prog3.EvalBindingAtInitialGuess(b3),
                                prog4.EvalBindingAtInitialGuess(b4), kTol));
    EXPECT_TRUE(CompareMatrices(b3.evaluator()->lower_bound(),
                                b4.evaluator()->lower_bound(), kCoeffTol));
  }

  // Test the A * x + b ∈ (c' * t + d) S variant with different sizes.
  MathematicalProgram prog5;
  auto x5 = prog5.NewContinuousVariables<8>("x");
  auto t5 = prog5.NewContinuousVariables<2>("t");
  spect.AddPointInNonnegativeScalingConstraints(
      &prog5, MatrixXd::Identity(6, 8), Vector6d::Zero(), Vector2d(1, 0), 0, x5,
      t5);
  prog5.SetInitialGuessForAllVariables(VectorXd::Zero(prog5.num_vars()));
  prog5.SetInitialGuess(x5.head<6>(), guess.head<6>());
  prog5.SetInitialGuess(t5.head<1>(), guess.tail<1>());
  for (int i = 0; i < static_cast<int>(prog3.linear_constraints().size());
       ++i) {
    const auto& b3 = prog3.linear_constraints()[i];
    const auto& b5 = prog5.linear_constraints()[i];
    // b4 depends on all of the variables, but b3 just depends on a subset, so
    // we eval instead of checking the coefficients.
    EXPECT_TRUE(CompareMatrices(prog3.EvalBindingAtInitialGuess(b3),
                                prog5.EvalBindingAtInitialGuess(b5), kTol));
    EXPECT_TRUE(CompareMatrices(b3.evaluator()->lower_bound(),
                                b5.evaluator()->lower_bound(), kCoeffTol));
    EXPECT_TRUE(CompareMatrices(b3.evaluator()->upper_bound(),
                                b5.evaluator()->upper_bound(), kCoeffTol));
  }
  // We only expect the first linear equality constraint to match (prog4 has
  // additional equality constraints for the PSD slack variables).
  {
    const auto& b3 = prog3.linear_equality_constraints()[0];
    const auto& b5 = prog5.linear_equality_constraints()[0];
    EXPECT_TRUE(CompareMatrices(prog3.EvalBindingAtInitialGuess(b3),
                                prog5.EvalBindingAtInitialGuess(b5), kTol));
    EXPECT_TRUE(CompareMatrices(b3.evaluator()->lower_bound(),
                                b5.evaluator()->lower_bound(), kCoeffTol));
  }
}

GTEST_TEST(SpectrahedronTest, AddPointInNonnegativeScalingConstraintsLinEqs) {
  MathematicalProgram prog;
  auto X1 = prog.NewSymmetricContinuousVariables<2>();
  prog.AddPositiveSemidefiniteConstraint(X1);

  const int kNumVars = 3;         // 3 free vars with SDP matrix of size 2x2
  const int kNumConstraints = 2;  // Test 2 constraints
  MatrixXd A(kNumConstraints, kNumVars);
  A << 0.2, 0.0, 1.7, -2.0, 2.3, 1.9;
  VectorXd b(kNumConstraints);
  b << 2.0, 1.0;

  prog.AddLinearEqualityConstraint(A, b, prog.decision_variables());

  auto result = Solve(prog);
  ASSERT_TRUE(result.is_success());

  auto spect = Spectrahedron(prog);

  MathematicalProgram prog2;
  auto x = prog2.NewContinuousVariables<kNumVars>("x");
  auto t = prog2.NewContinuousVariables<1>("t");
  spect.AddPointInNonnegativeScalingConstraints(&prog2, x, t[0]);

  EXPECT_EQ(prog2.positive_semidefinite_constraints().size(), 1);
  EXPECT_EQ(prog2.linear_equality_constraints().size(), 1);
  EXPECT_EQ(prog2.bounding_box_constraints().size(), 1);  // t >= 0
}

GTEST_TEST(SpectrahedronTest, AddPointInNonnegativeScalingConstraintsBBox) {
  MathematicalProgram prog;
  auto X1 = prog.NewSymmetricContinuousVariables<2>();
  prog.AddPositiveSemidefiniteConstraint(X1);

  const int kNumVars = 3;  // 3 free vars with SDP matrix of size 2x2
  VectorXd lb(kNumVars);
  lb << -0.1, -2.0, 2.0;
  VectorXd ub(kNumVars);
  ub << 1.0, 0.0, 4.0;

  prog.AddBoundingBoxConstraint(lb, ub, prog.decision_variables());

  auto result = Solve(prog);
  ASSERT_TRUE(result.is_success());

  auto spect = Spectrahedron(prog);

  MathematicalProgram prog2;
  auto x = prog2.NewContinuousVariables<kNumVars>("x");
  auto t = prog2.NewContinuousVariables<1>("t");
  spect.AddPointInNonnegativeScalingConstraints(&prog2, x, t[0]);

  EXPECT_EQ(prog2.positive_semidefinite_constraints().size(), 1);
  // bounding box constraints become two constraints, upper and lower bound
  EXPECT_EQ(prog2.linear_constraints().size(), 2);
  EXPECT_EQ(prog2.bounding_box_constraints().size(), 1);  // t >= 0
}

GTEST_TEST(SpectrahedronTest, Move) {
  auto make_sdp = []() {
    auto sdp = std::make_unique<MathematicalProgram>();
    sdp->AddPositiveSemidefiniteConstraint(
        sdp->NewSymmetricContinuousVariables<3>());
    return sdp;
  };
  Spectrahedron orig(*make_sdp());
  EXPECT_EQ(orig.ambient_dimension(), 6);

  // A move-constructed Spectrahedron takes over the original data.
  Spectrahedron dut(std::move(orig));
  EXPECT_EQ(dut.ambient_dimension(), 6);
  EXPECT_NO_THROW(dut.Clone());
  // In particular, make sure that the SDP is still intact by checking how many
  // constraints the spectrahedron emits.
  MathematicalProgram constraints;
  EXPECT_NO_THROW(dut.AddPointInSetConstraints(
      &constraints, constraints.NewContinuousVariables<6>()));
  EXPECT_EQ(constraints.positive_semidefinite_constraints().size(), 1);

  // The old set is in a valid but unspecified state. For convenience we'll
  // assert that it's empty, but that's not the only valid implementation,
  // just the one we happen to currently use.
  EXPECT_EQ(orig.ambient_dimension(), 0);
  EXPECT_NO_THROW(orig.Clone());
}

GTEST_TEST(SpectrahedronTest, NontriviallyEmpty) {
  // Construct an infeasible SDP, and check that its spectrahedron
  // ConvexSet is empty.
  MathematicalProgram prog;
  auto X1 = prog.NewSymmetricContinuousVariables<2>();
  prog.AddPositiveSemidefiniteConstraint(X1);

  // Enforce that X1 is diagonal, and that at least one of the
  // diagonal entries is negative, so X1 cannot be PSD.
  prog.AddLinearEqualityConstraint(X1(0, 0) + X1(1, 1), -1);
  prog.AddLinearEqualityConstraint(X1(0, 1), 0);

  Spectrahedron spect(prog);
  EXPECT_TRUE(spect.IsEmpty());
  EXPECT_FALSE(spect.MaybeGetFeasiblePoint().has_value());
}

GTEST_TEST(SpectrahedronTest, UnboundedTest) {
  // Construct an unconstrained SDP, and check that IsBounded notices.
  MathematicalProgram prog;
  auto X1 = prog.NewSymmetricContinuousVariables<2>();
  prog.AddPositiveSemidefiniteConstraint(X1);
  Spectrahedron spect(prog);
  EXPECT_FALSE(spect.IsBounded(Parallelism::None()));
  EXPECT_FALSE(spect.IsBounded(Parallelism::Max()));

  // Construct an unbounded but constrained SDP, and check that IsBounded
  // notices.
  prog.AddLinearConstraint(X1(0, 0) >= 0);
  Spectrahedron spect2(prog);
  EXPECT_FALSE(spect2.IsBounded(Parallelism::None()));
  EXPECT_FALSE(spect2.IsBounded(Parallelism::Max()));

  // Add more constraints that keep the program unbounded
  prog.AddLinearConstraint(X1(0, 0) == X1(1, 1));
  prog.AddLinearConstraint(X1(0, 1) >= 1);
  Spectrahedron spect3(prog);
  EXPECT_FALSE(spect3.IsBounded(Parallelism::None()));
  EXPECT_FALSE(spect3.IsBounded(Parallelism::Max()));

  // Add a constraint that makes the program bounded
  prog.AddLinearConstraint(X1(0, 0) + X1(0, 1) + X1(1, 1) <= 43);
  Spectrahedron spect4(prog);
  EXPECT_TRUE(spect4.IsBounded(Parallelism::None()));
  EXPECT_TRUE(spect4.IsBounded(Parallelism::Max()));

  // Add a constraint that makes the program infeasible (the empty
  // set is bounded)
  prog.AddLinearConstraint(X1(0, 0) >= 50);
  Spectrahedron spect5(prog);
  EXPECT_TRUE(spect5.IsEmpty());
  EXPECT_TRUE(spect5.IsBounded(Parallelism::None()));
  EXPECT_TRUE(spect5.IsBounded(Parallelism::Max()));
}

}  // namespace
}  // namespace optimization
}  // namespace geometry
}  // namespace drake
