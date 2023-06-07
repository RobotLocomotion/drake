#include "drake/multibody/contact_solvers/sap/sap_contact_problem.h"

#include <set>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/contact_solvers/sap/sap_constraint.h"

using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {
namespace {

// Useful SPD matrices for testing. The numbers specify their size.
// clang-format off
const Eigen::MatrixXd S00 = Eigen::MatrixXd::Zero(0, 0);
const Eigen::Matrix2d S22 =
    (Eigen::Matrix2d() << 2, 1,
                          1, 2).finished();
const Eigen::Matrix3d S33 =
    (Eigen::Matrix3d() << 4, 1, 2,
                          1, 5, 3,
                          2, 3, 6).finished();
const Eigen::Matrix4d S44 =
    (Eigen::Matrix4d() << 7, 1, 2, 3,
                          1, 8, 4, 5,
                          2, 4, 9, 6,
                          3, 5, 6, 10).finished();
// clang-format on

// Test constraint for which constraint and Jacobian values are not important.
// Only indexes and constraint sizes matter for the unit tests in this file.
class TestConstraint final : public SapConstraint<double> {
 public:
  // Constructor for a constraint on a single clique.
  // No objects are registered.
  TestConstraint(int num_constraint_equations, int clique, int clique_nv)
      : SapConstraint<double>(
            {clique, MatrixXd::Zero(num_constraint_equations, clique_nv)}) {}

  // Constructor for a constraint between two cliques.
  // Registers objects with index first_clique and second_clique, for testing.
  TestConstraint(int num_constraint_equations, int first_clique,
                 int first_clique_nv, int second_clique, int second_clique_nv)
      : SapConstraint<double>(
            {first_clique,
             MatrixXd::Zero(num_constraint_equations, first_clique_nv),
             second_clique,
             MatrixXd::Zero(num_constraint_equations, second_clique_nv)}) {
    this->RegisterObjects({first_clique, second_clique});
  }

  // N.B no-op overloads to allow us compile this testing constraint. These
  // methods are only tested for specific derived classes, not in this file.
  std::unique_ptr<AbstractValue> DoMakeData(
      const double&, const Eigen::Ref<const VectorXd>&) const final {
    return nullptr;
  }
  void DoCalcData(const Eigen::Ref<const VectorXd>&,
                  AbstractValue*) const final {}
  double DoCalcCost(const AbstractValue&) const final { return 0.0; }
  void DoCalcImpulse(const AbstractValue&, EigenPtr<VectorXd>) const final {}
  void DoCalcCostHessian(const AbstractValue&, MatrixX<double>*) const final {}

 private:
  TestConstraint(const TestConstraint&) = default;

  // Accumulates an arbitrary functional form for testing.
  void DoAccumulateGeneralizedImpulses(int c,
                                       const Eigen::Ref<const VectorXd>& gamma,
                                       EigenPtr<VectorXd> tau) const final {
    *tau += VectorXd::Ones(this->num_velocities(c)) * gamma.sum();
  }

  void DoAccumulateSpatialImpulses(int o,
                                   const Eigen::Ref<const VectorXd>& gamma,
                                   SpatialForce<double>* F) const final {
    // Vector6d v6 = o * gamma.sum() *
    // Vector6d::Ones();
    Vector6d v6 = o * Vector6d::Ones() * gamma.sum();
    *F += SpatialForce<double>(v6);
  }

  std::unique_ptr<SapConstraint<double>> DoClone() const final {
    return std::unique_ptr<TestConstraint>(new TestConstraint(*this));
  }
};

// Test construction of an empty problem.
GTEST_TEST(ContactProblem, EmptyProblem) {
  const double dt = 2.5e-4;
  SapContactProblem<double> problem(dt);
  EXPECT_EQ(problem.time_step(), dt);
  EXPECT_EQ(problem.num_cliques(), 0);
  EXPECT_EQ(problem.num_velocities(), 0);
  EXPECT_EQ(problem.num_constraints(), 0);
  EXPECT_EQ(problem.num_constraint_equations(), 0);
}

// Unit test the construction of a SapContactProblem.
GTEST_TEST(ContactProblem, Construction) {
  const double time_step = 0.01;
  const std::vector<MatrixXd> A{S22, S33, S44, S22};
  const VectorXd v_star = VectorXd::LinSpaced(11, 1.0, 11.0);
  SapContactProblem<double> problem(time_step, A, v_star);
  EXPECT_EQ(problem.time_step(), time_step);
  EXPECT_EQ(problem.num_cliques(), 4);
  EXPECT_EQ(problem.num_velocities(), 11);
  EXPECT_EQ(problem.num_velocities(0), 2);
  EXPECT_EQ(problem.num_velocities(1), 3);
  EXPECT_EQ(problem.num_velocities(2), 4);
  EXPECT_EQ(problem.num_velocities(3), 2);
  EXPECT_EQ(problem.dynamics_matrix(), A);
  EXPECT_EQ(problem.v_star(), v_star);
}

// Unit test that AddConstraint() throws for the right reasons when arguments
// are wrong.
GTEST_TEST(ContactProblem, AddConstraintWithWrongArguments) {
  const double time_step = 0.01;
  const std::vector<MatrixXd> A{S22, S33, S44, S22, S00};
  const VectorXd v_star = VectorXd::LinSpaced(11, 1.0, 11.0);
  SapContactProblem<double> problem(time_step, A, v_star);

  DRAKE_EXPECT_THROWS_MESSAGE(
      problem.AddConstraint(std::make_unique<TestConstraint>(
          3 /* num_equations */, 5 /* (wrong) first_clique */,
          2 /* first_clique_nv */, 1 /* second_clique */,
          3 /* second_clique_nv */)),
      "First clique index must be strictly lower than num_cliques\\(\\)");
  DRAKE_EXPECT_THROWS_MESSAGE(
      problem.AddConstraint(std::make_unique<TestConstraint>(
          3 /* num_equations */, 0 /* first_clique */, 2 /* first_clique_nv */,
          5 /* (wrong) second_clique */, 3 /* second_clique_nv */)),
      "Second clique index must be strictly lower than num_cliques\\(\\)");
  DRAKE_EXPECT_THROWS_MESSAGE(
      problem.AddConstraint(std::make_unique<TestConstraint>(
          3 /* num_equations */, 0 /* first_clique */,
          3 /* (wrong) first_clique_nv */, 1 /* second_clique */,
          3 /* second_clique_nv */)),
      ".* does not match the number of velocities in this problem for "
      "the first clique.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      problem.AddConstraint(std::make_unique<TestConstraint>(
          3 /* num_equations */, 0 /* first_clique */, 2 /* first_clique_nv */,
          1 /* second_clique */, 6 /* (wrong) second_clique_nv */)),
      ".* does not match the number of velocities in this problem for "
      "the second clique.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      problem.AddConstraint(std::make_unique<TestConstraint>(
          3 /* num_equations */, 0 /* first_clique */, 2 /* first_clique_nv */,
          4 /* (empty) second_clique */, 0 /* second_clique_nv */)),
      ".*Adding constraint.*zero.*velocities.*");
}

/* We test ContactProblem with the graph setup sketched below where each
 box corresponds to a clique (a node in the graph) an edges correspond to a
 cluster of constraints connecting one or more cliques. As documented in
 ContactProblemGraph's documentation, clusters are created (and assigned a
 non-negative increasing index) the first time a constraint between two cliques
 (or just one) is added with a call to AddConstraint(). The graph below is then
 the expected graph for the set of ContactProblem::AddConstraint() calls in the
 order listed in this test.

 The labels below correspond to:
  1. The first number corresponds to the expected cluster index.
  2. The numbers in square brackets correspond to the constraint index,
     assigned by each call to AddConstraint in the order it was called.
  3. The numbers in parentheses correspond to the number of equations of each of
     the constraints in the cluster, also in the order they were added.

                         0[0](1)
                          ┌─┐
                          │ │
 ┌───┐1[1](3)┌───┐3[3](2)┌┴─┴┐     ┌───┐
 │ 0 ├───────┤ 1 ├───────┤ 3 │     │ 2 │
 └─┬─┘       └───┘       └─┬─┘     └───┘
   │                       │
   └───────────────────────┘
         2[2, 4](6,5)

  TestConstraint with two cliques registers objects with index equal to the
  clique indexes. Therefore we expect the problem to have 3 objects with indexes
  0, 1 and 3.
*/
void AddConstraints(SapContactProblem<double>* problem) {
  auto add_and_verify_access =
      [problem](std::unique_ptr<TestConstraint> owned_constraint) {
        const int expected_index = problem->num_constraints();
        const TestConstraint& constraint = *owned_constraint;
        EXPECT_EQ(problem->AddConstraint(std::move(owned_constraint)),
                  expected_index);
        EXPECT_EQ(&problem->get_constraint(expected_index), &constraint);
      };

  add_and_verify_access(std::make_unique<TestConstraint>(
      1 /* num_equations */, 3 /* clique */, 2 /* clique_nv */));
  add_and_verify_access(std::make_unique<TestConstraint>(
      3 /* num_equations */, 0 /* first_clique */, 2 /* first_clique_nv */,
      1 /* second_clique */, 3 /* second_clique_nv */));
  add_and_verify_access(std::make_unique<TestConstraint>(
      6 /* num_equations */, 0 /* first_clique */, 2 /* first_clique_nv */,
      3 /* second_clique */, 2 /* second_clique_nv */));
  add_and_verify_access(std::make_unique<TestConstraint>(
      2 /* num_equations */, 1 /* first_clique */, 3 /* first_clique_nv */,
      3 /* second_clique */, 2 /* second_clique_nv */));
  add_and_verify_access(std::make_unique<TestConstraint>(
      5 /* num_equations */, 0 /* first_clique */, 2 /* first_clique_nv */,
      3 /* second_clique */, 2 /* second_clique_nv */));
}

GTEST_TEST(ContactProblem, AddConstraints) {
  const double time_step = 0.01;
  const std::vector<MatrixXd> A{S22, S33, S44, S22};
  const VectorXd v_star = VectorXd::LinSpaced(11, 1.0, 11.0);
  SapContactProblem<double> problem(time_step, std::move(A), std::move(v_star),
                                    4);
  AddConstraints(&problem);

  EXPECT_EQ(problem.num_constraints(), 5);
  EXPECT_EQ(problem.num_constraint_equations(), 17);

  // Verify object registration.
  EXPECT_EQ(problem.num_objects(), 4);
  EXPECT_EQ(problem.objects(), (std::set{0, 1, 3}));

  // Verify graph for this problem.
  const ContactProblemGraph& graph = problem.graph();
  EXPECT_EQ(graph.num_cliques(), 4);
  EXPECT_EQ(graph.num_constraints(), 5);
  EXPECT_EQ(graph.num_clusters(), 4);
  EXPECT_EQ(graph.num_constraint_equations(), 17);
}

GTEST_TEST(ContactProblem, Reset) {
  // Instantiate empty problem.
  const double dt = 2.5e-4;
  SapContactProblem<double> problem(dt);
  EXPECT_EQ(problem.time_step(), dt);
  EXPECT_EQ(problem.num_cliques(), 0);
  EXPECT_EQ(problem.num_velocities(), 0);
  EXPECT_EQ(problem.num_constraints(), 0);
  EXPECT_EQ(problem.num_constraint_equations(), 0);

  // Tests we can set dynamics and add constraints on an already instantiated
  // problem.
  const std::vector<MatrixXd> A{S22, S33, S44, S22};
  const VectorXd v_star = VectorXd::LinSpaced(11, 1.0, 11.0);
  problem.Reset(std::move(A), std::move(v_star), 4);
  AddConstraints(&problem);
  EXPECT_EQ(problem.num_cliques(), 4);
  EXPECT_EQ(problem.num_velocities(), 11);
  EXPECT_EQ(problem.num_constraints(), 5);
  EXPECT_EQ(problem.num_constraint_equations(), 17);

  // We test the call to Reset on a non-empty problem.
  problem.Reset({S22, S33}, v_star.segment(0, 5));
  EXPECT_EQ(problem.num_cliques(), 2);
  EXPECT_EQ(problem.num_velocities(), 5);
  EXPECT_EQ(problem.num_constraints(), 0);
  EXPECT_EQ(problem.num_constraint_equations(), 0);
  // The time step remains the same after calling Reset().
  EXPECT_EQ(problem.time_step(), dt);
}

GTEST_TEST(ContactProblem, Clone) {
  const double time_step = 0.01;
  const std::vector<MatrixXd> A{S22, S33, S44, S22};
  const VectorXd v_star = VectorXd::LinSpaced(11, 1.0, 11.0);
  SapContactProblem<double> problem(time_step, std::move(A), std::move(v_star),
                                    4);
  AddConstraints(&problem);

  std::unique_ptr<SapContactProblem<double>> clone = problem.Clone();
  EXPECT_EQ(clone->num_cliques(), problem.num_cliques());
  EXPECT_EQ(clone->num_constraints(), problem.num_constraints());
  EXPECT_EQ(clone->num_constraint_equations(),
            problem.num_constraint_equations());

  // Verify graph for this problem.
  const ContactProblemGraph& graph = clone->graph();
  EXPECT_EQ(graph.num_cliques(), 4);
  EXPECT_EQ(graph.num_constraints(), 5);
  EXPECT_EQ(graph.num_clusters(), 4);
  EXPECT_EQ(graph.num_constraint_equations(), 17);
}

GTEST_TEST(ContactProblem, CalcConstraintMultibodyForces) {
  const double time_step = 0.01;
  const std::vector<MatrixXd> A{S22, S33, S44, S22};
  const VectorXd v_star = VectorXd::LinSpaced(11, 1.0, 11.0);
  SapContactProblem<double> problem(time_step, std::move(A), std::move(v_star),
                                    4);
  AddConstraints(&problem);

  // Arbitrary vector of impulse values.
  const VectorXd gamma =
      VectorXd::LinSpaced(problem.num_constraint_equations(), -12.3, 24.5);

  // Compute the multibody forces resulting from the set of constraints in the
  // entire problem, given the known vector of impulses `gamma` for the problem.
  VectorXd tau = VectorXd::Zero(problem.num_velocities());
  std::vector<SpatialForce<double>> spatial_forces(
      problem.num_objects(), SpatialForce<double>::Zero());
  problem.CalcConstraintMultibodyForces(gamma, &tau, &spatial_forces);

  // Computed expected generalized forces due to the entire set of constraint in
  // problem. We know each constraint contributes:
  //  tau_c = time_step * E * gamma_c.sum()
  // where E is the vector of all ones.
  VectorXd tau_expected = VectorXd::Zero(problem.num_velocities());
  int offset = 0;
  for (int i = 0; i < problem.num_constraints(); ++i) {
    const auto& constraint = problem.get_constraint(i);
    const int ne = constraint.num_constraint_equations();
    const auto gamma_c = gamma.segment(offset, ne);
    for (int c = 0; c < constraint.num_cliques(); ++c) {
      const int clique = constraint.clique(c);
      tau_expected
          .segment(problem.velocities_start(clique),
                   problem.num_velocities(clique))
          .array() += gamma_c.sum();
    }
    offset += ne;
  }
  tau_expected /= problem.time_step();

  EXPECT_TRUE(CompareMatrices(tau, tau_expected,
                              std::numeric_limits<double>::epsilon(),
                              MatrixCompareType::relative));

  // TestConstraint applies spatial force:
  // F = (time_step + local_object) * gamma.sum() * Vector6d::Ones().
  std::vector<SpatialForce<double>> spatial_forces_expected(
      problem.num_objects(), SpatialForce<double>::Zero());
  offset = 0;
  for (int i = 0; i < problem.num_constraints(); ++i) {
    const auto& constraint = problem.get_constraint(i);
    const int ne = constraint.num_constraint_equations();
    const auto gamma_c = gamma.segment(offset, ne);

    for (int o = 0; o < constraint.num_objects(); ++o) {
      const int object = constraint.object(o);
      spatial_forces_expected[object].get_coeffs().array() +=
          (o * gamma_c.sum());
    }
    offset += ne;
  }
  for (SpatialForce<double>& F : spatial_forces_expected) {
    F.get_coeffs() /= problem.time_step();
  }

  for (int object = 0; object < problem.num_objects(); ++object) {
    EXPECT_TRUE(CompareMatrices(spatial_forces[object].get_coeffs(),
                                spatial_forces_expected[object].get_coeffs(),
                                std::numeric_limits<double>::epsilon(),
                                MatrixCompareType::relative));
  }
}

}  // namespace
}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
