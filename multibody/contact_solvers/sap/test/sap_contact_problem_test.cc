#include "drake/multibody/contact_solvers/sap/sap_contact_problem.h"

#include <gtest/gtest.h>

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
  TestConstraint(int num_constraint_equations, int clique, int clique_nv)
      : SapConstraint<double>(
            {clique, MatrixXd::Zero(num_constraint_equations, clique_nv)}) {}

  // Constructor for a constraint between two cliques.
  TestConstraint(int num_constraint_equations, int first_clique,
                 int first_clique_nv, int second_clique, int second_clique_nv)
      : SapConstraint<double>(
            {first_clique,
             MatrixXd::Zero(num_constraint_equations, first_clique_nv),
             second_clique,
             MatrixXd::Zero(num_constraint_equations, second_clique_nv)}) {}

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
  SapContactProblem<double> problem(time_step, std::move(A), std::move(v_star));
  AddConstraints(&problem);

  EXPECT_EQ(problem.num_constraints(), 5);
  EXPECT_EQ(problem.num_constraint_equations(), 17);

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
  problem.Reset(std::move(A), std::move(v_star));
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
  SapContactProblem<double> problem(time_step, std::move(A), std::move(v_star));
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

/* We test reducing the SapContactProblem. The graph setup sketched below
(having the same semantics as the graph described in AddConstraints())
corresponds to the graph of the contact problem that results from locking DoFs
{0,1,2,3,4,9} in the contact problem produced by AddConstraints(). The indices
in the graph below are the new mapped indices of the reduced problem. The
following tables explain the mapping between original clique and constraint
indices and their corresponding indices in the reduced problem:

┌────────────┬────────────────────┐┌─────────────────────────────────────────┐
│ clique idx │ reduced clique idx ││ constraint idx │ reduced constraint idx │
├────────────│────────────────────┤├────────────────│────────────────────────┤
│     0      │         N/A        ││       0        │           0            │
│     1      │         N/A        ││       1        │          N/A           │
│     2      │          0         ││       2        │           1            │
│     3      │          1         ││       3        │           2            │
└────────────┴────────────────────┘│       4        │           3            │
                                   └────────────────┴────────────────────────┘

Locking these DoFs has the effect of eliminating clique 0 and 1 from the
problem. One constraint from the original problem will be completely removed.
Other constraints that involved one of the eliminated clique now become single
clique constraints:

    0[0, 1, 2, 3](1, 6, 2, 5)
             ┌─┐
             │ │
            ┌┴─┴┐     ┌───┐
            │ 1 │     │ 0 │
            └───┘     └───┘

TODO(joemasterjohn): Make a fixture to consolidate the creation of this problem
and its constraints.
*/
GTEST_TEST(ContactProblem, MakeReduced) {
  const double time_step = 0.01;
  const std::vector<MatrixXd> A{S22, S33, S44, S22};
  const VectorXd v_star = VectorXd::LinSpaced(11, 1.0, 11.0);
  SapContactProblem<double> problem(time_step, std::move(A), std::move(v_star));
  AddConstraints(&problem);

  // Lock all the dofs of clique 0 and 1. Lock velocity index 9, local index 1
  // of clique 3.
  const std::vector<int> locked_indices{0, 1, 2, 3, 4, 9};
  const std::vector<std::vector<int>> clique_locked_indices{
      {0, 1}, {0, 1, 2}, {}, {1}};
  ReducedMapping mapping;
  std::unique_ptr<SapContactProblem<double>> reduced_problem =
      problem.MakeReduced(locked_indices, clique_locked_indices, &mapping);

  EXPECT_EQ(reduced_problem->num_cliques(), 2);
  EXPECT_EQ(reduced_problem->num_constraints(), 4);
  EXPECT_EQ(reduced_problem->num_constraint_equations(), 14);
  // Verify some expected data.
  EXPECT_EQ(reduced_problem->v_star().size(), 5);
  // Two cliques eliminated, so two remain.
  EXPECT_EQ(reduced_problem->dynamics_matrix().size(), 2);
  // Original clique 2 (reduced clique 0) is unaffected.
  EXPECT_EQ(reduced_problem->dynamics_matrix()[0], S44);
  // Original clique 3 (reduced clique 1) has only 1 dof now.
  EXPECT_EQ(reduced_problem->dynamics_matrix()[1].size(), 1);
  EXPECT_EQ(reduced_problem->dynamics_matrix()[1](0, 0), 2);

  const ContactProblemGraph& graph = reduced_problem->graph();
  EXPECT_EQ(graph.num_cliques(), 2);
  EXPECT_EQ(graph.num_constraints(), 4);
  EXPECT_EQ(graph.num_clusters(), 1);
  EXPECT_EQ(graph.num_constraint_equations(), 14);

  /* Clique permutation. We expect the first two cliques to be eliminated. */
  EXPECT_EQ(mapping.clique_permutation.domain_size(), problem.num_cliques());
  EXPECT_EQ(mapping.clique_permutation.permuted_domain_size(), 2);
  EXPECT_FALSE(mapping.clique_permutation.participates(0));
  EXPECT_FALSE(mapping.clique_permutation.participates(1));
  EXPECT_TRUE(mapping.clique_permutation.participates(2));
  EXPECT_TRUE(mapping.clique_permutation.participates(3));

  EXPECT_EQ(mapping.clique_permutation.permuted_index(2), 0);
  EXPECT_EQ(mapping.clique_permutation.permuted_index(3), 1);

  /* Constraint permutation. We expect the second constraint between the two
     eliminated cliques (0 and 1) to be eliminated. */
  EXPECT_EQ(mapping.constraint_permutation.domain_size(),
            problem.num_constraints());
  EXPECT_EQ(mapping.constraint_permutation.permuted_domain_size(), 4);
  EXPECT_TRUE(mapping.constraint_permutation.participates(0));
  EXPECT_FALSE(mapping.constraint_permutation.participates(1));
  EXPECT_TRUE(mapping.constraint_permutation.participates(2));
  EXPECT_TRUE(mapping.constraint_permutation.participates(3));
  EXPECT_TRUE(mapping.constraint_permutation.participates(4));

  EXPECT_EQ(mapping.constraint_permutation.permuted_index(0), 0);
  EXPECT_EQ(mapping.constraint_permutation.permuted_index(2), 1);
  EXPECT_EQ(mapping.constraint_permutation.permuted_index(3), 2);
  EXPECT_EQ(mapping.constraint_permutation.permuted_index(4), 3);

  /* Verify that the each constraint's index mapping is as expected and
     described by the graph. For each constraint, the table lists the original
     problem constraint properties and the reduced problem properties.*/

  /*
    ┌──────────┬───────┬───────────────┬────────────────┬─────────────────┐
    │ problem  │ index │ num_equations │ first_clique() │ second_clique() │
    ├──────────│───────│───────────────│────────────────│─────────────────┤
    │ original │   0   │      1        │       3        │      N/A        │
    │ reduced  │   0   │      1        │       1        │      N/A        │
    └──────────┴───────┴───────────────┴────────────────┴─────────────────┘
  */
  {
    const SapConstraint<double>& c = reduced_problem->get_constraint(0);
    EXPECT_EQ(c.num_cliques(), 1);
    EXPECT_EQ(c.first_clique(), mapping.clique_permutation.permuted_index(3));
    EXPECT_EQ(c.num_constraint_equations(), 1);
    EXPECT_EQ(c.first_clique_jacobian().cols(),
              reduced_problem->num_velocities(c.first_clique()));
  }

  /*
    ┌──────────┬───────┬───────────────┬────────────────┬─────────────────┐
    │ problem  │ index │ num_equations │ first_clique() │ second_clique() │
    ├──────────│───────│───────────────│────────────────│─────────────────┤
    │ original │   2   │      6        │       0        │       3         │
    │ reduced  │   1   │      6        │       1        │      N/A        │
    └──────────┴───────┴───────────────┴────────────────┴─────────────────┘
  */
  {
    const SapConstraint<double>& c = reduced_problem->get_constraint(1);
    EXPECT_EQ(c.num_cliques(), 1);
    EXPECT_EQ(c.first_clique(), mapping.clique_permutation.permuted_index(3));
    EXPECT_EQ(c.num_constraint_equations(), 6);
    EXPECT_EQ(c.first_clique_jacobian().cols(),
              reduced_problem->num_velocities(c.first_clique()));
  }

  /*
    ┌──────────┬───────┬───────────────┬────────────────┬─────────────────┐
    │ problem  │ index │ num_equations │ first_clique() │ second_clique() │
    ├──────────│───────│───────────────│────────────────│─────────────────┤
    │ original │   3   │      2        │       1        │       3         │
    │ reduced  │   2   │      2        │       1        │      N/A        │
    └──────────┴───────┴───────────────┴────────────────┴─────────────────┘
  */
  {
    const SapConstraint<double>& c = reduced_problem->get_constraint(2);
    EXPECT_EQ(c.num_cliques(), 1);
    EXPECT_EQ(c.first_clique(), mapping.clique_permutation.permuted_index(3));
    EXPECT_EQ(c.num_constraint_equations(), 2);
    EXPECT_EQ(c.first_clique_jacobian().cols(),
              reduced_problem->num_velocities(c.first_clique()));
  }

  /*
    ┌──────────┬───────┬───────────────┬────────────────┬─────────────────┐
    │ problem  │ index │ num_equations │ first_clique() │ second_clique() │
    ├──────────│───────│───────────────│────────────────│─────────────────┤
    │ original │   4   │      5        │       0        │       3         │
    │ reduced  │   3   │      5        │       1        │      N/A        │
    └──────────┴───────┴───────────────┴────────────────┴─────────────────┘
  */
  {
    const SapConstraint<double>& c = reduced_problem->get_constraint(3);
    EXPECT_EQ(c.num_cliques(), 1);
    EXPECT_EQ(c.first_clique(), mapping.clique_permutation.permuted_index(3));
    EXPECT_EQ(c.num_constraint_equations(), 5);
    EXPECT_EQ(c.first_clique_jacobian().cols(),
              reduced_problem->num_velocities(c.first_clique()));
  }
}

}  // namespace
}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
