#include "drake/multibody/contact_solvers/sap/sap_contact_problem.h"

#include <limits>
#include <memory>
#include <set>
#include <utility>
#include <vector>

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
// Only indices and constraint sizes matter for the unit tests in this file.
template <typename T = double>
class TestConstraint final : public SapConstraint<T> {
 public:
  // Constructor for a constraint on a single clique.
  // No objects are registered.
  TestConstraint(int num_constraint_equations, int clique, int clique_nv)
      : SapConstraint<T>(
            {clique, MatrixX<T>::Ones(num_constraint_equations, clique_nv)},
            {}) {}

  // Constructor for a constraint between two cliques.
  // Registers objects with index first_clique and second_clique, for testing.
  TestConstraint(int num_constraint_equations, int first_clique,
                 int first_clique_nv, int second_clique, int second_clique_nv)
      : SapConstraint<T>(
            {first_clique,
             MatrixX<T>::Ones(num_constraint_equations, first_clique_nv),
             second_clique,
             MatrixX<T>::Ones(num_constraint_equations, second_clique_nv)},
            {}) {}

  // N.B no-op overloads to allow us compile this testing constraint. These
  // methods are only tested for specific derived classes, not in this file.
  std::unique_ptr<AbstractValue> DoMakeData(
      const T&, const Eigen::Ref<const VectorX<T>>&) const final {
    return nullptr;
  }
  void DoCalcData(const Eigen::Ref<const VectorX<T>>&,
                  AbstractValue*) const final {}
  T DoCalcCost(const AbstractValue&) const final { return 0.0; }
  void DoCalcImpulse(const AbstractValue&, EigenPtr<VectorX<T>>) const final {}
  void DoCalcCostHessian(const AbstractValue&, MatrixX<T>*) const final {}

 private:
  TestConstraint(const TestConstraint&) = default;

  // These functions to report impulses are used to test that
  // SapContactProblem::CalcConstraintMultibodyForces() correctly accumulates
  // constraint contributions. Therefore their functional form must stay in sync
  // with testing code in ContactProblem__CalcConstraintMultibodyForces.
  void DoAccumulateGeneralizedImpulses(
      int c, const Eigen::Ref<const VectorX<T>>& gamma,
      EigenPtr<VectorX<T>> tau) const final {
    *tau += VectorXd::Ones(this->num_velocities(c)) * gamma.sum();
  }

  void DoAccumulateSpatialImpulses(int o,
                                   const Eigen::Ref<const VectorX<T>>& gamma,
                                   SpatialForce<T>* F) const final {
    Vector6<T> v6 = o * Vector6<T>::Ones() * gamma.sum();
    *F += SpatialForce<T>(v6);
  }

  std::unique_ptr<SapConstraint<T>> DoClone() const final {
    return std::unique_ptr<TestConstraint<T>>(new TestConstraint<T>(*this));
  }

  std::unique_ptr<SapConstraint<double>> DoToDouble() const final {
    if (this->num_cliques() == 1) {
      return std::make_unique<TestConstraint<double>>(
          this->num_constraint_equations(), this->first_clique(),
          this->num_velocities(0));
    }
    return std::make_unique<TestConstraint<double>>(
        this->num_constraint_equations(), this->first_clique(),
        this->num_velocities(0), this->second_clique(),
        this->num_velocities(1));
  }
};

// Test construction of an empty problem.
GTEST_TEST(ContactProblem, EmptyProblem) {
  const double dt = 2.5e-4;
  SapContactProblem<double> problem(dt, std::vector<MatrixXd>(), VectorXd());
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

  const std::vector<int> expected_velocities_start{0, 2, 5, 9};
  for (int i = 0; i < problem.num_cliques(); ++i) {
    EXPECT_EQ(problem.velocities_start(i), expected_velocities_start[i]);
  }
}

// Unit test that AddConstraint() throws for the right reasons when arguments
// are wrong.
GTEST_TEST(ContactProblem, AddConstraintWithWrongArguments) {
  const double time_step = 0.01;
  const std::vector<MatrixXd> A{S22, S33, S44, S22, S00};
  const VectorXd v_star = VectorXd::LinSpaced(11, 1.0, 11.0);
  SapContactProblem<double> problem(time_step, A, v_star);

  DRAKE_EXPECT_THROWS_MESSAGE(
      problem.AddConstraint(std::make_unique<TestConstraint<double>>(
          3 /* num_equations */, 5 /* (wrong) first_clique */,
          2 /* first_clique_nv */, 1 /* second_clique */,
          3 /* second_clique_nv */)),
      "First clique index must be strictly lower than num_cliques\\(\\)");
  DRAKE_EXPECT_THROWS_MESSAGE(
      problem.AddConstraint(std::make_unique<TestConstraint<double>>(
          3 /* num_equations */, 0 /* first_clique */, 2 /* first_clique_nv */,
          5 /* (wrong) second_clique */, 3 /* second_clique_nv */)),
      "Second clique index must be strictly lower than num_cliques\\(\\)");
  DRAKE_EXPECT_THROWS_MESSAGE(
      problem.AddConstraint(std::make_unique<TestConstraint<double>>(
          3 /* num_equations */, 0 /* first_clique */,
          3 /* (wrong) first_clique_nv */, 1 /* second_clique */,
          3 /* second_clique_nv */)),
      ".* does not match the number of velocities in this problem for "
      "the first clique.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      problem.AddConstraint(std::make_unique<TestConstraint<double>>(
          3 /* num_equations */, 0 /* first_clique */, 2 /* first_clique_nv */,
          1 /* second_clique */, 6 /* (wrong) second_clique_nv */)),
      ".* does not match the number of velocities in this problem for "
      "the second clique.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      problem.AddConstraint(std::make_unique<TestConstraint<double>>(
          3 /* num_equations */, 0 /* first_clique */, 2 /* first_clique_nv */,
          4 /* (empty) second_clique */, 0 /* second_clique_nv */)),
      ".*Adding constraint.*zero.*velocities.*");
}

/* We test ContactProblem with the graph setup sketched below where each
 box corresponds to a clique (a node in the graph) and edges correspond to a
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

 In this test setup, we consider there is a physical object per clique, and
 therefore we have four objects.
*/
template <typename T = double>
void AddConstraints(SapContactProblem<T>* problem) {
  auto add_and_verify_access =
      [problem](std::unique_ptr<TestConstraint<T>> owned_constraint) {
        const int expected_index = problem->num_constraints();
        const TestConstraint<T>& constraint = *owned_constraint;
        EXPECT_EQ(problem->AddConstraint(std::move(owned_constraint)),
                  expected_index);
        EXPECT_EQ(&problem->get_constraint(expected_index), &constraint);
      };

  // In this test, constraints will register one object per clique for testing.
  // Therefore we know we have four objects.
  problem->set_num_objects(4);

  add_and_verify_access(std::make_unique<TestConstraint<T>>(
      1 /* num_equations */, 3 /* clique */, 2 /* clique_nv */));
  add_and_verify_access(std::make_unique<TestConstraint<T>>(
      3 /* num_equations */, 0 /* first_clique */, 2 /* first_clique_nv */,
      1 /* second_clique */, 3 /* second_clique_nv */));
  add_and_verify_access(std::make_unique<TestConstraint<T>>(
      6 /* num_equations */, 0 /* first_clique */, 2 /* first_clique_nv */,
      3 /* second_clique */, 2 /* second_clique_nv */));
  add_and_verify_access(std::make_unique<TestConstraint<T>>(
      2 /* num_equations */, 1 /* first_clique */, 3 /* first_clique_nv */,
      3 /* second_clique */, 2 /* second_clique_nv */));
  add_and_verify_access(std::make_unique<TestConstraint<T>>(
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
  EXPECT_EQ(problem.num_objects(), 4);

  // Expected constraint equation indicies.
  const std::vector<int> constraint_equations_start_expected{0, 1, 4, 10, 12};
  for (int i = 0; i < problem.num_constraints(); ++i) {
    EXPECT_EQ(problem.constraint_equations_start(i),
              constraint_equations_start_expected[i]);
  }

  // Verify graph for this problem.
  const ContactProblemGraph& graph = problem.graph();
  EXPECT_EQ(graph.num_cliques(), 4);
  EXPECT_EQ(graph.num_constraints(), 5);
  EXPECT_EQ(graph.num_clusters(), 4);
  EXPECT_EQ(graph.num_constraint_equations(), 17);
}

GTEST_TEST(ContactProblem, Clone) {
  const double time_step = 0.01;
  std::vector<MatrixXd> A{S22, S33, S44, S22};
  const VectorXd v_star = VectorXd::LinSpaced(11, 1.0, 11.0);
  SapContactProblem<double> problem(time_step, std::move(A), v_star);
  AddConstraints(&problem);

  auto verify_clone = [&problem](const auto& clone) {
    EXPECT_EQ(clone.num_cliques(), problem.num_cliques());
    EXPECT_EQ(clone.num_constraints(), problem.num_constraints());
    EXPECT_EQ(clone.num_constraint_equations(),
              problem.num_constraint_equations());

    // Verify graph for this problem.
    const ContactProblemGraph& graph = clone.graph();
    EXPECT_EQ(graph.num_cliques(), 4);
    EXPECT_EQ(graph.num_constraints(), 5);
    EXPECT_EQ(graph.num_clusters(), 4);
    EXPECT_EQ(graph.num_constraint_equations(), 17);
  };

  std::unique_ptr<SapContactProblem<double>> clone = problem.Clone();
  verify_clone(*clone);

  // Make the same problem but with T = AutoDiffXd.
  std::vector<MatrixX<AutoDiffXd>> A_ad{S22, S33, S44, S22};
  VectorX<AutoDiffXd> v_star_ad = v_star;
  SapContactProblem<AutoDiffXd> problem_ad(time_step, std::move(A_ad),
                                           std::move(v_star_ad));
  AddConstraints(&problem_ad);

  // Unit test ToDouble().
  std::unique_ptr<SapContactProblem<double>> clone_from_ad =
      problem_ad.ToDouble();
  verify_clone(*clone_from_ad);
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

  // Lock all the dofs of clique 0 and 1. Lock velocity index 9, local index 0
  // of clique 3.
  const std::vector<int> unknown_indices{5, 6, 7, 8, 10};
  const std::vector<int> known_indices{0, 1, 2, 3, 4, 9};
  const std::vector<std::vector<int>> clique_known_indices{
      {0, 1}, {0, 1, 2}, {}, {0}};
  ReducedMapping mapping;
  std::unique_ptr<SapContactProblem<double>> reduced_problem =
      problem.MakeReduced(known_indices, clique_known_indices, &mapping);

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

  /* Velocity permutation. Expect velocities 5, 6, 7, 8, 10 to participate. */
  EXPECT_EQ(mapping.velocity_permutation.domain_size(),
            problem.num_velocities());
  EXPECT_EQ(mapping.velocity_permutation.permuted_domain_size(), 5);
  for (int i : unknown_indices) {
    EXPECT_TRUE(mapping.velocity_permutation.participates(i));
  }
  for (int i : known_indices) {
    EXPECT_FALSE(mapping.velocity_permutation.participates(i));
  }
  for (int i = 0; i < ssize(unknown_indices); ++i) {
    EXPECT_EQ(mapping.velocity_permutation.permuted_index(unknown_indices[i]),
              i);
  }

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
  EXPECT_EQ(mapping.constraint_equation_permutation.domain_size(),
            problem.num_constraint_equations());
  EXPECT_EQ(mapping.constraint_equation_permutation.permuted_domain_size(), 14);

  int constraint_equation_index = 0;
  int reduced_constraint_equation_index = 0;
  for (int i = 0; i < problem.num_constraints(); ++i) {
    const SapConstraint<double>& c = problem.get_constraint(i);
    for (int j = 0; j < c.num_constraint_equations(); ++j) {
      if (i == 1) {
        EXPECT_FALSE(mapping.constraint_equation_permutation.participates(
            constraint_equation_index + j));
      } else {
        EXPECT_TRUE(mapping.constraint_equation_permutation.participates(
            constraint_equation_index + j));
        EXPECT_EQ(mapping.constraint_equation_permutation.permuted_index(
                      constraint_equation_index + j),
                  reduced_constraint_equation_index + j);
      }
    }
    if (i != 1) {
      reduced_constraint_equation_index += c.num_constraint_equations();
    }
    constraint_equation_index += c.num_constraint_equations();
  }

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

GTEST_TEST(ContactProblem, ExpandContactSolverResults) {
  const double time_step = 0.01;
  const std::vector<MatrixXd> A{S22, S33, S44, S22};
  const VectorXd v_star = VectorXd::LinSpaced(11, 1.0, 11.0);
  SapContactProblem<double> problem(time_step, std::move(A), std::move(v_star));
  AddConstraints(&problem);

  // Lock all the dofs of clique 0 and 1. Lock velocity index 9, local index 0
  // of clique 3.
  const std::vector<int> unknown_indices{5, 6, 7, 8, 10};
  const std::vector<int> known_indices{0, 1, 2, 3, 4, 9};
  const std::vector<std::vector<int>> clique_known_indices{
      {0, 1}, {0, 1, 2}, {}, {0}};
  ReducedMapping mapping;
  std::unique_ptr<SapContactProblem<double>> reduced_problem =
      problem.MakeReduced(known_indices, clique_known_indices, &mapping);

  // Set up some dummy results.
  SapSolverResults<double> reduced_results;
  reduced_results.Resize(reduced_problem->num_velocities(),
                         reduced_problem->num_constraint_equations());
  reduced_results.v =
      VectorXd::LinSpaced(reduced_problem->num_velocities(), 1.0,
                          reduced_problem->num_velocities());
  reduced_results.gamma =
      VectorXd::LinSpaced(reduced_problem->num_constraint_equations(), 1.0,
                          reduced_problem->num_constraint_equations());
  reduced_results.vc =
      VectorXd::LinSpaced(reduced_problem->num_constraint_equations(), 1.0,
                          reduced_problem->num_constraint_equations());
  reduced_results.j =
      VectorXd::LinSpaced(reduced_problem->num_velocities(), 1.0,
                          reduced_problem->num_velocities());

  // Expand the results to the original problem.
  SapSolverResults<double> results;
  problem.ExpandContactSolverResults(mapping, reduced_results, &results);

  VectorX<double> v_expected = v_star;
  VectorX<double> gamma_expected =
      VectorX<double>::Zero(problem.num_constraint_equations());
  VectorX<double> vc_expected =
      VectorX<double>::Zero(problem.num_constraint_equations());
  VectorX<double> j_expected = VectorX<double>::Zero(problem.num_velocities());

  for (int i = 0; i < ssize(unknown_indices); ++i) {
    v_expected[unknown_indices[i]] = i + 1;
    j_expected[unknown_indices[i]] = i + 1;
  }

  EXPECT_TRUE(CompareMatrices(results.v, v_expected));
  EXPECT_TRUE(CompareMatrices(results.j, j_expected));

  // All constraint Jacobians set to Ones() so each constraint velocity should
  // be the sum of the segment of v_star corresponding to its cliques.
  for (int i = 0; i < problem.num_constraints(); ++i) {
    const SapConstraint<double>& c = problem.get_constraint(i);
    double sum = v_star
                     .segment(problem.velocities_start(c.first_clique()),
                              problem.num_velocities(c.first_clique()))
                     .sum();
    if (c.num_cliques() > 1) {
      sum += v_star
                 .segment(problem.velocities_start(c.second_clique()),
                          problem.num_velocities(c.second_clique()))
                 .sum();
    }
    vc_expected.segment(problem.constraint_equations_start(i),
                        c.num_constraint_equations()) =
        sum * VectorXd::Ones(c.num_constraint_equations());
  }

  int equation_index = 0;
  int reduced_equation_index = 0;
  for (int i = 0; i < problem.num_constraints(); ++i) {
    const SapConstraint<double>& c = problem.get_constraint(i);
    if (mapping.constraint_equation_permutation.participates(equation_index)) {
      for (int j = 0; j < c.num_constraint_equations(); ++j) {
        gamma_expected[equation_index] = reduced_equation_index + 1;
        vc_expected[equation_index] = reduced_equation_index + 1;
        ++equation_index;
        ++reduced_equation_index;
      }
    } else {
      equation_index += c.num_constraint_equations();
    }
  }

  EXPECT_TRUE(CompareMatrices(results.gamma, gamma_expected));
  EXPECT_TRUE(CompareMatrices(results.vc, vc_expected));
}

GTEST_TEST(ContactProblem, CalcConstraintMultibodyForces) {
  const double time_step = 0.01;
  const std::vector<MatrixXd> A{S22, S33, S44, S22};
  const VectorXd v_star = VectorXd::LinSpaced(11, 1.0, 11.0);
  SapContactProblem<double> problem(time_step, std::move(A), std::move(v_star));
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
  // tau_partial_expected only accumulates generalized forces for constraints
  // with indexes in start <= i <= end.
  const int start = 1;
  const int end = 2;
  VectorXd tau_partial_expected = VectorXd::Zero(problem.num_velocities());
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
      if (start <= i && i <= end) {
        tau_partial_expected
            .segment(problem.velocities_start(clique),
                     problem.num_velocities(clique))
            .array() += gamma_c.sum();
      }
    }
    offset += ne;
  }
  tau_expected /= problem.time_step();
  tau_partial_expected /= problem.time_step();

  EXPECT_TRUE(CompareMatrices(tau, tau_expected,
                              std::numeric_limits<double>::epsilon(),
                              MatrixCompareType::relative));

  // Verify computation of generalized forces for only a set of the constraints.
  problem.CalcConstraintGeneralizedForces(gamma, start, end, &tau);
  EXPECT_TRUE(CompareMatrices(tau, tau_partial_expected,
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
