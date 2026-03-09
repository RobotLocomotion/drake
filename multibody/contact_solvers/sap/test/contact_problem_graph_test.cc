#include "drake/multibody/contact_solvers/sap/contact_problem_graph.h"

#include <vector>

#include <gtest/gtest.h>

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {
namespace {

/* We test ContactProblemGraph with the graph setup sketched below where each
 box corresponds to a clique (a node in the graph) an edges correspond to a
 cluster of constraints connecting one or more cliques. As documented in
 ContactProblemGraph's documentation, clusters are created (and assigned a
 non-negative increasing index) the first time a constraint between two cliques
 (or just one) is added with a call to AddConstraint(). The graph below is then
 the expected graph for the set of AddConstraint() calls in the order listed in
 this test.

 The labels for the clusters below correspond to:
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
class ContactGraphTest : public ::testing::Test {
 public:
  static bool CompareClusters(const ContactProblemGraph::ConstraintCluster& a,
                              const ContactProblemGraph::ConstraintCluster& b) {
    return a.cliques() == b.cliques() &&
           a.num_constraints() == b.num_constraints() &&
           a.num_total_constraint_equations() ==
               b.num_total_constraint_equations() &&
           a.constraint_index() == b.constraint_index() &&
           a.constraint_num_equations() == b.constraint_num_equations();
  }

  static ContactProblemGraph MakeGraph() {
    ContactProblemGraph graph(4);
    AddGraphConstraints(&graph);
    return graph;
  }

  static void AddGraphConstraints(ContactProblemGraph* graph) {
    EXPECT_EQ(graph->AddConstraint(3, 1), 0);     // Defines cluster 0.
    EXPECT_EQ(graph->AddConstraint(0, 1, 3), 1);  // Defines cluster 1.
    EXPECT_EQ(graph->AddConstraint(0, 3, 6), 2);  // Defines cluster 2.
    EXPECT_EQ(graph->AddConstraint(1, 3, 2), 3);  // Defines cluster 3.
    EXPECT_EQ(graph->AddConstraint(3, 0, 5), 4);  // Cluster 2 already existed.
  }

  // Verifies for the expected graph created with MakeGraph().
  static void VerifyForExpectedGraph(const ContactProblemGraph& graph) {
    EXPECT_EQ(graph.num_cliques(), 4);
    EXPECT_EQ(graph.num_constraints(), 5);
    EXPECT_EQ(graph.num_clusters(), 4);
    EXPECT_EQ(graph.num_constraint_equations(), 17);

    // Expected clusters as documented in the schematic of the graph above.
    // Cluster | cliques pair  |  Constraints | Num. constraint equations.
    //    0    |    (3, 3)     |   0          |  1
    //    1    |    (0, 1)     |   1          |  3
    //    2    |    (0, 3)     |   2,4        | 11
    //    3    |    (1, 3)     |   3          |  2
    const auto cluster0 =
        ContactProblemGraph::ConstraintCluster(MakeSortedPair(3, 3))
            .AddConstraint(0, 1);
    const auto cluster1 =
        ContactProblemGraph::ConstraintCluster(MakeSortedPair(0, 1))
            .AddConstraint(1, 3);
    const auto cluster2 =
        ContactProblemGraph::ConstraintCluster(MakeSortedPair(0, 3))
            .AddConstraint(2, 6)
            .AddConstraint(4, 5);
    const auto cluster3 =
        ContactProblemGraph::ConstraintCluster(MakeSortedPair(1, 3))
            .AddConstraint(3, 2);

    // Verify clusters were created as expected in the diagram above:
    EXPECT_TRUE(CompareClusters(graph.get_cluster(0), cluster0));
    EXPECT_TRUE(CompareClusters(graph.get_cluster(1), cluster1));
    EXPECT_TRUE(CompareClusters(graph.get_cluster(2), cluster2));
    EXPECT_TRUE(CompareClusters(graph.get_cluster(3), cluster3));

    // Verify the expected vector of clusters.
    // N.B. We rely on the fact we tested get_cluster() above.
    for (int i = 0; i < graph.num_clusters(); ++i) {
      CompareClusters(graph.clusters()[i], graph.get_cluster(i));
    }

    // Verify participating cliques.
    const PartialPermutation& p = graph.participating_cliques();
    EXPECT_EQ(p.domain_size(), graph.num_cliques());
    EXPECT_EQ(p.permuted_domain_size(), 3);
    // Participating cliques are indexed in the order added to the problem.
    std::vector<int> expected_p = {1, 2, -1, 0};
    EXPECT_EQ(p.permutation(), expected_p);
  }
};

TEST_F(ContactGraphTest, EmptyConstruction) {
  ContactProblemGraph graph;
  EXPECT_EQ(graph.num_cliques(), 0);
  EXPECT_EQ(graph.num_constraints(), 0);
  EXPECT_EQ(graph.num_constraint_equations(), 0);
}

TEST_F(ContactGraphTest, ResetGraph) {
  ContactProblemGraph graph = MakeGraph();
  VerifyForExpectedGraph(graph);
  // Reset graph.
  graph.ResetNumCliques(4);
  EXPECT_EQ(graph.num_cliques(), 4);
  EXPECT_EQ(graph.num_constraints(), 0);
  EXPECT_EQ(graph.num_constraint_equations(), 0);
  // Rebuild again and verify we get the expected graph again.
  AddGraphConstraints(&graph);
  VerifyForExpectedGraph(graph);
}

}  // namespace
}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
