#include <cmath>
#include <numeric>

#include <gtest/gtest.h>

#include "drake/examples/planar_gripper/dev/contact_metric.h"
#include "drake/examples/planar_gripper/dev/contact_mode.h"
#include "drake/examples/planar_gripper/dev/contact_search.h"

namespace drake {
namespace examples {
namespace planar_gripper {

GTEST_TEST(ContactModeTest, ModeGeneration) {
  // 5 contact points (fingers) and 10 faces
  std::vector<int> contact_points(5);
  std::vector<int> contact_faces(10);
  std::iota(contact_points.begin(), contact_points.end(), 1);
  std::iota(contact_faces.begin(), contact_faces.end(), 1);

  // generate all contact modes
  auto cms =
      ContactMode::GenerateAllContactModes(contact_points, contact_faces);

  // check number of modes
  EXPECT_EQ(cms.size(), std::pow(contact_faces.size(), contact_points.size()));
}

GTEST_TEST(ContactModeTest, ModeConnection) {
  ContactMode cm1;
  ContactMode cm2;

  // connects mode 2 to mode 1
  cm1.AddConnectedMode(&cm2);

  // check that mode 2 is in mode 1's neighbors
  EXPECT_NE(cm1.get_connected_modes().find(&cm2),
            cm1.get_connected_modes().end());

  // check that the converse isn't true
  EXPECT_EQ(cm2.get_connected_modes().size(), 0);
}

GTEST_TEST(ContactMetricTest, MapBasedContactMetric) {
  ContactMode cm0({std::make_pair(0, 0)});
  ContactMode cm1({std::make_pair(1, 1)});
  ContactMode cm2({std::make_pair(2, 2)});

  MapBasedContactMetric<int> metric1;

  metric1.SetValue(cm1, cm2, 2);
  EXPECT_EQ(metric1.Eval(cm1, cm2), 2);
  EXPECT_EQ(metric1.Eval(cm0, cm2), 0);

  metric1.set_default_value(5);
  EXPECT_EQ(metric1.Eval(cm1, cm2), 2);
  EXPECT_EQ(metric1.Eval(cm0, cm2), 5);

  MapBasedContactMetric<double> metric2;

  metric2.SetValue(cm1, cm2, 2.);
  EXPECT_EQ(metric2.Eval(cm1, cm2), 2.);
  EXPECT_EQ(metric2.Eval(cm0, cm2), 0.);

  metric2.set_default_value(5.);
  EXPECT_EQ(metric2.Eval(cm1, cm2), 2.);
  EXPECT_EQ(metric2.Eval(cm0, cm2), 5.);
}

class ContactModeSearchTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // builds a small graph for testing
    ContactMode cm0({std::make_pair(0, 0)});
    ContactMode cm1({std::make_pair(1, 1)});
    ContactMode cm2({std::make_pair(2, 2)});
    ContactMode cm3({std::make_pair(3, 3)});
    ContactMode cm4({std::make_pair(4, 4)});
    ContactMode cm5({std::make_pair(5, 5)});

    graph_ = {cm0, cm1, cm2, cm3, cm4, cm5};
    graph_[0].AddConnectedMode(&graph_[1]);
    graph_[1].AddConnectedMode(&graph_[0]);
    graph_[1].AddConnectedMode(&graph_[2]);
    graph_[1].AddConnectedMode(&graph_[3]);
    graph_[2].AddConnectedMode(&graph_[0]);
    graph_[3].AddConnectedMode(&graph_[0]);
    graph_[3].AddConnectedMode(&graph_[4]);
    graph_[4].AddConnectedMode(&graph_[1]);
    graph_[5].AddConnectedMode(&graph_[4]);

    chain_graph_ = {cm0, cm1, cm2, cm3, cm4, cm5};
    chain_graph_[0].AddConnectedMode(&chain_graph_[1]);
    chain_graph_[1].AddConnectedMode(&chain_graph_[2]);
    chain_graph_[2].AddConnectedMode(&chain_graph_[3]);
    chain_graph_[3].AddConnectedMode(&chain_graph_[4]);
    chain_graph_[4].AddConnectedMode(&chain_graph_[5]);
    chain_graph_[0].AddConnectedMode(&chain_graph_[3]);
    chain_graph_[3].AddConnectedMode(&chain_graph_[5]);
  }

  void CheckPath(const std::vector<const ContactMode*>& path,
                 const std::vector<const ContactMode*>& expected_path) {
    ASSERT_EQ(path.size(), expected_path.size());

    for (uint i = 0; i < path.size(); i++) {
      EXPECT_EQ(path[i], expected_path[i]);
    }
  }

  std::vector<ContactMode> graph_;
  std::vector<ContactMode> chain_graph_;
};

TEST_F(ContactModeSearchTest, DepthFirstSearch) {
  // expected 0 -> 1 -> 3 -> 4
  std::vector<const ContactMode*> expected_path = {&graph_[0], &graph_[1],
                                                   &graph_[3], &graph_[4]};
  auto path = contact_search::DepthFirstSearch(graph_[0], graph_[4]);
  CheckPath(path, expected_path);

  expected_path = {&chain_graph_[0], &chain_graph_[1], &chain_graph_[2],
                   &chain_graph_[3], &chain_graph_[4], &chain_graph_[5]};
  path = contact_search::DepthFirstSearch(chain_graph_[0], chain_graph_[5]);
  CheckPath(path, expected_path);

  // expected empty
  expected_path.clear();
  path = contact_search::DepthFirstSearch(graph_[2], graph_[5]);
  CheckPath(path, expected_path);
}

TEST_F(ContactModeSearchTest, BreadthFirstSearch) {
  // expected 0 -> 1 -> 3 -> 4
  std::vector<const ContactMode*> expected_path = {&graph_[0], &graph_[1],
                                                   &graph_[3], &graph_[4]};
  auto path = contact_search::BreadthFirstSearch(graph_[0], graph_[4]);
  CheckPath(path, expected_path);

  expected_path = {&chain_graph_[0], &chain_graph_[3], &chain_graph_[5]};
  path = contact_search::BreadthFirstSearch(chain_graph_[0], chain_graph_[5]);
  CheckPath(path, expected_path);

  // expected empty
  expected_path.clear();
  path = contact_search::BreadthFirstSearch(graph_[2], graph_[5]);
  CheckPath(path, expected_path);
}

TEST_F(ContactModeSearchTest, AStarSearch) {
  // expected 0 -> 1 -> 3 -> 4
  MapBasedContactMetric<double> distance1;
  MapBasedContactMetric<double> heuristic1;
  std::vector<const ContactMode*> expected_path = {&graph_[0], &graph_[1],
                                                   &graph_[3], &graph_[4]};
  auto path = contact_search::AStarSearch<double>(graph_[0], graph_[4],
                                                  distance1, heuristic1);
  CheckPath(path, expected_path);

  // expected 1 -> 3 -> 0
  MapBasedContactMetric<double> distance2({
      {{graph_[1].get_id(), graph_[0].get_id()}, 10.},
      {{graph_[1].get_id(), graph_[2].get_id()}, 2.},
      {{graph_[1].get_id(), graph_[3].get_id()}, 2.},
      {{graph_[2].get_id(), graph_[0].get_id()}, 5.},
      {{graph_[3].get_id(), graph_[0].get_id()}, 1.},
  });
  distance2.set_default_value(2.);
  MapBasedContactMetric<double> heuristic2;
  heuristic2.SetValue(graph_[1], graph_[2], .5);
  heuristic2.set_default_value(1.);
  expected_path = {&graph_[1], &graph_[3], &graph_[0]};
  path =
      contact_search::AStarSearch(graph_[1], graph_[0], distance2, heuristic2);
  CheckPath(path, expected_path);

  // expected empty
  expected_path.clear();
  path =
      contact_search::AStarSearch(graph_[2], graph_[5], distance2, heuristic2);
  CheckPath(path, expected_path);
}

}  // namespace planar_gripper
}  // namespace examples
}  // namespace drake
