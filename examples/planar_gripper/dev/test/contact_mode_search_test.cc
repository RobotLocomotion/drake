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
      ContactMode::generate_all_contact_modes(contact_points, contact_faces);

  // check number of modes
  EXPECT_EQ(cms.size(), std::pow(contact_faces.size(), contact_points.size()));
}

GTEST_TEST(ContactModeTest, ModeConnection) {
  ContactMode cm1;
  ContactMode cm2;

  // connects mode 2 to mode 1
  cm1.add_connected_mode(&cm2);

  // check that mode 2 is in mode 1's neighbors
  auto cm_connected = cm1.get_connected_modes()[0];
  EXPECT_EQ(cm_connected, &cm2);

  // check that the converse isn't true
  auto connected_modes = cm2.get_connected_modes();
  EXPECT_EQ(connected_modes.size(), 0);
}

GTEST_TEST(ContactMetricTest, MapBasedContactMetric) {
  ContactMode cm0({{0, 0}});
  ContactMode cm1({{1, 1}});
  ContactMode cm2({{2, 2}});

  MapBasedContactMetric<int> metric1;

  metric1.add_value(&cm1, &cm2, 2);
  EXPECT_EQ(metric1.eval(&cm1, &cm2), 2);
  EXPECT_EQ(metric1.eval(&cm0, &cm2), 0);

  metric1.set_default_value(5);
  EXPECT_EQ(metric1.eval(&cm1, &cm2), 2);
  EXPECT_EQ(metric1.eval(&cm0, &cm2), 5);

  MapBasedContactMetric<double> metric2;

  metric2.add_value(&cm1, &cm2, 2.);
  EXPECT_EQ(metric2.eval(&cm1, &cm2), 2.);
  EXPECT_EQ(metric2.eval(&cm0, &cm2), 0.);

  metric2.set_default_value(5.);
  EXPECT_EQ(metric2.eval(&cm1, &cm2), 2.);
  EXPECT_EQ(metric2.eval(&cm0, &cm2), 5.);
}

class ContactModeSearchTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // builds a small graph for testing
    ContactMode cm0({{0, 0}});
    ContactMode cm1({{1, 1}});
    ContactMode cm2({{2, 2}});
    ContactMode cm3({{3, 3}});
    ContactMode cm4({{4, 4}});
    ContactMode cm5({{5, 5}});

    graph_ = {cm0, cm1, cm2, cm3, cm4, cm5};
    graph_[0].add_connected_mode(&graph_[1]);
    graph_[1].add_connected_mode(&graph_[0]);
    graph_[1].add_connected_mode(&graph_[2]);
    graph_[1].add_connected_mode(&graph_[3]);
    graph_[2].add_connected_mode(&graph_[0]);
    graph_[3].add_connected_mode(&graph_[0]);
    graph_[3].add_connected_mode(&graph_[4]);
    graph_[4].add_connected_mode(&graph_[1]);
    graph_[5].add_connected_mode(&graph_[4]);

    chain_graph_ = {cm0, cm1, cm2, cm3, cm4, cm5};
    chain_graph_[0].add_connected_mode(&chain_graph_[1]);
    chain_graph_[1].add_connected_mode(&chain_graph_[2]);
    chain_graph_[2].add_connected_mode(&chain_graph_[3]);
    chain_graph_[3].add_connected_mode(&chain_graph_[4]);
    chain_graph_[4].add_connected_mode(&chain_graph_[5]);
    chain_graph_[0].add_connected_mode(&chain_graph_[3]);
    chain_graph_[3].add_connected_mode(&chain_graph_[5]);
  }

  void check_path(std::vector<ContactMode*> path,
                  std::vector<ContactMode*> expected_path) {
    ASSERT_EQ(path.size(), expected_path.size());

    for (uint i = 0; i < path.size(); i++) {
      EXPECT_EQ(path[i], expected_path[i]);
    }
  };

  std::vector<ContactMode> graph_;
  std::vector<ContactMode> chain_graph_;
};

TEST_F(ContactModeSearchTest, DepthFirstSearch) {
  // expected 0 -> 1 -> 3 -> 4
  std::vector<ContactMode*> expected_path = {&graph_[0], &graph_[1], &graph_[3],
                                             &graph_[4]};
  auto path = contact_search::depth_first_search(&graph_[0], &graph_[4]);
  check_path(path, expected_path);

  expected_path = {&chain_graph_[0], &chain_graph_[1], &chain_graph_[2],
                   &chain_graph_[3], &chain_graph_[4], &chain_graph_[5]};
  path = contact_search::depth_first_search(&chain_graph_[0], &chain_graph_[5]);
  check_path(path, expected_path);

  // expected empty
  expected_path.clear();
  path = contact_search::depth_first_search(&graph_[2], &graph_[5]);
  check_path(path, expected_path);
}

TEST_F(ContactModeSearchTest, BreadthFirstSearch) {
  // expected 0 -> 1 -> 3 -> 4
  std::vector<ContactMode*> expected_path = {&graph_[0], &graph_[1], &graph_[3],
                                             &graph_[4]};
  auto path = contact_search::breadth_first_search(&graph_[0], &graph_[4]);
  check_path(path, expected_path);

  expected_path = {&chain_graph_[0], &chain_graph_[3], &chain_graph_[5]};
  path =
      contact_search::breadth_first_search(&chain_graph_[0], &chain_graph_[5]);
  check_path(path, expected_path);

  // expected empty
  expected_path.clear();
  path = contact_search::breadth_first_search(&graph_[2], &graph_[5]);
  check_path(path, expected_path);
}

TEST_F(ContactModeSearchTest, AStarSearch) {
  // expected 0 -> 1 -> 3 -> 4
  MapBasedContactMetric<double> distance1;
  MapBasedContactMetric<double> heuristic1;
  std::vector<ContactMode*> expected_path = {&graph_[0], &graph_[1], &graph_[3],
                                             &graph_[4]};
  auto path = contact_search::a_star_search(&graph_[0], &graph_[4], distance1,
                                            heuristic1);
  check_path(path, expected_path);

  // expected 1 -> 3 -> 0
  MapBasedContactMetric<double> distance2({
      {{&graph_[1], &graph_[0]}, 10.},
      {{&graph_[1], &graph_[2]}, 2.},
      {{&graph_[1], &graph_[3]}, 2.},
      {{&graph_[2], &graph_[0]}, 5.},
      {{&graph_[3], &graph_[0]}, 1.},
  });
  distance2.set_default_value(2.);
  MapBasedContactMetric<double> heuristic2;
  heuristic2.add_value(&graph_[1], &graph_[2], .5);
  heuristic2.set_default_value(1.);
  expected_path = {&graph_[1], &graph_[3], &graph_[0]};
  path = contact_search::a_star_search(&graph_[1], &graph_[0], distance2,
                                       heuristic2);
  check_path(path, expected_path);

  // expected empty
  expected_path.clear();
  path = contact_search::a_star_search(&graph_[2], &graph_[5], distance2,
                                       heuristic2);
  check_path(path, expected_path);
}

}  // namespace planar_gripper
}  // namespace examples
}  // namespace drake
