#pragma once

/* @file
This library provides a suite of tests for a concrete CollisionChecker
implementation. Developers can parameterize it to run against a specific
concrete checker implementation, e.g., scene_graph_collision_checker_test.
*/

#include <memory>
#include <ostream>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/multibody/parsing/model_directives.h"
#include "drake/planning/collision_checker.h"

namespace drake {
namespace planning {
namespace test {

/* All abstract tests assume the same model; an iiwa welded to a ground plane.
 */
multibody::parsing::ModelDirectives MakeCollisionCheckerTestScene();

/* All abstract tests assume some common constructor parameters for checkers. */
struct CollisionCheckerConstructionParams {
  const double edge_step_size{0.05};
  const double env_padding{0.0};
  const double self_padding{0.0};
};

/* All abstract tests use some common generalized position data, matched to
work with the test scene. */
struct CollisionCheckerTestConfigurationData {
  CollisionCheckerTestConfigurationData() {
    q2 << 0.0, M_PI_2, 0.0, 0.0, 0.0, 0.0, 0.0;
    q3 << 0.0, M_PI_2, 0.0, -M_PI_2, 0.0, 0.0, 0.0;
    q4 << M_PI_2, M_PI_2, 0.0, -M_PI_2, 0.0, 0.0, 0.0;
    configs = {q1, q2, q3, q4};
    edges = {std::make_pair(q1, q2), std::make_pair(q2, q3),
             std::make_pair(q3, q2), std::make_pair(q3, q4)};
  }
  const Eigen::VectorXd q1{Eigen::VectorXd::Zero(7)};
  Eigen::VectorXd q2{Eigen::VectorXd::Zero(7)};
  Eigen::VectorXd q3{Eigen::VectorXd::Zero(7)};
  Eigen::VectorXd q4{Eigen::VectorXd::Zero(7)};
  std::vector<Eigen::VectorXd> configs;
  std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> edges;
};

/* These parameters are passed to the abstract test cases to make them concrete;
a collision checker of some derived type, and options that tests may need to
adapt to the checker. */
struct CollisionCheckerTestParams {
  std::shared_ptr<CollisionChecker> checker;
  bool supports_added_world_obstacles{true};  // Most do, some don't.
  // Some derived classes may benefit from more thread stress testing than
  // others; the default here provides relatively light stress.
  int thread_stress_iterations{10};
};

std::ostream& operator<<(std::ostream& out,
                         const CollisionCheckerTestParams& p);

class CollisionCheckerAbstractTestSuite
    : public testing::TestWithParam<CollisionCheckerTestParams> {
 protected:
  void TestDiscreteQueries(const CollisionCheckerTestParams& params,
                           Parallelism parallelism);
  void TestDistanceQueries(const CollisionCheckerTestParams& params,
                           Parallelism parallelism);
  const CollisionCheckerTestConfigurationData qs_;
};

}  // namespace test
}  // namespace planning
}  // namespace drake
