#include "drake/planning/iris/iris_from_clique_cover.h"

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/ssize.h"
#include "drake/common/test_utilities/maybe_pause_for_user.h"
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/geometry/optimization/hyperrectangle.h"
#include "drake/geometry/optimization/vpolytope.h"
#include "drake/geometry/test_utilities/meshcat_environment.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/planning/graph_algorithms/max_clique_solver_via_mip.h"
#include "drake/planning/robot_diagram_builder.h"
#include "drake/planning/scene_graph_collision_checker.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/mosek_solver.h"
#include "drake/solvers/solver_options.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace planning {
namespace {
using common::MaybePauseForUser;
using Eigen::Vector2d;
using geometry::Meshcat;
using geometry::Rgba;
using geometry::optimization::ConvexSets;
using geometry::optimization::HPolyhedron;
using geometry::optimization::Hyperrectangle;
using geometry::optimization::IrisOptions;
using geometry::optimization::VPolytope;

// Draw a two dimensional polytope in meshcat.
void Draw2dVPolytope(const VPolytope& polytope, const std::string& meshcat_name,
                     const Eigen::Ref<const Eigen::Vector3d>& color,
                     std::shared_ptr<Meshcat> meshcat) {
  DRAKE_THROW_UNLESS(polytope.ambient_dimension() == 2);
  Eigen::Matrix3Xd points =
      Eigen::Matrix3Xd::Zero(3, polytope.vertices().cols() + 1);
  points.topLeftCorner(2, polytope.vertices().cols()) = polytope.vertices();
  points.topRightCorner(2, 1) = polytope.vertices().col(0);
  points.bottomRows<1>().setZero();

  meshcat->SetLine(meshcat_name, points, 2.0,
                   Rgba(color(0), color(1), color(2)));
}

GTEST_TEST(IrisInConfigurationSpaceFromCliqueCover,
           TestIrisFromCliqueCoverOptions) {
  IrisFromCliqueCoverOptions options;

  EXPECT_EQ(options.iris_options.iteration_limit, 1);
  options.iris_options.iteration_limit = 100;
  EXPECT_EQ(options.iris_options.iteration_limit, 100);

  EXPECT_EQ(options.coverage_termination_threshold, 0.7);
  options.coverage_termination_threshold = 0.1;
  EXPECT_EQ(options.coverage_termination_threshold, 0.1);

  EXPECT_EQ(options.iteration_limit, 100);
  options.iteration_limit = 10;
  EXPECT_EQ(options.iteration_limit, 10);

  EXPECT_EQ(options.num_points_per_coverage_check, 1000);
  options.num_points_per_coverage_check = 10;
  EXPECT_EQ(options.num_points_per_coverage_check, 10);

  EXPECT_EQ(options.parallelism.num_threads(),
            Parallelism::Max().num_threads());
  options.parallelism = Parallelism{100};
  EXPECT_EQ(options.parallelism.num_threads(), 100);

  EXPECT_EQ(options.minimum_clique_size, 3);
  options.minimum_clique_size = 5;
  EXPECT_EQ(options.minimum_clique_size, 5);

  EXPECT_EQ(options.num_points_per_visibility_round, 200);
  options.num_points_per_visibility_round = 10;
  EXPECT_EQ(options.num_points_per_visibility_round, 10);

  EXPECT_EQ(options.rank_tol_for_minimum_volume_circumscribed_ellipsoid, 1e-6);
  options.rank_tol_for_minimum_volume_circumscribed_ellipsoid = 1e-3;
  EXPECT_EQ(options.rank_tol_for_minimum_volume_circumscribed_ellipsoid, 1e-3);

  EXPECT_EQ(options.point_in_set_tol, 1e-6);
  options.point_in_set_tol = 1e-3;
  EXPECT_EQ(options.point_in_set_tol, 1e-3);
}

/* A movable sphere in a box.
┌───────────────┐
│               │
│               │
│               │
│       o       │
│               │
│               │
│               │
└───────────────┘ */
const char free_box[] = R"""(
<robot name="boxes">
  <link name="movable">
    <collision name="sphere">
      <geometry><sphere radius="0.1"/></geometry>
    </collision>
  </link>
  <link name="for_joint"/>

  <joint name="x" type="prismatic">
    <axis xyz="1 0 0"/>
    <limit lower="-2" upper="2"/>
    <parent link="world"/>
    <child link="for_joint"/>
  </joint>
  <joint name="y" type="prismatic">
    <axis xyz="0 1 0"/>
    <limit lower="-2" upper="2"/>
    <parent link="for_joint"/>
    <child link="movable"/>
  </joint>
</robot>
)""";
// Test that we get perfect coverage
GTEST_TEST(IrisInConfigurationSpaceFromCliqueCover, BoxConfigurationSpaceTest) {
  CollisionCheckerParams params;

  RobotDiagramBuilder<double> builder(0.0);
  params.robot_model_instances =
      builder.parser().AddModelsFromString(free_box, "urdf");
  params.edge_step_size = 0.01;

  params.model = builder.Build();
  auto checker =
      std::make_unique<SceneGraphCollisionChecker>(std::move(params));

  IrisFromCliqueCoverOptions options;

  options.num_points_per_coverage_check = 100;
  options.num_points_per_visibility_round = 100;
  options.iteration_limit = 1;
  // Set a large bounding region to test the path where this is set in the
  // IrisOptions.
  options.iris_options.bounding_region =
      HPolyhedron::MakeBox(Eigen::Vector2d{-2, -2}, Eigen::Vector2d{2, 2});
  // Run this test without parallelism to test that no bugs occur in the
  // non-parallel version.
  options.parallelism = Parallelism{1};
  std::vector<HPolyhedron> sets;

  RandomGenerator generator(0);

  IrisInConfigurationSpaceFromCliqueCover(*checker, options, &generator, &sets,
                                          nullptr);
  EXPECT_GE(ssize(sets), 1);

  // expect perfect coverage
  VPolytope vpoly(sets.at(0));
  EXPECT_EQ(vpoly.CalcVolume(), 16.0);
}

/* A movable sphere with fixed boxes in all corners.
┌───────────────┐
│┌────┐   ┌────┐│
││    │   │    ││
│└────┘   └────┘│
│       o       │
│┌────┐   ┌────┐│
││    │   │    ││
│└────┘   └────┘│
└───────────────┘ */
const char boxes_in_corners[] = R"""(
<robot name="boxes">
  <link name="fixed">
    <collision name="top_left">
      <origin rpy="0 0 0" xyz="-1 1 0"/>
      <geometry><box size="1.4 1.4 1.4"/></geometry>
    </collision>
    <collision name="top_right">
      <origin rpy="0 0 0" xyz="1 1 0"/>
      <geometry><box size="1.4 1.4 1.4"/></geometry>
    </collision>
    <collision name="bottom_left">
      <origin rpy="0 0 0" xyz="-1 -1 0"/>
      <geometry><box size="1.4 1.4 1.4"/></geometry>
    </collision>
    <collision name="bottom_right">
      <origin rpy="0 0 0" xyz="1 -1 0"/>
      <geometry><box size="1.4 1.4 1.4"/></geometry>
    </collision>
  </link>
  <joint name="fixed_link_weld" type="fixed">
    <parent link="world"/>
    <child link="fixed"/>
  </joint>
  <link name="movable">
    <collision name="sphere">
      <geometry><sphere radius="0.01"/></geometry>
    </collision>
  </link>
  <link name="for_joint"/>
  <joint name="x" type="prismatic">
    <axis xyz="1 0 0"/>
    <limit lower="-2" upper="2"/>
    <parent link="world"/>
    <child link="for_joint"/>
  </joint>
  <joint name="y" type="prismatic">
    <axis xyz="0 1 0"/>
    <limit lower="-2" upper="2"/>
    <parent link="for_joint"/>
    <child link="movable"/>
  </joint>
</robot>
)""";

class IrisInConfigurationSpaceFromCliqueCoverTestFixture
    : public ::testing::Test {
 protected:
  void SetUp() override {
    params = CollisionCheckerParams();

    meshcat = geometry::GetTestEnvironmentMeshcat();
    meshcat->Set2dRenderMode(math::RigidTransformd(Eigen::Vector3d{0, 0, 1}),
                             -3.25, 3.25, -3.25, 3.25);
    meshcat->SetProperty("/Grid", "visible", true);
    // Draw the true cspace.
    Eigen::Matrix3Xd env_points(3, 5);
    // clang-format off
        env_points << -2, 2,  2, -2, -2,
                        2, 2, -2, -2,  2,
                        0, 0,  0,  0,  0;
    // clang-format on
    meshcat->SetLine("Domain", env_points, 8.0, Rgba(0, 0, 0));
    Eigen::Matrix3Xd centers(3, 4);
    double c = 1.0;
    // clang-format off
        centers << -c, c,  c, -c,
                    c, c, -c, -c,
                    0, 0,  0,  0;
    // clang-format on
    Eigen::Matrix3Xd obs_points(3, 5);
    // approximating offset due to sphere radius with fixed offset
    double s = 0.7 + 0.01;
    // clang-format off
        obs_points << -s, s,  s, -s, -s,
                        s, s, -s, -s, s,
                        s, 0,  0,  0,  0;
    // clang-format on
    for (int obstacle_idx = 0; obstacle_idx < 4; ++obstacle_idx) {
      Eigen::Matrix3Xd obstacle = obs_points;
      obstacle.colwise() += centers.col(obstacle_idx);
      meshcat->SetLine(fmt::format("/obstacles/obs_{}", obstacle_idx), obstacle,
                       8.0, Rgba(0, 0, 0));
    }

    RobotDiagramBuilder<double> builder(0.0);
    params.robot_model_instances =
        builder.parser().AddModelsFromString(boxes_in_corners, "urdf");
    params.edge_step_size = 0.01;

    params.model = builder.Build();
    checker = std::make_unique<SceneGraphCollisionChecker>(std::move(params));
    options.iris_options.meshcat = meshcat;

    options.num_points_per_coverage_check = 1000;
    options.num_points_per_visibility_round = 180;
    options.coverage_termination_threshold = 0.95;
    options.minimum_clique_size = 25;

    generator = RandomGenerator(1);

    // A manual convex decomposition of the space.
    manual_decomposition.push_back(
        Hyperrectangle(Vector2d{-2, -2}, Vector2d{-1.7, 2}));
    manual_decomposition.push_back(
        Hyperrectangle(Vector2d{-2, -2}, Vector2d{2, -1.7}));
    manual_decomposition.push_back(
        Hyperrectangle(Vector2d{1.7, -2}, Vector2d{2, 2}));
    manual_decomposition.push_back(
        Hyperrectangle(Vector2d{-2, 1.7}, Vector2d{2, 2}));
    manual_decomposition.push_back(
        Hyperrectangle(Vector2d{-0.3, -2}, Vector2d{0.3, 2}));
    manual_decomposition.push_back(
        Hyperrectangle(Vector2d{-2, -0.3}, Vector2d{2, 0.3}));

    color = Eigen::VectorXd::Zero(3);
    // Show the manual decomposition in the meshcat debugger.
    for (int i = 0; i < ssize(manual_decomposition); ++i) {
      // Choose a random color.
      for (int j = 0; j < color.size(); ++j) {
        color[j] = abs(gaussian(generator));
      }
      color.normalize();
      VPolytope vregion =
          VPolytope(manual_decomposition.at(i).MakeHPolyhedron())
              .GetMinimalRepresentation();
      Draw2dVPolytope(vregion, fmt::format("manual_decomposition_{}", i), color,
                      meshcat);
    }
  }

  CollisionCheckerParams params;
  std::shared_ptr<Meshcat> meshcat;

  std::unique_ptr<SceneGraphCollisionChecker> checker;
  IrisFromCliqueCoverOptions options;
  std::vector<HPolyhedron> sets;
  RandomGenerator generator;
  std::vector<Hyperrectangle> manual_decomposition;
  std::normal_distribution<double> gaussian;
  Eigen::VectorXd color;
};

TEST_F(IrisInConfigurationSpaceFromCliqueCoverTestFixture,
       BoxWithCornerObstaclesTestMip) {
  // Set ILP settings for MaxCliqueSolverViaMip

  // limiting the work load for ILP solver
  solvers::SolverOptions solver_options;
  // Quit after finding 25 solutions.
  const int kFeasibleSolutionLimit = 25;
  // Quit at a 5% optimality gap
  const double kRelOptGap = 0.05;

  solver_options.SetOption(solvers::MosekSolver().id(),
                           "MSK_IPAR_MIO_MAX_NUM_SOLUTIONS",
                           kFeasibleSolutionLimit);
  solver_options.SetOption(solvers::MosekSolver().id(),
                           "MSK_DPAR_MIO_TOL_REL_GAP", kRelOptGap);
  solver_options.SetOption(solvers::GurobiSolver().id(), "SolutionLimit",
                           kFeasibleSolutionLimit);
  solver_options.SetOption(solvers::GurobiSolver().id(), "MIPGap", kRelOptGap);

  planning::graph_algorithms::MaxCliqueSolverViaMip solver{std::nullopt,
                                                           solver_options};

  IrisInConfigurationSpaceFromCliqueCover(*checker, options, &generator, &sets,
                                          &solver);
  EXPECT_EQ(ssize(sets), 6);
  // Show the IrisFromCliqueCoverDecomposition
  for (int i = 0; i < ssize(sets); ++i) {
    // Choose a random color.
    for (int j = 0; j < color.size(); ++j) {
      color[j] = abs(gaussian(generator));
    }
    color.normalize();
    VPolytope vregion = VPolytope(sets.at(i)).GetMinimalRepresentation();
    Draw2dVPolytope(vregion, fmt::format("iris_from_clique_cover_mip{}", i),
                    color, meshcat);
  }

  // Now check the coverage by drawing points from the manual decomposition and
  // checking if they are inside the IrisFromCliqueCover decomposition.
  int num_samples_per_set = 1000;
  int num_in_automatic_decomposition = 0;
  for (const auto& manual_set : manual_decomposition) {
    for (int i = 0; i < num_samples_per_set; ++i) {
      Eigen::Vector2d sample = manual_set.UniformSample(&generator);
      for (const auto& set : sets) {
        if (set.PointInSet(sample)) {
          ++num_in_automatic_decomposition;
          break;
        }
      }
    }
  }
  double coverage_estimate =
      static_cast<double>(num_in_automatic_decomposition) /
      static_cast<double>(num_samples_per_set * ssize(manual_decomposition));
  EXPECT_GE(coverage_estimate, 0.9);

  MaybePauseForUser();
}

TEST_F(IrisInConfigurationSpaceFromCliqueCoverTestFixture,
       BoxWithCornerObstaclesTestGreedy) {
  // use default solver MaxCliqueSovlerViaGreedy
  IrisInConfigurationSpaceFromCliqueCover(*checker, options, &generator, &sets,
                                          nullptr);

  EXPECT_EQ(ssize(sets), 6);

  // Show the IrisFromCliqueCoverDecomposition
  for (int i = 0; i < ssize(sets); ++i) {
    // Choose a random color.
    for (int j = 0; j < color.size(); ++j) {
      color[j] = abs(gaussian(generator));
    }
    color.normalize();
    VPolytope vregion = VPolytope(sets.at(i)).GetMinimalRepresentation();
    Draw2dVPolytope(vregion, fmt::format("iris_from_clique_cover_greedy{}", i),
                    color, meshcat);
  }

  // Now check the coverage by drawing points from the manual decomposition and
  // checking if they are inside the IrisFromCliqueCover decomposition.
  int num_samples_per_set = 1000;
  int num_in_automatic_decomposition = 0;
  for (const auto& manual_set : manual_decomposition) {
    for (int i = 0; i < num_samples_per_set; ++i) {
      Eigen::Vector2d sample = manual_set.UniformSample(&generator);
      for (const auto& set : sets) {
        if (set.PointInSet(sample)) {
          ++num_in_automatic_decomposition;
          break;
        }
      }
    }
  }
  double coverage_estimate =
      static_cast<double>(num_in_automatic_decomposition) /
      static_cast<double>(num_samples_per_set * ssize(manual_decomposition));
  EXPECT_GE(coverage_estimate, 0.9);

  MaybePauseForUser();
}

}  // namespace
}  // namespace planning
}  // namespace drake
