#include "drake/planning/scene_graph_collision_checker.h"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/text_logging.h"
#include "drake/planning/linear_distance_and_interpolation_provider.h"
#include "drake/planning/robot_diagram_builder.h"
#include "drake/planning/test/planning_test_helpers.h"
#include "drake/planning/test_utilities/collision_checker_abstract_test_suite.h"

namespace drake {
namespace planning {
namespace test {
namespace {

using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;
using geometry::GeometryId;
using geometry::SceneGraphInspector;
using multibody::BodyIndex;
using multibody::RigidBody;
using testing::ElementsAre;

enum class MakeCheckerOptions { kMakeNone, kMakeProvider, kMakeFunction };

CollisionCheckerTestParams MakeSceneGraphCollisionCheckerParams(
    MakeCheckerOptions make_checker_option) {
  CollisionCheckerTestParams result;
  const CollisionCheckerConstructionParams p;
  auto model = MakePlanningTestModel(MakeCollisionCheckerTestScene());

  std::unique_ptr<LinearDistanceAndInterpolationProvider> provider = nullptr;
  ConfigurationDistanceFunction distance_function = nullptr;

  if (make_checker_option == MakeCheckerOptions::kMakeProvider) {
    provider = std::make_unique<LinearDistanceAndInterpolationProvider>(
        model->plant(), GetIiwaDistanceWeights());
  } else if (make_checker_option == MakeCheckerOptions::kMakeFunction) {
    distance_function = MakeWeightedIiwaConfigurationDistanceFunction();
  }

  const auto robot_instance = model->plant().GetModelInstanceByName("iiwa");
  result.checker.reset(new SceneGraphCollisionChecker(
      {.model = std::move(model),
       .distance_and_interpolation_provider = std::move(provider),
       .robot_model_instances = {robot_instance},
       .configuration_distance_function = std::move(distance_function),
       .edge_step_size = p.edge_step_size,
       .env_collision_padding = p.env_padding,
       .self_collision_padding = p.self_padding}));
  return result;
}

// This is equivalent to CollisionChecker::GenerateFilteredCollisionMatrix, but
// without any of the entries set by CollisionChecker itself, for purposes of
// checking filters in SceneGraph.
Eigen::MatrixXi GenerateFilteredCollisionMatrixFromSceneGraphOnly(
    const SceneGraphCollisionChecker& checker,
    const SceneGraphInspector<double>& inspector) {
  const int num_bodies = checker.plant().num_bodies();
  // Initialize matrix to zero (no filtered collisions).
  Eigen::MatrixXi filtered_collisions =
      Eigen::MatrixXi::Zero(num_bodies, num_bodies);

  // Loop variables below use `int` for Eigen indexing compatibility.
  for (int i = 0; i < num_bodies; ++i) {
    const RigidBody<double>& body_i = checker.get_body(BodyIndex(i));

    const std::vector<GeometryId>& geometries_i =
        checker.plant().GetCollisionGeometriesForBody(body_i);

    for (int j = i; j < num_bodies; ++j) {
      const RigidBody<double>& body_j = checker.get_body(BodyIndex(j));

      // Check if collisions between the geometries are already filtered.
      bool collisions_filtered = false;
      const std::vector<GeometryId>& geometries_j =
          checker.plant().GetCollisionGeometriesForBody(body_j);
      if (geometries_i.size() > 0 && geometries_j.size() > 0) {
        collisions_filtered =
            inspector.CollisionFiltered(geometries_i.at(0), geometries_j.at(0));
        // Ensure that the collision filtering is homogeneous across all body
        // geometries.
        for (const auto& id_i : geometries_i) {
          for (const auto& id_j : geometries_j) {
            const bool current_filtered =
                inspector.CollisionFiltered(id_i, id_j);
            if (current_filtered != collisions_filtered) {
              throw std::runtime_error(fmt::format(
                  "SceneGraph's collision filters on the geometries of bodies "
                  " {} [{}] and {} [{}] are not homogeneous",
                  i, body_i.scoped_name(), j, body_j.scoped_name()));
            }
          }
        }
      }

      // Add the filter accordingly.
      if (collisions_filtered) {
        // Filter the collision
        log()->debug(
            "Collision between body {} [{}] and body {} [{}] filtered in "
            "SceneGraph",
            body_i.scoped_name(), i, body_j.scoped_name(), j);
        filtered_collisions(i, j) = 1;
        filtered_collisions(j, i) = 1;
      } else {
        log()->debug(
            "Collision between body {} [{}] and body {} [{}] not filtered in "
            "SceneGraph",
            body_i.scoped_name(), i, body_j.scoped_name(), j);
      }
    }
  }
  return filtered_collisions;
}

// Enforce that the collision filter matrix of the provided collision checker is
// consistent with the filters applied in the SceneGraph context of every
// collision checker context. Note: this check only covers bodies with
// associated collision geometries, as SceneGraph cannot express filters between
// bodies without geometries.
void EnforceCollisionFilterConsistency(
    const SceneGraphCollisionChecker& checker) {
  const Eigen::MatrixXi& current_collision_filter_matrix =
      checker.GetFilteredCollisionMatrix();
  drake::log()->info("Current filter matrix:\n{}",
                     fmt_eigen(current_collision_filter_matrix));

  const auto body_has_geometries = [&](BodyIndex index) {
    const auto& body = checker.get_body(index);
    return checker.plant().GetCollisionGeometriesForBody(body).size() > 0;
  };

  int inconsistencies = 0;

  using Operation = const std::function<void(const RobotDiagram<double>&,
                                             CollisionCheckerContext*)>;
  const Operation operation = [&](const RobotDiagram<double>& model,
                                  CollisionCheckerContext* model_context) {
    const auto& inspector = model_context->GetQueryObject().inspector();
    const Eigen::MatrixXi context_collision_filter_matrix =
        GenerateFilteredCollisionMatrixFromSceneGraphOnly(checker, inspector);
    drake::log()->info("Context filter matrix:\n{}",
                       fmt_eigen(context_collision_filter_matrix));

    const int num_bodies = model.plant().num_bodies();
    for (BodyIndex i(0); i < num_bodies; ++i) {
      const bool i_has_geometries = body_has_geometries(i);

      for (BodyIndex j(0); j < num_bodies; ++j) {
        const bool j_has_geometries = body_has_geometries(j);

        const bool current_filtered =
            current_collision_filter_matrix(int{i}, int{j}) != 0;
        const bool context_filtered =
            context_collision_filter_matrix(int{i}, int{j}) != 0;

        if (current_filtered != context_filtered) {
          // SceneGraph only has a notion of allowed/excluded collision for
          // bodies with geometries, so we only check pairs where both bodies
          // have associated geometries.
          if (i_has_geometries && j_has_geometries) {
            ++inconsistencies;
            drake::log()->error("Entry {},{} is inconsistent", i, j);
          } else {
            drake::log()->info("Entry {},{} is inconsistent (ignored)", i, j);
          }
        }
      }
    }
  };

  // We know that the operation does not actually mutate the contexts.
  const_cast<SceneGraphCollisionChecker&>(checker)
      .PerformOperationAgainstAllModelContexts(operation);

  if (inconsistencies != 0) {
    throw std::runtime_error("Collision filters are inconsistent");
  }
}

}  // namespace

INSTANTIATE_TEST_SUITE_P(
    SceneGraphCollisionCheckerTestSuite, CollisionCheckerAbstractTestSuite,
    testing::Values(
        MakeSceneGraphCollisionCheckerParams(MakeCheckerOptions::kMakeNone),
        MakeSceneGraphCollisionCheckerParams(MakeCheckerOptions::kMakeProvider),
        MakeSceneGraphCollisionCheckerParams(
            MakeCheckerOptions::kMakeFunction)));

// Creates three spheres (each on a prismatic joint with its parent set to
// the world origin) and checks their RobotClearance query.
//
// The three spheres are configured so that when q = (5, 5, 0) (for Boston,
// Seattle, and Dallas, respectively), the distances from Do to So and Bo are 5
// (they form 3-4-5 triangles) and the distance between So and Bo is 8.
//
//       So                      Bo
//       |  ╲                  ╱  |
//       |    ╲  5        5  ╱    |
//    3  |      ╲          ╱      |  3
//       |        ╲      ╱        |
//       |          ╲  ╱          |
//       └-----------Do-----------┘
//            4             4
//
// Boston's sphere has a radius of 0.5.
// Seattle's sphere has a radius of 0.25.
// Dallas's sphere has a radius of 1.0.
GTEST_TEST(SceneGraphCollisionCheckerTest, ClearanceThreeSpheres) {
  RobotDiagramBuilder<double> builder;
  const std::string model_data = R"""(
<?xml version='1.0'?>
<sdf xmlns:drake='http://drake.mit.edu' version='1.9'>
<world name='default'>
  <model name='robot'>
    <link name='boston'>
      <collision name='boston_collision'>
        <geometry><sphere><radius>0.5</radius></sphere></geometry>
      </collision>
    </link>
    <joint name='boston_joint' type='prismatic'>
      <parent>world</parent>
      <child>boston</child>
      <axis><xyz>0.8 0.6 0</xyz></axis>
    </joint>
    <link name='seattle'>
      <collision name='seattle_collision'>
        <geometry><sphere><radius>0.25</radius></sphere></geometry>
      </collision>
    </link>
    <joint name='seattle_joint' type='prismatic'>
      <parent>world</parent>
      <child>seattle</child>
      <axis><xyz>-0.8 0.6 0</xyz></axis>
    </joint>
  </model>
  <model name='environment'>
    <link name='dallas'>
      <collision name='dallas_collision'>
        <geometry><sphere><radius>1.0</radius></sphere></geometry>
      </collision>
    </link>
    <joint name='dallas_joint' type='prismatic'>
      <parent>world</parent>
      <child>dallas</child>
      <axis><xyz>0 -1 0</xyz></axis>
    </joint>
  </model>
</world>
</sdf>
)""";
  builder.parser().AddModelsFromString(model_data, "sdf");

  const auto& plant = builder.plant();

  CollisionCheckerParams params;
  params.model = builder.Build();
  params.robot_model_instances.push_back(plant.GetModelInstanceByName("robot"));
  params.configuration_distance_function = [](const VectorXd& q1,
                                              const VectorXd& q2) {
    return (q1 - q2).norm();
  };
  params.edge_step_size = 0.05;
  SceneGraphCollisionChecker dut(std::move(params));

  const BodyIndex boston = plant.GetBodyByName("boston").index();
  const BodyIndex seattle = plant.GetBodyByName("seattle").index();
  const BodyIndex dallas = plant.GetBodyByName("dallas").index();

  // Posture Seattle's center at (-4, 3), Boston's center at (4, 3), and
  // Dallas's center at (0, 0); see the test overview above for a figure.
  Vector3d q(5.0, 5.0, 0.0);
  const double influence = 99999999;
  auto clearance = dut.CalcRobotClearance(q, influence);

  // The current order of return values is like so:
  //
  //  boston, seattle
  //  boston, dallas
  //  seattle, dallas
  //
  // We don't care what the order is, but it's easier for now to just hard-code
  // one in the test.
  const double tol = 1e-9;
  EXPECT_EQ(clearance.size(), 3);
  EXPECT_EQ(clearance.num_positions(), 3);
  EXPECT_THAT(clearance.robot_indices(), ElementsAre(boston, boston, seattle));
  EXPECT_THAT(clearance.other_indices(), ElementsAre(seattle, dallas, dallas));
  EXPECT_THAT(clearance.collision_types(),
              ElementsAre(RobotCollisionType::kSelfCollision,
                          RobotCollisionType::kEnvironmentCollision,
                          RobotCollisionType::kEnvironmentCollision));
  // The Boston <-> Seattle distance on-center is 8.0.
  // The Boston <-> Dallas distance on-center is 5.0.
  // The Seattle <-> Dallas distance on-center is 5.0.
  // In each case, the measured distance is less than on-center by each of the
  // two cities' sphere radii.
  Vector3d expected_distances;
  expected_distances(0) = 8.0 - 0.5 - 0.25;
  expected_distances(1) = 5.0 - 0.5 - 1.0;
  expected_distances(2) = 5.0 - 0.25 - 1.0;
  EXPECT_TRUE(CompareMatrices(clearance.distances(), expected_distances, tol));
  // Recall that:
  // - The prismatic joint for Boston moves (+0.6, +0.8) for each +1.0 q₀.
  // - The prismatic joint for Seattle moves (-0.6, +0.8) for each +1.0 q₁.
  // - Dallas is not part of the robot, so its jacobian will always be zero.
  Matrix3d expected_jacobians(3, 3);
  expected_jacobians.setZero();
  // Increasing either q₀ or q₁ will increase the Boston-Seattle distance.
  expected_jacobians(0, 0) = 0.8;
  expected_jacobians(0, 1) = 0.8;
  // Increasing q₀ will increase the Boston-Dallas distance.
  expected_jacobians(1, 0) = 1.0;
  // Increasing q₁ will increase the Seattle-Dallas distance.
  expected_jacobians(2, 1) = 1.0;
  EXPECT_TRUE(CompareMatrices(clearance.jacobians(), expected_jacobians, tol));

  // Now reposture Seattle to be directly south of Boston (tectonic plates!).
  // The distances will change in the obvious ways; what we care about most is
  // that the jacobian and especially its sign update correctly.
  q(1) = -5.0;
  clearance = dut.CalcRobotClearance(q, influence);
  EXPECT_THAT(clearance.robot_indices(), ElementsAre(boston, boston, seattle));
  EXPECT_THAT(clearance.other_indices(), ElementsAre(seattle, dallas, dallas));
  EXPECT_THAT(clearance.collision_types(),
              ElementsAre(RobotCollisionType::kSelfCollision,
                          RobotCollisionType::kEnvironmentCollision,
                          RobotCollisionType::kEnvironmentCollision));
  // The Boston <-> Seattle distance on-center is 6.0.
  // The Boston <-> Dallas distance on-center is 5.0.
  // The Seattle <-> Dallas distance on-center is 5.0.
  // In each case, the measured distance is less than on-center by each of the
  // two cities' sphere radii.
  expected_distances(0) = 6.0 - 0.5 - 0.25;
  expected_distances(1) = 5.0 - 0.5 - 1.0;
  expected_distances(2) = 5.0 - 0.25 - 1.0;
  EXPECT_TRUE(CompareMatrices(clearance.distances(), expected_distances, tol));
  // Recall that:
  // - The prismatic joint for Boston moves (+0.6, +0.8) for each +1.0 q₀.
  // - The prismatic joint for Seattle moves (-0.6, +0.8) for each +1.0 q₁.
  // - Dallas is not part of the robot, so its jacobian will always be zero.
  expected_jacobians.setZero();
  // Increasing q₀ will increase the Boston-Seattle distance.
  expected_jacobians(0, 0) = 0.6;
  // Increasing q₁ will decrease the Boston-Seattle distance.
  expected_jacobians(0, 1) = -0.6;
  // Increasing q₀ will increase the Boston-Dallas distance.
  expected_jacobians(1, 0) = 1.0;
  // Increasing q₁ will decrease the Seattle-Dallas distance.
  expected_jacobians(2, 1) = -1.0;
  EXPECT_TRUE(CompareMatrices(clearance.jacobians(), expected_jacobians, tol));
}

// Checks RobotClearance when the robot is an arm (only) but inboard of the
// arm we have a mobile chassis that is not part of a robot from the point
// of view of CollisionChecker (e.g., we're only planning for the arm).
GTEST_TEST(SceneGraphCollisionCheckerTest, ClearanceFloatingBase) {
  // Build a dut with a ground plane + floating chassis with welded arm.
  RobotDiagramBuilder<double> builder;
  const std::string model_directives = R"""(
directives:
- add_model:
    name: ground
    file: package://drake/planning/test_utilities/collision_ground_plane.sdf
- add_weld:
    parent: world
    child: ground::ground_plane_box
- add_model:
    name: chassis
    file: package://drake_models/ycb/010_potted_meat_can.sdf
- add_model:
    name: arm
    file: package://drake_models/iiwa_description/urdf/iiwa14_spheres_dense_collision.urdf
- add_weld:
    parent: chassis::base_link_meat
    child: arm::base
)""";
  builder.parser().AddModelsFromString(model_directives, "dmd.yaml");

  const auto& plant = builder.plant();
  CollisionCheckerParams params;
  params.model = builder.Build();
  params.robot_model_instances.push_back(plant.GetModelInstanceByName("arm"));
  params.configuration_distance_function = [](const VectorXd& q1,
                                              const VectorXd& q2) {
    return (q1 - q2).norm();
  };
  params.edge_step_size = 0.05;
  SceneGraphCollisionChecker dut(std::move(params));

  // In the testing below, we assume that the iiwa's floating base is
  // quaternion, and that the floating base precedes the arm in q.
  const int floating_dofs = 7;
  const int arm_dofs = 7;
  const int total_dofs = floating_dofs + arm_dofs;
  DRAKE_DEMAND(plant.num_positions() == total_dofs);
  const auto& first_arm_joint = plant.GetJointByName("iiwa_joint_1");
  DRAKE_DEMAND(first_arm_joint.position_start() == floating_dofs);

  // Check the clearance of the default posture.
  const VectorXd q = plant.GetPositions(*plant.CreateDefaultContext());
  const double influence = 99999999;
  auto clearance = dut.CalcRobotClearance(q, influence);
  EXPECT_EQ(clearance.num_positions(), total_dofs);
  EXPECT_GT(clearance.size(), 0);
  int num_non_zero = 0;
  for (int i = 0; i < clearance.size(); ++i) {
    const VectorXd jacobian = clearance.jacobians().row(i);
    ASSERT_EQ(jacobian.size(), total_dofs);
    // Non-robot dofs must always be zero.
    ASSERT_TRUE(CompareMatrices(jacobian.head(floating_dofs),
                                VectorXd::Zero(floating_dofs)));
    // Robot dofs are generally non-zero.
    if (!jacobian.tail(arm_dofs).isZero(0.0)) {
      ++num_non_zero;
    }
  }
  // At least 95% of the distances should have a non-singular jacobian. In any
  // given posture, some small number of distances will have a zero jacobian.
  // We aren't trying drill down into that here; we're only really concerned
  // with the non-robot dofs being zeroed out without the entire jacobian
  // accidentally being zeroed out.
  EXPECT_GT(static_cast<double>(num_non_zero) / clearance.size(), 0.95);
}

// Checks that collision filters are properly updated.
GTEST_TEST(SceneGraphCollisionCheckerTest, CollisionFilterUpdate) {
  // Build a dut with a ground plane + floating chassis with welded arm.
  RobotDiagramBuilder<double> builder;
  const std::string model_directives = R"""(
directives:
- add_model:
    name: ground
    file: package://drake/planning/test_utilities/collision_ground_plane.sdf
- add_weld:
    parent: world
    child: ground::ground_plane_box
- add_model:
    name: chassis
    file: package://drake_models/ycb/010_potted_meat_can.sdf
- add_model:
    name: arm
    file: package://drake_models/iiwa_description/urdf/iiwa14_spheres_dense_collision.urdf
- add_weld:
    parent: chassis::base_link_meat
    child: arm::base
)""";
  builder.parser().AddModelsFromString(model_directives, "dmd.yaml");

  const auto& plant = builder.plant();
  CollisionCheckerParams params;
  params.model = builder.Build();
  params.robot_model_instances.push_back(plant.GetModelInstanceByName("arm"));
  params.configuration_distance_function = [](const VectorXd& q1,
                                              const VectorXd& q2) {
    return (q1 - q2).norm();
  };
  params.edge_step_size = 0.05;
  SceneGraphCollisionChecker dut(std::move(params));

  // As constructed, collision filters must be consistent.
  EXPECT_NO_THROW(EnforceCollisionFilterConsistency(dut));

  // Set collision filtering for a specific pair of robot bodies.
  // Note: this also tests setting the equivalent filtering via body indices.
  {
    const Eigen::MatrixXi pre_filter_matrix = dut.GetFilteredCollisionMatrix();
    dut.SetCollisionFilteredBetween(dut.plant().GetBodyByName("iiwa_link_0"),
                                    dut.plant().GetBodyByName("iiwa_link_6"),
                                    true);
    ASSERT_FALSE(
        CompareMatrices(pre_filter_matrix, dut.GetFilteredCollisionMatrix()));
    EXPECT_NO_THROW(EnforceCollisionFilterConsistency(dut));
  }

  // Set the entire collision filter matrix (this is also used to reset for the
  // following test).
  {
    const Eigen::MatrixXi pre_filter_matrix = dut.GetFilteredCollisionMatrix();
    ASSERT_FALSE(CompareMatrices(pre_filter_matrix,
                                 dut.GetNominalFilteredCollisionMatrix()));
    dut.SetCollisionFilterMatrix(dut.GetNominalFilteredCollisionMatrix());
    ASSERT_FALSE(
        CompareMatrices(pre_filter_matrix, dut.GetFilteredCollisionMatrix()));
    EXPECT_NO_THROW(EnforceCollisionFilterConsistency(dut));
  }

  // Set collision filtering between one robot body and all other bodies.
  // Note: this also tests setting the equivalent filtering via body index.
  {
    const Eigen::MatrixXi pre_filter_matrix = dut.GetFilteredCollisionMatrix();
    dut.SetCollisionFilteredWithAllBodies(
        dut.plant().GetBodyByName("iiwa_link_0"));
    ASSERT_FALSE(
        CompareMatrices(pre_filter_matrix, dut.GetFilteredCollisionMatrix()));
    EXPECT_NO_THROW(EnforceCollisionFilterConsistency(dut));
  }
}

}  // namespace test
}  // namespace planning
}  // namespace drake
