#include "drake/multibody/parsing/detail_urdf_parser.h"

#include <limits>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/filesystem.h"
#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/multibody/parsing/detail_path_utils.h"

namespace drake {
namespace multibody {
namespace internal {
namespace {

using Eigen::Vector3d;
using geometry::GeometryId;
using geometry::SceneGraph;

// Verifies that the URDF loader can leverage a specified package map.
GTEST_TEST(MultibodyPlantUrdfParserTest, PackageMapSpecified) {
  // We start with the world and default model instances (model_instance.h
  // explains why there are two).
  MultibodyPlant<double> plant;
  geometry::SceneGraph<double> scene_graph;
  ASSERT_EQ(plant.num_model_instances(), 2);

  const std::string full_urdf_filename = FindResourceOrThrow(
      "drake/multibody/parsing/test/box_package/urdfs/box.urdf");
  filesystem::path package_path = full_urdf_filename;
  package_path = package_path.parent_path();
  package_path = package_path.parent_path();

  // Construct the PackageMap.
  PackageMap package_map;
  package_map.PopulateFromFolder(package_path.string());

  // Read in the URDF file.
  AddModelFromUrdfFile(full_urdf_filename, "", package_map, &plant,
                       &scene_graph);
  plant.Finalize();

  // Verify the number of model instances.
  EXPECT_EQ(plant.num_model_instances(), 3);
}

GTEST_TEST(MultibodyPlantUrdfParserTest, DoublePendulum) {
  MultibodyPlant<double> plant;
  SceneGraph<double> scene_graph;
  std::string full_name = FindResourceOrThrow(
      "drake/multibody/benchmarks/acrobot/double_pendulum.urdf");
  PackageMap package_map;
  package_map.PopulateUpstreamToDrake(full_name);
  AddModelFromUrdfFile(full_name, "", package_map, &plant, &scene_graph);
  plant.Finalize();

  EXPECT_EQ(plant.num_bodies(), 4);
  EXPECT_EQ(plant.num_frames(), 10);

  ASSERT_TRUE(plant.HasFrameNamed("frame_on_link1"));
  ASSERT_TRUE(plant.HasFrameNamed("frame_on_link2"));
  ASSERT_TRUE(plant.HasFrameNamed("link1_com"));
  ASSERT_TRUE(plant.HasFrameNamed("link2_com"));

  // Sample a couple of frames.
  const Frame<double>& frame_on_link1 = plant.GetFrameByName("frame_on_link1");
  EXPECT_EQ(frame_on_link1.body().index(),
            plant.GetBodyByName("link1").index());

  math::RollPitchYaw<double> rpy_expected(-1, 0.1, 0.2);
  Vector3d xyz_expected(0.8, -0.2, 0.3);
  math::RigidTransform<double> X_BF_expected(rpy_expected.ToRotationMatrix(),
                                             xyz_expected);

  EXPECT_TRUE(CompareMatrices(frame_on_link1.GetFixedPoseInBodyFrame().matrix(),
                              X_BF_expected.GetAsMatrix4(), 1e-10));

  const Frame<double>& link2_com = plant.GetFrameByName("link2_com");
  EXPECT_EQ(link2_com.body().index(), plant.GetBodyByName("link2").index());
}

// This test verifies that we're able to successfully look up meshes using the
// "package://" syntax internally to the URDF (at least for packages which are
// successfully found in the same directory at the URDF.
GTEST_TEST(MultibodyPlantUrdfParserTest, TestAtlasMinimalContact) {
  MultibodyPlant<double> plant;
  SceneGraph<double> scene_graph;
  std::string full_name = FindResourceOrThrow(
      "drake/examples/atlas/urdf/atlas_minimal_contact.urdf");
  PackageMap package_map;
  package_map.PopulateUpstreamToDrake(full_name);

  AddModelFromUrdfFile(full_name, "", package_map, &plant, &scene_graph);
  plant.Finalize();

  EXPECT_EQ(plant.num_positions(), 37);
  EXPECT_EQ(plant.num_velocities(), 36);
}

GTEST_TEST(MultibodyPlantUrdfParserTest, TestAddWithQuaternionFloatingDof) {
  const std::string resource_dir{
      "drake/multibody/parsing/test/urdf_parser_test/"};
  const std::string model_file =
      FindResourceOrThrow(resource_dir + "zero_dof_robot.urdf");
  PackageMap package_map;
  package_map.PopulateUpstreamToDrake(model_file);

  MultibodyPlant<double> plant;
  SceneGraph<double> scene_graph;
  AddModelFromUrdfFile(model_file, "", package_map, &plant, &scene_graph);
  plant.Finalize();

  EXPECT_EQ(plant.num_positions(), 7);
  EXPECT_EQ(plant.num_velocities(), 6);
}

GTEST_TEST(MultibodyPlantUrdfParserTest, TestOptionalSceneGraph) {
  const std::string full_name = FindResourceOrThrow(
      "drake/examples/atlas/urdf/atlas_minimal_contact.urdf");
  PackageMap package_map;
  package_map.PopulateUpstreamToDrake(full_name);
  int num_visuals_explicit{};
  {
    // Test explicitly specifying `scene_graph`.
    MultibodyPlant<double> plant;
    SceneGraph<double> scene_graph;
    AddModelFromUrdfFile(full_name, "", package_map, &plant, &scene_graph);
    plant.Finalize();
    num_visuals_explicit = plant.num_visual_geometries();
  }
  EXPECT_NE(num_visuals_explicit, 0);
  {
    // Test implicitly specifying.
    MultibodyPlant<double> plant;
    SceneGraph<double> scene_graph;
    plant.RegisterAsSourceForSceneGraph(&scene_graph);
    AddModelFromUrdfFile(full_name, "", package_map, &plant);
    plant.Finalize();
    EXPECT_EQ(plant.num_visual_geometries(), num_visuals_explicit);
  }
}

GTEST_TEST(MultibodyPlantUrdfParserTest, JointParsingTest) {
  const std::string full_name = FindResourceOrThrow(
      "drake/multibody/parsing/test/urdf_parser_test/"
      "joint_parsing_test.urdf");
  PackageMap package_map;
  package_map.PopulateUpstreamToDrake(full_name);

  MultibodyPlant<double> plant;
  SceneGraph<double> scene_graph;
  AddModelFromUrdfFile(full_name, "", package_map, &plant, &scene_graph);
  plant.Finalize();

  const Joint<double>& revolute_joint = plant.GetJointByName("revolute_joint");
  EXPECT_TRUE(
      CompareMatrices(revolute_joint.position_lower_limits(), Vector1d(-1)));
  EXPECT_TRUE(
      CompareMatrices(revolute_joint.position_upper_limits(), Vector1d(2)));
  EXPECT_TRUE(
      CompareMatrices(revolute_joint.velocity_lower_limits(), Vector1d(-100)));
  EXPECT_TRUE(
      CompareMatrices(revolute_joint.velocity_upper_limits(), Vector1d(100)));

  const Joint<double>& prismatic_joint =
      plant.GetJointByName("prismatic_joint");
  EXPECT_TRUE(
      CompareMatrices(prismatic_joint.position_lower_limits(), Vector1d(-2)));
  EXPECT_TRUE(
      CompareMatrices(prismatic_joint.position_upper_limits(), Vector1d(1)));
  EXPECT_TRUE(
      CompareMatrices(prismatic_joint.velocity_lower_limits(), Vector1d(-5)));
  EXPECT_TRUE(
      CompareMatrices(prismatic_joint.velocity_upper_limits(), Vector1d(5)));

  const Joint<double>& no_limit_joint =
      plant.GetJointByName("revolute_joint_no_limits");
  const Vector1d inf(std::numeric_limits<double>::infinity());
  const Vector1d neg_inf(-std::numeric_limits<double>::infinity());

  EXPECT_TRUE(CompareMatrices(no_limit_joint.position_lower_limits(), neg_inf));
  EXPECT_TRUE(CompareMatrices(no_limit_joint.position_upper_limits(), inf));
  EXPECT_TRUE(CompareMatrices(no_limit_joint.velocity_lower_limits(), neg_inf));
  EXPECT_TRUE(CompareMatrices(no_limit_joint.velocity_upper_limits(), inf));
}

GTEST_TEST(MultibodyPlantUrdfParserTest, CollisionFilterGroupParsingTest) {
  const std::string full_name = FindResourceOrThrow(
      "drake/multibody/parsing/test/urdf_parser_test/"
      "collision_filter_group_parsing_test.urdf");
  PackageMap package_map;
  package_map.PopulateUpstreamToDrake(full_name);

  MultibodyPlant<double> plant;
  SceneGraph<double> scene_graph;
  AddModelFromUrdfFile(full_name, "", package_map, &plant, &scene_graph);

  // Get geometry ids for all the bodies.
  const geometry::SceneGraphInspector<double>& inspector =
      scene_graph.model_inspector();
  const auto geometry_id_link1 = inspector.GetGeometryIdByName(
      plant.GetBodyFrameIdOrThrow(plant.GetBodyByName("link1").index()),
      geometry::Role::kProximity,
      "collision_filter_group_parsing_test::link1_sphere");
  const auto geometry_id_link2 = inspector.GetGeometryIdByName(
      plant.GetBodyFrameIdOrThrow(plant.GetBodyByName("link2").index()),
      geometry::Role::kProximity,
      "collision_filter_group_parsing_test::link2_sphere");
  const auto geometry_id_link3 = inspector.GetGeometryIdByName(
      plant.GetBodyFrameIdOrThrow(plant.GetBodyByName("link3").index()),
      geometry::Role::kProximity,
      "collision_filter_group_parsing_test::link3_sphere");
  const auto geometry_id_link4 = inspector.GetGeometryIdByName(
      plant.GetBodyFrameIdOrThrow(plant.GetBodyByName("link4").index()),
      geometry::Role::kProximity,
      "collision_filter_group_parsing_test::link4_sphere");

  // Make sure the plant is not finalized such that the adjacent joint filter
  // has not taken into effect yet. This guarantees that the collision filtering
  // is applied due to the collision filter group parsing.
  ASSERT_FALSE(plant.is_finalized());

  // We have four geometries and six possible pairs, each with a particular
  // disposition.
  // (1, 2) - unfiltered
  // (1, 3) - filtered by group_link_3 ignores group_link_14
  // (1, 4) - filtered by group_link_14 ignores itself
  // (2, 3) - filtered by group_link_2 ignores group_link_3
  // (2, 4) - unfiltered (although declared in an *ignored* self-filtering
  // group_link_24).
  // (3, 4) - filtered by group_link_3 ignores group_link_14
  EXPECT_FALSE(
      inspector.CollisionFiltered(geometry_id_link1, geometry_id_link2));
  EXPECT_TRUE(
      inspector.CollisionFiltered(geometry_id_link1, geometry_id_link3));
  EXPECT_TRUE(
      inspector.CollisionFiltered(geometry_id_link1, geometry_id_link4));
  EXPECT_TRUE(
      inspector.CollisionFiltered(geometry_id_link2, geometry_id_link3));
  EXPECT_FALSE(
      inspector.CollisionFiltered(geometry_id_link2, geometry_id_link4));
  EXPECT_TRUE(
      inspector.CollisionFiltered(geometry_id_link3, geometry_id_link4));
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
