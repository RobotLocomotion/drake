#include "drake/multibody/multibody_tree/parsing/multibody_plant_urdf_parser.h"

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/multibody_tree/parsing/parser_common.h"

namespace drake {

using multibody::parsing::AddModelFromUrdfFile;
using Eigen::Vector3d;
using geometry::GeometryId;
using geometry::SceneGraph;
using multibody::parsing::AddModelFromUrdfFile;
using multibody::parsing::default_friction;

namespace multibody {
namespace multibody_plant {
namespace {

GTEST_TEST(MultibodyPlantUrdfParserTest, DoublePendulum) {
  MultibodyPlant<double> plant;
  SceneGraph<double> scene_graph;
  std::string full_name = FindResourceOrThrow(
      "drake/multibody/benchmarks/acrobot/double_pendulum.urdf");
  AddModelFromUrdfFile(full_name, &plant, &scene_graph);
  plant.Finalize();

  const MultibodyTree<double>& tree = plant.tree();
  EXPECT_EQ(tree.num_bodies(), 4);
  EXPECT_EQ(tree.num_frames(), 10);

  ASSERT_TRUE(tree.HasFrameNamed("frame_on_link1"));
  ASSERT_TRUE(tree.HasFrameNamed("frame_on_link2"));
  ASSERT_TRUE(tree.HasFrameNamed("link1_com"));
  ASSERT_TRUE(tree.HasFrameNamed("link2_com"));

  // Sample a couple of frames.
  const Frame<double>& frame_on_link1 = tree.GetFrameByName("frame_on_link1");
  EXPECT_EQ(frame_on_link1.body().index(), tree.GetBodyByName("link1").index());

  math::RollPitchYaw<double> rpy_expected(-1, 0.1, 0.2);
  Vector3d xyz_expected(0.8, -0.2, 0.3);
  math::RigidTransform<double> X_BF_expected(
      rpy_expected.ToRotationMatrix(), xyz_expected);

  EXPECT_TRUE(CompareMatrices(
      frame_on_link1.GetFixedPoseInBodyFrame().matrix(),
      X_BF_expected.GetAsMatrix4(), 1e-10));

  const Frame<double>& link2_com = tree.GetFrameByName("link2_com");
  EXPECT_EQ(link2_com.body().index(), tree.GetBodyByName("link2").index());
}

// This test verifies that we're able to successfully look up meshes using the
// "package://" syntax internally to the URDF (at least for packages which are
// successfully found in the same directory at the URDF.
GTEST_TEST(MultibodyPlantUrdfParserTest, TestAtlasMinimalContact) {
  MultibodyPlant<double> plant;
  SceneGraph<double> scene_graph;
  std::string full_name = FindResourceOrThrow(
      "drake/examples/atlas/urdf/atlas_minimal_contact.urdf");

  AddModelFromUrdfFile(full_name, &plant, &scene_graph);
  plant.Finalize();

  EXPECT_EQ(plant.num_positions(), 37);
  EXPECT_EQ(plant.num_velocities(), 36);
}

GTEST_TEST(MultibodyPlantUrdfParserTest, TestAddWithQuaternionFloatingDof) {
  const std::string resource_dir{
    "drake/multibody/multibody_tree/parsing/test/urdf_parser_test/"};
  const std::string model_file = FindResourceOrThrow(
      resource_dir + "zero_dof_robot.urdf");

  MultibodyPlant<double> plant;
  SceneGraph<double> scene_graph;
  AddModelFromUrdfFile(model_file, &plant, &scene_graph);
  plant.Finalize(&scene_graph);

  EXPECT_EQ(plant.num_positions(), 7);
  EXPECT_EQ(plant.num_velocities(), 6);
}

GTEST_TEST(MultibodyPlantUrdfParserTest, TestOptionalSceneGraph) {
  const std::string full_name = FindResourceOrThrow(
      "drake/examples/atlas/urdf/atlas_minimal_contact.urdf");
  int num_visuals_explicit{};
  {
    // Test explicitly specifying `scene_graph`.
    MultibodyPlant<double> plant;
    SceneGraph<double> scene_graph;
    AddModelFromUrdfFile(full_name, &plant, &scene_graph);
    plant.Finalize();
    num_visuals_explicit = plant.num_visual_geometries();
  }
  EXPECT_NE(num_visuals_explicit, 0);
  {
    // Test implicitly specifying.
    MultibodyPlant<double> plant;
    SceneGraph<double> scene_graph;
    plant.RegisterAsSourceForSceneGraph(&scene_graph);
    AddModelFromUrdfFile(full_name, &plant);
    plant.Finalize();
    EXPECT_EQ(plant.num_visual_geometries(), num_visuals_explicit);
  }
}

}  // namespace
}  // namespace multibody_plant
}  // namespace multibody
}  // namespace drake
