#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/sdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {

using Eigen::Isometry3d;
using Eigen::Vector3d;
using multibody::joints::FloatingBaseType;
using parsers::sdf::AddModelInstancesFromSdfFileToWorld;

namespace parsers {
namespace {

// TODO(eric.cousineau): Figure out a core location for a sugar method like
// this.
Isometry3d XyzRpy(const Vector3d& xyz, const Vector3d& rpy) {
  return math::RigidTransform<double>(
      math::RollPitchYaw<double>(rpy).ToRotationMatrix(), xyz).GetAsIsometry3();
}

GTEST_TEST(SdfParserTest, ParseFrames) {
  // Minimal test of backporting *proper* frame parsing from MultibodyPlant to
  // RigidBodyTree.
  RigidBodyTree<double> tree;
  const auto id_table = AddModelInstancesFromSdfFileToWorld(
      FindResourceOrThrow(
          "drake/attic/multibody/parsers/test/rbt_sdf_parser_test_model.sdf"),
      FloatingBaseType::kExperimentalMultibodyPlantStyle, &tree);
  ASSERT_EQ(id_table.size(), 2);
  const int world_instance = tree.world().get_model_instance_id();
  ASSERT_EQ(id_table.at("dummy_model"), world_instance);
  const int test_instance = id_table.at("sdf_parser_test_model");

  // Check model scope frames.
  // Backport from MultibodyPlant test:
  //   multibody/multibody_tree/parsing/test/multibody_plant_sdf_parser_test.cc
  //   at line 158, git sha 5687173
  auto cache = tree.doKinematics(tree.getZeroConfiguration());
  const double eps = std::numeric_limits<double>::epsilon();
  auto check_frame = [&tree, test_instance, &cache, eps](
      std::shared_ptr<RigidBodyFrame<double>> parent_frame, std::string name,
      const Isometry3d& X_PF_expected) {
    std::shared_ptr<RigidBodyFrame<double>> frame = tree.findFrame(
        name, test_instance);
    const Isometry3d X_PF = tree.relativeTransform(
        cache, parent_frame->get_frame_index(), frame->get_frame_index());
    EXPECT_TRUE(CompareMatrices(X_PF_expected.matrix(), X_PF.matrix(), eps))
        << name;
  };

  const Isometry3d X_L1F1 = XyzRpy(
      Vector3d(0.1, 0.2, 0.3), Vector3d(0.4, 0.5, 0.6));
  check_frame(
      tree.findFrame("link1", test_instance),
      "model_scope_link1_frame", X_L1F1);
  const Isometry3d X_F1F2 = XyzRpy(Vector3d(0.1, 0.0, 0.0), Vector3d::Zero());
  check_frame(
      tree.findFrame("model_scope_link1_frame", test_instance),
      "model_scope_link1_frame_child", X_F1F2);
  // TODO(eric.cousineau): Consider using model frame.
  const Isometry3d X_MF3 = XyzRpy(Vector3d(0.7, 0.8, 0.9), Vector3d::Zero());
  auto world_frame = tree.findFrame("world", world_instance);
  check_frame(world_frame, "model_scope_model_frame_implicit", X_MF3);
  const Isometry3d X_WF4 = XyzRpy(Vector3d(1.1, 1.2, 1.3), Vector3d::Zero());
  check_frame(world_frame, "model_scope_world_frame", X_WF4);

  // Old-style frame:
  const Isometry3d X_L1O1 = XyzRpy(Vector3d(1, 2, 3), Vector3d(4, 5, 6));
  check_frame(
      tree.findFrame("link1", test_instance), "old_style_frame", X_L1O1);
}

}  // namespace
}  // namespace parsers
}  // namespace drake
