#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"

#include <map>
#include <string>

#include "drake/common/drake_assert.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

void AddGround(RigidBodyTree* tree) {
  double kBoxWidth = 3;
  double kBoxDepth = 0.2;
  DrakeShapes::Box geom(Eigen::Vector3d(kBoxWidth, kBoxWidth, kBoxDepth));
  Eigen::Isometry3d T_element_to_link = Eigen::Isometry3d::Identity();
  T_element_to_link.translation() << 0, 0,
      -kBoxDepth / 2.0;  // top of the box is at z = 0

  RigidBody& world = tree->world();
  Eigen::Vector4d color;
  color << 0.9297, 0.7930, 0.6758, 1;
  world.AddVisualElement(
      DrakeShapes::VisualElement(geom, T_element_to_link, color));
  tree->addCollisionElement(
      DrakeCollision::Element(geom, T_element_to_link, &world), world,
      "terrain");
  tree->updateStaticCollisionElements();
}

void VerifyIiwaTree(const RigidBodyTree& tree) {
  std::map<std::string, int> name_to_idx =
      tree.computePositionNameToIndexMap();

  int joint_idx = 0;
  DRAKE_DEMAND(name_to_idx.count("iiwa_joint_1"));
  DRAKE_DEMAND(name_to_idx["iiwa_joint_1"] == joint_idx++);
  DRAKE_DEMAND(name_to_idx.count("iiwa_joint_2"));
  DRAKE_DEMAND(name_to_idx["iiwa_joint_2"] == joint_idx++);
  DRAKE_DEMAND(name_to_idx.count("iiwa_joint_3"));
  DRAKE_DEMAND(name_to_idx["iiwa_joint_3"] == joint_idx++);
  DRAKE_DEMAND(name_to_idx.count("iiwa_joint_4"));
  DRAKE_DEMAND(name_to_idx["iiwa_joint_4"] == joint_idx++);
  DRAKE_DEMAND(name_to_idx.count("iiwa_joint_5"));
  DRAKE_DEMAND(name_to_idx["iiwa_joint_5"] == joint_idx++);
  DRAKE_DEMAND(name_to_idx.count("iiwa_joint_6"));
  DRAKE_DEMAND(name_to_idx["iiwa_joint_6"] == joint_idx++);
  DRAKE_DEMAND(name_to_idx.count("iiwa_joint_7"));
  DRAKE_DEMAND(name_to_idx["iiwa_joint_7"] == joint_idx++);
}

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
