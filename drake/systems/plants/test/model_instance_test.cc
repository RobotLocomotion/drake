#include <gtest/gtest.h>

#include "drake/systems/plants/joints/RevoluteJoint.h"
#include "drake/systems/plants/model_instance.h"

namespace drake {
namespace systems {
namespace plants {

// Tests the basic functionality of ModelElementId.
GTEST_TEST(ModelInstanceTest, BasicTest) {
  ModelInstance model_instance("Foo_Instance");
  EXPECT_EQ(model_instance.get_model_intance_name(), "Foo_Instance");

  model_instance.set_model_name("Bar_Model");
  EXPECT_EQ(model_instance.get_model_name(), "Bar_Model");

  // Tests the ability to add a DrakeJoint to the ModelInstance.
  {
    std::string joint_name = "Baz_Joint";
    Eigen::Isometry3d transform_to_parent_body = Eigen::Isometry3d::Identity();
    Eigen::Vector3d rotation_axis;
    rotation_axis << 1, 0, 0;

    std::unique_ptr<DrakeJoint> drake_joint(new RevoluteJoint(joint_name,
        transform_to_parent_body, rotation_axis));
    model_instance.add_joint(std::move(drake_joint));

    const std::vector<std::unique_ptr<DrakeJoint>>& joints =
        model_instance.get_joints();
    EXPECT_EQ(joints.size(), 1);
    EXPECT_EQ(joints[0]->getName(), joint_name);
    EXPECT_EQ(joints[0]->getNumPositions(), 1);
    EXPECT_EQ(joints[0]->getNumVelocities(), 1);
    EXPECT_FALSE(joints[0]->isFloating());
  }

  // TODO(liang.fok) Make this unit test more rigorous.
}


}  // namespace plants
}  // namespace systems
}  // namespace drake
