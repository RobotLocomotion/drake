#pragma once

#include <memory>
#include <string>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "drake/common/eigen_matrix_compare.h"
#include "drake/multibody/joints/quaternion_floating_joint.h"
#include "drake/multibody/rigid_body.h"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace systems {
namespace plants {
namespace rigid_body_plant {
namespace test {

// Utility function to facilitate comparing matrices for equivalency.
template <typename DerivedA, typename DerivedB>
bool CompareMatrices(const Eigen::MatrixBase<DerivedA>& m1,
                     const Eigen::MatrixBase<DerivedB>& m2) {
  return CompareMatrices(m1, m2, Eigen::NumTraits<double>::dummy_precision(),
                         MatrixCompareType::absolute);
}

// Base class for testing the CompliantContactModel class as well as the
// RigidBodyPlant's logic for populating its output port for collision response
// data.
class ContactResultTestCommon : public ::testing::Test {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ContactResultTestCommon)

  // Default constructor.
  ContactResultTestCommon() {}

 protected:
  // These pointers are merely reference pointers; the underlying instances
  //  are owned by objects which, ultimately, are owned by the test class.
  RigidBody<double>* body1_{};
  RigidBody<double>* body2_{};
  RigidBodyTree<double>* tree_{};

  // The point around which the test is run. Each sphere is offset along the
  //  x-axis from this point.
  double x_anchor_{};

  ContactResults<double> contact_results_{};

  const double kRadius = 1.0;

  // Contact parameters
  const double kStiffness = 150;
  const double kDissipation = 2.0;
  const double kStaticFriction = 0.9;
  const double kDynamicFriction = 0.5;
  const double kVStictionTolerance = 0.01;

  // Places two spheres are on the x-y plane mirrored across the origin from
  // each other such there is 2 * `distance` units gap between them.  Negative
  // numbers imply collision.
  std::unique_ptr<RigidBodyTree<double>> GenerateTestTree(double distance) {
    auto unique_tree = std::make_unique<RigidBodyTree<double>>();
    tree_ = unique_tree.get();

    x_anchor_ = 1.5;
    Eigen::Vector3d pos(x_anchor_ - (kRadius + distance), 0, 0);
    body1_ = AddSphere(pos, "sphere1");
    pos << x_anchor_ + (kRadius + distance), 0, 0;
    body2_ = AddSphere(pos, "sphere2");

    tree_->compile();
    return unique_tree;
  }

  // Add a sphere with default radius, placed at the given position.
  //  Returns a raw pointer so that tests can use it for result validation.
  RigidBody<double>* AddSphere(const Eigen::Vector3d& pos,
                               const std::string& name) {
    RigidBody<double>* body;
    tree_->add_rigid_body(
        std::unique_ptr<RigidBody<double>>(body = new RigidBody<double>()));
    body->set_name(name);
    body->set_mass(1.0);
    body->set_spatial_inertia(Matrix6<double>::Identity());
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translate(pos);
    body->add_joint(&tree_->world(),
                    std::make_unique<QuaternionFloatingJoint>("base", pose));
    DrakeShapes::Sphere sphere(kRadius);
    DrakeCollision::Element collision_element(sphere);
    collision_element.set_body(body);
    tree_->addCollisionElement(collision_element, *body, "group1");
    return body;
  }

  virtual const ContactResults<double>& RunTest(double distance) = 0;
};

}  // namespace test
}  // namespace rigid_body_plant
}  // namespace plants
}  // namespace systems
}  // namespace drake
