#pragma once

#include <memory>
#include <string>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
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
::testing::AssertionResult CompareMatrices(
    const Eigen::MatrixBase<DerivedA>& m1,
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

  ContactResultTestCommon() {}

  /// Computes the default material properties for derived classes to set.
  CompliantMaterial MakeDefaultMaterial() {
    CompliantMaterial material;
    material.set_youngs_modulus(kYoungsModulus);
    material.set_dissipation(kDissipation);
    material.set_friction(kStaticFriction, kDynamicFriction);
    return material;
  }

 private:
  // Compliant _material_ parameters applied to each *object*. These are private
  // to make sure they are not confused as contact parameters in derived tests
  // (see below).
  const double kYoungsModulus = 150;  // Pa
  const double kDissipation = 2.0;  // s/m
  const double kStaticFriction = 0.9;
  const double kDynamicFriction = 0.5;

 protected:
  const double kRadius = 1.0;  // m

  // The *material* parameters given above are *not* the exact values that
  // determine the compliant contact force. The actual computation uses net
  // values derived from material elasticity, dissipation, and friction
  // coefficients. Generally, the value for the contact will be different than
  // the per-material values. That said, in these tests we're using the same
  // materials on all objects. As such, the contact parameters for dissipation,
  // and friction are the same, but the elasticity is *half*. That is captured
  // here. This is not generally true for contact between objects with different
  // compliant material parameters. See contact_model_doxygen.h for details.
  const double kContactYoungsModulus = kYoungsModulus;
  const double kContactDissipation = kDissipation;
  const double kContactStaticFriction = kStaticFriction;
  const double kConstantDynamicFriction = kDynamicFriction;
  const double kVStictionTolerance = 0.01;  // m/s
  const double kContactArea = 1.0;  // m^2

  // Places two spheres on the x-y plane mirrored across the x_anchor_ from
  // each other such there is 2 * `distance` units gap between them.  Negative
  // numbers imply collision.
  std::unique_ptr<RigidBodyTree<double>> GenerateTestTree(double distance) {
    auto unique_tree = std::make_unique<RigidBodyTree<double>>();

    x_anchor_ = 1.5;
    Eigen::Vector3d pos(x_anchor_ - (kRadius + distance), 0, 0);
    body1_ = AddSphere(unique_tree.get(), pos, "sphere1");
    pos << x_anchor_ + (kRadius + distance), 0, 0;
    body2_ = AddSphere(unique_tree.get(), pos, "sphere2");

    unique_tree->compile();
    DoGenerateTestTree(unique_tree.get());
    return unique_tree;
  }

  // Opportunity for sub-classes to manipulate the tree *after* it is compiled.
  virtual void DoGenerateTestTree(RigidBodyTree<double>* tree) {}

  // Add a sphere with default radius, placed at the given position.
  //  Returns a raw pointer so that tests can use it for result validation.
  RigidBody<double>* AddSphere(RigidBodyTree<double> *tree,
      const Eigen::Vector3d& pos, const std::string& name) {
    RigidBody<double>* body;
    tree->add_rigid_body(
        std::unique_ptr<RigidBody<double>>(body = new RigidBody<double>()));
    body->set_name(name);
    body->set_mass(1.0);
    body->set_spatial_inertia(Matrix6<double>::Identity());
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translate(pos);
    body->add_joint(&tree->world(),
                    std::make_unique<QuaternionFloatingJoint>("base", pose));
    DrakeShapes::Sphere sphere(kRadius);
    drake::multibody::collision::Element collision_element(sphere);
    collision_element.set_body(body);
    tree->addCollisionElement(collision_element, *body, "group1");
    return body;
  }

  // Abstract method to be implemented by child classes.
  virtual const ContactResults<double>& RunTest(double distance) = 0;

  // These pointers are merely reference pointers; the underlying instances
  //  are owned by objects which, ultimately, are owned by the test class.
  RigidBody<double>* body1_{};
  RigidBody<double>* body2_{};

  // The point around which the test is run. Each sphere is offset along the
  //  x-axis from this point.
  double x_anchor_{};

  ContactResults<double> contact_results_{};
};

}  // namespace test
}  // namespace rigid_body_plant
}  // namespace plants
}  // namespace systems
}  // namespace drake
