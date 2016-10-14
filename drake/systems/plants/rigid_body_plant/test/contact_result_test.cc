#include <iostream>
#include <memory>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/eigen_types.h"
#include <drake/systems/plants/RigidBody.h>
#include <drake/systems/plants/RigidBodyTree.h>
#include "drake/systems/plants/collision/DrakeCollision.h"
#include <drake/systems/plants/joints/QuaternionFloatingJoint.h>
#include <drake/systems/plants/rigid_body_plant/rigid_body_plant.h>

using Eigen::Isometry3d;
using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;
using std::make_unique;
using std::move;
using std::unique_ptr;

namespace drake {
namespace systems {
namespace plants {
namespace rigid_body_plant {
namespace test {

template <class T>
std::unique_ptr<FreestandingInputPort> MakeInput(
    std::unique_ptr<BasicVector<T>> data) {
  return make_unique<FreestandingInputPort>(std::move(data));
}

template <typename DerivedA, typename DerivedB>
bool CompareMatrices(const Eigen::MatrixBase<DerivedA>& m1,
                     const Eigen::MatrixBase<DerivedB>& m2) {
  return CompareMatrices(m1, m2, 1e-14 /*threshold*/,
                         MatrixCompareType::absolute);
}

class ContactResultTest : public ::testing::Test {
 protected:
  void SetUp() override {
  }

  // These pointers are merely reference pointers; the underlying instances
  //  are owned by objects which, ultimately, are owned by the test class.
  RigidBody* body1_{};
  RigidBody* body2_{};
  RigidBodyTree* tree_{};

  // instances owned by the test class
  unique_ptr<RigidBodyPlant<double>> plant_{};
  unique_ptr<Context<double>> context_{};
  unique_ptr<SystemOutput<double>> output_{};
  const double kRadius = 1.0;


  const ContactResults<double>& RunTest(double distance) {
    // Places two spheres are on the x-y plane mirrored across the origin from
    //  each other such there is `distance` units gap between them.  Negative
    //  numbers imply collision.

    tree_ = new RigidBodyTree();

    Vector3d pos;
    pos << -(kRadius + distance), 0, 0;
    body1_ = AddSphere(pos, "sphere1");
    pos << (kRadius + distance), 0, 0;
    body2_ = AddSphere(pos, "sphere2");

    tree_->compile();

    // Populate the plant
    plant_ = make_unique<RigidBodyPlant<double>>(
        move(unique_ptr<RigidBodyTree>(tree_)));
    context_ = plant_->CreateDefaultContext();
    output_ = plant_->AllocateOutput(*context_);
    context_->SetInputPort(0, MakeInput(
        make_unique<BasicVector<double>>(0)));
    plant_->SetZeroConfiguration(context_.get());
    plant_->EvalOutput(*context_.get(), output_.get());

    // TODO(SeanCurtis-TRI): This hard-coded value is unfortuante. However,
    //  there is no mechanism for finding out the port id for a known port
    //  (e.g., contact results).
    return output_->get_data(2)->GetValue<ContactResults<double>>();
  }

  // Add a sphere with default radius, placed at the given position.
  RigidBody* AddSphere(const Vector3d& pos, const std::string& name) {
    RigidBody *body;
    tree_->add_rigid_body(unique_ptr<RigidBody>(body = new RigidBody()));
    body->set_name(name);
    body->set_mass(1.0);
    body->set_spatial_inertia(Matrix6<double>::Identity());
    Isometry3d pose = Isometry3d::Identity();
    pose.translate(pos);
    body->add_joint(
        &tree_->world(),
        make_unique<QuaternionFloatingJoint>("base", pose));
    DrakeShapes::Sphere sphere(kRadius);
    DrakeCollision::Element cElement(sphere);
    cElement.set_body(body);
    tree_->addCollisionElement(cElement, *body, "group1");
    return body;
  }
};

// Confirms a contact result for two colliding spheres.
TEST_F(ContactResultTest, NoCollision) {
  auto& contact_results = RunTest(0.1);
  ASSERT_EQ(contact_results.get_num_contacts(), 0);
}

// Confirms a contact result for two colliding spheres.
TEST_F(ContactResultTest, Touching) {
  // For now, osculation is not considered a "contact" for reporting purposes.
  //  If the definition changes, this will likewise change.
  auto& contact_results = RunTest(0.0);
  ASSERT_EQ(contact_results.get_num_contacts(), 0);
}

// Confirms a contact result for two colliding spheres.
TEST_F(ContactResultTest, SingleCollision) {
  double displace = 0.1;
  auto& contact_results = RunTest(-displace);
  ASSERT_EQ(contact_results.get_num_contacts(), 1);
  const auto info = contact_results.get_contact_info(0);
  DrakeCollision::ElementId e1 = info.get_element_id_1();
  DrakeCollision::ElementId e2 = info.get_element_id_2();
  const ContactManifold<double>& manifold = info.get_contact_manifold();
  const RigidBody* b1 = tree_->FindBody(e1);
  const RigidBody* b2 = tree_->FindBody(e2);
  ASSERT_NE(e1, e2);
  ASSERT_TRUE(b1 == body1_ || b1 == body2_);
  ASSERT_TRUE(b2 == body1_ || b2 == body2_);
  ASSERT_EQ(manifold.get_num_contacts(), 1);
  auto detail = manifold.get_ith_contact(0);
  Vector3d expectedPt = Vector3d::Zero();
  ASSERT_TRUE(CompareMatrices(detail->get_application_point(), expectedPt));
  // note: this is fragile.  This is the value copied from rigid_body_plant.h
  //  THis value depends on
  //    a) What is hard-coded in that location.
  //    b) Any dynamic changes which may affect it.
  const double stiffness = 150.0;
  double force = stiffness * displace * 2;
  WrenchVector<double> expectedF;
  // force goes from body2 to body2
  expectedF << -force, 0, 0, 0, 0, 0;
  ASSERT_TRUE(CompareMatrices(detail->get_force(), expectedF));
}
}  // namespace test
}  // namespace rigid_body_plant
}  // namespace plants
}  // namespace systems
}  // namespace drake
