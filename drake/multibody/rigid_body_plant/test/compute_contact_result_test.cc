#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"

#include <memory>

#include <gtest/gtest.h>
#include <Eigen/Geometry>

#include "drake/common/eigen_matrix_compare.h"
#include "drake/multibody/rigid_body.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/joints/quaternion_floating_joint.h"

// The ContactResult class is largely a container for the data that is computed
//  by the RigidBodyPlant while determining contact forces.  This test confirms
//  that for a known set of contacts, that the expected contact forces are
//  generated and stashed into the ContactResult data structure.
//
//  Thus, a rigid body tree is created with a known configuration such that the
//  contacts and corresponding contact forces are known.  The RigidBodyPlant's
//  EvalOutput is invoked on the ContactResult port and the ContactResult
//  contents are evaluated to see if they contain the expected results.

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
namespace {

// Utility function to create an input port.
template <class T>
unique_ptr<FreestandingInputPort> MakeInput(
    std::unique_ptr<BasicVector<T>> data) {
  return make_unique<FreestandingInputPort>(std::move(data));
}

// Utility function to facilitate comparing matrices for equivalency.
template <typename DerivedA, typename DerivedB>
bool CompareMatrices(const Eigen::MatrixBase<DerivedA>& m1,
                     const Eigen::MatrixBase<DerivedB>& m2) {
  return CompareMatrices(m1, m2, Eigen::NumTraits<double>::dummy_precision(),
                         MatrixCompareType::absolute);
}

// Base class for testing the RigidBodyPlant's logic for populating its
// output port for collision response data.
class ContactResultTest : public ::testing::Test {
 protected:
  // These pointers are merely reference pointers; the underlying instances
  //  are owned by objects which, ultimately, are owned by the test class.
  RigidBody<double>* body1_{};
  RigidBody<double>* body2_{};
  RigidBodyTree<double>* tree_{};

  // The point around which the test is run. Each sphere is offset along the
  //  x-axis from this point.
  double x_anchor_{};

  // instances owned by the test class
  unique_ptr<RigidBodyPlant<double>> plant_{};
  unique_ptr<Context<double>> context_{};
  unique_ptr<SystemOutput<double>> output_{};
  const double kRadius = 1.0;

  // Places two spheres are on the x-y plane mirrored across the origin from
  //  each other such there is 2 * `distance` units gap between them.  Negative
  //  numbers imply collision.
  const ContactResults<double>& RunTest(double distance) {
    auto unique_tree = make_unique<RigidBodyTree<double>>();
    tree_ = unique_tree.get();

    x_anchor_ = 1.5;
    Vector3d pos;
    pos << x_anchor_ - (kRadius + distance), 0, 0;
    body1_ = AddSphere(pos, "sphere1");
    pos << x_anchor_ + (kRadius + distance), 0, 0;
    body2_ = AddSphere(pos, "sphere2");

    tree_->compile();

    // Populate the plant.
    // Note: This is done here instead of the SetUp method because it appears
    //  the plant requires a *compiled* tree at constructor time.
    plant_ = make_unique<RigidBodyPlant<double>>(move(unique_tree));
    context_ = plant_->CreateDefaultContext();
    output_ = plant_->AllocateOutput(*context_);
    context_->SetInputPort(0, MakeInput(make_unique<BasicVector<double>>(0)));
    plant_->SetZeroConfiguration(context_.get());
    plant_->EvalOutput(*context_.get(), output_.get());

    // TODO(SeanCurtis-TRI): This hard-coded value is unfortunate. However,
    //  there is no mechanism for finding out the port id for a known port
    //  (e.g., contact results). Update when such a mechanism exists.
    return output_->get_data(2)->GetValue<ContactResults<double>>();
  }

  // Add a sphere with default radius, placed at the given position.
  //  Returns a raw pointer so that tests can use it for result validation.
  RigidBody<double>* AddSphere(const Vector3d& pos, const std::string& name) {
    RigidBody<double>* body;
    tree_->add_rigid_body(
        unique_ptr<RigidBody<double>>(body = new RigidBody<double>()));
    body->set_name(name);
    body->set_mass(1.0);
    body->set_spatial_inertia(Matrix6<double>::Identity());
    Isometry3d pose = Isometry3d::Identity();
    pose.translate(pos);
    body->add_joint(&tree_->world(),
                    make_unique<QuaternionFloatingJoint>("base", pose));
    DrakeShapes::Sphere sphere(kRadius);
    DrakeCollision::Element collision_element(sphere);
    collision_element.set_body(body);
    tree_->addCollisionElement(collision_element, *body, "group1");
    return body;
  }
};

// Confirms a contact result for two non-colliding spheres -- expects no
// reported collisions.
TEST_F(ContactResultTest, NoCollision) {
  auto& contact_results = RunTest(0.1);
  ASSERT_EQ(contact_results.get_num_contacts(), 0);
}

// Confirms a contact result for two touching spheres -- expects no reported
// collisions. For now, osculation is not considered a "contact" for reporting
// purposes. If the definition changes, this will likewise change.
TEST_F(ContactResultTest, Touching) {
  auto& contact_results = RunTest(0.0);
  ASSERT_EQ(contact_results.get_num_contacts(), 0);
}

// Confirms a contact result for two colliding spheres.
TEST_F(ContactResultTest, SingleCollision) {
  double offset = 0.1;
  auto& contact_results = RunTest(-offset);
  ASSERT_EQ(contact_results.get_num_contacts(), 1);
  const auto info = contact_results.get_contact_info(0);

  // Confirms that the proper bodies are in contact.
  DrakeCollision::ElementId e1 = info.get_element_id_1();
  DrakeCollision::ElementId e2 = info.get_element_id_2();
  const RigidBody<double>* b1 = tree_->FindBody(e1);
  const RigidBody<double>* b2 = tree_->FindBody(e2);
  ASSERT_NE(e1, e2);
  ASSERT_TRUE(b1 == body1_ || b1 == body2_);
  ASSERT_TRUE(b2 == body1_ || b2 == body2_);

  // Confirms the contact details are as expected.
  const auto& resultant = info.get_resultant_force();
  SpatialForce<double> expected_spatial_force;
  // Note: This is fragile.  This is the value copied from rigid_body_plant.h
  //  If the hard-coded value changes, or the code changes for the value to
  //  be set in some other manner, then this test may fail.
  const double stiffness = 150.0;
  double force = stiffness * offset * 2;
  expected_spatial_force << 0, 0, 0, -force, 0, 0;
  ASSERT_TRUE(
      CompareMatrices(resultant.get_spatial_force(), expected_spatial_force));

  const auto& details = info.get_contact_details();
  ASSERT_EQ(details.size(), 1);
  auto detail_force = details[0]->ComputeContactForce();
  ASSERT_TRUE(CompareMatrices(detail_force.get_spatial_force(),
                              expected_spatial_force));
  Vector3<double> expected_point;
  expected_point << x_anchor_, 0, 0;
  ASSERT_TRUE(CompareMatrices(detail_force.get_application_point(),
                              expected_point));
}
}  // namespace
}  // namespace test
}  // namespace rigid_body_plant
}  // namespace plants
}  // namespace systems
}  // namespace drake
