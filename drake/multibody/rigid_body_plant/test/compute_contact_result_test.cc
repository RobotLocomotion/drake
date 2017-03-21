/* clang-format off */
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
/* clang-format on */

#include <memory>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "drake/common/eigen_matrix_compare.h"
#include "drake/multibody/joints/quaternion_floating_joint.h"
#include "drake/multibody/rigid_body.h"
#include "drake/multibody/rigid_body_tree.h"

// The ContactResult class is largely a container for the data that is computed
// by the RigidBodyPlant while determining contact forces.  This test confirms
// that for a known set of contacts, that the expected contact forces are
// generated and stashed into the ContactResult data structure.
//
// Thus, a rigid body tree is created with a known configuration such that the
// contacts and corresponding contact forces are known.  The RigidBodyPlant's
// CalcOutput is invoked on the ContactResult port and the ContactResult
// contents are evaluated to see if they contain the expected results.

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

  // Contact parameters
  const double kStiffness = 150;
  const double kDissipation = 2.0;
  const double kStaticFriction = 0.9;
  const double kDynamicFriction = 0.5;
  const double kVStictionTolerance = 0.01;

  // Places two spheres are on the x-y plane mirrored across the origin from
  //  each other such there is 2 * `distance` units gap between them.  Negative
  //  numbers imply collision.
  const ContactResults<double>& RunTest(double distance) {
    auto unique_tree = make_unique<RigidBodyTree<double>>();
    tree_ = unique_tree.get();

    x_anchor_ = 1.5;
    Vector3d pos(x_anchor_ - (kRadius + distance), 0, 0);
    body1_ = AddSphere(pos, "sphere1");
    pos << x_anchor_ + (kRadius + distance), 0, 0;
    body2_ = AddSphere(pos, "sphere2");

    tree_->compile();

    // Populate the plant.
    // Note: This is done here instead of the SetUp method because it appears
    //  the plant requires a *compiled* tree at constructor time.
    plant_ = make_unique<RigidBodyPlant<double>>(move(unique_tree));
    plant_->set_normal_contact_parameters(kStiffness, kDissipation);
    plant_->set_friction_contact_parameters(kStaticFriction, kDynamicFriction,
                                            kVStictionTolerance);
    context_ = plant_->CreateDefaultContext();
    output_ = plant_->AllocateOutput(*context_);
    plant_->CalcOutput(*context_.get(), output_.get());

    const int port_index = plant_->contact_results_output_port().get_index();
    return output_->get_data(port_index)->GetValue<ContactResults<double>>();
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
  ASSERT_TRUE((b1 == body1_ && b2 == body2_) || (b1 == body2_ && b2 == body1_));

  // The direction of the force depends on which body is 1 and which is 2. We
  // assume b1 is body1_, if not, we reverse the sign of the force.
  double force_sign = -1;
  if (b2 == body1_) {
    force_sign = 1;
  }

  // Confirms the contact details are as expected.
  const auto& resultant = info.get_resultant_force();
  SpatialForce<double> expected_spatial_force;
  // Note: This is fragile. It assumes a particular collision model.  Once the
  // model has been generalized, this will have to adapt to account for that.

  // NOTE: Because there is zero velocity, there is no frictional force and no
  // damping on the normal force.  Simply the kx term.  Penetration is twice
  // the offset.
  double force = kStiffness * offset * 2;
  expected_spatial_force << 0, 0, 0, force_sign * force, 0, 0;
  ASSERT_TRUE(
      CompareMatrices(resultant.get_spatial_force(), expected_spatial_force));

  const auto& details = info.get_contact_details();
  ASSERT_EQ(details.size(), 1u);
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
