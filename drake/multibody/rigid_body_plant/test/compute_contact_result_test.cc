/* clang-format off to disable clang-format-includes */
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
/* clang-format on */

#include <memory>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/joints/quaternion_floating_joint.h"
#include "drake/multibody/rigid_body.h"
#include "drake/multibody/rigid_body_plant/test/contact_result_test_common.h"
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

// Base class for testing the RigidBodyPlant's logic for populating its
// output port for collision response data.
class ContactResultTest : public ContactResultTestCommon {
 protected:
  // Runs the test on the RigidBodyPlant.
  const ContactResults<double>& RunTest(double distance) {
    // Populate the plant.
    plant_ = make_unique<RigidBodyPlant<double>>(GenerateTestTree(distance));

    plant_->set_default_compliant_material(MakeDefaultMaterial());

    systems::CompliantContactModelParameters model_parameters;
    model_parameters.characteristic_area = kContactArea;
    model_parameters.v_stiction_tolerance = kVStictionTolerance;
    plant_->set_contact_model_parameters(model_parameters);

    context_ = plant_->CreateDefaultContext();
    output_ = plant_->AllocateOutput(*context_);
    plant_->CalcOutput(*context_.get(), output_.get());

    const int port_index = plant_->contact_results_output_port().get_index();
    contact_results_ =
        output_->get_data(port_index)->GetValue<ContactResults<double>>();
    return contact_results_;
  }

  // Returns a constant reference to the RigidBodyTree that is within the plant.
  const RigidBodyTree<double>& GetTree() {
    return plant_->get_rigid_body_tree();
  }

  // instances owned by the test class
  unique_ptr<RigidBodyPlant<double>> plant_{};
  unique_ptr<Context<double>> context_{};
  unique_ptr<SystemOutput<double>> output_{};
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
  drake::multibody::collision::ElementId e1 = info.get_element_id_1();
  drake::multibody::collision::ElementId e2 = info.get_element_id_2();
  const RigidBody<double>* b1 = GetTree().FindBody(e1);
  const RigidBody<double>* b2 = GetTree().FindBody(e2);
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

  // NOTE: the *effective* Young's modulus of the contact is half of the
  // material Young's modulus.
  const double effective_elasticity = kContactYoungsModulus * 0.5;
  // NOTE: Because there is zero velocity, there is no frictional force and no
  // damping on the normal force.  Simply the kx term.  Penetration is twice
  // the offset.
  double force = effective_elasticity * offset * 2;
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
  ASSERT_TRUE(
      CompareMatrices(detail_force.get_application_point(), expected_point));
}
}  // namespace
}  // namespace test
}  // namespace rigid_body_plant
}  // namespace plants
}  // namespace systems
}  // namespace drake
