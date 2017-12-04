#include "drake/multibody/rigid_body_plant/compliant_contact_model.h"

#include <memory>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/joints/quaternion_floating_joint.h"
#include "drake/multibody/rigid_body_plant/compliant_material.h"
#include "drake/multibody/rigid_body_plant/test/contact_result_test_common.h"
#include "drake/multibody/rigid_body_tree.h"

// The CompliantContactModel is a class that wraps the algorithms that perform
// contact computations. The ContactResult class is largely a container for the
// data that is computed by the CompliantContactModel while determining contact
// forces.  This test confirms that for a known set of contacts, that the
// expected contact forces are generated and stashed into the ContactResult
// data structure.
//
// Thus, a rigid body tree is created with a known configuration such that the
// contacts and corresponding contact forces are known.
// The CompliantContactModel's ComputeContactForce method is evaluated to
// populate the ContactResult, the contents of which are evaluated to see if
// they contain the expected results.

using Eigen::Isometry3d;
using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;
using drake::multibody::collision::Element;
using std::make_unique;
using std::move;
using std::unique_ptr;
using std::shared_ptr;

namespace drake {
namespace systems {
namespace plants {
namespace rigid_body_plant {
namespace test {
namespace {

// Base class for testing the CompliantContactModel logic for contact force
// computations.
class CompliantContactModelTest : public ContactResultTestCommon {
 protected:
  const ContactResults<double>& RunTest(double distance) override {
    unique_tree_ = GenerateTestTree(distance);
    // Populate the CompliantContactModel.
    compliant_contact_model_ =
        make_unique<CompliantContactModel<double>>();
    CompliantMaterial material = MakeDefaultMaterial();
    compliant_contact_model_->set_default_material(material);
    CompliantContactModelParameters contact_parameters;
    contact_parameters.v_stiction_tolerance = kVStictionTolerance;
    contact_parameters.characteristic_area = kContactArea;
    compliant_contact_model_->set_model_parameters(contact_parameters);

    // The state to test is the default state of the tree (0 velocities
    // and default configuration positions of the tree)

    VectorX<double> q0 = VectorX<double>::Zero(
        unique_tree_->get_num_positions());

    VectorXd v0 = VectorXd::Zero(unique_tree_->get_num_velocities());

    q0 = unique_tree_->getZeroConfiguration();
    auto kinsol = unique_tree_->doKinematics(q0, v0);

    compliant_contact_model_->ComputeContactForce(*unique_tree_.get(), kinsol,
                                                  &contact_results_);
    return contact_results_;
  }

  // Instances owned by the test class.
  unique_ptr<CompliantContactModel<double>> compliant_contact_model_{};
  // Holds the unique pointer to the tree.
  unique_ptr<RigidBodyTree<double>> unique_tree_{};
};

// Confirms a contact result for two non-colliding spheres -- expects no
// reported collisions.
TEST_F(CompliantContactModelTest, ModelNoCollision) {
  auto& contact_results = RunTest(0.1);
  ASSERT_EQ(contact_results.get_num_contacts(), 0);
}

// Confirms a contact result for two touching spheres -- expects no reported
// collisions. For now, osculation is not considered a "contact" for reporting
// purposes. If the definition changes, this will likewise change.
TEST_F(CompliantContactModelTest, ModelTouching) {
  auto& contact_results = RunTest(0.0);
  ASSERT_EQ(contact_results.get_num_contacts(), 0);
}

// Confirms a contact result for two colliding spheres with identical contact
// material and zero relative velocity.
TEST_F(CompliantContactModelTest, ModelSingleCollision) {
  const double offset = 0.1;
  auto& contact_results = RunTest(-offset);
  ASSERT_EQ(contact_results.get_num_contacts(), 1);
  const auto info = contact_results.get_contact_info(0);

  // Confirms that the proper bodies are in contact.
  drake::multibody::collision::ElementId e1 = info.get_element_id_1();
  drake::multibody::collision::ElementId e2 = info.get_element_id_2();
  const RigidBody<double>* b1 = unique_tree_->FindBody(e1);
  const RigidBody<double>* b2 = unique_tree_->FindBody(e2);
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

  // NOTE: the *effective* Young's modulus of the contact is half of the
  // material Young's modulus.
  const double effective_stiffness = kContactYoungsModulus * 0.5;
  // NOTE: Each sphere is moved toward the other offset distance which makes
  // the penetration depth 2 * offset.
  const double force = effective_stiffness * offset * 2;
  // NOTE: The spatial force contains the torque component followed by the
  // linear component.
  expected_spatial_force << 0, 0, 0, force_sign * force, 0, 0;
  // The force reported as the *resultant* force from ContactInfo.
  ASSERT_TRUE(
      CompareMatrices(resultant.get_spatial_force(), expected_spatial_force));

  // The force reported by the single ContactDetail. Should be the same.
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

// This class introduces heterogeneous compliant material parameters. It does
// so by wrapping the tree generation and directly setting element contact
// parameters.
class CompliantHeterogeneousModelTest : public CompliantContactModelTest {
 protected:
  void DoGenerateTestTree(RigidBodyTree<double>*) override {
    EXPECT_EQ(body1_->get_num_collision_elements(), 1);
    EXPECT_EQ(body2_->get_num_collision_elements(), 1);
    Element& element1 = **body1_->collision_elements_begin();
    Element& element2 = **body2_->collision_elements_begin();

    auto set_params = [this](Element* element, int index) {
      CompliantMaterial material;
      material.set_youngs_modulus(kMaterialYoungsModulus[index]);
      element->set_compliant_material(material);
    };
    set_params(&element1, 0);
    set_params(&element2, 1);
  }

  // Per-element contact parameters. Note; this is only testing simple contact;
  // there is just a normal force with zero relative velocity. Young's modulus
  // is the only parameter that matters.
  static const double kMaterialYoungsModulus[2];

  static double contact_youngs_modulus() {
    return kMaterialYoungsModulus[0] * kMaterialYoungsModulus[1] /
           (kMaterialYoungsModulus[0] + kMaterialYoungsModulus[1]);
  }
};

const double CompliantHeterogeneousModelTest::kMaterialYoungsModulus[2]{10000,
                                                                        20000};

// Confirms a contact result for two colliding spheres with heterogeneous
// contact material and zero relative velocity.
TEST_F(CompliantHeterogeneousModelTest, ModelSingleCollision) {
  const double offset = 0.1;
  auto& contact_results = RunTest(-offset);
  ASSERT_EQ(contact_results.get_num_contacts(), 1);
  const auto info = contact_results.get_contact_info(0);

  // Confirms that the proper bodies are in contact.
  drake::multibody::collision::ElementId e1 = info.get_element_id_1();
  drake::multibody::collision::ElementId e2 = info.get_element_id_2();
  const RigidBody<double>* b1 = unique_tree_->FindBody(e1);
  const RigidBody<double>* b2 = unique_tree_->FindBody(e2);
  ASSERT_NE(e1, e2);
  ASSERT_TRUE((b1 == body1_ && b2 == body2_) || (b1 == body2_ && b2 == body1_));

  // These values depend on the interpretation of which element belongs to body1
  // and body2, respectively. Assume b1 == body1_ and reverse if untrue.
  double force_sign = -1;
  double s1 = kMaterialYoungsModulus[1] /
              (kMaterialYoungsModulus[0] + kMaterialYoungsModulus[1]);
  if (b2 == body1_) {
    force_sign = 1;
    s1 = 1 - s1;
  }

  // Confirms the contact details are as expected.
  const auto& resultant = info.get_resultant_force();
  SpatialForce<double> expected_spatial_force;

  // NOTE: Each sphere is moved toward the other offset distance which makes
  // the penetration depth 2 * offset.
  const double force = contact_youngs_modulus() * (offset * 2);
  expected_spatial_force << 0, 0, 0, force_sign * force, 0, 0;
  ASSERT_TRUE(
      CompareMatrices(resultant.get_spatial_force(), expected_spatial_force));

  const auto& details = info.get_contact_details();
  ASSERT_EQ(details.size(), 1u);
  auto detail_force = details[0]->ComputeContactForce();
  ASSERT_TRUE(CompareMatrices(detail_force.get_spatial_force(),
                              expected_spatial_force));
  Vector3<double> expected_point;
  // Sphere 1 is placed on the left of the anchor point, and Sphere 2 on the
  // right.
  const double x_pos =
      (x_anchor_ - offset) * s1 + (x_anchor_ + offset) * (1 - s1);
  expected_point << x_pos, 0, 0;
  ASSERT_TRUE(
      CompareMatrices(detail_force.get_application_point(), expected_point));
}
}  // namespace
}  // namespace test
}  // namespace rigid_body_plant
}  // namespace plants
}  // namespace systems
}  // namespace drake
