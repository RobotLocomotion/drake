/* clang-format off to disable clang-format-includes */
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
/* clang-format on */

#include <memory>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/joints/quaternion_floating_joint.h"
#include "drake/multibody/parsers/sdf_parser.h"
#include "drake/multibody/rigid_body.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/test/contact_result_test_common.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"

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
class ContactResultTest : public ContactResultTestCommon<double>,
                          public ::testing::TestWithParam<bool> {
 protected:
  // Runs the test on the RigidBodyPlant.
  const ContactResults<double>& RunTest(double distance) {
    // Set the time step, based on whether the continuous or discrete model is
    // used. The contact forces from the discretized plant match better as the
    // step size gets smaller. 1e-18 is necessary to get the contact force
    // outputs to match to the requested accuracy.
    const double timestep = (GetParam()) ? 1e-18 : 0.0;

    // Populate the plant.
    plant_ = make_unique<RigidBodyPlant<double>>(GenerateTestTree(distance),
        timestep);

    plant_->set_default_compliant_material(MakeDefaultMaterial());

    systems::CompliantContactModelParameters model_parameters;
    model_parameters.characteristic_radius = kContactRadius;
    model_parameters.v_stiction_tolerance = kVStictionTolerance;
    plant_->set_contact_model_parameters(model_parameters);

    context_ = plant_->CreateDefaultContext();
    output_ = plant_->AllocateOutput(*context_);

    // TODO(edrumwri): Eliminate this call once the caching system is in place-
    // it will then no longer be necessary.
    plant_->CalcDiscreteVariableUpdates(*context_,
       &context_->get_mutable_discrete_state());

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
TEST_P(ContactResultTest, NoCollision) {
  auto& contact_results = RunTest(0.1);
  ASSERT_EQ(contact_results.get_num_contacts(), 0);
}

// Confirms a contact result for two touching spheres -- expects no reported
// collisions. For now, osculation is not considered a "contact" for reporting
// purposes. If the definition changes, this will likewise change.
TEST_P(ContactResultTest, Touching) {
  auto& contact_results = RunTest(0.0);
  ASSERT_EQ(contact_results.get_num_contacts(), 0);
}

// Confirms a contact result for two colliding spheres.
TEST_P(ContactResultTest, SingleCollision) {
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
  ASSERT_TRUE(CompareMatrices(resultant.get_spatial_force(),
              expected_spatial_force));

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

class ThreeLeggedStoolTest : public ::testing::TestWithParam<bool> {
 protected:
  void SetUp() {
  }

  // Gets the forces for the specified sphere from the stool in contact with the
  // ground.
  ContactInfo<double> GetContactInfo(
      const ContactResults<double>& contact_results,
      const RigidBodyTree<double>& tree,
      const std::string& sphere_name) {
    for (int i = 0; i < contact_results.get_num_contacts(); ++i) {
      const auto info = contact_results.get_contact_info(i);

      // Confirms that the proper bodies are in contact.
      drake::multibody::collision::ElementId e1 = info.get_element_id_1();
      drake::multibody::collision::ElementId e2 = info.get_element_id_2();
      const RigidBody<double>* b1 = tree.FindBody(e1);
      const RigidBody<double>* b2 = tree.FindBody(e2);
      if (b1->get_name() == sphere_name || b2->get_name() == sphere_name)
        return info;
    }

    DRAKE_ABORT();
    return contact_results.get_contact_info(0);  // To keep compiler happy.
  }

  const char* kThreeLeggedStoolSdf =
      "drake/multibody/rigid_body_plant/test/ThreeLeggedStool.sdf";

  // We loosen the default absolute comparison tolerance slightly to make the
  // normal force roughly "2".
  const double kMatchTol = 1e-10;

  // Coefficient of friction for the discretized system (it *always* uses the
  // static coefficient of friction material property).
  const double kMuStatic = 0.18;

  // Coefficient of friction for the continuous system (it uses the appropriate
  // coefficient of friction, i.e, sliding, in the tests below).
  const double kMuDynamic = 0.166666666666667;

  // Expected normal force at each contact.
  const double kNormalForce = 2;
};

// Checks the contact forces in a multi-contact scenario with dynamic friction.
TEST_P(ThreeLeggedStoolTest, ContactForces) {
  // Create RigidBodyTree.
  auto tree_ptr = make_unique<RigidBodyTree<double>>();
  parsers::sdf::AddModelInstancesFromSdfFile(
      FindResourceOrThrow(kThreeLeggedStoolSdf),
      multibody::joints::kQuaternion, nullptr /* weld to frame */,
      tree_ptr.get());
  multibody::AddFlatTerrainToWorld(tree_ptr.get(), 100., 10.);

  // Set the time step, based on whether the continuous or discrete model is
  // used. The contact forces from the discretized plant match better as the
  // step size gets smaller. 1e-18 is necessary to get the contact force
  // outputs to match to the requested accuracy.
  const double dt = (GetParam()) ? 1e-18 : 0.0;

  // Instantiate a RigidBodyPlant from the RigidBodyTree.
  DiagramBuilder<double> builder;
  auto& plant = *builder.AddSystem<RigidBodyPlant<double>>(move(tree_ptr), dt);
  plant.set_name("plant");

  // Build the diagram.
  auto diagram = builder.Build();

  // Create the context and allocate the output.
  std::unique_ptr<Context<double>> context = plant.CreateDefaultContext();
  std::unique_ptr<SystemOutput<double>> output = plant.AllocateOutput(*context);

  // Set the deformation between the bodies and the state to a sliding velocity.
  const double deformation = 1e-4;
  const double horizontal_velocity = 1.0;
  VectorX<double> old_x;
  if (dt > 0) {
    context->get_mutable_discrete_state(0).get_mutable_value()[2] =
        -deformation;
    context->get_mutable_discrete_state(0).get_mutable_value()[11] =
        horizontal_velocity;
    old_x = context->get_discrete_state(0).get_value();

    // Compute the contact results.
    plant.CalcDiscreteVariableUpdates(*context,
                                      &context->get_mutable_discrete_state());
  } else {
    context->get_mutable_continuous_state().get_mutable_vector()[2] =
        -deformation;
    context->get_mutable_continuous_state().get_mutable_vector()[11] =
        horizontal_velocity;
  }

  // Get the appropriate friction coefficient.
  const double mu = (dt > 0) ? kMuStatic : kMuDynamic;

  // Compute the contact results.
  plant.CalcOutput(*context, output.get());
  const int port_index = plant.contact_results_output_port().get_index();
  ContactResults<double> contact_results =
      output->get_data(port_index)->GetValue<ContactResults<double>>();

  // Verify the correct number of contacts.
  const auto& tree = plant.get_rigid_body_tree();
  ASSERT_EQ(contact_results.get_num_contacts(), 3);

  // Get the various contact infos.
  ContactInfo<double> info1 = GetContactInfo(contact_results, tree, "SphereA");
  ContactInfo<double> info2 = GetContactInfo(contact_results, tree, "SphereB");
  ContactInfo<double> info3 = GetContactInfo(contact_results, tree, "SphereC");

  // Determine force signs.
  double force_sign1 = 1, force_sign2 = 1, force_sign3 = 1;
  if (tree.FindBody(info1.get_element_id_1())->get_name() == "world")
    force_sign1 = -1;
  if (tree.FindBody(info2.get_element_id_1())->get_name() == "world")
    force_sign2 = -1;
  if (tree.FindBody(info3.get_element_id_1())->get_name() == "world")
    force_sign3 = -1;

  // Confirms the contact details are as expected.
  const auto& resultant1 = info1.get_resultant_force();
  const auto& resultant2 = info2.get_resultant_force();
  const auto& resultant3 = info3.get_resultant_force();
  SpatialForce<double> expected_spatial_force1, expected_spatial_force2,
      expected_spatial_force3;

  expected_spatial_force1 << 0, 0, 0, 0,
      force_sign1 * -mu * kNormalForce, force_sign1 * kNormalForce;
  expected_spatial_force2 << 0, 0, 0, 0, 0, force_sign2 *  kNormalForce;
  expected_spatial_force3 << 0, 0, 0, 0, 0, force_sign3 * kNormalForce;
  EXPECT_TRUE(CompareMatrices(resultant1.get_spatial_force(),
                              expected_spatial_force1, kMatchTol));
  EXPECT_TRUE(CompareMatrices(resultant2.get_spatial_force(),
                              expected_spatial_force2, kMatchTol));
  EXPECT_TRUE(CompareMatrices(resultant3.get_spatial_force(),
                              expected_spatial_force3, kMatchTol));

  const auto& details1 = info1.get_contact_details();
  ASSERT_EQ(details1.size(), 1u);
  auto detail_force1 = details1[0]->ComputeContactForce();
  EXPECT_TRUE(CompareMatrices(detail_force1.get_spatial_force(),
                              expected_spatial_force1, kMatchTol));

  const auto& details2 = info2.get_contact_details();
  ASSERT_EQ(details2.size(), 1u);
  auto detail_force2 = details2[0]->ComputeContactForce();
  EXPECT_TRUE(CompareMatrices(detail_force2.get_spatial_force(),
                              expected_spatial_force2, kMatchTol));

  const auto& details3 = info3.get_contact_details();
  ASSERT_EQ(details3.size(), 1u);
  auto detail_force3 = details3[0]->ComputeContactForce();
  EXPECT_TRUE(CompareMatrices(detail_force3.get_spatial_force(),
                              expected_spatial_force3, kMatchTol));

  // If this is the discretized system, test again using more edges in the
  // friction cone.
  if (dt > 0) {
    plant.set_default_half_num_friction_cone_edges(16);
    context->get_mutable_discrete_state(0).get_mutable_value() = old_x;
    plant.CalcDiscreteVariableUpdates(*context,
                                    &context->get_mutable_discrete_state());
    plant.CalcOutput(*context, output.get());
    contact_results =
        output->get_data(port_index)->GetValue<ContactResults<double>>();

    // Verify the correct number of contacts.
    ASSERT_EQ(contact_results.get_num_contacts(), 3);

    // Get the various contact infos.
    info1 = GetContactInfo(contact_results, tree, "SphereA");
    info2 = GetContactInfo(contact_results, tree, "SphereB");
    info3 = GetContactInfo(contact_results, tree, "SphereC");

    // Verify that the force signs have not changed.
    ASSERT_TRUE(tree.FindBody(info1.get_element_id_1())->get_name() != "world"
        || force_sign1 == -1);
    ASSERT_TRUE(tree.FindBody(info2.get_element_id_1())->get_name() != "world"
        || force_sign2 == -1);
    ASSERT_TRUE(tree.FindBody(info3.get_element_id_1())->get_name() != "world"
        || force_sign3 == -1);

    // Confirms the contact details are as expected.
    EXPECT_TRUE(CompareMatrices(info1.get_resultant_force().get_spatial_force(),
                                expected_spatial_force1, kMatchTol));
    EXPECT_TRUE(CompareMatrices(info2.get_resultant_force().get_spatial_force(),
                                expected_spatial_force2, kMatchTol));
    EXPECT_TRUE(CompareMatrices(info3.get_resultant_force().get_spatial_force(),
                                expected_spatial_force3, kMatchTol));
  }
}

/*
// Checks the contact forces in a multi-contact scenario with dynamic friction.
GTEST_TEST(AllReportingTest, ThreeLeggedStoolDiscretized) {
  // Create RigidBodyTree.
  auto tree_ptr = make_unique<RigidBodyTree<double>>();
  parsers::sdf::AddModelInstancesFromSdfFile(
      FindResourceOrThrow(kThreeLeggedStoolSdf),
      multibody::joints::kQuaternion, nullptr,
      tree_ptr.get());
  multibody::AddFlatTerrainToWorld(tree_ptr.get(), 100., 10.);

  // Instantiate a RigidBodyPlant from the RigidBodyTree.
  const double dt = 1e-12;
  DiagramBuilder<double> builder;
  auto& plant = *builder.AddSystem<RigidBodyPlant<double>>(move(tree_ptr), dt);
  plant.set_name("plant");

  // Build the diagram.
  auto diagram = builder.Build();

  // Create the context and allocate the output.
  std::unique_ptr<Context<double>> context = plant.CreateDefaultContext();
  std::unique_ptr<SystemOutput<double>> output = plant.AllocateOutput(*context);

  // Set the velocity for the body to cause sliding.
  context->get_mutable_discrete_state(0).get_mutable_value()[2] = -1e-4;
  context->get_mutable_discrete_state(0).get_mutable_value()[11] = 1.0;
  VectorX<double> old_x = context->get_discrete_state(0).get_value();

  // Compute the contact results.
  plant.CalcDiscreteVariableUpdates(*context,
                                    &context->get_mutable_discrete_state());
  plant.CalcOutput(*context, output.get());
  const int port_index = plant.contact_results_output_port().get_index();
  ContactResults<double> contact_results =
      output->get_data(port_index)->GetValue<ContactResults<double>>();

  // Verify the correct number of contacts.
  const auto& tree = plant.get_rigid_body_tree();
  ASSERT_EQ(contact_results.get_num_contacts(), 3);

  // Get the various contact infos.
  ContactInfo<double> info1 = GetContactInfo(contact_results, tree, "SphereA");
  ContactInfo<double> info2 = GetContactInfo(contact_results, tree, "SphereB");
  ContactInfo<double> info3 = GetContactInfo(contact_results, tree, "SphereC");

  // Determine force signs.
  double force_sign1 = 1, force_sign2 = 1, force_sign3 = 1;
  if (tree.FindBody(info1.get_element_id_1())->get_name() == "world")
    force_sign1 = -1;
  if (tree.FindBody(info2.get_element_id_1())->get_name() == "world")
    force_sign2 = -1;
  if (tree.FindBody(info3.get_element_id_1())->get_name() == "world")
    force_sign3 = -1;

  // Confirms the contact details are as expected.
  const auto& resultant1 = info1.get_resultant_force();
  const auto& resultant2 = info2.get_resultant_force();
  const auto& resultant3 = info3.get_resultant_force();
  SpatialForce<double> expected_spatial_force1, expected_spatial_force2,
      expected_spatial_force3;

  expected_spatial_force1 << 0, 0, 0, 0,
      force_sign1 * -kMuStatic * kNormalForce, force_sign1 * kNormalForce;
  expected_spatial_force2 << 0, 0, 0, 0, 0, force_sign2 * kNormalForce;
  expected_spatial_force3 << 0, 0, 0, 0, 0, force_sign3 * kNormalForce;
  EXPECT_TRUE(CompareMatrices(resultant1.get_spatial_force(),
                              expected_spatial_force1, kMatchTol));
  EXPECT_TRUE(CompareMatrices(resultant2.get_spatial_force(),
                              expected_spatial_force2, kMatchTol));
  EXPECT_TRUE(CompareMatrices(resultant3.get_spatial_force(),
                              expected_spatial_force3, kMatchTol));

  const auto& details1 = info1.get_contact_details();
  ASSERT_EQ(details1.size(), 1u);
  auto detail_force1 = details1[0]->ComputeContactForce();
  EXPECT_TRUE(CompareMatrices(detail_force1.get_spatial_force(),
                              expected_spatial_force1, kMatchTol));

  const auto& details2 = info2.get_contact_details();
  ASSERT_EQ(details2.size(), 1u);
  auto detail_force2 = details2[0]->ComputeContactForce();
  EXPECT_TRUE(CompareMatrices(detail_force2.get_spatial_force(),
                              expected_spatial_force2, kMatchTol));

  const auto& details3 = info3.get_contact_details();
  ASSERT_EQ(details3.size(), 1u);
  auto detail_force3 = details3[0]->ComputeContactForce();
  EXPECT_TRUE(CompareMatrices(detail_force3.get_spatial_force(),
                              expected_spatial_force3, kMatchTol));

  // Change the plant to use many more edges in the friction cone
  // approximation. The answer should still be the same.
  plant.set_default_half_num_friction_cone_edges(16);
  context->get_mutable_discrete_state(0).get_mutable_value() = old_x;
  plant.CalcDiscreteVariableUpdates(*context,
                                    &context->get_mutable_discrete_state());
  plant.CalcOutput(*context, output.get());
  contact_results =
      output->get_data(port_index)->GetValue<ContactResults<double>>();

  // Verify the correct number of contacts.
  ASSERT_EQ(contact_results.get_num_contacts(), 3);

  // Get the various contact infos.
  info1 = GetContactInfo(contact_results, tree, "SphereA");
  info2 = GetContactInfo(contact_results, tree, "SphereB");
  info3 = GetContactInfo(contact_results, tree, "SphereC");

  // Verify that the force signs have not changed.
  ASSERT_TRUE(tree.FindBody(info1.get_element_id_1())->get_name() != "world" ||
    force_sign1 == -1);
  ASSERT_TRUE(tree.FindBody(info2.get_element_id_1())->get_name() != "world" ||
    force_sign2 == -1);
  ASSERT_TRUE(tree.FindBody(info3.get_element_id_1())->get_name() != "world" ||
    force_sign3 == -1);

  // Confirms the contact details are as expected.
  EXPECT_TRUE(CompareMatrices(info1.get_resultant_force().get_spatial_force(),
                              expected_spatial_force1, kMatchTol));
  EXPECT_TRUE(CompareMatrices(info2.get_resultant_force().get_spatial_force(),
                              expected_spatial_force2, kMatchTol));
  EXPECT_TRUE(CompareMatrices(info3.get_resultant_force().get_spatial_force(),
                              expected_spatial_force3, kMatchTol));
}
*/
GTEST_TEST(AdditionalContactResultsTest, AutoDiffTest) {
  // Simply test that I can instantiate the AutoDiffXd type.
  ContactResults<AutoDiffXd> result;
  EXPECT_EQ(result.get_num_contacts(), 0);
}

// Instantiate the tests.
INSTANTIATE_TEST_CASE_P(ContinuousAndDiscretizedTest, ContactResultTest,
                        ::testing::Bool());
INSTANTIATE_TEST_CASE_P(ContinuousAndDiscretizedTest, ThreeLeggedStoolTest,
                        ::testing::Bool());

}  // namespace
}  // namespace test
}  // namespace rigid_body_plant
}  // namespace plants
}  // namespace systems
}  // namespace drake

