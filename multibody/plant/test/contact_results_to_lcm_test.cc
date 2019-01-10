#include "drake/multibody/plant/contact_results_to_lcm.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/lcmt_contact_results_for_viz.hpp"
#include "drake/multibody/benchmarks/acrobot/make_acrobot_plant.h"
#include "drake/multibody/plant/contact_results.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/context.h"

namespace drake {

using geometry::GeometryId;
using geometry::PenetrationAsPointPair;
using multibody::benchmarks::acrobot::MakeAcrobotPlant;
using multibody::benchmarks::acrobot::AcrobotParameters;

namespace multibody {
namespace {

// Confirm that an empty multibody plant produces an empty lcm message.
GTEST_TEST(ContactResultToLcmSystem, EmptyMultibodyPlant) {
  MultibodyPlant<double> plant;
  plant.Finalize();
  ContactResultsToLcmSystem<double> lcm_system(plant);
  auto lcm_context = lcm_system.AllocateContext();
  lcm_context->FixInputPort(
      lcm_system.get_contact_result_input_port().get_index(),
      systems::Value<ContactResults<double>>());

  Value<lcmt_contact_results_for_viz> lcm_message_value;
  lcm_system.get_lcm_message_output_port().Calc(*lcm_context,
                                                &lcm_message_value);

  const lcmt_contact_results_for_viz& lcm_message =
      lcm_message_value.GetValue<lcmt_contact_results_for_viz>();

  // We haven't stepped, so we should assume the time is the context's default
  // value.
  EXPECT_EQ(lcm_message.timestamp, 0);
  EXPECT_EQ(lcm_message.num_contacts, 0);
}

// Common case: confirm that the reported contacts map to the right lcm message.
// In this test, we're using a MBP that doesn't actually have collision
// geometry, but simulate collision results by reporting that two bodies are
// colliding. That is enough to test the ContactResultsToLcmSystem.
GTEST_TEST(ContactResultToLcmSystem, NonEmptyMultibodyPlantEmptyContact) {
  using std::to_string;
  const AcrobotParameters parameters;
  std::unique_ptr<MultibodyPlant<double>> plant =
      MakeAcrobotPlant(parameters, true /* finalize */);

  ContactResultsToLcmSystem<double> lcm_system(*plant);

  // Create ContactResults with single reported contact.
  ContactResults<double> contacts;
  // NOTE: The values in penetration_data are irrelevant except for nhat_BA_W.
  // So, only that value is set; all other values are left as default.
  PenetrationAsPointPair<double> penetration_data;
  penetration_data.nhat_BA_W << 1, 2, 3;
  const auto& link1 = plant->GetBodyByName(parameters.link1_name());
  const BodyIndex index1 = link1.index();
  const ModelInstanceIndex model_instance = link1.model_instance();
  const std::string name1 =
      parameters.link1_name() + "(" + to_string(model_instance) + ")";
  const BodyIndex index2 =
      plant->GetBodyByName(parameters.link2_name()).index();
  // Assume that link1 and link2 belong to the same model instance.
  const std::string name2 =
      parameters.link2_name() + "(" + to_string(model_instance) + ")";
  const Vector3<double> f_BC_W = Vector3<double>{1, 2, 3};
  const Vector3<double> p_WC = Vector3<double>{-1, -2, -2};
  const double separation_speed = 0.25;
  const double slip_speed = 0.5;
  PointPairContactInfo<double> pair_info{
      index1,           index2,     f_BC_W,          p_WC,
      separation_speed, slip_speed, penetration_data};
  contacts.AddContactInfo(pair_info);
  Value<ContactResults<double>> contacts_value(contacts);
  auto lcm_context = lcm_system.AllocateContext();
  lcm_context->FixInputPort(
      lcm_system.get_contact_result_input_port().get_index(),
      systems::Value<ContactResults<double>>(contacts));

  Value<lcmt_contact_results_for_viz> lcm_message_value;
  lcm_system.get_lcm_message_output_port().Calc(*lcm_context,
                                                &lcm_message_value);
  const lcmt_contact_results_for_viz& lcm_message =
      lcm_message_value.GetValue<lcmt_contact_results_for_viz>();

  // We haven't stepped, so we should assume the time is the context's default
  // value.
  EXPECT_EQ(lcm_message.timestamp, 0);
  ASSERT_EQ(lcm_message.num_contacts, 1);
  const lcmt_contact_info_for_viz& info_msg = lcm_message.contact_info[0];
  EXPECT_EQ(info_msg.timestamp, 0);
  EXPECT_EQ(info_msg.body1_name, name1);
  EXPECT_EQ(info_msg.body2_name, name2);
  CompareMatrices(Vector3<double>(info_msg.contact_point), p_WC, 0,
                  MatrixCompareType::absolute);
  CompareMatrices(Vector3<double>(info_msg.contact_force), f_BC_W, 0,
                  MatrixCompareType::absolute);
  CompareMatrices(Vector3<double>(info_msg.normal), penetration_data.nhat_BA_W,
                  0, MatrixCompareType::absolute);
}

// Confirm that the system can be transmogrified to other supported scalars.
GTEST_TEST(ContactResultToLcmSystem, Transmogrify) {
  MultibodyPlant<double> plant;
  plant.Finalize();
  ContactResultsToLcmSystem<double> lcm_system(plant);

  ContactResultsToLcmSystem<AutoDiffXd> lcm_system_ad(lcm_system);
}

GTEST_TEST(ConnectContactResultsToDrakeVisualizer, BasicTest) {
  systems::DiagramBuilder<double> builder;

  // Make a trivial plant with at least one body and a discrete time step.
  auto plant = builder.AddSystem<MultibodyPlant>(0.001);
  plant->AddRigidBody("link", SpatialInertia<double>());
  plant->Finalize();

  auto publisher = ConnectContactResultsToDrakeVisualizer(&builder, *plant);

  // Confirm that we get a non-null result.
  EXPECT_NE(publisher, nullptr);

  // Check that the publishing event was set as documented.
  auto periodic_events = publisher->GetPeriodicEvents();
  EXPECT_EQ(periodic_events.size(), 1);
  EXPECT_EQ(periodic_events.begin()->first.period_sec(), 1/60.0);
}

GTEST_TEST(ConnectContactResultsToDrakeVisualizer, NestedDiagramTest) {
  systems::DiagramBuilder<double> interior_builder;

  // Make a trivial plant with at least one body and a discrete time step.
  auto plant = interior_builder.AddSystem<MultibodyPlant>(0.001);
  plant->AddRigidBody("link", SpatialInertia<double>());
  plant->Finalize();

  interior_builder.ExportOutput(plant->get_contact_results_output_port(),
      "contact_results");

  systems::DiagramBuilder<double> builder;
  auto interior_diagram = builder.AddSystem(interior_builder.Build());

  auto publisher = ConnectContactResultsToDrakeVisualizer(
      &builder, *plant, interior_diagram->GetOutputPort("contact_results"));

  // Confirm that we get a non-null result.
  EXPECT_NE(publisher, nullptr);

  // Check that the publishing event was set as documented.
  auto periodic_events = publisher->GetPeriodicEvents();
  EXPECT_EQ(periodic_events.size(), 1);
  EXPECT_EQ(periodic_events.begin()->first.period_sec(), 1/60.0);
}

}  // namespace
}  // namespace multibody
}  // namespace drake
