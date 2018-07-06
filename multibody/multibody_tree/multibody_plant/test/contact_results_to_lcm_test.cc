#include "drake/multibody/multibody_tree/multibody_plant/contact_results_to_lcm.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/lcmt_contact_results_for_viz.hpp"
#include "drake/multibody/benchmarks/acrobot/make_acrobot_plant.h"
#include "drake/multibody/multibody_tree/multibody_plant/contact_results.h"
#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"
#include "drake/systems/framework/context.h"

namespace drake {

using geometry::GeometryId;
using geometry::PenetrationAsPointPair;
using multibody::benchmarks::acrobot::MakeAcrobotPlant;
using multibody::benchmarks::acrobot::AcrobotParameters;
using systems::Value;

namespace multibody {
namespace multibody_plant {
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

// Case where the the MBP has bodies, but no collisions exist.
GTEST_TEST(ContactResultToLcmSystem, NonEmptyMultibodyPlantEmptyContact) {
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
  BodyIndex index1 = plant->GetBodyByName(parameters.link1_name()).index();
  // Assumes the acrobot belongs to model instance 1.
  const std::string name1 = parameters.link1_name() + "(1)";
  BodyIndex index2 = plant->GetBodyByName(parameters.link2_name()).index();
  const std::string name2 = parameters.link2_name() + "(1)";
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
GTEST_TEST(ContractResultToLcmSystem, Transmogrify) {
  MultibodyPlant<double> plant;
  plant.Finalize();
  ContactResultsToLcmSystem<double> lcm_system(plant);

  ContactResultsToLcmSystem<AutoDiffXd> lcm_system_ad(lcm_system);
}

}  // namespace
}  // namespace multibody_plant
}  // namespace multibody
}  // namespace drake
