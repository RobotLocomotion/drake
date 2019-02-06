#include "drake/multibody/plant/externally_applied_spatial_force.h"

#include <functional>
#include <limits>
#include <utility>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"

namespace drake {

using multibody::Parser;
using systems::ConstantVectorSource;
using systems::Context;
using systems::VectorBase;

namespace multibody {
namespace {

class AcrobotGravityCompensator : public systems::LeafSystem<double> {
 public:
  explicit AcrobotGravityCompensator(const MultibodyPlant<double>* plant)
      : plant_(plant) {
    this->DeclareAbstractOutputPort(
        "spatial_forces",
        &AcrobotGravityCompensator::CalcSpatialForceGravityCompensation);
  }

 private:
  void CalcSpatialForceGravityCompensation(
      const Context<double>& context,
      std::vector<ExternallyAppliedSpatialForce<double>>* output) const {
    const double g = UniformGravityFieldElement<double>::kDefaultStrength;

    // Verify that the links are in the expected order.
    const RigidBody<double>& link1 =
        dynamic_cast<const RigidBody<double>&>(plant_->get_body(BodyIndex(1)));
    const RigidBody<double>& link2 =
        dynamic_cast<const RigidBody<double>&>(plant_->get_body(BodyIndex(2)));
    ASSERT_EQ(&link1, &plant_->GetBodyByName("Link1"));
    ASSERT_EQ(&link2, &plant_->GetBodyByName("Link2"));

    // Get position vector from L1o (link1 origin) to L1cm (link1
    // center-of-mass), expressed in link1 frame, and do the same for the
    // position vector from L2o to L2cm.
    const Vector3<double> p_L1oL1cm_L1 = link1.default_com();
    const Vector3<double> p_L2oL2cm_L2 = link2.default_com();

    // One way to do gravity compensation for link1 is to apply forces of
    // magnitude 1/2 * mass * gravity to two arbitrary points L1q and L1r on
    // link1, where p_Lcm_L1r = -p_Lcm_L1q, and similarly for gravity
    // compensation of link2.
    const Vector3<double> p_L1cmL1q_L1(1, 2, 3);
    const Vector3<double> p_L1cmL1r_L1 = -p_L1cmL1q_L1;
    const Vector3<double> p_L2cmL2q_L2(-1, 3, -5);
    const Vector3<double> p_L2cmL2r_L2 = -p_L2cmL2q_L2;

    // Construct position vectors from L1o to L1q and from L1o to L1r,
    // expressed in link1 frame and do the same for position vectors on link2.
    const Vector3<double> p_L1oL1q_L1 = p_L1oL1cm_L1 + p_L1cmL1q_L1;
    const Vector3<double> p_L1oL1r_L1 = p_L1oL1cm_L1 + p_L1cmL1r_L1;
    const Vector3<double> p_L2oL2q_L2 = p_L2oL2cm_L2 + p_L2cmL2q_L2;
    const Vector3<double> p_L2oL2r_L2 = p_L2oL2cm_L2 + p_L2cmL2r_L2;

    // Construct position vectors from L1o to L1q and from L1o to L1r,
    // all expressed in link1 frame. Do the same for position vectors on link2.
    const Vector3<double> up_W(0, 0, 1);
    const SpatialForce<double> F_L1q_W(
        Vector3<double>::Zero() /* no torque */,
        link1.get_default_mass() * g / 2 * up_W);
    const SpatialForce<double> F_L1r_W(
        Vector3<double>::Zero() /* no torque */,
        link1.get_default_mass() * g / 2 * up_W);
    const SpatialForce<double> F_L2q_W(
        Vector3<double>::Zero() /* no torque */,
        link2.get_default_mass() * g / 2 * up_W);
    const SpatialForce<double> F_L2r_W(
        Vector3<double>::Zero() /* no torque */,
        link2.get_default_mass() * g / 2 * up_W);

    output->resize(4 /* number of forces */);
    (*output)[0].body_index = BodyIndex(1);
    (*output)[0].p_BoBq_B = p_L1oL1q_L1;
    (*output)[0].F_Bq_W = F_L1q_W;

    (*output)[1].body_index = BodyIndex(1);
    (*output)[1].p_BoBq_B = p_L1oL1r_L1;
    (*output)[1].F_Bq_W = F_L1r_W;

    (*output)[2].body_index = BodyIndex(2);
    (*output)[2].p_BoBq_B = p_L2oL2q_L2;
    (*output)[2].F_Bq_W = F_L2q_W;

    (*output)[3].body_index = BodyIndex(2);
    (*output)[3].p_BoBq_B = p_L2oL2r_L2;
    (*output)[3].F_Bq_W = F_L2r_W;
  }

  const MultibodyPlant<double>* plant_{nullptr};
};

GTEST_TEST(MultibodyPlantTest, CheckExternalAppliedInput) {
  // Load the acrobot model.
  const std::string full_name = FindResourceOrThrow(
        "drake/multibody/benchmarks/acrobot/acrobot.sdf");
  systems::DiagramBuilder<double> builder;
  auto plant = builder.AddSystem<MultibodyPlant<double>>();
  Parser(plant).AddModelFromFile(full_name);
  plant->AddForceElement<UniformGravityFieldElement>();
  plant->Finalize();

  // Add the system that applies inverse gravitational forces to the link
  // endpoints.
  auto acrobot_gravity_compensator =
      builder.AddSystem<AcrobotGravityCompensator>(plant);

  // Connect the system to the MBP.
  builder.Connect(
      acrobot_gravity_compensator->get_output_port(0),
      plant->get_applied_spatial_force_input_port());
  auto zero_source =
      builder.AddSystem<ConstantVectorSource<double>>(Vector1<double>(0));
  builder.Connect(zero_source->get_output_port(),
                  plant->get_actuation_input_port());
  auto diagram = builder.Build();

  // Create a context.
  auto context = diagram->CreateDefaultContext();
  auto& acrobot_context = diagram->GetMutableSubsystemContext(
      *plant, context.get());

  // Put the acrobot into a configuration where it has nonzero potential
  // energy.
  VectorBase<double>& acrobot_state =
    acrobot_context.get_mutable_continuous_state_vector();
  acrobot_state[0] = M_PI_4;
  acrobot_state[1] = M_PI_4;

  // Compute time derivatives and ensure that they're sufficiently near zero.
  auto derivatives = context->get_continuous_state().Clone();
  ASSERT_EQ(derivatives->size(), 4);
  diagram->CalcTimeDerivatives(*context, derivatives.get());

  // Ensure that the acceleration is zero.
  const VectorBase<double>& derivatives_vector = derivatives->get_vector();
  const double eps = 100 * std::numeric_limits<double>::epsilon();
  EXPECT_NEAR(derivatives_vector[2], 0.0, eps);
  EXPECT_NEAR(derivatives_vector[3], 0.0, eps);
}

}  // namespace
}  // namespace multibody
}  // namespace drake
