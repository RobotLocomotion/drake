#include "drake/multibody/plant/externally_applied_spatial_force.h"

#include <functional>
#include <limits>
#include <utility>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"

namespace drake {

using Eigen::VectorXd;
using math::RigidTransform;
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
        link1.default_mass() * g / 2 * up_W);
    const SpatialForce<double> F_L1r_W(
        Vector3<double>::Zero() /* no torque */,
        link1.default_mass() * g / 2 * up_W);
    const SpatialForce<double> F_L2q_W(
        Vector3<double>::Zero() /* no torque */,
        link2.default_mass() * g / 2 * up_W);
    const SpatialForce<double> F_L2r_W(
        Vector3<double>::Zero() /* no torque */,
        link2.default_mass() * g / 2 * up_W);

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

// Fixture for tests that should be performed on multibody plants either in
// continuous or discrete mode.
class ExternallyAppliedForcesTest : public ::testing::Test {
 protected:
  void MakePlantWithGravityCompensator(double time_step) {
    // Load the acrobot model.
    const std::string full_name =
        FindResourceOrThrow("drake/multibody/benchmarks/acrobot/acrobot.sdf");
    systems::DiagramBuilder<double> builder;
    plant_ = builder.AddSystem<MultibodyPlant<double>>(time_step);
    Parser(plant_).AddModels(full_name);
    plant_->Finalize();

    // Add the system that applies inverse gravitational forces to the link
    // endpoints.
    acrobot_gravity_compensator_ =
        builder.AddSystem<AcrobotGravityCompensator>(plant_);

    // Connect the system to the MBP.
    builder.Connect(acrobot_gravity_compensator_->get_output_port(0),
                    plant_->get_applied_spatial_force_input_port());
    auto zero_source =
        builder.AddSystem<ConstantVectorSource<double>>(Vector1<double>(0));
    builder.Connect(zero_source->get_output_port(),
                    plant_->get_actuation_input_port());
    diagram_ = builder.Build();

    // Create a context.
    context_ = diagram_->CreateDefaultContext();
    auto& acrobot_context =
        diagram_->GetMutableSubsystemContext(*plant_, context_.get());

    // Put the acrobot into a configuration where it has nonzero potential
    // energy.
    const VectorXd q = M_PI_4 * VectorXd::Ones(2);
    plant_->SetPositions(&acrobot_context, q);
  }
  MultibodyPlant<double>* plant_{nullptr};
  AcrobotGravityCompensator* acrobot_gravity_compensator_{nullptr};
  std::unique_ptr<systems::Diagram<double>> diagram_;
  std::unique_ptr<systems::Context<double>> context_;
};

TEST_F(ExternallyAppliedForcesTest, ContinuousPlant) {
  MakePlantWithGravityCompensator(0.0);

  // Compute time derivatives and ensure that they're sufficiently near zero.
  auto derivatives = context_->get_continuous_state().Clone();
  ASSERT_EQ(derivatives->size(), 4);
  diagram_->CalcTimeDerivatives(*context_, derivatives.get());

  // Ensure that the acceleration is zero.
  const VectorXd derivatives_vector = derivatives->get_vector().CopyToVector();
  const double eps = 100 * std::numeric_limits<double>::epsilon();
  EXPECT_TRUE(CompareMatrices(derivatives_vector, VectorXd::Zero(4), eps,
                              MatrixCompareType::absolute));
}

TEST_F(ExternallyAppliedForcesTest, DiscretePlant) {
  MakePlantWithGravityCompensator(1.0e-3);

  auto updates = diagram_->AllocateDiscreteVariables();
  diagram_->CalcForcedDiscreteVariableUpdate(*context_, updates.get());

  // Copies to plain Eigen vectors to verify the math.
  auto& acrobot_context =
      diagram_->GetMutableSubsystemContext(*plant_, context_.get());
  const VectorXd x0 = plant_->GetPositionsAndVelocities(acrobot_context);
  ASSERT_EQ(updates->get_vector().size(), 4);
  const VectorXd xnext = updates->get_vector().CopyToVector();
  const VectorXd delta_x = xnext - x0;

  // Ensure the discrete change in the state is zero.
  const double eps = 100 * std::numeric_limits<double>::epsilon();
  EXPECT_TRUE(CompareMatrices(delta_x, VectorXd::Zero(4), eps,
                              MatrixCompareType::absolute));
}

TEST_F(ExternallyAppliedForcesTest, CalcGeneralizedForcesDueToMultibodyForces) {
  // N.B. Whether the model is discrete or continuous is irrelevant. In this
  // case we arbitrarily chose to use a continuous model.
  MakePlantWithGravityCompensator(0.0);

  const Context<double>& plant_context =
      diagram_->GetSubsystemContext(*plant_, *context_);
  const Context<double>& compensator_context =
      diagram_->GetSubsystemContext(*acrobot_gravity_compensator_, *context_);
  const auto& externally_applied_spatial_forces =
      acrobot_gravity_compensator_->get_output_port(0)
          .Eval<std::vector<ExternallyAppliedSpatialForce<double>>>(
              compensator_context);

  MultibodyForces<double> forces(*plant_);
  for (const ExternallyAppliedSpatialForce<double>& a_force :
       externally_applied_spatial_forces) {
    const Body<double>& body = plant_->get_body(a_force.body_index);

    // Get the pose for this body in the world frame.
    const RigidTransform<double>& X_WB = body.EvalPoseInWorld(plant_context);

    // Get the position vector from the body origin (Bo) to the point of
    // force application (Bq), expressed in the world frame (W).
    const Vector3<double> p_BoBq_W = X_WB.rotation() * a_force.p_BoBq_B;

    // Shift the spatial force from Bq to Bo.
    const SpatialForce<double> F_Bo_W = a_force.F_Bq_W.Shift(-p_BoBq_W);

    // Add contribution.
    body.AddInForceInWorld(plant_context, F_Bo_W, &forces);
  }

  // For completeness, we will add an arbitrary set of generalized forces.
  const VectorX<double> tau_external =
      (VectorX<double>(2) << 1.4, -3.14).finished();
  forces.mutable_generalized_forces() = tau_external;

  // Generalized forces result of applying "forces".
  VectorX<double> generalized_forces(plant_->num_velocities());
  plant_->CalcGeneralizedForces(plant_context, forces, &generalized_forces);

  // We expect the total generalized forces to be the result of the generalized
  // forces from the gravity compensator, which balance gravity (minus sign),
  // and the externally applied generalized forces.
  const VectorX<double> expected_generalized_forces =
      -plant_->CalcGravityGeneralizedForces(plant_context) + tau_external;

  const double tolerance = 5.0 * std::numeric_limits<double>::epsilon();
  EXPECT_TRUE(CompareMatrices(generalized_forces, expected_generalized_forces,
                              tolerance, MatrixCompareType::relative));
}

}  // namespace
}  // namespace multibody
}  // namespace drake
