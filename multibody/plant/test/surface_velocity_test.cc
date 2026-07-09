#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"
#include "drake/common/random.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/value.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/bus_value.h"

namespace drake {
namespace multibody {

// Exposes ComputeSurfaceVelocity for testing.
class MultibodyPlantTester {
 public:
  MultibodyPlantTester() = delete;
  static Vector3<double> ComputeSurfaceVelocity(
      const MultibodyPlant<double>& plant, BodyIndex body_index,
      const systems::Context<double>& context, const Eigen::Vector3d& n_W) {
    return plant.ComputeSurfaceVelocity(body_index, context, n_W);
  }

  // Invokes the (private) periodic displacement-update handler directly. This
  // lets tests exercise the accumulation math independent of how (or whether)
  // the event that would normally trigger it is scheduled.
  static systems::EventStatus CallSurfaceDisplacementUpdate(
      const MultibodyPlant<double>& plant,
      const systems::Context<double>& context, systems::State<double>* state) {
    return plant.CalcSurfaceDisplacementUpdate(context, state);
  }

  // Sets every surface-displacement value to `value`, dispatching on the
  // plant's continuous/discrete mode (continuous: misc continuous state z;
  // discrete: the surface-displacement abstract state).
  static void SetAllSurfaceDisplacements(const MultibodyPlant<double>& plant,
                                         systems::Context<double>* context,
                                         double value) {
    if (plant.is_discrete()) {
      auto& displacements =
          context->get_mutable_abstract_state<std::vector<double>>(
              plant.surface_displacement_abstract_state_index_);
      displacements.assign(displacements.size(), value);
    } else {
      auto& z = context->get_mutable_continuous_state()
                    .get_mutable_misc_continuous_state();
      z.SetFromVector(Eigen::VectorXd::Constant(z.size(), value));
    }
  }
};

namespace {

using Eigen::Vector3d;
using math::RigidTransformd;
using math::RollPitchYawd;

// Reads a body's cumulative surface displacement via the (mode-agnostic)
// "surface_displacements" output port.
double ReadSurfaceDisplacement(const MultibodyPlant<double>& plant,
                               const systems::Context<double>& context,
                               const RigidBody<double>& body) {
  const auto& output =
      plant.get_surface_displacements_output_port().Eval<systems::BusValue>(
          context);
  const AbstractValue* value = output.Find(body.scoped_name().to_string());
  DRAKE_DEMAND(value != nullptr);
  return value->get_value<double>();
}

// Fixes a constant `speed` signal for `body` on the "surface_speeds" input
// port (leaving any unlisted body's signal absent from the bus).
void FixSurfaceSpeed(const MultibodyPlant<double>& plant,
                     systems::Context<double>* context,
                     const RigidBody<double>& body, double speed) {
  systems::BusValue bus;
  bus.Set(body.scoped_name().to_string(), Value<double>(speed));
  plant.get_surface_speeds_input_port().FixValue(context, bus);
}

GTEST_TEST(MultibodyPlantTest, SetSurfaceVelocityAxisErrors) {
  MultibodyPlant<double> plant(0.0);
  const RigidBody<double>& belt =
      plant.AddRigidBody("belt", SpatialInertia<double>::MakeUnitary());

  // World body is not allowed.
  EXPECT_THROW(
      plant.SetSurfaceVelocityAxis(plant.world_body(), Vector3d{1, 0, 0}),
      std::exception);

  // Zero vector is not allowed.
  EXPECT_THROW(plant.SetSurfaceVelocityAxis(belt, Vector3d::Zero()),
               std::exception);

  // Registration after Finalize() is not allowed.
  plant.Finalize();
  EXPECT_THROW(plant.SetSurfaceVelocityAxis(belt, Vector3d{1, 0, 0}),
               std::exception);
}

// Initializes a plant with a couple of bodies to test the surface velocity API.
// The choice of a *continuous* plant is arbitrary and has no bearing on the
// test.
class SurfaceVelocityTest : public ::testing::Test {
 protected:
  void SetUp() override {
    belt_ = &plant_.AddRigidBody("belt", SpatialInertia<double>::MakeUnitary());
    other_ =
        &plant_.AddRigidBody("other", SpatialInertia<double>::MakeUnitary());
  }

  MultibodyPlant<double> plant_{0.0};
  const RigidBody<double>* belt_{nullptr};
  const RigidBody<double>* other_{nullptr};
};

// Tests on the Get/SetSurfaceVelocityAxis() API. We're testing the following:
//
// SetSurfaceVelocityAxis()
//  S1. Can be called pre-Finalize().
//  S2. Unnormalized vectors get normalized.
//  S3. New axis overwrites old axis.
//  S4. Setting nullopt clears axis.
//  S5. Can't be called post-finalize().
//  S6. Zero vectors can't be normalized (throws).
//
// GetSurfaceVelocityAxis()
//  G1. Can be called pre-Finalize().
//  G2. Return nullopt for unregistered body.
//  G3. Value still present post finalize.
TEST_F(SurfaceVelocityTest, GetSetSurfaceVelocityAxis) {
  // (S1) Can set pre-finalize().
  EXPECT_NO_THROW(plant_.SetSurfaceVelocityAxis(*belt_, Vector3d(0, 2, 0)));
  // (G1) Can get pre-finalize(), (S2) vector gets normalized.
  EXPECT_TRUE(CompareMatrices(*plant_.GetSurfaceVelocityAxis(*belt_),
                              Vector3d(0, 1, 0)));
  // (G2) Unregistered body returns nullopt.
  EXPECT_EQ(plant_.GetSurfaceVelocityAxis(*other_), std::nullopt);
  // (S3) Overwrite with new axis; (S2) tiny vector also normalized.
  EXPECT_NO_THROW(plant_.SetSurfaceVelocityAxis(*belt_, Vector3d(1e-12, 0, 0)));
  EXPECT_TRUE(CompareMatrices(*plant_.GetSurfaceVelocityAxis(*belt_),
                              Vector3d(1, 0, 0)));
  // (S4) Setting with null clears the registration.
  EXPECT_NO_THROW(plant_.SetSurfaceVelocityAxis(*belt_, {}));
  EXPECT_EQ(plant_.GetSurfaceVelocityAxis(*belt_), std::nullopt);

  // (S6) Can't set a zero vector.
  EXPECT_THROW(plant_.SetSurfaceVelocityAxis(*belt_, Vector3d::Zero()),
               std::exception);

  // Put an axis back onto belt.
  plant_.SetSurfaceVelocityAxis(*belt_, Vector3d(0, 0, 1));
  plant_.Finalize();

  // (G3) Value still present after Finalize().
  EXPECT_TRUE(CompareMatrices(*plant_.GetSurfaceVelocityAxis(*belt_),
                              Vector3d(0, 0, 1)));
  // (S5) - Can't set post-finalize.
  EXPECT_THROW(plant_.SetSurfaceVelocityAxis(*belt_, Vector3d(0, 1, 0)),
               std::exception);
}

// Confirm port names.
TEST_F(SurfaceVelocityTest, SurfaceSpeedsPortName) {
  // Ports don't exist until we finalize.
  plant_.Finalize();
  EXPECT_EQ(plant_.get_surface_speeds_input_port().get_name(),
            "surface_speeds");
  EXPECT_EQ(plant_.get_surface_displacements_output_port().get_name(),
            "surface_displacements");
}

// Tests the ComputeSurfaceVelocity() function.
//
// 1. If the port is unconnected, the velocity is zero.
// 2. A connected bus missing the signal reports zero velocity.
// 3. Velocity is zero for a body with no axis, even if the bus mistakenly
//    provides a signal.
// 4. Finite speeds with various signs work.
// 5. Axis is truly treated as being in the body frame.
TEST_F(SurfaceVelocityTest, ComputeSurfaceVelocity) {
  // Only the belt body has a surface velocity.
  plant_.SetSurfaceVelocityAxis(*belt_, Vector3d(0, 1, 0));

  plant_.Finalize();
  std::unique_ptr<systems::Context<double>> context =
      plant_.CreateDefaultContext();

  // Fixes a constant speed for the given body on the surface_speeds port.
  auto fix_belt_speed = [this, &context](double speed,
                                         const RigidBody<double>& body) {
    systems::BusValue bus;
    bus.Set(body.scoped_name().to_string(), Value<double>(speed));
    plant_.get_surface_speeds_input_port().FixValue(context.get(), bus);
  };

  // (1) Unconnected port → zero velocity.
  EXPECT_TRUE(
      CompareMatrices(MultibodyPlantTester::ComputeSurfaceVelocity(
                          plant_, belt_->index(), *context, Vector3d(0, 0, 1)),
                      Vector3d::Zero()));

  // (2) Connected port but missing signal --> zero velocity.
  fix_belt_speed(1.0, *other_);  // Setting other, *not* belt.
  EXPECT_TRUE(
      CompareMatrices(MultibodyPlantTester::ComputeSurfaceVelocity(
                          plant_, belt_->index(), *context, Vector3d(0, 0, 1)),
                      Vector3d::Zero()));

  // (3) Although bus has a signal for "other", no axis --> zero velocity.
  EXPECT_TRUE(
      CompareMatrices(MultibodyPlantTester::ComputeSurfaceVelocity(
                          plant_, other_->index(), *context, Vector3d(0, 0, 1)),
                      Vector3d::Zero()));

  // (4) Finite speeds all work.
  for (double speed : {0.25, 0.0, -0.75}) {
    SCOPED_TRACE(fmt::format("with speed = {}", speed));
    fix_belt_speed(speed, *belt_);
    // X_WB = I, so n_W = n_B and axis_B = axis_W. The computed velocity is
    // v = a X n = (0,1,0) X (0,0,1) = (1,0,0) * speed = (speed, 0, 0).
    const Vector3d n_W(0, 0, 1);  // Normal in world frame.
    const Vector3d v_B_expected(speed, 0, 0);
    EXPECT_TRUE(CompareMatrices(MultibodyPlantTester::ComputeSurfaceVelocity(
                                    plant_, belt_->index(), *context, n_W),
                                v_B_expected));
  }

  // (5) Axis follows the body pose.
  const RigidTransformd X_WB(RollPitchYawd(0.1, 0.2, 0.3),
                             Vector3d(0.5, -0.4, 0.3));
  plant_.SetFloatingBaseBodyPoseInWorldFrame(context.get(), *belt_, X_WB);
  fix_belt_speed(1.0, *belt_);
  const Vector3d n_W = X_WB.rotation().col(2);  // Normal in world frame.
  const Vector3d v_B_expected(1, 0, 0);
  EXPECT_TRUE(CompareMatrices(MultibodyPlantTester::ComputeSurfaceVelocity(
                                  plant_, belt_->index(), *context, n_W),
                              v_B_expected, 1e-16));
}

// Fixture for testing the surface displacement state and its accumulation.
//
class SurfaceDisplacementTest : public ::testing::Test {
 protected:
  void ConfigurePlant(double time_step) {
    plant_ = std::make_unique<MultibodyPlant<double>>(time_step);

    belt_ =
        &plant_->AddRigidBody("belt", SpatialInertia<double>::MakeUnitary());
    plant_->SetSurfaceVelocityAxis(*belt_, Vector3d(1, 0, 0));

    roller_ =
        &plant_->AddRigidBody("roller", SpatialInertia<double>::MakeUnitary());
    plant_->SetSurfaceVelocityAxis(*roller_, Vector3d(1, 0, 0));

    plant_->Finalize();
    context_ = plant_->CreateDefaultContext();
  }

  // Reads current surface displacement state. By reading from the output port
  // we ignore the mode-dependent details of state implementation and can
  // simply assess the effect.
  double ReadBeltDisplacement() const {
    const auto& output =
        plant_->get_surface_displacements_output_port().Eval<systems::BusValue>(
            *context_);
    const AbstractValue* value = output.Find(belt_->scoped_name().to_string());
    DRAKE_DEMAND(value != nullptr);
    return value->get_value<double>();
  }

  std::unique_ptr<MultibodyPlant<double>> plant_;
  std::unique_ptr<systems::Context<double>> context_;
  const RigidBody<double>* belt_{nullptr};
  const RigidBody<double>* roller_{nullptr};
};

// Confirms the events that should reset the plant's surface displacement state
// to zero.
//
// This is written to cover both discrete and continuous plants. This is
// important because where the surface displacement state is stored, depends on
// the plant's mode; correctness in one mode does *not* imply correctness in
// the other.
TEST_F(SurfaceDisplacementTest, StateResets) {
  for (double time_step : {0.0, 0.01}) {
    ConfigurePlant(time_step);

    // The allocation of a default context guarantees initial zero values.
    EXPECT_EQ(ReadBeltDisplacement(), 0.0);

    // SetDefaultContext() resets a dirtied displacement.
    MultibodyPlantTester::SetAllSurfaceDisplacements(*plant_, context_.get(),
                                                     1.25);
    ASSERT_EQ(ReadBeltDisplacement(), 1.25);
    plant_->SetDefaultContext(context_.get());
    EXPECT_EQ(ReadBeltDisplacement(), 0.0);

    // SetRandomContext() likewise resets a dirtied displacement. There is no
    // support for assigning distribution to surface displacement, so, as
    // documented, it reverts to default values.
    MultibodyPlantTester::SetAllSurfaceDisplacements(*plant_, context_.get(),
                                                     2.5);
    ASSERT_EQ(ReadBeltDisplacement(), 2.5);
    RandomGenerator generator;
    plant_->SetRandomContext(context_.get(), &generator);
    EXPECT_EQ(ReadBeltDisplacement(), 0.0);
  }
}

// Update of surface displacement for discrete plants. We're testing:
//
//  1. Each invocation accumulates one time_step.
//  2. The accumulation is on top of the value in the *context* and not in the
//     mutable state passed in.
//  3. Bodies with no signal on the surface_speeds bus do not accumulate.
//  4. Using a simulator to advance time one time step triggers accumulation.
TEST_F(SurfaceDisplacementTest, DiscreteAccumulation) {
  constexpr double kTimeStep = 0.01;
  ConfigurePlant(kTimeStep);  // discrete.

  const double speed = 2.0;
  FixSurfaceSpeed(*plant_, context_.get(), *belt_, speed);

  // Prime the context's displacement to a known non-zero baseline.
  const double baseline = 10.0;
  MultibodyPlantTester::SetAllSurfaceDisplacements(*plant_, context_.get(),
                                                   baseline);
  // The scratch state carries a bogus value; the handler must ignore it.
  auto scratch = context_->Clone();
  MultibodyPlantTester::SetAllSurfaceDisplacements(*plant_, scratch.get(),
                                                   999.0);
  // context's state is different from scratch's. The reported displacements
  // will reflect the value in context and not in scratch's state.
  MultibodyPlantTester::CallSurfaceDisplacementUpdate(
      *plant_, *context_, &scratch->get_mutable_state());
  EXPECT_NEAR(ReadSurfaceDisplacement(*plant_, *scratch, *belt_),
              baseline + speed * kTimeStep, 1e-14);
  EXPECT_NEAR(ReadSurfaceDisplacement(*plant_, *scratch, *roller_), baseline,
              1e-14);

  for (int k = 1; k <= 5; ++k) {
    SCOPED_TRACE(fmt::format("after {} update(s)", k));
    MultibodyPlantTester::CallSurfaceDisplacementUpdate(
        *plant_, *context_, &context_->get_mutable_state());
    EXPECT_NEAR(ReadSurfaceDisplacement(*plant_, *context_, *belt_),
                baseline + k * speed * kTimeStep, 1e-14);
    EXPECT_NEAR(ReadSurfaceDisplacement(*plant_, *context_, *roller_), baseline,
                1e-14);
  }

  // Reset context.
  context_->SetTime(0.0);
  MultibodyPlantTester::SetAllSurfaceDisplacements(*plant_, context_.get(),
                                                   baseline);
  systems::Simulator<double> simulator(*plant_, std::move(context_));
  const double t_final = kTimeStep * 5;
  simulator.AdvanceTo(t_final);

  // Over [0, t_final] with a period of time_step, the event integrates the
  // constant speed to speed*t_final.
  EXPECT_NEAR(ReadSurfaceDisplacement(*plant_, simulator.get_context(), *belt_),
              baseline + speed * t_final, 1e-12);
}

// Update of surface displacement for continuous plants. We're testing:
//
//  1. Derivatives of surface displacement are exactly the surface speed.
//  2. Derivative for signal missing from input bus is zero.
//  3. As continuous state, the integral is simply speed * time.
TEST_F(SurfaceDisplacementTest, ContinuousIntegration) {
  ConfigurePlant(/* time_step= */ 0.0);  // continuous.

  const double speed = 2.0;
  FixSurfaceSpeed(*plant_, context_.get(), *belt_, speed);

  // The time derivative of the displacement is exactly the surface speed.
  const systems::ContinuousState<double>& derivatives =
      plant_->EvalTimeDerivatives(*context_);
  ASSERT_EQ(derivatives.get_misc_continuous_state().size(), 2);
  EXPECT_EQ(derivatives.get_misc_continuous_state().GetAtIndex(0), speed);
  EXPECT_EQ(derivatives.get_misc_continuous_state().GetAtIndex(1), 0.0);

  // Integrating over time accumulates speed * t.
  systems::Simulator<double> simulator(*plant_, std::move(context_));
  const double t_final = 0.5;
  simulator.AdvanceTo(t_final);
  EXPECT_NEAR(ReadSurfaceDisplacement(*plant_, simulator.get_context(), *belt_),
              speed * t_final, 1e-9);
  EXPECT_EQ(ReadSurfaceDisplacement(*plant_, simulator.get_context(), *roller_),
            0.0);
}

}  // namespace
}  // namespace multibody
}  // namespace drake
