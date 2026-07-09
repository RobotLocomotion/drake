#include <algorithm>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"
#include "drake/common/random.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/value.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/geometry/scene_graph.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/plant/contact_results.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/multibody_plant_config_functions.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/bus_value.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace multibody {

// Exposes surface velocity APIs for testing.
class MultibodyPlantTester {
 public:
  MultibodyPlantTester() = delete;
  static Vector3<double> ComputeSurfaceVelocity(
      const MultibodyPlant<double>& plant, BodyIndex body_index,
      const systems::Context<double>& context, const Eigen::Vector3d& n_W) {
    return plant.ComputeSurfaceVelocity(body_index, context, n_W);
  }

  // Invokes the (private) periodic displacement-update handler directly.
  // If `state` is defined, the result is written to that.
  // If `state` is not provided, the result is ultimately written back to
  // `context` (akin to a Simulator evaluating an event).
  // This enables testing the integration math without worrying about framework
  // triggers.
  static systems::EventStatus CallSurfaceDisplacementUpdate(
      const MultibodyPlant<double>& plant, systems::Context<double>* context,
      systems::State<double>* state = nullptr) {
    DRAKE_DEMAND(context != nullptr);
    if (state != nullptr) {
      return plant.CalcSurfaceDisplacementUpdate(*context, state);
    }
    // We have no way to simply allocate a State object, so we'll use a cloned
    // context to make one.
    auto cloned_context = context->Clone();
    systems::EventStatus status = plant.CalcSurfaceDisplacementUpdate(
        *cloned_context, &cloned_context->get_mutable_state());
    context->SetStateAndParametersFrom(*cloned_context);
    return status;
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
              plant.surface_displacements_abstract_state_index_);
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
using geometry::AddCompliantHydroelasticProperties;
using geometry::AddContactMaterial;
using geometry::AddRigidHydroelasticProperties;
using geometry::Box;
using geometry::GeometryId;
using geometry::GeometryInstance;
using geometry::ProximityProperties;
using geometry::Sphere;
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
//  S1. Can be called pre-finalize.
//  S2. Unnormalized vectors get normalized.
//  S3. New axis overwrites old axis.
//  S4. Setting nullopt clears axis.
//  S5. Can't be called post-finalize().
//  S6. Zero vectors can't be normalized (throws).
//  S7. Can't be called on the world body.
//
// GetSurfaceVelocityAxis()
//  G1. Can be called pre-finalize.
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

  // (S7) Can't call on the world body.
  EXPECT_THROW(
      plant_.SetSurfaceVelocityAxis(plant_.world_body(), Vector3d{1, 0, 0}),
      std::exception);

  // Put an axis back onto belt.
  plant_.SetSurfaceVelocityAxis(*belt_, Vector3d(0, 0, 1));
  plant_.Finalize();

  // (G3) Value still present after finalize.
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

  // (1) Unconnected port → zero velocity.
  EXPECT_TRUE(
      CompareMatrices(MultibodyPlantTester::ComputeSurfaceVelocity(
                          plant_, belt_->index(), *context, Vector3d(0, 0, 1)),
                      Vector3d::Zero()));

  // (2) Connected port but missing BusValue signal --> zero velocity.
  // Setting other, *not* belt.
  FixSurfaceSpeed(plant_, context.get(), *other_, 1.0);
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
    FixSurfaceSpeed(plant_, context.get(), *belt_, speed);
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
  FixSurfaceSpeed(plant_, context.get(), *belt_, 1.0);
  const Vector3d n_W = X_WB.rotation().col(2);  // Normal in world frame.
  const Vector3d v_B_expected(1, 0, 0);
  EXPECT_TRUE(CompareMatrices(MultibodyPlantTester::ComputeSurfaceVelocity(
                                  plant_, belt_->index(), *context, n_W),
                              v_B_expected, 1e-14));
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

  double ReadBodySurfaceDisplacement(
      const RigidBody<double>& body,
      const systems::Context<double>* context = nullptr) {
    if (context == nullptr) context = context_.get();
    return ReadSurfaceDisplacement(*plant_, *context, body);
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
    EXPECT_EQ(ReadBodySurfaceDisplacement(*belt_), 0.0);

    // SetDefaultContext() resets a dirtied displacement.
    MultibodyPlantTester::SetAllSurfaceDisplacements(*plant_, context_.get(),
                                                     1.25);
    ASSERT_EQ(ReadBodySurfaceDisplacement(*belt_), 1.25);
    plant_->SetDefaultContext(context_.get());
    EXPECT_EQ(ReadBodySurfaceDisplacement(*belt_), 0.0);

    // SetRandomContext() likewise resets a dirtied displacement. There is no
    // support for assigning distribution to surface displacement, so, as
    // documented, it reverts to default values.
    MultibodyPlantTester::SetAllSurfaceDisplacements(*plant_, context_.get(),
                                                     2.5);
    ASSERT_EQ(ReadBodySurfaceDisplacement(*belt_), 2.5);
    RandomGenerator generator;
    plant_->SetRandomContext(context_.get(), &generator);
    EXPECT_EQ(ReadBodySurfaceDisplacement(*belt_), 0.0);
  }
}

// In the case where the plant has no connection on the surface_speeds input
// port, both continuous and discrete plants return a fully-populated output
// bus. The values in the bus are the unchanged state -- equivalent to
// connecting input speeds all equal to zero.
TEST_F(SurfaceDisplacementTest, NoInputConnection) {
  for (double time_step : {0.0, 0.01}) {
    SCOPED_TRACE(fmt::format("time_step = {}", time_step));
    ConfigurePlant(time_step);
    ASSERT_NE(plant_->GetSurfaceVelocityAxis(*belt_), std::nullopt);
    ASSERT_NE(plant_->GetSurfaceVelocityAxis(*roller_), std::nullopt);

    // Arbitrary non-zero value to distinguish initial value from unchanging
    // value.
    const double baseline = 10.0;
    MultibodyPlantTester::SetAllSurfaceDisplacements(*plant_, context_.get(),
                                                     baseline);

    // We'll use a simulator to exercise the continuous/discrete state update
    // machinery agnostically.
    systems::Simulator<double> simulator(*plant_, std::move(context_));
    simulator.AdvanceTo(0.1);

    // Check the output - confirm both signals unchanged from baseline.
    EXPECT_EQ(ReadBodySurfaceDisplacement(*belt_, &simulator.get_context()),
              baseline);
    EXPECT_EQ(ReadBodySurfaceDisplacement(*roller_, &simulator.get_context()),
              baseline);
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

  // With the surface_speeds input port unconnected, attempting to update the
  // displacement state should report that nothing happened.
  EXPECT_EQ(MultibodyPlantTester::CallSurfaceDisplacementUpdate(*plant_,
                                                                context_.get())
                .severity(),
            systems::EventStatus::DidNothing().severity());

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
      *plant_, context_.get(), &scratch->get_mutable_state());
  EXPECT_NEAR(ReadBodySurfaceDisplacement(*belt_, scratch.get()),
              baseline + speed * kTimeStep, 1e-14);
  EXPECT_NEAR(ReadBodySurfaceDisplacement(*roller_), baseline, 1e-14);

  for (int k = 1; k <= 5; ++k) {
    SCOPED_TRACE(fmt::format("after {} update(s)", k));
    MultibodyPlantTester::CallSurfaceDisplacementUpdate(*plant_,
                                                        context_.get());
    EXPECT_NEAR(ReadBodySurfaceDisplacement(*belt_),
                baseline + k * speed * kTimeStep, 1e-14);
    EXPECT_NEAR(ReadBodySurfaceDisplacement(*roller_), baseline, 1e-14);
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
  EXPECT_NEAR(ReadBodySurfaceDisplacement(*belt_, &simulator.get_context()),
              baseline + speed * t_final, 1e-14);

  // As a side effect, simply invoking a forced event increments surface
  // displacement by speed * time_step per invocation.
  // Reconfigure a fresh plant/context.
  ConfigurePlant(kTimeStep);
  FixSurfaceSpeed(*plant_, context_.get(), *belt_, speed);
  MultibodyPlantTester::SetAllSurfaceDisplacements(*plant_, context_.get(),
                                                   baseline);
  for (int k = 1; k <= 5; ++k) {
    SCOPED_TRACE(fmt::format("after {} forced event(s)", k));
    plant_->ExecuteForcedEvents(context_.get());
    EXPECT_NEAR(ReadBodySurfaceDisplacement(*belt_),
                baseline + k * speed * kTimeStep, 1e-14);
    EXPECT_NEAR(ReadBodySurfaceDisplacement(*roller_), baseline, 1e-14);
  }
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
  EXPECT_NEAR(ReadBodySurfaceDisplacement(*belt_, &simulator.get_context()),
              speed * t_final, 1e-9);
  EXPECT_EQ(ReadBodySurfaceDisplacement(*roller_, &simulator.get_context()),
            0.0);
}

// Generalized physics test harness.
//
// We've established that the surface velocity reports the right thing based
// on port values and body poses and the like. What remains is some positive
// indication that observed contact introduces the expected effect. This test
// is an attempt to do that in a *general* way. We want to make sure the
// contact models do the following:
//
// a. Consider both bodies in contact for surface velocity.
// b. Account for the bodies' poses in mapping surface velocity to the world.
// c. Combine the surface velocity contributions in a consistent way.
// d. The effect should affect both dynamics and reported contact forces; they
//    are independent code paths and are therefore tested independently (see
//    below).
//
// If it does all that, we should get appropriate forces (due to friction).
// Generally, examining the forces isn't trivial across contact models and
// time steppers. So, we'll adopt a more indirect, but universal, approach.
//
// - Take a small time step and observe the contact forces (as reported
//   in the plant's output port). Does it have a tangential component we can
//   clearly attribute to the surface velocity?
// - With no other source of motion, do bodies move relative to each other in a
//   way which, again, can be clearly attributed to the surface velocity?
//
// To that end, we will place a box on a ground plane. The configuration will
// help us achieve the testing goals:
//
// a. Both bodies will be given surface velocities in orthogonal directions.
// b. Both bodies will be rotated 90° around Wz.
// c. Given the surface velocities, we'll predict force and movement
//    *directions*.
//
// Here's what that looks like.
//
//                          ┏━━━━━━━━━━━━┓
//           Gy             ┃   By       ┃
//           │              ┃   │        ┃
//           └── Gx         ┃   └── Bx   ┃                ┌── Wy
//           → axis_G       ┃   ↓        ┃               ╱│
//                          ┃   axis_B   ┃              ╱ Wx
//                          ┗━━━━━━━━━━━━┛             ╱
//                                                  v_ss_GB_W
//
// Surface velocity axes:
//  - R_WG = R_WB = 90° Wz-rotation.
//  - axis_B = -By  --> axis_B_W = R_WB⋅(-By) = Wx
//  - axis_G = Gx   --> axis_G_W = R_WG⋅Gx = Wy
//
// Surface speeds:
//  - ss_B = surface speed for the box.
//  - ss_G = surface speed for the ground.
//
// The box rests on top of ground creating the following contact normals:
//  - n_C_B = -Wz  (Contact on the *bottom* of the box --> contact normal points
//                  down.)
//  - n_C_G = +Wz  (The contact normal on the ground points up.)
//
// The expected surface velocities at contact are:
//  - v_ss_B_W = ss_B * axis_B_W × n_C_B = ss_B * Wx × -Wz = ss_B * Wy
//  - v_ss_G_W = ss_G * axis_G_W × n_C_G = ss_G * Wy × Wz = ss_G * Wx
//
// The surface velocity of the box w.r.t. the ground at the contact point is
// (drawn in the illustration):
//  - v_ss_GB_W = v_ss_B_W - v_ss_G_W
//              = ss_B * Wy - ss_G * Wx
//              = [-ss_G, ss_B, 0]_W.
//
// This non-zero velocity is orthogonal to the contact normal so it will lead
// to a frictional force in the opposite direction. We'd expect the force on the
// box to be:
//  - f_B_W ∝ -[-ss_G, ss_B, 0]_W = [ss_G, -ssB, 0]_W.
//
// We'll test for a force in that direction and verify that a force applied
// in that direction leads to motion in the same direction.

struct OrthogonalContactTestConfig {
  std::string description;
  MultibodyPlantConfig plant_config;
};

// Formatter for OrthogonalContactTestConfig so that if a test fails, we get
// meaningful feedback about the test configuration instead of a byte string.
void PrintTo(const OrthogonalContactTestConfig& config, std::ostream* os) {
  const MultibodyPlantConfig& plant_config = config.plant_config;
  *os << "{ time_step: " << plant_config.time_step
      << ", contact_model: " << plant_config.contact_model
      << ", discrete_contact_approximation: "
      << plant_config.discrete_contact_approximation << " }";
}

class OrthogonalSurfaceVelocityTest
    : public ::testing::TestWithParam<OrthogonalContactTestConfig> {
 protected:
  static constexpr double kTheta = M_PI / 2;    // 90° rotation around world Z
  static constexpr double kHalfSize = 0.1;      // box is 0.2 m cube
  static constexpr double kPenetration = 1e-3;  // initial overlap, m
  static constexpr double kStiffness = 1e5;     // N/m, per body (point contact)
  static constexpr double kHydroModulus = 1e6;  // Pa (hydroelastic compliance)
  static constexpr double kMu = 1.0;
  static constexpr double kGroundSpeed = 1.0;  // m/s
  static constexpr double kBoxSpeed = 1.0;     // m/s

  void SetUp() override {
    const auto& config = GetParam().plant_config;

    systems::DiagramBuilder<double> builder;
    auto [plant_ref, scene_graph_ref] =
        AddMultibodyPlantSceneGraph(&builder, config.time_step);
    ApplyMultibodyPlantConfig(config, &plant_ref);
    plant_ = &plant_ref;

    ProximityProperties material;
    AddContactMaterial(0.0, kStiffness, CoulombFriction<double>(kMu, kMu),
                       &material);
    ProximityProperties rigid(material);
    AddRigidHydroelasticProperties(&rigid);
    ProximityProperties compliant(material);
    AddCompliantHydroelasticProperties(kHalfSize, kHydroModulus, &compliant);

    const RollPitchYawd Rz_90(0.0, 0.0, kTheta);

    // Ground: large Box welded to world (top face at z=0), rotated kTheta
    // around Z. axis_ss_B = (1,0,0) → world surface velocity = kGroundSpeed *
    // Wx.
    ground_ =
        &plant_->AddRigidBody("ground", SpatialInertia<double>::MakeUnitary());
    plant_->WeldFrames(plant_->world_frame(), ground_->body_frame(),
                       RigidTransformd(Rz_90, Vector3d::Zero()));
    plant_->RegisterCollisionGeometry(*ground_,
                                      RigidTransformd(Vector3d(0, 0, -0.5)),
                                      Box(10.0, 10.0, 1.0), "ground", rigid);
    plant_->SetSurfaceVelocityAxis(*ground_, Vector3d(1, 0, 0));

    // Box: free floating, 0.2 m cube.
    // axis_ss_B = (0,-1,0) → world surface velocity = kBoxSpeed * Wy.
    // (Box is body B; contact pairs pass −Ẑ to B, so the cross product
    // flips sign.)
    box_ = &plant_->AddRigidBody(
        "box", SpatialInertia<double>::SolidBoxWithMass(
                   1.0, 2 * kHalfSize, 2 * kHalfSize, 2 * kHalfSize));
    plant_->RegisterCollisionGeometry(
        *box_, RigidTransformd::Identity(),
        Box(2 * kHalfSize, 2 * kHalfSize, 2 * kHalfSize), "box", compliant);
    plant_->SetSurfaceVelocityAxis(*box_, Vector3d(0, -1, 0));

    plant_->Finalize();
    auto diagram = builder.Build();
    sim_ = std::make_unique<systems::Simulator<double>>(std::move(diagram));
    auto& plant_context =
        plant_->GetMyMutableContextFromRoot(&sim_->get_mutable_context());

    // Place box rotated kTheta around Z with bottom face at z = -kPenetration.
    plant_->SetFloatingBaseBodyPoseInWorldFrame(
        &plant_context, *box_,
        RigidTransformd(Rz_90, Vector3d(0, 0, kHalfSize - kPenetration)));

    systems::BusValue bus;
    bus.Set(ground_->scoped_name().to_string(), Value<double>(kGroundSpeed));
    bus.Set(box_->scoped_name().to_string(), Value<double>(kBoxSpeed));
    plant_->get_surface_speeds_input_port().FixValue(&plant_context, bus);
    sim_->Initialize();
  }

  // Returns the total contact force on the contacting body (rigid box or
  // deformable sphere) from ContactResults.
  Vector3d ContactForceOnContactingBody(
      const systems::Context<double>& plant_context) const {
    const auto& results =
        plant_->get_contact_results_output_port().Eval<ContactResults<double>>(
            plant_context);

    // The total force on the box as accumulated from point and hydro contact.
    // The code below simply sums up all apparently relevant contact forces.
    // This is a cheat because each reported force would be applied to the box
    // at different positions. However, we exploit the fact that the test has
    // been set up such that there is only a single contact force acting on the
    // box for any given experiment. This allows us to ignore the contact point.
    // We enforce the assumption with the test on active_forces.
    int active_forces = 0;
    Vector3d f_Box = Vector3d::Zero();

    // Point contacts: contact_force() is the force on body B.
    for (int i = 0; i < results.num_point_pair_contacts(); ++i) {
      const auto& info = results.point_pair_contact_info(i);
      if (info.bodyB_index() == box_->index()) {
        f_Box += info.contact_force();
        ++active_forces;
      } else if (info.bodyA_index() == box_->index()) {
        f_Box -= info.contact_force();
        ++active_forces;
      }
    }

    // Hydroelastic contacts: F_Ac_W() is the force on body A (id_M geometry).
    // If the box is body A in the contact results, we apply the force,
    // otherwise, we reverse the force.
    const auto& box_geom_ids = plant_->GetCollisionGeometriesForBody(*box_);
    for (int i = 0; i < results.num_hydroelastic_contacts(); ++i) {
      const auto& hydro_info = results.hydroelastic_contact_info(i);
      const GeometryId id_M = hydro_info.contact_surface().id_M();
      const bool box_is_body_A =
          std::find(box_geom_ids.begin(), box_geom_ids.end(), id_M) !=
          box_geom_ids.end();
      if (box_is_body_A) {
        f_Box += hydro_info.F_Ac_W().translational();
        ++active_forces;
      } else {
        f_Box -= hydro_info.F_Ac_W().translational();
        ++active_forces;
      }
    }

    DRAKE_DEMAND(active_forces <= 1);

    return f_Box;
  }

  // Returns the position of the box.
  Vector3d BoxPositionInWorld(
      const systems::Context<double>& plant_context) const {
    return plant_->EvalBodyPoseInWorld(plant_context, *box_).translation();
  }

  MultibodyPlant<double>* plant_{nullptr};
  const RigidBody<double>* ground_{nullptr};
  const RigidBody<double>* box_{nullptr};
  std::optional<DeformableBodyId> deformable_id_;
  std::unique_ptr<systems::Simulator<double>> sim_;
};

// Confirms the effect of the surface velocity appears in the contact results.
// After one contact step, the tangential force on the box should lie in the
// (+Wx, -Wy) direction: +Wx from the ground belt, -Wy from the box's own
// surface. With kGroundSpeed == kBoxSpeed the two components are equal in
// magnitude.
TEST_P(OrthogonalSurfaceVelocityTest, ContactForceTangentialDirection) {
  const double time_step = GetParam().plant_config.time_step;
  // Discrete: advance one step to populate DiscreteStepMemory.
  // Continuous: contact results are available on demand at t = 0.
  if (time_step > 0.0) sim_->AdvanceTo(time_step);
  const Vector3d f = ContactForceOnContactingBody(
      plant_->GetMyContextFromRoot(sim_->get_context()));

  EXPECT_GT(f.z(), 0.0);  // normal force pushes body up
  EXPECT_GT(f.x(), 0.0)
      << "ground belt (kGroundSpeed in +Wx) should push body in +Wx";
  EXPECT_LT(f.y(), 0.0) << "box surface (kBoxSpeed in +Wy) should react in -Wy";
  // Equal speeds → equal-magnitude tangential components, within 10%.
  EXPECT_NEAR(f.x(), -f.y(), 0.1 * f.x());
}

// Confirms the effect of the surface velocity on continuous dynamics.
// After integrating for 0.3 s the box should have displaced in (+Wx, -Wy).
// Contact results and dynamics are independent code paths and require
// independent tests.
TEST_P(OrthogonalSurfaceVelocityTest, BoxDisplacementDirection) {
  // Continuous simulation is *very* slow; don't advance too far.
  sim_->AdvanceTo(0.1);
  const auto& final_ctx = plant_->GetMyContextFromRoot(sim_->get_context());
  const Vector3d p_WBody = BoxPositionInWorld(final_ctx);

  constexpr double kMinDisplacement = 0.01;  // 1 cm
  EXPECT_GT(p_WBody.x(), kMinDisplacement)
      << "body should have moved in +Wx due to ground belt";
  EXPECT_LT(p_WBody.y(), -kMinDisplacement)
      << "box should have moved in -Wy due to box surface velocity";
}

INSTANTIATE_TEST_SUITE_P(
    AllContactRegimes, OrthogonalSurfaceVelocityTest, testing::ValuesIn([] {
      MultibodyPlantConfig continuous_point;
      continuous_point.time_step = 0.0;
      continuous_point.contact_model = "point";

      MultibodyPlantConfig continuous_hydro;
      continuous_hydro.time_step = 0.0;
      continuous_hydro.contact_model = "hydroelastic_with_fallback";

      return std::vector<OrthogonalContactTestConfig>{
          // Keep continuous_point and continuous_hydro next to each other in
          // this list so that the sharding will make sure they end up in
          // different shards. Continuous simulation in debug build is
          // terrifyingly slow.
          {"continuous_point", continuous_point},
          {"continuous_hydro", continuous_hydro},
      };
    }()),
    [](const testing::TestParamInfo<OrthogonalContactTestConfig>& param_info) {
      return param_info.param.description;
    });

}  // namespace
}  // namespace multibody
}  // namespace drake
