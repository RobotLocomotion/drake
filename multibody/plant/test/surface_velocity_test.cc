#include <memory>
#include <optional>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"
#include "drake/common/random.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/value.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/multibody/plant/multibody_plant.h"
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
      for (int i = 0; i < z.size(); ++i) {
        z.SetAtIndex(i, value);
      }
    }
  }
};

namespace {

using Eigen::Vector3d;
using math::RigidTransformd;
using math::RollPitchYawd;

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

// Fixture: finalized standalone plant, standalone context
//
// "belt" is registered with non-unit axis (2,0,0) to exercise normalization.
// "other" is intentionally left without surface velocity. Tests that need a
// speed wired to the surface_speeds port call FixValue().
class SurfaceVelocityTest : public ::testing::Test {
 protected:
  void SetUp() override {
    belt_ = &plant_.AddRigidBody("belt", SpatialInertia<double>::MakeUnitary());
    other_ =
        &plant_.AddRigidBody("other", SpatialInertia<double>::MakeUnitary());
    // Intentionally non-unit to exercise normalization.
    plant_.SetSurfaceVelocityAxis(*belt_, Vector3d(2, 0, 0));
    plant_.Finalize();
    context_ = plant_.CreateDefaultContext();
  }

  // Fixes a constant speed for "belt" on the surface_speeds port.
  void FixBeltSpeed(double speed) {
    systems::BusValue bus;
    bus.Set(belt_->scoped_name().to_string(), Value<double>(speed));
    plant_.get_surface_speeds_input_port().FixValue(context_.get(), bus);
  }

  MultibodyPlant<double> plant_{0.0};
  const RigidBody<double>* belt_{nullptr};
  const RigidBody<double>* other_{nullptr};
  std::unique_ptr<systems::Context<double>> context_;
};

// SetSurfaceVelocityAxis normalizes the stored direction: (2,0,0) → (1,0,0).
TEST_F(SurfaceVelocityTest, SetAxisNormalizesInput) {
  EXPECT_TRUE(CompareMatrices(*plant_.GetSurfaceVelocityAxis(*belt_),
                              Vector3d(1, 0, 0)));
}

// GetSurfaceVelocityAxis returns nullopt for an unregistered body.
TEST_F(SurfaceVelocityTest, GetAxisNulloptForUnregisteredBody) {
  EXPECT_EQ(plant_.GetSurfaceVelocityAxis(*other_), std::nullopt);
}

// SetSurfaceVelocityAxis is pre-Finalize only.
TEST_F(SurfaceVelocityTest, SetAxisThrowsAfterFinalize) {
  EXPECT_THROW(plant_.SetSurfaceVelocityAxis(*belt_, Vector3d(0, 1, 0)),
               std::exception);
}

// SetSurfaceVelocityAxis normalizes its input, can overwrite an existing
// registration, and the value survives Finalize().
GTEST_TEST(MultibodyPlantTest, SetSurfaceVelocityAxisNormalizesAndPersists) {
  MultibodyPlant<double> plant{0.0};
  const auto& belt =
      plant.AddRigidBody("belt", SpatialInertia<double>::MakeUnitary());
  plant.SetSurfaceVelocityAxis(belt, Vector3d(1, 0, 0));
  // Overwrite with a non-unit vector; should be normalized to (0,1,0).
  plant.SetSurfaceVelocityAxis(belt, Vector3d(0, 1e-12, 0));
  EXPECT_TRUE(
      CompareMatrices(*plant.GetSurfaceVelocityAxis(belt), Vector3d(0, 1, 0)));
  plant.Finalize();
  // Value is unchanged after finalization.
  EXPECT_TRUE(
      CompareMatrices(*plant.GetSurfaceVelocityAxis(belt), Vector3d(0, 1, 0)));
}

// SetSurfaceVelocityAxis(body, nullopt) clears a registration.
GTEST_TEST(MultibodyPlantTest, SetSurfaceVelocityAxisNulloptClears) {
  MultibodyPlant<double> plant{0.0};
  const auto& belt =
      plant.AddRigidBody("belt", SpatialInertia<double>::MakeUnitary());
  plant.SetSurfaceVelocityAxis(belt, Vector3d(1, 0, 0));
  EXPECT_TRUE(plant.GetSurfaceVelocityAxis(belt).has_value());
  plant.SetSurfaceVelocityAxis(belt, std::nullopt);
  EXPECT_FALSE(plant.GetSurfaceVelocityAxis(belt).has_value());
}

// Unconnected port (no FixValue called) → speed treated as zero.
TEST_F(SurfaceVelocityTest, ZeroVelocityWhenPortUnconnected) {
  const Vector3d v = MultibodyPlantTester::ComputeSurfaceVelocity(
      plant_, belt_->index(), *context_, Vector3d(0, 0, 1));
  EXPECT_TRUE(CompareMatrices(v, Vector3d::Zero()));
}

// Port connected but carrying no signal for this body → zero.
TEST_F(SurfaceVelocityTest, ZeroVelocityWhenSignalAbsent) {
  plant_.get_surface_speeds_input_port().FixValue(context_.get(),
                                                  systems::BusValue{});
  const Vector3d v = MultibodyPlantTester::ComputeSurfaceVelocity(
      plant_, belt_->index(), *context_, Vector3d(0, 0, 1));
  EXPECT_TRUE(CompareMatrices(v, Vector3d::Zero()));
}

// Body never registered → zero regardless of port state.
TEST_F(SurfaceVelocityTest, ZeroVelocityForUnregisteredBody) {
  FixBeltSpeed(1.0);
  const Vector3d v = MultibodyPlantTester::ComputeSurfaceVelocity(
      plant_, other_->index(), *context_, Vector3d(0, 0, 1));
  EXPECT_TRUE(CompareMatrices(v, Vector3d::Zero()));
}

// The speed can be positive or negative, flipping the velocity direction.
// a_ss_B = (1,0,0), n_W = (0,0,1) (R_WB = I, so n_B = n_W).
// speed = s: v = s*(1,0,0)×(0,0,1) = s*(0, -1, 0) = (0, -s, 0).
TEST_F(SurfaceVelocityTest, WorksWithFiniteSpeeds) {
  for (double speed : {0.25, 0.0, -0.75}) {
    SCOPED_TRACE(fmt::format("with speed = {}", speed));
    FixBeltSpeed(speed);
    const Vector3d v = MultibodyPlantTester::ComputeSurfaceVelocity(
        plant_, belt_->index(), *context_, Vector3d(0, 0, 1));
    EXPECT_TRUE(CompareMatrices(v, Vector3d(0, -speed, 0)));
  }
}

TEST_F(SurfaceVelocityTest, SurfaceSpeedsPortName) {
  EXPECT_EQ(plant_.get_surface_speeds_input_port().get_name(),
            "surface_speeds");
}

// Confirms that a non-zero surface displacement is reset to zero by both
// SetDefaultContext() and SetRandomContext(). The displacement lives in the
// misc continuous state (continuous plants) or the abstract state (discrete
// plants); `time_step` selects the mode. Surface displacement has no random
// distribution, so randomizing must fall back to the default (zero) value.
void CheckSurfaceDisplacementResets(double time_step) {
  SCOPED_TRACE(time_step == 0.0 ? "continuous" : "discrete");
  MultibodyPlant<double> plant(time_step);
  const RigidBody<double>& belt =
      plant.AddRigidBody("belt", SpatialInertia<double>::MakeUnitary());
  plant.SetSurfaceVelocityAxis(belt, Vector3d(1, 0, 0));
  plant.Finalize();
  auto context = plant.CreateDefaultContext();

  // Reads belt's displacement via the output port (mode-agnostic).
  auto belt_displacement = [&]() {
    const auto& output =
        plant.get_surface_displacement_output_port().Eval<systems::BusValue>(
            *context);
    const AbstractValue* value = output.Find(belt.scoped_name().to_string());
    DRAKE_DEMAND(value != nullptr);
    return value->get_value<double>();
  };

  // The default is zero.
  EXPECT_EQ(belt_displacement(), 0.0);

  // SetDefaultContext() resets a dirtied displacement.
  MultibodyPlantTester::SetAllSurfaceDisplacements(plant, context.get(), 1.25);
  ASSERT_EQ(belt_displacement(), 1.25);
  plant.SetDefaultContext(context.get());
  EXPECT_EQ(belt_displacement(), 0.0);

  // SetRandomContext() likewise resets a dirtied displacement.
  MultibodyPlantTester::SetAllSurfaceDisplacements(plant, context.get(), 2.5);
  ASSERT_EQ(belt_displacement(), 2.5);
  RandomGenerator generator;
  plant.SetRandomContext(context.get(), &generator);
  EXPECT_EQ(belt_displacement(), 0.0);
}

// Continuous plant: displacement (misc continuous state z) resets to zero.
GTEST_TEST(MultibodyPlantTest, ContinuousSurfaceDisplacementResets) {
  CheckSurfaceDisplacementResets(0.0);
}

// Discrete plant: displacement (abstract accumulation state) resets to zero.
GTEST_TEST(MultibodyPlantTest, DiscreteSurfaceDisplacementResets) {
  CheckSurfaceDisplacementResets(0.01);
}

// Confirm that the surface velocity follows the body pose in the world; put the
// body in an arbitrary, non-trival pose.
TEST_F(SurfaceVelocityTest, SurfaceVelocityPosedInWorld) {
  const RigidTransformd X_WB(RollPitchYawd(0.1, 0.2, 0.3),
                             Vector3d(0.5, -0.4, 0.3));
  plant_.SetFloatingBaseBodyPoseInWorldFrame(context_.get(), *belt_, X_WB);

  // For contact normal n_C_B = (0, 0, 1), the expected surface velocity,
  // v_ss_B is simply (0, -s, 0). So, given X_WB, we pass in n_C_W and should
  // get the expected v_ss_B.
  const Vector3d n_C_B(0, 0, 1);
  double speed = 0.75;
  const Vector3d v_ss_B_expected(0, -speed, 0);
  FixBeltSpeed(speed);
  const Vector3d n_C_W = X_WB.rotation() * n_C_B;

  const Vector3d v_ss_B = MultibodyPlantTester::ComputeSurfaceVelocity(
      plant_, belt_->index(), *context_, n_C_W);
  EXPECT_TRUE(CompareMatrices(v_ss_B, v_ss_B_expected, 1e-15));
}

}  // namespace
}  // namespace multibody
}  // namespace drake
