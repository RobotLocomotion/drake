#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/examples/quadrotor/quadrotor_plant.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"

namespace drake {
namespace examples {
namespace quadrotor {
namespace {

using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::VectorXd;
using math::RigidTransformd;
using math::RollPitchYawd;
using multibody::MultibodyPlant;
using multibody::Parser;
using multibody::RigidBody;
using multibody::SpatialVelocity;
using systems::ConstantVectorSource;
using systems::Context;
using systems::ContinuousState;
using systems::Diagram;
using systems::DiagramBuilder;
using systems::Simulator;

constexpr double kEpsilon = std::numeric_limits<double>::epsilon();

// This test program compares the behaviour of two plants:
//
// (#1) a GenericQuadrotor created from a QuadrotorPlant, and
//
// (#2) a MultibodyQuadrotor created from parsing a corresponding model URDF
// file into a MultibodyPlant.
//
// Both types of plant provide a method SetState whose argument is organized as
// [x, y, z, r, p, y, ẋ, ẏ, ż, ṙ, ṗ, ẏ], but the underlying state representation
// in the Context is NOT necessarily organized that way.
//
// Both types also provide GetPose.

// The models are integrated and compared for the duration specified by the
// following constant.
const double kSimulationDuration = 0.1;

// Case (#1): The Diagram that wraps a QuadrotorPlant.
class GenericQuadrotor : public Diagram<double> {
 public:
  GenericQuadrotor() {
    DiagramBuilder<double> builder;
    plant_ = builder.template AddSystem<QuadrotorPlant<double>>();
    auto* source = builder.template AddSystem<ConstantVectorSource<double>>(
        VectorXd::Zero(plant_->num_total_inputs()));
    builder.Cascade(*source, *plant_);
    builder.BuildInto(this);
  }

  // @param[in,out] context a mutable Context to be set from the state vector x.
  // @param[in] x 12-element state vector [x, y, z, r, p, y, ẋ, ẏ, ż, ṙ, ṗ, ẏ].
  void SetState(Context<double>* context, const VectorXd& x) const {
    DRAKE_DEMAND(x.size() == 12);
    Context<double>& plant_context =
        this->GetMutableSubsystemContext(*plant_, context);
    plant_context.SetContinuousState(x);
  }

  // Returns X_WB, the pose relating world W to the base-link body B.
  // @param[in] context contains the state of the multibody system.
  RigidTransformd GetPose(const Context<double>& context) const {
    // Reminder: The state vector is [x, y, z, r, p, y, ẋ, ẏ, ż, ṙ, ṗ, ẏ].
    const VectorXd state =
        context.get_continuous_state().get_vector().CopyToVector();
    const Vector3d xyz = state.segment(0, 3);
    const Vector3d rpy = state.segment(3, 3);
    return RigidTransformd(RollPitchYawd(rpy), xyz);
  }

  // Returns V_WB_W, base-link B's spatial velocity measured and expressed in
  // the world frame W.
  // @param[in] context contains the state of the multibody system.
  SpatialVelocity<double> GetVelocity(const Context<double>& context) const {
    // Reminder: The state vector is [x, y, z, r, p, y, ẋ, ẏ, ż, ṙ, ṗ, ẏ].
    const VectorXd state =
        context.get_continuous_state().get_vector().CopyToVector();
    const Vector3d xyz_dot = state.segment(6, 3);  // v_WBo_W = [ẋ, ẏ, ż].
    const Vector3d rpy_dot = state.segment(9, 3);
    const math::RollPitchYaw<double> rpy(state.segment(3, 3));
    const Vector3d w_WB_W = rpy.CalcAngularVelocityInParentFromRpyDt(rpy_dot);
    return SpatialVelocity<double>(w_WB_W, xyz_dot);
  }

 private:
  QuadrotorPlant<double>* plant_{};
};

// Case (#2): The Diagram that wraps a MultibodyPlant.
class MultibodyQuadrotor : public Diagram<double> {
 public:
  MultibodyQuadrotor() {
    auto owned_plant = std::make_unique<MultibodyPlant<double>>(0.0);
    plant_ = owned_plant.get();
    Parser(plant_).AddModelsFromUrl(
        "package://drake_models/skydio_2/quadrotor.urdf");
    plant_->Finalize();
    body_ = &plant_->GetBodyByName("base_link");
    DiagramBuilder<double> builder;
    builder.AddSystem(std::move(owned_plant));
    builder.BuildInto(this);
  }

  // @param[in,out] context a mutable Context to be set from the state vector x.
  // @param[in] x 12-element state vector [x, y, z, r, p, y, ẋ, ẏ, ż, ṙ, ṗ, ẏ].
  void SetState(Context<double>* context, const VectorXd& x) const {
    const Vector3d xyz = x.segment(0, 3);
    const Vector3d rpy = x.segment(3, 3);
    const Vector3d xyz_dot = x.segment(6, 3);
    const Vector3d rpy_dot = x.segment(9, 3);

    // Convert the state values into our conventional multibody types.
    const Vector3d p_WB(xyz);
    const RollPitchYawd R_WB(rpy);
    const Vector3d v_WB(xyz_dot);
    const Vector3d w_WB = R_WB.CalcAngularVelocityInParentFromRpyDt(rpy_dot);
    const RigidTransformd X_WB(R_WB, p_WB);
    const SpatialVelocity<double> V_WB(w_WB, v_WB);

    // Set the state of the base_link.
    Context<double>& plant_context =
        this->GetMutableSubsystemContext(*plant_, context);
    plant_->SetFloatingBaseBodyPoseInWorldFrame(&plant_context, *body_, X_WB);
    plant_->SetFreeBodySpatialVelocity(&plant_context, *body_, V_WB);
  }

  // Returns X_WB, the pose relating world W to the base-link body B.
  // @param[in] context contains the state of the multibody system.
  RigidTransformd GetPose(const Context<double>& context) const {
    const Context<double>& plant_context =
        this->GetSubsystemContext(*plant_, context);
    return plant_->EvalBodyPoseInWorld(plant_context, *body_);
  }

  // Returns V_WB_W, base-link B's spatial velocity measured and expressed in
  // the world frame W.
  // @param[in] context contains the state of the multibody system.
  SpatialVelocity<double> GetVelocity(const Context<double>& context) const {
    const Context<double>& plant_context =
        this->GetSubsystemContext(*plant_, context);
    const SpatialVelocity<double>& V_WB_W =
        plant_->EvalBodySpatialVelocityInWorld(plant_context, *body_);
    return V_WB_W;
  }

 private:
  MultibodyPlant<double>* plant_{};
  const RigidBody<double>* body_{};
};

void TestPassiveBehavior(const VectorXd& x0) {
  const GenericQuadrotor ge_model;
  const MultibodyQuadrotor mb_model;
  Simulator<double> ge_simulator(ge_model);
  Simulator<double> mb_simulator(mb_model);
  auto& ge_context = ge_simulator.get_mutable_context();
  auto& mb_context = mb_simulator.get_mutable_context();

  ge_model.SetState(&ge_context, x0);
  mb_model.SetState(&mb_context, x0);

  // Verify initial configuration (rotation matrix and position).
  const RigidTransformd ge_initial_pose = ge_model.GetPose(ge_context);
  const RigidTransformd mb_initial_pose = mb_model.GetPose(mb_context);
  EXPECT_TRUE(CompareMatrices(ge_initial_pose.translation(),
                              mb_initial_pose.translation(), 4 * kEpsilon,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(ge_initial_pose.rotation().matrix(),
                              mb_initial_pose.rotation().matrix(), 4 * kEpsilon,
                              MatrixCompareType::absolute));

  // Verify initial motion (translational and rotational velocities).
  const SpatialVelocity<double> ge_initial_velocity =
      ge_model.GetVelocity(ge_context);
  const SpatialVelocity<double> mb_initial_velocity =
      mb_model.GetVelocity(mb_context);
  EXPECT_TRUE(CompareMatrices(ge_initial_velocity.translational(),
                              mb_initial_velocity.translational(), 4 * kEpsilon,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(ge_initial_velocity.rotational(),
                              mb_initial_velocity.rotational(), 4 * kEpsilon,
                              MatrixCompareType::absolute));

  // Simulate and advance time.
  ge_simulator.AdvanceTo(kSimulationDuration);
  mb_simulator.AdvanceTo(kSimulationDuration);

  // Verify configuration after advancing time. Herein, the allowable error in
  // rotation is much higher than the allowable error in translation.
  const RigidTransformd ge_pose = ge_model.GetPose(ge_context);
  const RigidTransformd mb_pose = mb_model.GetPose(mb_context);
  EXPECT_TRUE(CompareMatrices(ge_pose.translation(), mb_pose.translation(),
                              1e-10, MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(ge_pose.rotation().matrix(),
                              mb_pose.rotation().matrix(), 1e-5,
                              MatrixCompareType::absolute));

  // Verify motion after advancing time. Herein, the allowable error in
  // rotation is much higher than the allowable error in translation.
  const SpatialVelocity<double> ge_velocity = ge_model.GetVelocity(ge_context);
  const SpatialVelocity<double> mb_velocity = mb_model.GetVelocity(mb_context);
  EXPECT_TRUE(CompareMatrices(ge_velocity.translational(),
                              mb_velocity.translational(), 1E-14,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(ge_velocity.rotational(),
                              mb_velocity.rotational(), 1E-4,
                              MatrixCompareType::absolute));
}

// Test comparing the state for of each kind of plant under passive behaviour.
// Each plant is dropped from rest at the origin.
GTEST_TEST(QuadrotorDynamicsTest, DropFromRest) {
  VectorXd x0 = VectorXd::Zero(12);
  TestPassiveBehavior(x0);
}

// Test comparing the state for of each kind of plant under passive behaviour.
// Each plant is dropped from the origin with an initial translational velocity.
GTEST_TEST(QuadrotorDynamicsTest, DropFromInitialVelocity) {
  VectorXd x0 = VectorXd::Zero(12);
  x0.segment(6, 3) << 1, 1, 1;  // Some translational velocity.
  TestPassiveBehavior(x0);
}

// Test comparing the state for of each kind of plant under passive behaviour.
// Each plant is dropped from the origin with an initial rotational velocity.
GTEST_TEST(QuadrotorDynamicsTest, DropFromInitialRotation) {
  VectorXd x0 = VectorXd::Zero(12);
  x0.segment(9, 3) << 1, 1, 1;  // Some rotary velocity.
  TestPassiveBehavior(x0);
}

// Test comparing the state for of each kind of plant under passive behaviour.
// Each plant is dropped from and arbitrary initial state.
GTEST_TEST(QuadrotorDynamicsTest, DropFromArbitraryState) {
  VectorXd x0 = VectorXd::Zero(12);
  x0 << 1, 2, 3, 4, 5, 6, 1, 2, 3, 4, 5, 6;  // Some initial state.
  TestPassiveBehavior(x0);
}

}  // namespace
}  // namespace quadrotor
}  // namespace examples
}  // namespace drake
