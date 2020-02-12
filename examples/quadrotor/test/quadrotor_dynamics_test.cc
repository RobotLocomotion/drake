#include <stdexcept>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/examples/quadrotor/quadrotor_plant.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/test_utilities/scalar_conversion.h"
#include "drake/systems/primitives/constant_vector_source.h"

// The following sequence of tests compares the behaviour of two kinds of
// plants : (i) A GenericQuadrotor created from the QuadrotorPlant, and
// (ii) MultibodyPlant created from parsing a corresponding model URDF file.
namespace drake {
namespace examples {
namespace quadrotor {
namespace {

using Eigen::Quaterniond;
using math::RollPitchYawd;
using multibody::MultibodyPlant;

GTEST_TEST(QuadrotorPlantTest, DirectFeedthrough) {
  QuadrotorPlant<double> quadrotor;
  EXPECT_FALSE(quadrotor.HasAnyDirectFeedthrough());
}

// The models are integrated and compared for the duration specified by the
// following constant.
const double kSimulationDuration = 0.1;

// A Generic Quadrotor Plant Diagram with the plant created from
// QuadrotorPlant.
template<typename T>
class GenericQuadrotor: public systems::Diagram<T> {
 public:
  GenericQuadrotor() {
    systems::DiagramBuilder<T> builder;

    plant_ = builder.template AddSystem<QuadrotorPlant<T>>();

    VectorX<T> hover_input(plant_->num_total_inputs());
    hover_input.setZero();
    systems::ConstantVectorSource<T>* source =
        builder.template AddSystem<systems::ConstantVectorSource<T>>(
            hover_input);

    builder.Connect(source->get_output_port(), plant_->get_input_port(0));

    builder.BuildInto(this);
  }

  // Set state given [x, y, z, r, p, y, xdot, ydot, zdot, rdot, pdot, ydot].
  void SetState(systems::Context<T>* context, VectorX<T> x) const {
    systems::Context<T>& plant_context =
        this->GetMutableSubsystemContext(*plant_, context);
    plant_context.SetContinuousState(x);
  }

 private:
  QuadrotorPlant<T>* plant_{};
};

//  A Quadrotor as a MultibodyPlant that is created from a model
// specified in a URDF file.
template<typename T>
class MultibodyQuadrotor: public systems::Diagram<T> {
 public:
  MultibodyQuadrotor() {
    auto owned_plant = std::make_unique<MultibodyPlant<T>>(0.0);
    plant_ = owned_plant.get();

    multibody::Parser(plant_).AddModelFromFile(FindResourceOrThrow(
        "drake/examples/quadrotor/quadrotor.urdf"));
    plant_->Finalize();

    systems::DiagramBuilder<T> builder;
    builder.AddSystem(std::move(owned_plant));
    builder.BuildInto(this);
  }

  // Set state given [x, y, z, r, p, y, xdot, ydot, zdot, rdot, pdot, ydot].
  void SetState(systems::Context<T>* context, VectorX<T> x) const {
    // Unpack the unit test's nominal order.
    systems::Context<T>& plant_context =
        this->GetMutableSubsystemContext(*plant_, context);
    DRAKE_DEMAND(x.size() == 12);
    const VectorX<double> q_xyz = x.segment(0, 3);
    const VectorX<double> q_rpy = x.segment(3, 3);
    const VectorX<double> v_xyz = x.segment(6, 3);
    const VectorX<double> v_rpy = x.segment(9, 3);
    const auto quat = RollPitchYawd(x.segment(3, 3)).ToQuaternion();

    // Pack into QuaternionFloatingMobilizer's state vector.
    VectorX<double> qv(13);
    qv(0) = quat.w();
    qv.segment(1, 3) = quat.vec();
    qv.segment(4, 3) = q_xyz;
    qv.segment(7, 3) = v_rpy;
    qv.segment(10, 3) = v_xyz;
    plant_->SetPositionsAndVelocities(&plant_context, qv);
  }

 private:
  MultibodyPlant<T>* plant_{};
};

//  Combines test setup for both kinds of plants:
//  ge_model_:  GenericQuadrotor
//  mb_model_:  MultibodyPlant
class QuadrotorTest: public ::testing::Test {
 public:
  QuadrotorTest() {
    ge_model_ = std::make_unique<GenericQuadrotor<double>>();
    mb_model_ = std::make_unique<MultibodyQuadrotor<double>>();

    ge_simulator_ = std::make_unique<systems::Simulator<double>>(*ge_model_);
    mb_simulator_ = std::make_unique<systems::Simulator<double>>(*mb_model_);

    ge_derivatives_ = ge_model_->AllocateTimeDerivatives();
    mb_derivatives_ = mb_model_->AllocateTimeDerivatives();
  }

  void SetUp() override {
    ge_context_ = ge_model_->CreateDefaultContext();
    mb_context_ = mb_model_->CreateDefaultContext();

    ge_model_->SetState(ge_context_.get(), x0_);
    mb_model_->SetState(mb_context_.get(), x0_);

    ge_simulator_->Initialize();
    mb_simulator_->Initialize();
  }

  void SetState(const VectorX<double> x0) {
    ge_context_ = ge_model_->CreateDefaultContext();
    mb_context_ = mb_model_->CreateDefaultContext();

    ge_model_->SetState(ge_context_.get(), x0);
    mb_model_->SetState(mb_context_.get(), x0);
  }

  // Report state as [x, y, z, r, p, y, xdot, ydot, zdot, rdot, pdot, ydot],
  // given the GenericQuadrotor's state.
  VectorX<double> CopyGeState(const systems::ContinuousState<double>& x) {
    const VectorX<double> result = x.get_vector().CopyToVector();
    DRAKE_DEMAND(result.size() == 12);
    return result;
  }

  // Report state as [x, y, z, r, p, y, xdot, ydot, zdot, rdot, pdot, ydot],
  // given the MultibodyQuadrotor's state.
  VectorX<double> CopyMbState(const systems::ContinuousState<double>& x) {
    // Unpack QuaternionFloatingMobilizer's state vector.
    const VectorX<double> x_vec = x.get_vector().CopyToVector();
    DRAKE_DEMAND(x_vec.size() == 13);
    const VectorX<double> q_wxyz = x_vec.segment(0, 4);
    const VectorX<double> q_xyz = x_vec.segment(4, 3);
    const VectorX<double> v_rpy = x_vec.segment(7, 3);
    const VectorX<double> v_xyz = x_vec.segment(10, 3);
    const Quaterniond q_quat(q_wxyz(0), q_wxyz(1), q_wxyz(2), q_wxyz(3));

    // Pack it back into the unit test's expected order.
    VectorX<double> result(12);
    result.segment(0, 3) = q_xyz;
    result.segment(3, 3) = RollPitchYawd(q_quat).vector();
    result.segment(6, 3) = v_xyz;
    result.segment(9, 3) = v_rpy;
    return result;
  }

  void Simulate(const double t) {
    ge_simulator_->Initialize();
    mb_simulator_->Initialize();

    ge_simulator_->AdvanceTo(t);
    mb_simulator_->AdvanceTo(t);
  }

  void PassiveBehaviorTest(VectorX<double> x0) {
    SetState(x0);
    Simulate(kSimulationDuration);
    VectorX<double> ge_state = CopyGeState(
        ge_simulator_->get_context().get_continuous_state());
    VectorX<double> mb_state = CopyMbState(
        mb_simulator_->get_context().get_continuous_state());
    double tol = 1e-10;
    EXPECT_TRUE(
        CompareMatrices(ge_state, mb_state, tol, MatrixCompareType::absolute));
  }

 protected:
  VectorX<double> x0_ = VectorX<double>::Zero(12);

  std::unique_ptr<GenericQuadrotor<double>> ge_model_;
  std::unique_ptr<MultibodyQuadrotor<double>> mb_model_;

  std::unique_ptr<systems::Simulator<double>> ge_simulator_, mb_simulator_;

  std::unique_ptr<systems::Context<double>> ge_context_, mb_context_;
  std::unique_ptr<systems::ContinuousState<double>> ge_derivatives_,
      mb_derivatives_;
};

//  Test comparing the computation of derivatives for a fixed state.
TEST_F(QuadrotorTest, derivatives) {
  VectorX<double> x0 = VectorX<double>::Ones(12);  // Set state to ones.
  SetState(x0);

  ge_model_->CalcTimeDerivatives(*ge_context_, ge_derivatives_.get());
  mb_model_->CalcTimeDerivatives(*mb_context_, mb_derivatives_.get());

  VectorX<double> ge_derivative_vector = CopyGeState(*ge_derivatives_);
  VectorX<double> mb_derivative_vector = CopyMbState(*mb_derivatives_);

  EXPECT_TRUE(CompareMatrices(ge_derivative_vector, mb_derivative_vector,
                              1e-10 /* tolerance */,
                              MatrixCompareType::absolute));
}

// Test comparing the state for of each kind of plant under passive behaviour
// after a 1.0 second motion. Each plant is dropped from rest at the origin.
TEST_F(QuadrotorTest, drop_from_rest) {
  VectorX<double> x0 = VectorX<double>::Zero(12);
  PassiveBehaviorTest(x0);
}

// Test comparing the state for of each kind of plant under passive behaviour
// after a 1.0 second motion. Each plant is dropped from the origin with an
// initial translational velocity.
TEST_F(QuadrotorTest, drop_from_initial_velocity) {
  VectorX<double> x0 = VectorX<double>::Zero(12);
  x0.segment(6, 3) << 1, 1, 1;  // Some translational velocity.
  PassiveBehaviorTest(x0);
}

// Test comparing the state for of each kind of plant under passive behaviour
// after a 1.0 second motion. Each plant is dropped from the origin with an
// initial rotational velocity.
TEST_F(QuadrotorTest, drop_from_initial_rotation) {
  VectorX<double> x0 = VectorX<double>::Zero(12);
  x0.segment(9, 3) << 1, 1, 1;  // Some rotary velocity.
  PassiveBehaviorTest(x0);
}

// Test comparing the state for of each kind of plant under passive behaviour
// after a 1.0 second motion. Each plant is dropped from and arbitrary initial
// state.
TEST_F(QuadrotorTest, drop_from_arbitrary_state) {
  VectorX<double> x0 = VectorX<double>::Zero(12);
  x0 << 1, 2, 3, 4, 5, 6, 1, 2, 3, 4, 5, 6;  // Some initial state.
  PassiveBehaviorTest(x0);
}

TEST_F(QuadrotorTest, ToAutoDiff) {
  const QuadrotorPlant<double> plant;
  EXPECT_TRUE(is_autodiffxd_convertible(plant));
}

TEST_F(QuadrotorTest, ToSymbolic) {
  const QuadrotorPlant<double> plant;
  EXPECT_FALSE(is_symbolic_convertible(plant));
}

}  // namespace
}  // namespace quadrotor
}  // namespace examples
}  // namespace drake
