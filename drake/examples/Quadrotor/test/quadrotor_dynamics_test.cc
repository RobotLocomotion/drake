#include <stdexcept>

#include <gtest/gtest.h>

#include "drake/common/drake_path.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/examples/Quadrotor/quadrotor_plant.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"

// The following sequence of tests compares the behaviour of two kinds of
// plants : (i) A GenericQuadrotor created from the QuadrotorPlant, and
// (ii) RigidBodyPlant created from parsing a corresponding model URDF file.
namespace drake {
namespace examples {
namespace quadrotor {
namespace {

// The models are integrated and compared for the duration specified by the
// following constant.
const double kSimulationDuration = 0.1;

// A Generic Quadrotor Plant Diagram with the plant created from
// QuadrotorPlant.
template<typename T>
class GenericQuadrotor: public systems::Diagram<T> {
 public:
  GenericQuadrotor() {
    this->set_name("QuadrotorTest");

    systems::DiagramBuilder<T> builder;

    plant_ = builder.template AddSystem<QuadrotorPlant<T>>();

    VectorX<T> hover_input(plant_->get_input_size());
    hover_input.setZero();
    systems::ConstantVectorSource<T> *source =
        builder.template AddSystem<systems::ConstantVectorSource<T>>(
            hover_input);

    builder.Connect(source->get_output_port(), plant_->get_input_port(0));

    builder.BuildInto(this);
  }

  void SetState(systems::Context<T> *context, VectorX<T> x) const {
    systems::Context<T> *plant_context =
        this->GetMutableSubsystemContext(context, plant_);
    plant_->set_state(plant_context, x);
  }

 private:
  QuadrotorPlant<T> *plant_{};
};

//  A Quadrotor as a RigidBodyPlant that is created from a model
// specified in a URDF file.
template<typename T>
class RigidBodyQuadrotor: public systems::Diagram<T> {
 public:
  RigidBodyQuadrotor() {
    this->set_name("Quadrotor");

    auto tree = std::make_unique<RigidBodyTree<T>>();

    drake::parsers::urdf::AddModelInstanceFromUrdfFile(
        drake::GetDrakePath() + "/examples/Quadrotor/quadrotor.urdf",
        multibody::joints::kRollPitchYaw, nullptr, tree.get());

    systems::DiagramBuilder<T> builder;

    plant_ =
        builder.template AddSystem<systems::RigidBodyPlant<T>>(std::move(tree));

    builder.BuildInto(this);
  }

  void SetState(systems::Context<T> *context, VectorX<T> x) const {
    systems::Context<T> *plant_context =
        this->GetMutableSubsystemContext(context, plant_);
    plant_->set_state_vector(plant_context, x);
  }

 private:
  systems::RigidBodyPlant<T> *plant_{};
};

//  Combines test setup for both kinds of plants:
//  ge_model_:  GenericQuadrotor
//  rb_model_:  RigidBodyPlant
class QuadrotorTest: public ::testing::Test {
 public:
  QuadrotorTest() {
    ge_model_ = std::make_unique<GenericQuadrotor<double>>();
    rb_model_ = std::make_unique<RigidBodyQuadrotor<double>>();

    ge_simulator_ = std::make_unique<systems::Simulator<double>>(*ge_model_);
    rb_simulator_ = std::make_unique<systems::Simulator<double>>(*rb_model_);

    ge_derivatives_ = ge_model_->AllocateTimeDerivatives();
    rb_derivatives_ = rb_model_->AllocateTimeDerivatives();
  }

  void SetUp() override {
    ge_context_ = ge_model_->CreateDefaultContext();
    rb_context_ = rb_model_->CreateDefaultContext();

    ge_model_->SetState(ge_context_.get(), x0_);
    rb_model_->SetState(rb_context_.get(), x0_);

    ge_simulator_->Initialize();
    rb_simulator_->Initialize();
  }

  void SetState(const VectorX<double> x0) {
    ge_context_ = ge_model_->CreateDefaultContext();
    rb_context_ = rb_model_->CreateDefaultContext();

    ge_model_->SetState(ge_context_.get(), x0);
    rb_model_->SetState(rb_context_.get(), x0);
  }

  void Simulate(const double t) {
    ge_simulator_->Initialize();
    rb_simulator_->Initialize();

    ge_simulator_->StepTo(t);
    rb_simulator_->StepTo(t);
  }

  VectorX<double> GetState(systems::Simulator<double> *simulator) {
    return simulator->get_context()
        .get_continuous_state_vector()
        .CopyToVector();
  }

  void PassiveBehaviorTest(VectorX<double> x0) {
    SetState(x0);
    Simulate(kSimulationDuration);
    VectorX<double> my_state = ge_simulator_->get_context()
        .get_continuous_state_vector()
        .CopyToVector();
    VectorX<double> rb_state = rb_simulator_->get_context()
        .get_continuous_state_vector()
        .CopyToVector();
    double tol = 1e-10;
    EXPECT_TRUE(
        CompareMatrices(my_state, rb_state, tol, MatrixCompareType::absolute));
  }

 protected:
  VectorX<double> x0_ = VectorX<double>::Zero(12);

  std::unique_ptr<GenericQuadrotor<double>> ge_model_;
  std::unique_ptr<RigidBodyQuadrotor<double>> rb_model_;

  std::unique_ptr<systems::Simulator<double>> ge_simulator_, rb_simulator_;

  std::unique_ptr<systems::Context<double>> ge_context_, rb_context_;
  std::unique_ptr<systems::ContinuousState<double>> ge_derivatives_,
      rb_derivatives_;
};

//  Test comparing the computation of derivatives for a fixed state.
TEST_F(QuadrotorTest, derivatives) {
  VectorX<double> x0 = VectorX<double>::Ones(12);  // Set state to ones.
  SetState(x0);

  ge_model_->CalcTimeDerivatives(*ge_context_, ge_derivatives_.get());
  rb_model_->CalcTimeDerivatives(*rb_context_, rb_derivatives_.get());

  VectorX<double> my_derivative_vector = ge_derivatives_->CopyToVector();
  VectorX<double> rb_derivative_vector = rb_derivatives_->CopyToVector();

  EXPECT_TRUE(CompareMatrices(my_derivative_vector, rb_derivative_vector,
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

}  // namespace
}  // namespace quadrotor
}  // namespace examples
}  // namespace drake
