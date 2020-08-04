/*
Adapted for Drake from a Cassie benchmark by Michael Posa:
Copyright (c) 2020, Dynamic Autonomy and Intelligent Robotics Lab
BSD 3-clause license: https://github.com/DAIRLab/dairlib/blob/master/LICENSE
See https://github.com/DAIRLab/dairlib/issues/181 for the original benchmark.
*/

#include <benchmark/benchmark.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/limit_malloc.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"

using drake::multibody::MultibodyPlant;
using drake::systems::Context;

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace drake {
namespace examples {
namespace {

// According to research from @sherm1, the caching system has some
// debug-only asserts that allocate memory. Hence there are two different
// ceilings for allocations until that is addressed.
#ifdef NDEBUG
#define RELEASE_LIMIT_MALLOC(x) \
  drake::test::LimitMalloc guard({.max_num_allocations = (x)});
#else
#define RELEASE_LIMIT_MALLOC(x) /* empty */
#endif

class DoubleFixture : public benchmark::Fixture {
 public:
  // This apparently futile using statement works around "overloaded virtual"
  // errors in g++. All of this is a consequence of the weird deprecation of
  // const-ref State versions of SetUp() and TearDown() in benchmark.h.
  using benchmark::Fixture::SetUp;
  void SetUp(benchmark::State&) override {
    plant_ = builder_.AddSystem<MultibodyPlant>(0);

    multibody::Parser parser(plant_);
    const auto& model =
        "drake/examples/multibody/cassie_benchmark/cassie_v2.urdf";
    parser.AddModelFromFile(FindResourceOrThrow(model));
    plant_->Finalize();

    nq_ = plant_->num_positions();
    nv_ = plant_->num_velocities();
    nu_ = plant_->num_actuators();

    context_ = plant_->CreateDefaultContext();

    u_ = VectorXd::Zero(nu_);

    // Use default state to avoid problems with all-zero quaternions.
    x_ = context_->get_continuous_state_vector().CopyToVector();
  }

  systems::DiagramBuilder<double> builder_;
  MultibodyPlant<double>* plant_{};
  std::unique_ptr<Context<double>> context_;
  int nq_{};
  int nv_{};
  int nu_{};
  VectorXd u_{};
  VectorXd x_{};
};

// NOLINTNEXTLINE(runtime/references) cpplint disapproves of gbench choices.
BENCHMARK_F(DoubleFixture, DoubleMassMatrix)(benchmark::State& state) {
  MatrixXd M(nv_, nv_);
  for (auto _ : state) {
    RELEASE_LIMIT_MALLOC(175);
    context_->NoteContinuousStateChange();
    plant_->SetPositionsAndVelocities(context_.get(), x_);
    plant_->CalcMassMatrix(*context_, &M);
  }
}

// NOLINTNEXTLINE(runtime/references) cpplint disapproves of gbench choices.
BENCHMARK_F(DoubleFixture, DoubleInverseDynamics)(benchmark::State& state) {
  VectorXd desired_vdot = VectorXd::Zero(nv_);
  multibody::MultibodyForces<double> external_forces(*plant_);
  for (auto _ : state) {
    RELEASE_LIMIT_MALLOC(3);
    context_->NoteContinuousStateChange();
    plant_->SetPositionsAndVelocities(context_.get(), x_);
    plant_->CalcInverseDynamics(*context_, desired_vdot, external_forces);
  }
}

// NOLINTNEXTLINE(runtime/references) cpplint disapproves of gbench choices.
BENCHMARK_F(DoubleFixture, DoubleForwardDynamics)(benchmark::State& state) {
  auto derivatives = plant_->AllocateTimeDerivatives();
  auto& port_value =
      plant_->get_actuation_input_port().FixValue(context_.get(), u_);
  for (auto _ : state) {
    RELEASE_LIMIT_MALLOC(22);
    context_->NoteContinuousStateChange();
    port_value.GetMutableData();
    plant_->SetPositionsAndVelocities(context_.get(), x_);
    plant_->CalcTimeDerivatives(*context_, derivatives.get());
  }
}

class AutodiffFixture : public DoubleFixture {
 public:
  using DoubleFixture::SetUp;
  void SetUp(benchmark::State& state) override {
    DoubleFixture::SetUp(state);
    plant_autodiff_ = systems::System<double>::ToAutoDiffXd(*plant_);
    context_autodiff_ = plant_autodiff_->CreateDefaultContext();
  }

  std::unique_ptr<MultibodyPlant<AutoDiffXd>> plant_autodiff_;
  std::unique_ptr<Context<AutoDiffXd>> context_autodiff_;
};

// NOLINTNEXTLINE(runtime/references) cpplint disapproves of gbench choices.
BENCHMARK_F(AutodiffFixture, AutodiffMassMatrix)(benchmark::State& state) {
  MatrixX<AutoDiffXd> M_autodiff(nv_, nv_);
  auto x_autodiff = math::initializeAutoDiff(x_);
  for (auto _ : state) {
    RELEASE_LIMIT_MALLOC(62476);
    context_autodiff_->NoteContinuousStateChange();
    plant_autodiff_->SetPositionsAndVelocities(context_autodiff_.get(),
                                               x_autodiff);
    plant_autodiff_->CalcMassMatrix(*context_autodiff_, &M_autodiff);
  }
}

// NOLINTNEXTLINE(runtime/references) cpplint disapproves of gbench choices.
BENCHMARK_F(AutodiffFixture, AutodiffInverseDynamics)(benchmark::State& state) {
  VectorXd desired_vdot = VectorXd::Zero(nv_);
  multibody::MultibodyForces<AutoDiffXd> external_forces_autodiff(
      *plant_autodiff_);
  auto x_autodiff = math::initializeAutoDiff(x_, nq_ + 2 * nv_);
  auto vdot_autodiff =
      math::initializeAutoDiff(desired_vdot, nq_ + 2 * nv_, nq_ + nv_);
  for (auto _ : state) {
    RELEASE_LIMIT_MALLOC(70301);
    context_autodiff_->NoteContinuousStateChange();
    plant_autodiff_->SetPositionsAndVelocities(context_autodiff_.get(),
                                               x_autodiff);
    plant_autodiff_->CalcInverseDynamics(*context_autodiff_,
                                         vdot_autodiff,
                                         external_forces_autodiff);
  }
}

// NOLINTNEXTLINE(runtime/references) cpplint disapproves of gbench choices.
BENCHMARK_F(AutodiffFixture, AutodiffForwardDynamics)(benchmark::State& state) {
  auto derivatives_autodiff = plant_autodiff_->AllocateTimeDerivatives();
  auto u_autodiff = math::initializeAutoDiff(u_, nq_ + nv_ + nu_, nq_ + nv_);
  auto& port_value = plant_autodiff_->get_actuation_input_port().FixValue(
      context_autodiff_.get(), u_autodiff);
  auto x_autodiff = math::initializeAutoDiff(x_, nq_ + nv_ + nu_);
  for (auto _ : state) {
    RELEASE_LIMIT_MALLOC(105675);
    context_autodiff_->NoteContinuousStateChange();
    port_value.GetMutableData();
    plant_autodiff_->SetPositionsAndVelocities(context_autodiff_.get(),
                                               x_autodiff);
    plant_autodiff_->CalcTimeDerivatives(*context_autodiff_,
                                         derivatives_autodiff.get());
  }
}

#undef RELEASE_LIMIT_MALLOC

}  // namespace
}  // namespace examples
}  // namespace drake

BENCHMARK_MAIN();
