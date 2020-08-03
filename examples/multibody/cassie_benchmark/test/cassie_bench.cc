/*
Adapted for Drake from a Cassie benchmark by Michael Posa:
Copyright (c) 2020, Dynamic Autonomy and Intelligent Robotics Lab
BSD 3-clause license: https://github.com/DAIRLab/dairlib/blob/master/LICENSE
See https://github.com/DAIRLab/dairlib/issues/181 for the original benchmark.
*/

#include <benchmark/benchmark.h>
#include <gflags/gflags.h>

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

class DoubleFixture : public benchmark::Fixture {
 public:
  using benchmark::Fixture::SetUp;
  void SetUp(const benchmark::State&) override {
    plant_ = builder_.AddSystem<MultibodyPlant>(0);

    multibody::Parser parser(plant_);
    const auto& model =
        "drake/examples/multibody/cassie_benchmark/cassie_v2.urdf";
    parser.AddModelFromFile(FindResourceOrThrow(model));

    plant_->WeldFrames(plant_->world_frame(),
                       plant_->GetFrameByName("pelvis"));
    plant_->Finalize();

    nq_ = plant_->num_positions();
    nv_ = plant_->num_velocities();
    nu_ = plant_->num_actuators();

    x_ = VectorXd::Zero(nq_ + nv_);
    u_ = VectorXd::Zero(nu_);

    context_ = plant_->CreateDefaultContext();
  }

  systems::DiagramBuilder<double> builder_;
  MultibodyPlant<double>* plant_{};
  std::unique_ptr<Context<double>> context_;
  int nq_{};
  int nv_{};
  int nu_{};
  VectorXd x_{};
  VectorXd u_{};
};

// NOLINTNEXTLINE(runtime/references) cpplint disapproves of gbench choices.
BENCHMARK_F(DoubleFixture, DoubleMassMatrix)(benchmark::State& state) {
  MatrixXd M(nv_, nv_);
  int i = 0;
  for (auto _ : state) {
    x_(0) = i;
    plant_->SetPositionsAndVelocities(context_.get(), x_);
    plant_->CalcMassMatrix(*context_, &M);
    i++;
  }
}

// NOLINTNEXTLINE(runtime/references) cpplint disapproves of gbench choices.
BENCHMARK_F(DoubleFixture, DoubleInverseDynamics)(benchmark::State& state) {
  VectorXd desired_vdot;
  VectorXd x = VectorXd::Zero(nq_ + nv_);
  multibody::MultibodyForces<double> external_forces(*plant_);
  int i = 0;
  for (auto _ : state) {
    x = VectorXd::Constant(nq_ + nv_, i);
    desired_vdot = VectorXd::Constant(nv_, i);
    plant_->SetPositionsAndVelocities(context_.get(), x);
    plant_->CalcInverseDynamics(*context_, desired_vdot, external_forces);
    i++;
  }
}

// NOLINTNEXTLINE(runtime/references) cpplint disapproves of gbench choices.
BENCHMARK_F(DoubleFixture, DoubleForwardDynamics)(benchmark::State& state) {
  VectorXd x = VectorXd::Zero(nq_ + nv_);
  VectorXd u = VectorXd::Zero(nu_);
  auto derivatives = plant_->AllocateTimeDerivatives();
  int i = 0;
  for (auto _ : state) {
    x = VectorXd::Constant(nq_ + nv_, i);
    u = VectorXd::Constant(nu_, i);
    context_->FixInputPort(plant_->get_actuation_input_port().get_index(), u);
    plant_->SetPositionsAndVelocities(context_.get(), x);
    plant_->CalcTimeDerivatives(*context_, derivatives.get());
    i++;
  }
}

class AutodiffFixture : public DoubleFixture {
 public:
  using DoubleFixture::SetUp;
  void SetUp(const benchmark::State& state) override {
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
  int i = 0;
  for (auto _ : state) {
    // According to research from @sherm1, the caching system has some
    // debug-only asserts that allocate memory. Hence there are two different
    // ceilings for allocations until that is addressed.
#ifdef NDEBUG
    drake::test::LimitMalloc guard({.max_num_allocations = 53438});
#else
    drake::test::LimitMalloc guard({.max_num_allocations = 59897});
#endif
    x_(0) = i;
    plant_autodiff_->SetPositionsAndVelocities(
        context_autodiff_.get(), math::initializeAutoDiff(x_));
    plant_autodiff_->CalcMassMatrix(*context_autodiff_, &M_autodiff);
    i++;
  }
}

// NOLINTNEXTLINE(runtime/references) cpplint disapproves of gbench choices.
BENCHMARK_F(AutodiffFixture, AutodiffInverseDynamics)(benchmark::State& state) {
  VectorXd desired_vdot;
  VectorXd x = VectorXd::Zero(nq_ + nv_);
  multibody::MultibodyForces<AutoDiffXd> external_forces_autodiff(
      *plant_autodiff_);
  int i = 0;
  for (auto _ : state) {
    x = VectorXd::Constant(nq_ + nv_, i);
    desired_vdot = VectorXd::Constant(nv_, i);
    plant_autodiff_->SetPositionsAndVelocities(
        context_autodiff_.get(),
        math::initializeAutoDiff(x, nq_ + 2 * nv_));
    plant_autodiff_->CalcInverseDynamics(
        *context_autodiff_,
        math::initializeAutoDiff(desired_vdot, nq_ + 2 * nv_, nq_ + nv_),
        external_forces_autodiff);
    i++;
  }
}

// NOLINTNEXTLINE(runtime/references) cpplint disapproves of gbench choices.
BENCHMARK_F(AutodiffFixture, AutodiffForwardDynamics)(benchmark::State& state) {
  VectorXd x = VectorXd::Zero(nq_ + nv_);
  VectorXd u = VectorXd::Zero(nu_);
  auto derivatives_autodiff = plant_autodiff_->AllocateTimeDerivatives();
  int i = 0;
  for (auto _ : state) {
    i++;
    x = VectorXd::Constant(nq_ + nv_, i);
    u = VectorXd::Constant(nu_, i);

    context_autodiff_->FixInputPort(
        plant_autodiff_->get_actuation_input_port().get_index(),
        math::initializeAutoDiff(u, nq_ + nv_ + nu_, nq_ + nv_));
    plant_autodiff_->SetPositionsAndVelocities(
        context_autodiff_.get(),
        math::initializeAutoDiff(x, nq_ + nv_ + nu_));
    plant_autodiff_->CalcTimeDerivatives(*context_autodiff_,
                                         derivatives_autodiff.get());
  }
}

}  // namespace
}  // namespace examples
}  // namespace drake

BENCHMARK_MAIN();
