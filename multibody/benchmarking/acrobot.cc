// @file
// Benchmarks for Acrobot autodiff, with and without MultibodyPlant.
//
// This program is a successor to Hongkai Dai's original benchmark; see #8482.

#include <benchmark/benchmark.h>

#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/common/pointer_cast.h"
#include "drake/examples/acrobot/acrobot_plant.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/benchmarks/acrobot/make_acrobot_plant.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/tools/performance/fixture_common.h"

using Eigen::MatrixXd;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;
using drake::systems::System;

namespace drake {
namespace examples {
namespace acrobot {
namespace {

template <typename T>
class FixtureBase : public benchmark::Fixture {
 public:
  FixtureBase() {
    tools::performance::AddMinMaxStatistics(this);
  }

  void Populate(const System<T>& plant) {
    context_ = plant.CreateDefaultContext();
    x_ = context_->get_continuous_state_vector().CopyToVector();
  }

  void InvalidateState() {
    context_->NoteContinuousStateChange();
  }

 protected:
  std::unique_ptr<Context<T>> context_;
  VectorX<T> x_{};
};

template <typename T>
class AcrobotFixture : public FixtureBase<T> {
 public:
  using benchmark::Fixture::SetUp;
  void SetUp(benchmark::State&) override {
    plant_ = std::make_unique<AcrobotPlant<T>>();
    this->Populate(*plant_);
  }

 protected:
  std::unique_ptr<AcrobotPlant<T>> plant_{};
};

using AcrobotFixtureD = AcrobotFixture<double>;
using AcrobotFixtureAdx = AcrobotFixture<AutoDiffXd>;

// NOLINTNEXTLINE(runtime/references) cpplint disapproves of gbench choices.
BENCHMARK_F(AcrobotFixtureD, AcrobotDMassMatrix)(benchmark::State& state) {
  for (auto _ : state) {
    InvalidateState();
    plant_->MassMatrix(*context_);
  }
}

// NOLINTNEXTLINE(runtime/references) cpplint disapproves of gbench choices.
BENCHMARK_F(AcrobotFixtureAdx, AcrobotAdxMassMatrix)(benchmark::State& state) {
  for (auto _ : state) {
    InvalidateState();
    plant_->MassMatrix(*context_);
  }
}

template <typename T>
class MultibodyFixture : public FixtureBase<T> {
 public:
  using benchmark::Fixture::SetUp;
  void SetUp(benchmark::State&) override {
    auto double_plant = multibody::benchmarks::acrobot::MakeAcrobotPlant(
            multibody::benchmarks::acrobot::AcrobotParameters(), true);
    if constexpr (std::is_same_v<T, double>) {
      plant_ = std::move(double_plant);
    } else {
      plant_ = dynamic_pointer_cast<MultibodyPlant<AutoDiffXd>>(
          double_plant->ToAutoDiffXd());
    }
    nv_ = plant_->num_velocities();
    this->Populate(*plant_);
  }

 protected:
  std::unique_ptr<MultibodyPlant<T>> plant_{};
  int nv_{};
};

using MultibodyFixtureD = MultibodyFixture<double>;
using MultibodyFixtureAdx = MultibodyFixture<AutoDiffXd>;

// NOLINTNEXTLINE(runtime/references) cpplint disapproves of gbench choices.
BENCHMARK_F(MultibodyFixtureD, MultibodyDMassMatrix)(benchmark::State& state) {
  MatrixXd M(nv_, nv_);
  for (auto _ : state) {
    InvalidateState();
    plant_->CalcMassMatrix(*context_, &M);
  }
}

BENCHMARK_F(MultibodyFixtureAdx, MultibodyAdxMassMatrix)
    // NOLINTNEXTLINE(runtime/references) cpplint disapproves of gbench choices.
    (benchmark::State& state) {
  MatrixX<AutoDiffXd> M(nv_, nv_);
  for (auto _ : state) {
    InvalidateState();
    plant_->CalcMassMatrix(*context_, &M);
  }
}

BENCHMARK_F(MultibodyFixtureD, MultibodyDMassMatrixViaInverseDynamics)
    // NOLINTNEXTLINE(runtime/references) cpplint disapproves of gbench choices.
    (benchmark::State& state) {
  MatrixXd M(nv_, nv_);
  for (auto _ : state) {
    InvalidateState();
    plant_->CalcMassMatrixViaInverseDynamics(*context_, &M);
  }
}

BENCHMARK_F(MultibodyFixtureAdx, MultibodyAdxMassMatrixViaInverseDynamics)
    // NOLINTNEXTLINE(runtime/references) cpplint disapproves of gbench choices.
    (benchmark::State& state) {
  MatrixX<AutoDiffXd> M(nv_, nv_);
  for (auto _ : state) {
    InvalidateState();
    plant_->CalcMassMatrixViaInverseDynamics(*context_, &M);
  }
}

}  // namespace
}  // namespace acrobot
}  // namespace examples
}  // namespace drake

BENCHMARK_MAIN();
