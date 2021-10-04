#include <benchmark/benchmark.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/limit_malloc.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/tools/performance/fixture_common.h"

using drake::multibody::MultibodyPlant;
using drake::symbolic::Expression;
using drake::systems::Context;
using drake::test::LimitMalloc;

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace drake {
namespace examples {
namespace {

// @note LimitMalloc: This program uses LimitMalloc to indicate the count of
// malloc calls measured at the time the benchmark cases were written. At best,
// they are empirical observations. If there is a good reason to exceed these
// limits, maintainers should not hesitate to change them. If they are exceeded
// without a good reason, maintainers should revisit their changes to see why
// heap usage has increased.

// TODO(sherm1) Remove this if AutoDiffXd heap usage can be made the same
//   in Release and Debug builds (higher in Debug currently).
// Use this to suppress heap limiting in Debug builds.
drake::test::LimitMallocParams LimitReleaseOnly(int max_num_allocations) {
  if (kDrakeAssertIsArmed) { return {}; }
  return { .max_num_allocations = max_num_allocations };
}

// Track and report simple streaming statistics on allocations. Variance
// tracking is adapted from:
// https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance#Welford's_online_algorithm
class AllocationTracker {
 public:
  AllocationTracker() {}

  void Report(benchmark::State* state) {
    state->counters["Allocs.min"] = min_;
    state->counters["Allocs.max"] = max_;
    state->counters["Allocs.mean"] = mean_;
    state->counters["Allocs.stddev"] =
        updates_ < 2 ? std::numeric_limits<double>::quiet_NaN()
                     : std::sqrt(m2_ / (updates_ - 1));
  }

  void Update(int allocs) {
    min_ = std::min(min_, allocs);
    max_ = std::max(max_, allocs);
    ++updates_;
    double delta = allocs - mean_;
    mean_ += delta / updates_;
    m2_ += delta * (allocs - mean_);
  }

 private:
  int min_{std::numeric_limits<int>::max()};
  int max_{std::numeric_limits<int>::min()};
  int updates_{};
  double mean_{};
  double m2_{};
};

// Fixture that holds a Cassie robot model in a MultibodyPlant<double>. The
// class also holds a default context for the plant, and dimensions of its
// state and inputs.
class CassieDoubleFixture : public benchmark::Fixture {
 public:
  CassieDoubleFixture() {
    tools::performance::AddMinMaxStatistics(this);
  }

  // This apparently futile using statement works around "overloaded virtual"
  // errors in g++. All of this is a consequence of the weird deprecation of
  // const-ref State versions of SetUp() and TearDown() in benchmark.h.
  using benchmark::Fixture::SetUp;
  void SetUp(benchmark::State&) override {
    plant_ = std::make_unique<MultibodyPlant<double>>(0);

    multibody::Parser parser(plant_.get());
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

  // Use this method to invalidate state-dependent computations within each
  // benchmarked step. Disabling the cache entirely could affect the performance
  // differently because it would suppress any internal use of the cache during
  // complicated computations like forward dynamics. For example, if there are
  // multiple places in forward dynamics that access body positions, currently
  // those would get computed once and re-used (like in real applications) but
  // with caching off they would get recalculated repeatedly, affecting the
  // timing results.
  virtual void InvalidateState() {
    context_->NoteContinuousStateChange();
  }

 protected:
  AllocationTracker tracker_;
  std::unique_ptr<MultibodyPlant<double>> plant_{};
  std::unique_ptr<Context<double>> context_;
  int nq_{};
  int nv_{};
  int nu_{};
  VectorXd u_{};
  VectorXd x_{};
};

// NOLINTNEXTLINE(runtime/references) cpplint disapproves of gbench choices.
BENCHMARK_F(CassieDoubleFixture, DoubleMassMatrix)(benchmark::State& state) {
  MatrixXd M(nv_, nv_);
  for (auto _ : state) {
    // @see LimitMalloc note above.
    LimitMalloc guard({.max_num_allocations = 0});
    InvalidateState();
    plant_->CalcMassMatrix(*context_, &M);
    tracker_.Update(guard.num_allocations());
  }
  tracker_.Report(&state);
}

BENCHMARK_F(CassieDoubleFixture, DoubleInverseDynamics)
    // NOLINTNEXTLINE(runtime/references) cpplint disapproves of gbench choices.
    (benchmark::State& state) {
  VectorXd desired_vdot = VectorXd::Zero(nv_);
  multibody::MultibodyForces<double> external_forces(*plant_);
  for (auto _ : state) {
    // @see LimitMalloc note above.
    LimitMalloc guard({.max_num_allocations = 3});
    InvalidateState();
    plant_->CalcInverseDynamics(*context_, desired_vdot, external_forces);
    tracker_.Update(guard.num_allocations());
  }
  tracker_.Report(&state);
}

BENCHMARK_F(CassieDoubleFixture, DoubleForwardDynamics)
    // NOLINTNEXTLINE(runtime/references) cpplint disapproves of gbench choices.
    (benchmark::State& state) {
  auto derivatives = plant_->AllocateTimeDerivatives();
  auto& port_value =
      plant_->get_actuation_input_port().FixValue(context_.get(), u_);
  for (auto _ : state) {
    // @see LimitMalloc note above.
    LimitMalloc guard({.max_num_allocations = 22});
    InvalidateState();
    port_value.GetMutableData();  // Invalidates caching of inputs.
    plant_->CalcTimeDerivatives(*context_, derivatives.get());
    tracker_.Update(guard.num_allocations());
  }
  tracker_.Report(&state);
}

// Fixture that holds a Cassie robot model in a MultibodyPlant<AutoDiffXd>. It
// also holds a default context for autodiff.
class CassieAutodiffFixture : public CassieDoubleFixture {
 public:
  using CassieDoubleFixture::SetUp;
  void SetUp(benchmark::State& state) override {
    CassieDoubleFixture::SetUp(state);
    plant_autodiff_ = systems::System<double>::ToAutoDiffXd(*plant_);
    context_autodiff_ = plant_autodiff_->CreateDefaultContext();
  }

  // @see CassieDoubleFixture::InvalidateState(). This method invalidates the
  // autodiff version of state.
  void InvalidateState() override {
    context_autodiff_->NoteContinuousStateChange();
  }

 protected:
  std::unique_ptr<MultibodyPlant<AutoDiffXd>> plant_autodiff_;
  std::unique_ptr<Context<AutoDiffXd>> context_autodiff_;
};

BENCHMARK_F(CassieAutodiffFixture, AutodiffMassMatrix)
    // NOLINTNEXTLINE(runtime/references) cpplint disapproves of gbench choices.
    (benchmark::State& state) {
  MatrixX<AutoDiffXd> M_autodiff(nv_, nv_);
  auto x_autodiff = math::InitializeAutoDiff(x_);
  plant_autodiff_->SetPositionsAndVelocities(context_autodiff_.get(),
      x_autodiff);

  auto compute = [&]() {
    InvalidateState();
    plant_autodiff_->CalcMassMatrix(*context_autodiff_, &M_autodiff);
  };

  // The first iteration allocates more memory than subsequent runs.
  compute();

  for (int k = 0; k < 3; k++) {
    // @see LimitMalloc note above.
    LimitMalloc guard(LimitReleaseOnly(31426));

    compute();

    tracker_.Update(guard.num_allocations());
  }

  for (auto _ : state) {
    compute();
  }
  tracker_.Report(&state);
}

BENCHMARK_F(CassieAutodiffFixture, AutodiffInverseDynamics)
    // NOLINTNEXTLINE(runtime/references) cpplint disapproves of gbench choices.
    (benchmark::State& state) {
  VectorXd desired_vdot = VectorXd::Zero(nv_);
  multibody::MultibodyForces<AutoDiffXd> external_forces_autodiff(
      *plant_autodiff_);
  auto x_autodiff = math::InitializeAutoDiff(x_, nq_ + 2 * nv_);
  auto vdot_autodiff = math::InitializeAutoDiff(desired_vdot,
                                                nq_ + 2 * nv_,
                                                nq_ + nv_);
  plant_autodiff_->SetPositionsAndVelocities(context_autodiff_.get(),
      x_autodiff);

  auto compute = [&]() {
    InvalidateState();
    plant_autodiff_->CalcInverseDynamics(*context_autodiff_,
                                         vdot_autodiff,
                                         external_forces_autodiff);
  };

  // The first iteration allocates more memory than subsequent runs.
  compute();

  for (int k = 0; k < 3; k++) {
    // @see LimitMalloc note above.
    LimitMalloc guard(LimitReleaseOnly(38049));

    compute();

    tracker_.Update(guard.num_allocations());
  }

  for (auto _ : state) {
    compute();
  }
  tracker_.Report(&state);
}

BENCHMARK_F(CassieAutodiffFixture, AutodiffForwardDynamics)
    // NOLINTNEXTLINE(runtime/references) cpplint disapproves of gbench choices.
    (benchmark::State& state) {
  auto derivatives_autodiff = plant_autodiff_->AllocateTimeDerivatives();
  auto u_autodiff = math::InitializeAutoDiff(u_, nq_ + nv_ + nu_, nq_ + nv_);
  auto& port_value = plant_autodiff_->get_actuation_input_port().FixValue(
      context_autodiff_.get(), u_autodiff);
  auto x_autodiff = math::InitializeAutoDiff(x_, nq_ + nv_ + nu_);
  plant_autodiff_->SetPositionsAndVelocities(context_autodiff_.get(),
      x_autodiff);

  auto compute = [&]() {
    InvalidateState();
    port_value.GetMutableData();  // Invalidates caching of inputs.
    plant_autodiff_->CalcTimeDerivatives(*context_autodiff_,
                                         derivatives_autodiff.get());
  };

  // The first iteration allocates more memory than subsequent runs.
  compute();

  for (int k = 0; k < 3; k++) {
    // @see LimitMalloc note above. For this particular limit, the high water
    // mark comes when building against Eigen 3.4, so only tighten the limit
    // here if you've tested vs Eigen 3.4 specifically -- our default build
    // of Drake currently uses the Eigen 3.3 series.
    LimitMalloc guard(LimitReleaseOnly(57916));

    compute();

    tracker_.Update(guard.num_allocations());
  }

  for (auto _ : state) {
    compute();
  }
  tracker_.Report(&state);
}

// Fixture that holds a Cassie robot model in a MultibodyPlant<Expression>. It
// also holds a default context for Expression.
class CassieExpressionFixture : public CassieDoubleFixture {
 public:
  using CassieDoubleFixture::SetUp;
  void SetUp(benchmark::State& state) override {
    CassieDoubleFixture::SetUp(state);
    plant_expression_ = systems::System<double>::ToSymbolic(*plant_);
    context_expression_ = plant_expression_->CreateDefaultContext();
  }

  // @see CassieDoubleFixture::InvalidateState(). This method invalidates the
  // expression version of state.
  void InvalidateState() override {
    context_expression_->NoteContinuousStateChange();
  }

 protected:
  std::unique_ptr<MultibodyPlant<Expression>> plant_expression_;
  std::unique_ptr<Context<Expression>> context_expression_;
};

// This uses T=Expression, but all values are still constants (not variables).
BENCHMARK_F(CassieExpressionFixture, ExpressionMassMatrix)
    // NOLINTNEXTLINE(runtime/references) cpplint disapproves of gbench choices.
    (benchmark::State& state) {
  MatrixX<Expression> M(nv_, nv_);
  for (auto _ : state) {
    InvalidateState();
    plant_expression_->CalcMassMatrix(*context_expression_, &M);
  }
}

// This uses T=Expression, but all values are still constants (not variables).
BENCHMARK_F(CassieExpressionFixture, ExpressionInverseDynamics)
    // NOLINTNEXTLINE(runtime/references) cpplint disapproves of gbench choices.
    (benchmark::State& state) {
  VectorX<Expression> desired_vdot = VectorXd::Zero(nv_);
  multibody::MultibodyForces<Expression> external_forces(*plant_expression_);
  for (auto _ : state) {
    InvalidateState();
    plant_expression_->CalcInverseDynamics(
        *context_expression_, desired_vdot, external_forces);
  }
}

// This uses T=Expression, but all values are still constants (not variables).
BENCHMARK_F(CassieExpressionFixture, ExpressionForwardDynamics)
    // NOLINTNEXTLINE(runtime/references) cpplint disapproves of gbench choices.
    (benchmark::State& state) {
  auto derivatives = plant_expression_->AllocateTimeDerivatives();
  auto& port_value = plant_expression_->get_actuation_input_port().FixValue(
      context_expression_.get(), u_.cast<Expression>().eval());
  for (auto _ : state) {
    InvalidateState();
    port_value.GetMutableData();  // Invalidates caching of inputs.
    plant_expression_->CalcTimeDerivatives(
        *context_expression_, derivatives.get());
  }
}

}  // namespace
}  // namespace examples
}  // namespace drake

BENCHMARK_MAIN();
