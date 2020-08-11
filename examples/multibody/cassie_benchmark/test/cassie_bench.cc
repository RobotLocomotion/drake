#include <benchmark/benchmark.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/limit_malloc.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"

using drake::multibody::MultibodyPlant;
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
// TODO(rpoyner-tri): replace LimitMalloc usage with a benchmark statistics
// implementation that counts malloc use, but doesn't try to enforce caps.

// According to research from @sherm1, the caching system has some debug-only
// asserts that allocate memory. Hence this function only enforces release-only
// ceilings for allocations until that is addressed.
drake::test::LimitMallocParams LimitReleaseOnly(int max_num_allocations) {
  if (kDrakeAssertIsArmed) { return {}; }
  return { .max_num_allocations = max_num_allocations };
}

// Fixture that holds a Cassie robot model in a MultibodyPlant<double>. The
// class also holds a default context for the plant, and dimensions of its
// state and inputs.
class CassieDoubleFixture : public benchmark::Fixture {
 public:
  CassieDoubleFixture() {
    ComputeStatistics("min", [](const std::vector<double>& v) -> double {
        return *(std::min_element(std::begin(v), std::end(v)));
      });
    ComputeStatistics("max", [](const std::vector<double>& v) -> double {
        return *(std::max_element(std::begin(v), std::end(v)));
      });
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
    LimitMalloc guard(LimitReleaseOnly(175));
    InvalidateState();
    plant_->CalcMassMatrix(*context_, &M);
  }
}

BENCHMARK_F(CassieDoubleFixture, DoubleInverseDynamics)
    // NOLINTNEXTLINE(runtime/references) cpplint disapproves of gbench choices.
    (benchmark::State& state) {
  VectorXd desired_vdot = VectorXd::Zero(nv_);
  multibody::MultibodyForces<double> external_forces(*plant_);
  for (auto _ : state) {
    // @see LimitMalloc note above.
    LimitMalloc guard(LimitReleaseOnly(3));
    InvalidateState();
    plant_->CalcInverseDynamics(*context_, desired_vdot, external_forces);
  }
}

BENCHMARK_F(CassieDoubleFixture, DoubleForwardDynamics)
    // NOLINTNEXTLINE(runtime/references) cpplint disapproves of gbench choices.
    (benchmark::State& state) {
  auto derivatives = plant_->AllocateTimeDerivatives();
  auto& port_value =
      plant_->get_actuation_input_port().FixValue(context_.get(), u_);
  for (auto _ : state) {
    // @see LimitMalloc note above.
    LimitMalloc guard(LimitReleaseOnly(22));
    InvalidateState();
    port_value.GetMutableData();  // Invalidates caching of inputs.
    plant_->CalcTimeDerivatives(*context_, derivatives.get());
  }
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
  auto x_autodiff = math::initializeAutoDiff(x_);
  plant_autodiff_->SetPositionsAndVelocities(context_autodiff_.get(),
      x_autodiff);
  for (auto _ : state) {
    // @see LimitMalloc note above.
    LimitMalloc guard(LimitReleaseOnly(62476));
    InvalidateState();
    plant_autodiff_->CalcMassMatrix(*context_autodiff_, &M_autodiff);
  }
}

BENCHMARK_F(CassieAutodiffFixture, AutodiffInverseDynamics)
    // NOLINTNEXTLINE(runtime/references) cpplint disapproves of gbench choices.
    (benchmark::State& state) {
  VectorXd desired_vdot = VectorXd::Zero(nv_);
  multibody::MultibodyForces<AutoDiffXd> external_forces_autodiff(
      *plant_autodiff_);
  auto x_autodiff = math::initializeAutoDiff(x_, nq_ + 2 * nv_);
  auto vdot_autodiff =
      math::initializeAutoDiff(desired_vdot, nq_ + 2 * nv_, nq_ + nv_);
  plant_autodiff_->SetPositionsAndVelocities(context_autodiff_.get(),
      x_autodiff);
  for (auto _ : state) {
    // @see LimitMalloc note above.
    LimitMalloc guard(LimitReleaseOnly(70301));
    InvalidateState();
    plant_autodiff_->CalcInverseDynamics(*context_autodiff_,
                                         vdot_autodiff,
                                         external_forces_autodiff);
  }
}

BENCHMARK_F(CassieAutodiffFixture, AutodiffForwardDynamics)
    // NOLINTNEXTLINE(runtime/references) cpplint disapproves of gbench choices.
    (benchmark::State& state) {
  auto derivatives_autodiff = plant_autodiff_->AllocateTimeDerivatives();
  auto u_autodiff = math::initializeAutoDiff(u_, nq_ + nv_ + nu_, nq_ + nv_);
  auto& port_value = plant_autodiff_->get_actuation_input_port().FixValue(
      context_autodiff_.get(), u_autodiff);
  auto x_autodiff = math::initializeAutoDiff(x_, nq_ + nv_ + nu_);
  plant_autodiff_->SetPositionsAndVelocities(context_autodiff_.get(),
      x_autodiff);
  for (auto _ : state) {
    // @see LimitMalloc note above.
    LimitMalloc guard(LimitReleaseOnly(105675));

    InvalidateState();
    port_value.GetMutableData();  // Invalidates caching of inputs.
    plant_autodiff_->CalcTimeDerivatives(*context_autodiff_,
                                         derivatives_autodiff.get());
  }
}

}  // namespace
}  // namespace examples
}  // namespace drake

BENCHMARK_MAIN();
