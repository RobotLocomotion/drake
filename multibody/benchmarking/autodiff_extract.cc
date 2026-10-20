// @file
// Benchmarks for math::ExtractValue() and math::ExtractGradient() reading an
// AutoDiffXd vector through Eigen::Ref<const AutoDiffVecXd> (the type that
// every EvaluatorBase::DoEval() receives) versus through a plain vector, and
// for the AutoDiffXd evaluation of a few cheap evaluators that call them.

#include <memory>

#include <benchmark/benchmark.h>

#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/inverse_kinematics/orientation_constraint.h"
#include "drake/multibody/inverse_kinematics/position_constraint.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/solvers/cost.h"
#include "drake/tools/performance/fixture_common.h"

namespace drake {
namespace multibody {
namespace {

using Eigen::MatrixXd;
using Eigen::VectorXd;

AutoDiffVecXd MakeAutoDiffVector(int size, int num_derivatives) {
  return math::InitializeAutoDiff(VectorXd::LinSpaced(size, -1, 1),
                                  MatrixXd::Identity(size, num_derivatives));
}

__attribute__((noinline)) VectorXd
ExtractValueFromRef(const Eigen::Ref<const AutoDiffVecXd>& x) {
  return math::ExtractValue(x);
}

__attribute__((noinline)) VectorXd
ExtractValueFromVector(const AutoDiffVecXd& x) {
  return math::ExtractValue(x);
}

__attribute__((noinline)) MatrixXd
ExtractGradientFromRef(const Eigen::Ref<const AutoDiffVecXd>& x) {
  return math::ExtractGradient(x);
}

__attribute__((noinline)) MatrixXd
ExtractGradientFromVector(const AutoDiffVecXd& x) {
  return math::ExtractGradient(x);
}

// Arguments are {size, number of derivatives}.
void HelperArgs(benchmark::Benchmark* b) {
  for (int size : {7, 14, 56}) {
    b->Args({size, 1});
    b->Args({size, size});
  }
  b->Unit(benchmark::kNanosecond);
}

// NOLINTNEXTLINE(runtime/references) cpplint disapproves of gbench choices.
void ExtractValueRef(benchmark::State& state) {
  const AutoDiffVecXd x = MakeAutoDiffVector(state.range(0), state.range(1));
  for (auto _ : state) {
    benchmark::DoNotOptimize(ExtractValueFromRef(x));
  }
}
BENCHMARK(ExtractValueRef)->Apply(HelperArgs);

// NOLINTNEXTLINE(runtime/references) cpplint disapproves of gbench choices.
void ExtractValueVector(benchmark::State& state) {
  const AutoDiffVecXd x = MakeAutoDiffVector(state.range(0), state.range(1));
  for (auto _ : state) {
    benchmark::DoNotOptimize(ExtractValueFromVector(x));
  }
}
BENCHMARK(ExtractValueVector)->Apply(HelperArgs);

// NOLINTNEXTLINE(runtime/references) cpplint disapproves of gbench choices.
void ExtractGradientRef(benchmark::State& state) {
  const AutoDiffVecXd x = MakeAutoDiffVector(state.range(0), state.range(1));
  for (auto _ : state) {
    benchmark::DoNotOptimize(ExtractGradientFromRef(x));
  }
}
BENCHMARK(ExtractGradientRef)->Apply(HelperArgs);

// NOLINTNEXTLINE(runtime/references) cpplint disapproves of gbench choices.
void ExtractGradientVector(benchmark::State& state) {
  const AutoDiffVecXd x = MakeAutoDiffVector(state.range(0), state.range(1));
  for (auto _ : state) {
    benchmark::DoNotOptimize(ExtractGradientFromVector(x));
  }
}
BENCHMARK(ExtractGradientVector)->Apply(HelperArgs);

constexpr char kIiwaUrl[] =
    "package://drake_models/iiwa_description/urdf/"
    "iiwa14_spheres_collision.urdf";

// A row of welded iiwas; evaluators act on all of their positions, with the
// identity gradient that a solver passes.
class IiwaRowFixture : public benchmark::Fixture {
 public:
  void SetUp(::benchmark::State& state) override {
    tools::performance::AddMinMaxStatistics(this);

    const int num_iiwas = state.range(0);
    plant_ = std::make_unique<MultibodyPlant<double>>(0.0);
    Parser parser(plant_.get());
    parser.SetAutoRenaming(true);
    ModelInstanceIndex iiwa;
    for (int i = 0; i < num_iiwas; ++i) {
      iiwa = parser.AddModelsFromUrl(kIiwaUrl).at(0);
      plant_->WeldFrames(plant_->world_frame(),
                         plant_->GetFrameByName("base", iiwa),
                         math::RigidTransformd(Eigen::Vector3d(0.6 * i, 0, 0)));
    }
    plant_->Finalize();
    context_ = plant_->CreateDefaultContext();
    const int nq = plant_->num_positions();
    q_ = VectorXd::LinSpaced(nq, -0.5, 0.5);
    gradient_ = MatrixXd::Identity(nq, nq);

    const Frame<double>& end_effector =
        plant_->GetFrameByName("iiwa_link_7", iiwa);
    position_ = std::make_unique<PositionConstraint>(
        plant_.get(), plant_->world_frame(), Eigen::Vector3d::Constant(-1),
        Eigen::Vector3d::Constant(1), end_effector, Eigen::Vector3d(0, 0, 0.1),
        context_.get());
    orientation_ = std::make_unique<OrientationConstraint>(
        plant_.get(), plant_->world_frame(), math::RotationMatrixd(),
        end_effector, math::RotationMatrixd(), 0.1, context_.get());
    quadratic_cost_ = std::make_unique<solvers::QuadraticCost>(
        MatrixXd::Identity(nq, nq), VectorXd::Ones(nq), 1.0);
  }

  void TearDown(::benchmark::State&) override {
    quadratic_cost_.reset();
    orientation_.reset();
    position_.reset();
    context_.reset();
    plant_.reset();
  }

 protected:
  // NOLINTNEXTLINE(runtime/references) cpplint disapproves of gbench choices.
  void RunEval(benchmark::State& state,
               const solvers::EvaluatorBase& evaluator) {
    for (auto _ : state) {
      q_(0) += 1e-9;  // Avoid caching.
      const AutoDiffVecXd q_autodiff = math::InitializeAutoDiff(q_, gradient_);
      AutoDiffVecXd y_autodiff;
      evaluator.Eval(q_autodiff, &y_autodiff);
      benchmark::DoNotOptimize(y_autodiff);
    }
    state.counters["nq"] = q_.size();
  }

  std::unique_ptr<MultibodyPlant<double>> plant_;
  std::unique_ptr<systems::Context<double>> context_;
  std::unique_ptr<PositionConstraint> position_;
  std::unique_ptr<OrientationConstraint> orientation_;
  std::unique_ptr<solvers::QuadraticCost> quadratic_cost_;
  VectorXd q_;
  MatrixXd gradient_;
};

// Arguments are {number of iiwas}.
void IiwaRowArgs(benchmark::Benchmark* b) {
  for (int num_iiwas : {1, 2, 8}) {
    b->Args({num_iiwas});
  }
  b->Unit(benchmark::kNanosecond);
}

BENCHMARK_DEFINE_F(IiwaRowFixture, PositionConstraint)
// NOLINTNEXTLINE(runtime/references) cpplint disapproves of gbench choices.
(benchmark::State& state) {
  RunEval(state, *position_);
}
BENCHMARK_REGISTER_F(IiwaRowFixture, PositionConstraint)->Apply(IiwaRowArgs);

BENCHMARK_DEFINE_F(IiwaRowFixture, OrientationConstraint)
// NOLINTNEXTLINE(runtime/references) cpplint disapproves of gbench choices.
(benchmark::State& state) {
  RunEval(state, *orientation_);
}
BENCHMARK_REGISTER_F(IiwaRowFixture, OrientationConstraint)->Apply(IiwaRowArgs);

BENCHMARK_DEFINE_F(IiwaRowFixture, QuadraticCost)
// NOLINTNEXTLINE(runtime/references) cpplint disapproves of gbench choices.
(benchmark::State& state) {
  RunEval(state, *quadratic_cost_);
}
BENCHMARK_REGISTER_F(IiwaRowFixture, QuadraticCost)->Apply(IiwaRowArgs);

}  // namespace
}  // namespace multibody
}  // namespace drake

BENCHMARK_MAIN();
