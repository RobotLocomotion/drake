#include <benchmark/benchmark.h>

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/tools/performance/fixture_common.h"

namespace drake {
namespace multibody {
namespace {

using math::RigidTransform;
using math::RollPitchYaw;
using symbolic::Expression;
using symbolic::MakeMatrixVariable;
using systems::Context;
using systems::ContinuousState;
using systems::FixedInputPortValue;
using systems::System;

// We use this alias to silence cpplint barking at mutable references.
using BenchmarkStateRef = benchmark::State&;

// Fixture that holds a Cassie robot model and offers helper functions to
// configure the benchmark case.
template <typename T>
class Cassie : public benchmark::Fixture {
 public:
  // Loads the plant, allocates memory, and sets default values.
  Cassie() {
    tools::performance::AddMinMaxStatistics(this);

    // Set the plant to have non-zero state. In some cases, computing using
    // zeros will not tickle the relevant paths through the code.
    context_->get_mutable_continuous_state_vector().SetFromVector(
        VectorX<T>::LinSpaced(nq_ + nv_, 0.1, 0.9));

    // Set any quaternions back to a sane value.
    for (const BodyIndex& index : plant_->GetFloatingBaseBodies()) {
      const Body<T>& body = plant_->get_body(index);
      const RigidTransform<T> pose(
          RollPitchYaw<T>(0.1, 0.2, 0.3), Vector3<T>(0.4, 0.5, 0.6));
      plant_->SetFreeBodyPose(context_.get(), body, pose);
    }
  }

 protected:
  // Loads the plant.
  static std::unique_ptr<MultibodyPlant<T>> MakePlant() {
    auto plant = std::make_unique<MultibodyPlant<double>>(0);
    Parser parser(plant.get());
    const auto& model = "drake/multibody/benchmarking/cassie_v2.urdf";
    parser.AddModelFromFile(FindResourceOrThrow(model));
    plant->Finalize();
    if constexpr (std::is_same_v<T, double>) {
      return plant;
    } else {
      return System<double>::ToScalarType<T>(*plant);
    }
  }

  VectorX<T> GetPlantState() const {
    return context_->get_continuous_state_vector().CopyToVector();
  }

  const VectorX<T>& GetPlantInput() const {
    return input_.get_vector_value<T>().value();
  }

  // Take gradients w.r.t the plant state 'x'.
  void SetGradX() {
    auto x = math::DiscardGradient(GetPlantState());
    auto x_grad = math::InitializeAutoDiff(x);
    plant_->SetPositionsAndVelocities(context_.get(), x_grad);
  }

  // Take gradients w.r.t the inverse dynamics 'vdot'.
  void SetGradVdot() {
    auto vd = math::DiscardGradient(desired_vdot_);
    auto vd_grad = math::InitializeAutoDiff(vd);
    desired_vdot_ = vd_grad;
  }

  // Take gradients w.r.t the plant state 'x' and the inverse dynamics 'vdot'.
  void SetGradXVdot() {
    auto x = math::DiscardGradient(GetPlantState());
    auto vd = math::DiscardGradient(desired_vdot_);
    auto [x_grad, vd_grad] = math::InitializeAutoDiffTuple(x, vd);
    plant_->SetPositionsAndVelocities(context_.get(), x_grad);
    desired_vdot_ = vd_grad;
  }

  // Take gradients w.r.t the actuation input 'u'.
  void SetGradU() {
    auto u = math::DiscardGradient(GetPlantInput());
    auto u_grad = math::InitializeAutoDiff(u);
    input_.GetMutableVectorData<AutoDiffXd>()->SetFromVector(u_grad);
  }

  // Take gradients w.r.t the plant state 'x' and actuation input 'u'.
  void SetGradXU() {
    auto x = math::DiscardGradient(GetPlantState());
    auto u = math::DiscardGradient(GetPlantInput());
    auto [x_grad, u_grad] = math::InitializeAutoDiffTuple(x, u);
    plant_->SetPositionsAndVelocities(context_.get(), x_grad);
    input_.GetMutableVectorData<AutoDiffXd>()->SetFromVector(u_grad);
  }

  // Use variables for the plant state 'x'.
  void SetSymbolicX() {
    auto x = MakeMatrixVariable(nq_ + nv_, 1, "x");
    plant_->SetPositionsAndVelocities(context_.get(), x.cast<Expression>());
  }

  // Use variables for the inverse dynamics 'vdot'.
  void SetSymbolicVdot() {
    desired_vdot_ = MakeMatrixVariable(nv_, 1, "vd");
  }

  // Use these functions to invalidate input- or state-dependent computations
  // each benchmarked step. Disabling the cache entirely would affect the
  // performance because it would suppress any internal use of the cache during
  // complicated computations like forward dynamics. For example, if there are
  // multiple places in forward dynamics that access body positions, currently
  // those would get computed once and re-used (like in real applications) but
  // with caching off they would get recalculated repeatedly, affecting the
  // timing results.
  void InvalidateInput() {
    input_.GetMutableData();
  }
  void InvalidateState() {
    context_->NoteContinuousStateChange();
  }

  // Run the MassMatrix benchmark.
  void DoMassMatrix(BenchmarkStateRef state) {
    for (auto _ : state) {
      InvalidateState();
      plant_->CalcMassMatrix(*context_, &mass_matrix_out_);
    }
  }

  // Run the InverseDynamics benchmark.
  void DoInverseDynamics(BenchmarkStateRef state) {
    for (auto _ : state) {
      InvalidateState();
      plant_->CalcInverseDynamics(*context_, desired_vdot_, external_forces_);
    }
  }

  // Run the ForwardDynamics benchmark.
  void DoForwardDynamics(BenchmarkStateRef state) {
    for (auto _ : state) {
      InvalidateInput();
      InvalidateState();
      plant_->EvalTimeDerivatives(*context_);
    }
  }

  // The plant itself.
  const std::unique_ptr<const MultibodyPlant<T>> plant_{MakePlant()};
  const int nq_{plant_->num_positions()};
  const int nv_{plant_->num_velocities()};
  const int nu_{plant_->num_actuators()};

  // The plant's context.
  std::unique_ptr<Context<T>> context_{plant_->CreateDefaultContext()};
  FixedInputPortValue& input_ = plant_->get_actuation_input_port().FixValue(
      context_.get(), VectorX<T>::Constant(nu_, 0.15));

  // Data used in the MassMatrix cases (only).
  MatrixX<T> mass_matrix_out_{nv_, nv_};

  // Data used in the InverseDynamics cases (only).
  VectorX<T> desired_vdot_{VectorX<T>::Constant(nv_, 0.125)};
  MultibodyForces<T> external_forces_{*plant_};
};

using CassieDouble = Cassie<double>;
using CassieAutoDiff = Cassie<AutoDiffXd>;
using CassieExpression = Cassie<Expression>;

// All that remains is to add the sensible combinations of benchmark configs.
//
// For T=double, there's only a single config. We still use a range arg "/0" so
// that its correspondence with the non-double cases is apparrent.
//
// For T=AutoDiff, the range arg sets which gradients to use; "/0" means no
// gradients; non-zero args each cover more and more of the function's inputs.
//
// For T=Expression, the range arg sets which variables to use; "/0" means no
// variables; non-zerzo args each cover more and move of the function's inputs.

BENCHMARK_DEFINE_F(CassieDouble, MassMatrix)(BenchmarkStateRef state) {
  DoMassMatrix(state);
}
BENCHMARK_REGISTER_F(CassieDouble, MassMatrix)
  ->DenseRange(0, 0)->Unit(benchmark::kMicrosecond);

BENCHMARK_DEFINE_F(CassieDouble, InverseDynamics)(BenchmarkStateRef state) {
  DoInverseDynamics(state);
}
BENCHMARK_REGISTER_F(CassieDouble, InverseDynamics)
  ->DenseRange(0, 0)->Unit(benchmark::kMicrosecond);

BENCHMARK_DEFINE_F(CassieDouble, ForwardDynamics)(BenchmarkStateRef state) {
  DoForwardDynamics(state);
}
BENCHMARK_REGISTER_F(CassieDouble, ForwardDynamics)
  ->DenseRange(0, 0)->Unit(benchmark::kMicrosecond);

BENCHMARK_DEFINE_F(CassieAutoDiff, MassMatrix)(BenchmarkStateRef state) {
  switch (state.range(0)) {
    case 0: break;
    case 1: SetGradX(); break;
    default: DRAKE_UNREACHABLE();
  }
  DoMassMatrix(state);
}
BENCHMARK_REGISTER_F(CassieAutoDiff, MassMatrix)
  ->DenseRange(0, 1)->Unit(benchmark::kMicrosecond);

BENCHMARK_DEFINE_F(CassieAutoDiff, InverseDynamics)(BenchmarkStateRef state) {
  switch (state.range(0)) {
    case 0: break;
    case 1: SetGradVdot(); break;
    case 2: SetGradX(); break;
    case 3: SetGradXVdot(); break;
    default: DRAKE_UNREACHABLE();
  }
  DoInverseDynamics(state);
}
BENCHMARK_REGISTER_F(CassieAutoDiff, InverseDynamics)
  ->DenseRange(0, 3)->Unit(benchmark::kMicrosecond);

BENCHMARK_DEFINE_F(CassieAutoDiff, ForwardDynamics)(BenchmarkStateRef state) {
  switch (state.range(0)) {
    case 0: break;
    case 1: SetGradU(); break;
    case 2: SetGradX(); break;
    case 3: SetGradXU(); break;
    default: DRAKE_UNREACHABLE();
  }
  DoForwardDynamics(state);
}
BENCHMARK_REGISTER_F(CassieAutoDiff, ForwardDynamics)
  ->DenseRange(0, 3)->Unit(benchmark::kMicrosecond);

BENCHMARK_DEFINE_F(CassieExpression, MassMatrix)(BenchmarkStateRef state) {
  switch (state.range(0)) {
    case 0: break;
    case 1: SetSymbolicX(); break;
    default: DRAKE_UNREACHABLE();
  }
  DoMassMatrix(state);
}
BENCHMARK_REGISTER_F(CassieExpression, MassMatrix)
  ->DenseRange(0, 1)->Unit(benchmark::kMicrosecond);

BENCHMARK_DEFINE_F(CassieExpression, InverseDynamics)(BenchmarkStateRef state) {
  switch (state.range(0)) {
    case 0: break;
    case 1: SetSymbolicVdot(); break;
    case 2: SetSymbolicX(); break;
    case 3: SetSymbolicX(); SetSymbolicVdot(); break;
    default: DRAKE_UNREACHABLE();
  }
  DoInverseDynamics(state);
}
BENCHMARK_REGISTER_F(CassieExpression, InverseDynamics)
  ->DenseRange(0, 3)->Unit(benchmark::kMicrosecond);

BENCHMARK_DEFINE_F(CassieExpression, ForwardDynamics)(BenchmarkStateRef state) {
  switch (state.range(0)) {
    case 0: break;
    default: DRAKE_UNREACHABLE();
  }
  DoForwardDynamics(state);
}
BENCHMARK_REGISTER_F(CassieExpression, ForwardDynamics)
  ->DenseRange(0, 0)->Unit(benchmark::kMicrosecond);

}  // namespace
}  // namespace multibody
}  // namespace drake

BENCHMARK_MAIN();
