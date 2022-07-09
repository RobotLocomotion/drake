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

// In the benchmark case instantiations at the bottom of this file, we'll use
// a bitmask for the case's "Arg" to denote which quantities are in scope as
// either gradients (for T=AutoDiffXd) or variables (for T=Expression).
constexpr int kWantGradX = 1;
constexpr int kWantGradVdot = 2;
constexpr int kWantGradU = 4;

// Fixture that holds a Cassie robot model and offers helper functions to
// configure the benchmark case.
template <typename T>
class Cassie : public benchmark::Fixture {
 public:
  Cassie() {
    tools::performance::AddMinMaxStatistics(this);
  }

  // This apparently futile using statement works around "overloaded virtual"
  // errors in g++. All of this is a consequence of the weird deprecation of
  // const-ref State versions of SetUp() and TearDown() in benchmark.h.
  using benchmark::Fixture::SetUp;
  void SetUp(BenchmarkStateRef state) override {
    SetUpNonZeroState();
    SetUpGradientsOrVariables(state);
  }

 protected:
  // Loads the plant.
  static std::unique_ptr<MultibodyPlant<T>> MakePlant();

  // Sets the plant to have non-zero state and input. In some cases, computing
  // using zeros will not tickle the relevant paths through the code.
  void SetUpNonZeroState();

  VectorX<T> GetPlantState() const {
    return context_->get_continuous_state_vector().CopyToVector();
  }

  const VectorX<T>& GetPlantInput() const {
    return input_.get_vector_value<T>().value();
  }

  // In the benchmark case instantiations at the bottom of this file, we'll use
  // a bitmask for the case's "Arg" to denote which quantities are in scope as
  // either gradients (for T=AutoDiffXd) or variables (for T=Expression).
  static bool want_grad_x(const benchmark::State& state) {
    return state.range(0) & kWantGradX;
  }
  static bool want_grad_vdot(const benchmark::State& state) {
    return state.range(0) & kWantGradVdot;
  }
  static bool want_grad_u(const benchmark::State& state) {
    return state.range(0) & kWantGradU;
  }

  // Using the "Arg" from the given benchmark state, sets up the MbP
  // state and/or input to use gradients and/or symbolic variables
  // as configured in this benchmark case.
  //
  // For T=double, any request for gradients is an error.
  // For T=AutoDiffXd, sets the specified gradients to the identity matrix.
  // For T=Expression, sets the specified quantities to symbolic variables.
  void SetUpGradientsOrVariables(BenchmarkStateRef state);

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

  // Runs the MassMatrix benchmark.
  void DoMassMatrix(BenchmarkStateRef state) {
    DRAKE_DEMAND(want_grad_vdot(state) == false);
    DRAKE_DEMAND(want_grad_u(state) == false);
    for (auto _ : state) {
      InvalidateState();
      plant_->CalcMassMatrix(*context_, &mass_matrix_out_);
    }
  }

  // Runs the InverseDynamics benchmark.
  void DoInverseDynamics(BenchmarkStateRef state) {
    DRAKE_DEMAND(want_grad_u(state) == false);
    for (auto _ : state) {
      InvalidateState();
      plant_->CalcInverseDynamics(*context_, desired_vdot_, external_forces_);
    }
  }

  // Runs the ForwardDynamics benchmark.
  void DoForwardDynamics(BenchmarkStateRef state) {
    DRAKE_DEMAND(want_grad_vdot(state) == false);
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
      context_.get(), VectorX<T>::Zero(nu_));

  // Data used in the MassMatrix cases (only).
  MatrixX<T> mass_matrix_out_;

  // Data used in the InverseDynamics cases (only).
  VectorX<T> desired_vdot_;
  const MultibodyForces<T> external_forces_{*plant_};
};

using CassieDouble = Cassie<double>;
using CassieAutoDiff = Cassie<AutoDiffXd>;
using CassieExpression = Cassie<Expression>;

template <typename T>
std::unique_ptr<MultibodyPlant<T>> Cassie<T>::MakePlant() {
  auto plant = std::make_unique<MultibodyPlant<double>>(0.0);
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

template <typename T>
void Cassie<T>::SetUpNonZeroState() {
  // Reset 'x'; be sure to set quaternions back to a sane value.
  context_->get_mutable_continuous_state_vector().SetFromVector(
      VectorX<T>::LinSpaced(nq_ + nv_, 0.1, 0.9));
  for (const BodyIndex& index : plant_->GetFloatingBaseBodies()) {
    const Body<T>& body = plant_->get_body(index);
    const RigidTransform<T> pose(
        RollPitchYaw<T>(0.1, 0.2, 0.3), Vector3<T>(0.4, 0.5, 0.6));
    plant_->SetFreeBodyPose(context_.get(), body, pose);
  }

  // Reset 'vdot'.
  desired_vdot_ = VectorX<T>::Constant(nv_, 0.5);

  // Reset 'u'.
  input_.GetMutableVectorData<T>()->SetFromVector(
      VectorX<T>::Constant(nu_, 0.5));

  // Reset temporaries.
  mass_matrix_out_ = MatrixX<T>::Zero(nv_, nv_);
}

template <>
void Cassie<double>::SetUpGradientsOrVariables(BenchmarkStateRef state) {
  DRAKE_DEMAND(want_grad_x(state) == false);
  DRAKE_DEMAND(want_grad_vdot(state) == false);
  DRAKE_DEMAND(want_grad_u(state) == false);
}

namespace {
// Conditionally sets the identity gradient vector on 'a' and/or 'b'.
// Returns a tuple of [a_grad, b_grad].
std::tuple<VectorX<AutoDiffXd>, VectorX<AutoDiffXd>> AddGradient(
    const VectorX<double>& a, const VectorX<double>& b,
    bool want_grad_a, bool want_grad_b) {
  std::tuple<VectorX<AutoDiffXd>, VectorX<AutoDiffXd>> result;
  if (want_grad_a && want_grad_b) {
    result = math::InitializeAutoDiffTuple(a, b);
  } else if (want_grad_a) {
    std::get<0>(result) = math::InitializeAutoDiff(a);
    std::get<1>(result) = b;
  } else if (want_grad_b) {
    std::get<0>(result) = a;
    std::get<1>(result) = math::InitializeAutoDiff(b);
  } else {
    std::get<0>(result) = a;
    std::get<1>(result) = b;
  }
  return result;
}
}  // namespace

template <>
void Cassie<AutoDiffXd>::SetUpGradientsOrVariables(BenchmarkStateRef state) {
  // Read the default plant values (without any gradients).
  const VectorX<double> x = math::DiscardGradient(GetPlantState());
  const VectorX<double> vd = math::DiscardGradient(desired_vdot_);
  const VectorX<double> u = math::DiscardGradient(GetPlantInput());

  // Initialize the desired gradients.
  VectorX<AutoDiffXd> x_grad;
  VectorX<AutoDiffXd> vd_grad;
  VectorX<AutoDiffXd> u_grad;
  if (want_grad_u(state)) {
    DRAKE_DEMAND(want_grad_vdot(state) == false);
    vd_grad = vd;
    std::tie(x_grad, u_grad) =
        AddGradient(x, u, want_grad_x(state), want_grad_u(state));
  } else {
    DRAKE_DEMAND(want_grad_u(state) == false);
    u_grad = u;
    std::tie(x_grad, vd_grad) =
        AddGradient(x, vd, want_grad_x(state), want_grad_vdot(state));
  }

  // Write back to the plant.
  plant_->SetPositionsAndVelocities(context_.get(), x_grad);
  desired_vdot_ = vd_grad;
  input_.GetMutableVectorData<AutoDiffXd>()->SetFromVector(u_grad);
}

template <>
void Cassie<Expression>::SetUpGradientsOrVariables(BenchmarkStateRef state) {
  if (want_grad_x(state)) {
    const VectorX<Expression> x = MakeMatrixVariable(nq_ + nv_, 1, "x");
    plant_->SetPositionsAndVelocities(context_.get(), x);
  }
  if (want_grad_vdot(state)) {
    desired_vdot_ = MakeMatrixVariable(nv_, 1, "vd");
  }
  if (want_grad_u(state)) {
    const VectorX<Expression> u = MakeMatrixVariable(nu_, 1, "u");
    input_.GetMutableVectorData<Expression>()->SetFromVector(u);
  }
}

// All that remains is to add the sensible combinations of benchmark configs.
//
// For T=double, there's only a single config. We still use a range arg "0" so
// that its correspondence with the non-double cases is apparent.
//
// For T=AutoDiff, the range arg sets which gradients to use; "0" means no
// gradients; non-zero args each cover more and more of the function's inputs.
//
// For T=Expression, the range arg sets which variables to use; "0" means no
// variables; non-zero args each cover more and move of the function's inputs.

BENCHMARK_DEFINE_F(CassieDouble, MassMatrix)(BenchmarkStateRef state) {
  DoMassMatrix(state);
}
BENCHMARK_REGISTER_F(CassieDouble, MassMatrix)
  ->Unit(benchmark::kMicrosecond)
  ->Arg(0);

BENCHMARK_DEFINE_F(CassieDouble, InverseDynamics)(BenchmarkStateRef state) {
  DoInverseDynamics(state);
}
BENCHMARK_REGISTER_F(CassieDouble, InverseDynamics)
  ->Unit(benchmark::kMicrosecond)
  ->Arg(0);

BENCHMARK_DEFINE_F(CassieDouble, ForwardDynamics)(BenchmarkStateRef state) {
  DoForwardDynamics(state);
}
BENCHMARK_REGISTER_F(CassieDouble, ForwardDynamics)
  ->Unit(benchmark::kMicrosecond)
  ->Arg(0);

BENCHMARK_DEFINE_F(CassieAutoDiff, MassMatrix)(BenchmarkStateRef state) {
  DoMassMatrix(state);
}
BENCHMARK_REGISTER_F(CassieAutoDiff, MassMatrix)
  ->Unit(benchmark::kMicrosecond)
  ->Arg(0)
  ->Arg(kWantGradX);

BENCHMARK_DEFINE_F(CassieAutoDiff, InverseDynamics)(BenchmarkStateRef state) {
  DoInverseDynamics(state);
}
BENCHMARK_REGISTER_F(CassieAutoDiff, InverseDynamics)
  ->Unit(benchmark::kMicrosecond)
  ->Arg(0)
  ->Arg(kWantGradX)
  ->Arg(kWantGradVdot)
  ->Arg(kWantGradX|kWantGradVdot);

BENCHMARK_DEFINE_F(CassieAutoDiff, ForwardDynamics)(BenchmarkStateRef state) {
  DoForwardDynamics(state);
}
BENCHMARK_REGISTER_F(CassieAutoDiff, ForwardDynamics)
  ->Unit(benchmark::kMicrosecond)
  ->Arg(0)
  ->Arg(kWantGradX)
  ->Arg(kWantGradU)
  ->Arg(kWantGradX|kWantGradU);

BENCHMARK_DEFINE_F(CassieExpression, MassMatrix)(BenchmarkStateRef state) {
  DoMassMatrix(state);
}
BENCHMARK_REGISTER_F(CassieExpression, MassMatrix)
  ->Unit(benchmark::kMicrosecond)
  ->Arg(0)
  ->Arg(kWantGradX);

BENCHMARK_DEFINE_F(CassieExpression, InverseDynamics)(BenchmarkStateRef state) {
  DoInverseDynamics(state);
}
BENCHMARK_REGISTER_F(CassieExpression, InverseDynamics)
  ->Unit(benchmark::kMicrosecond)
  ->Arg(0)
  ->Arg(kWantGradX)
  ->Arg(kWantGradVdot)
  ->Arg(kWantGradX|kWantGradVdot);

BENCHMARK_DEFINE_F(CassieExpression, ForwardDynamics)(BenchmarkStateRef state) {
  DoForwardDynamics(state);
}
BENCHMARK_REGISTER_F(CassieExpression, ForwardDynamics)
  ->Unit(benchmark::kMicrosecond)
  ->Arg(0)
  // N.B. MbP does not support forward dynamics with Variables in 'x'.
  ->Arg(kWantGradU);

}  // namespace
}  // namespace multibody
}  // namespace drake

BENCHMARK_MAIN();
