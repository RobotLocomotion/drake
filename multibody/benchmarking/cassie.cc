#include <memory>

#include <benchmark/benchmark.h>

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/tools/performance/fixture_common.h"
#include "drake/tools/performance/fixture_memory.h"

namespace drake {
namespace multibody {
namespace {

using math::RigidTransform;
using math::RollPitchYaw;
using symbolic::Expression;
using symbolic::MakeVectorVariable;
using systems::Context;
using systems::ContinuousState;
using systems::FixedInputPortValue;
using systems::System;

// In the benchmark case instantiations at the bottom of this file, we'll use
// a bitmask for the case's "Arg" to denote which quantities are in scope as
// either gradients (for T=AutoDiffXd) or variables (for T=Expression).
constexpr int kWantNoGrad = 0x0;
constexpr int kWantGradQ = 0x1;
constexpr int kWantGradV = 0x2;
constexpr int kWantGradX = kWantGradQ | kWantGradV;
constexpr int kWantGradVdot = 0x4;
constexpr int kWantGradU = 0x8;

// Fixture that holds a Cassie robot model and offers helper functions to
// configure the benchmark case.
template <typename T>
class Cassie : public benchmark::Fixture {
 public:
  Cassie() { tools::performance::AddMinMaxStatistics(this); }

  void SetUp(benchmark::State& state) override {
    SetUpNonZeroState();
    SetUpGradientsOrVariables(state);
    tools::performance::TareMemoryManager();
  }

 protected:
  // Loads the plant.
  static std::unique_ptr<MultibodyPlant<T>> MakePlant();

  // Sets the plant to have non-zero state and input. In some cases, computing
  // using zeros will not tickle the relevant paths through the code.
  void SetUpNonZeroState();

  // In the benchmark case instantiations at the bottom of this file, we'll use
  // a bitmask for the case's "Arg" to denote which quantities are in scope as
  // either gradients (for T=AutoDiffXd) or variables (for T=Expression).
  static bool want_grad_q(const benchmark::State& state) {
    return state.range(0) & kWantGradQ;
  }
  static bool want_grad_v(const benchmark::State& state) {
    return state.range(0) & kWantGradV;
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
  // NOLINTNEXTLINE(runtime/references)
  void SetUpGradientsOrVariables(benchmark::State& state);

  // Use these functions to invalidate input- or state-dependent computations
  // each benchmarked step. Disabling the cache entirely would affect the
  // performance because it would suppress any internal use of the cache during
  // complicated computations like forward dynamics. For example, if there are
  // multiple places in forward dynamics that access body positions, currently
  // those would get computed once and re-used (like in real applications) but
  // with caching off they would get recalculated repeatedly, affecting the
  // timing results.
  void InvalidateInput() { input_.GetMutableData(); }
  void InvalidateState() { context_->NoteContinuousStateChange(); }

  // Runs the PositionKinematics benchmark.
  // NOLINTNEXTLINE(runtime/references)
  void DoPositionKinematics(benchmark::State& state) {
    DRAKE_DEMAND(want_grad_vdot(state) == false);
    DRAKE_DEMAND(want_grad_u(state) == false);
    for (auto _ : state) {
      InvalidateState();
      plant_->EvalPositionKinematics(*context_);
    }
  }

  // Runs the CompositeBodyInertiaInWorld benchmark.
  // NOLINTNEXTLINE(runtime/references)
  void DoCompositeBodyInertiaInWorld(benchmark::State& state) {
    DRAKE_DEMAND(want_grad_vdot(state) == false);
    DRAKE_DEMAND(want_grad_u(state) == false);
    for (auto _ : state) {
      InvalidateState();
      plant_->EvalCompositeBodyInertiaInWorldCache(*context_);
    }
  }

  // Runs the SlowSystemJacobian benchmark.
  // NOLINTNEXTLINE(runtime/references)
  void DoSlowSystemJacobian(benchmark::State& state) {
    DRAKE_DEMAND(want_grad_vdot(state) == false);
    DRAKE_DEMAND(want_grad_u(state) == false);
    const int num_mobods = ssize(plant_->graph().forest().mobods());
    MatrixX<double> Jv_V_WB_W(6 * num_mobods, plant_->num_velocities());
    for (auto _ : state) {
      InvalidateState();
      Jv_V_WB_W.setZero();
      for (BodyIndex index{1}; index < plant_->num_bodies(); ++index) {
        const Frame<double>& body_frame = plant_->get_body(index).body_frame();
        auto J = Jv_V_WB_W.block(6 * index, 0, 6, plant_->num_velocities());
        plant_->CalcJacobianSpatialVelocity(
            *context_, JacobianWrtVariable::kV, body_frame,
            Eigen::Vector3<double>::Zero(), plant_->world_frame(),
            plant_->world_frame(), &J);
      }
    }
  }

  // Runs the BlockSystemJacobian benchmark.
  // NOLINTNEXTLINE(runtime/references)
  void DoBlockSystemJacobian(benchmark::State& state) {
    DRAKE_DEMAND(want_grad_vdot(state) == false);
    DRAKE_DEMAND(want_grad_u(state) == false);
    for (auto _ : state) {
      InvalidateState();
      (void)plant_->EvalBlockSystemJacobian(*context_);
    }
  }

  // Runs the PosAndVelKinematics benchmark.
  // NOLINTNEXTLINE(runtime/references)
  void DoPosAndVelKinematics(benchmark::State& state) {
    DRAKE_DEMAND(want_grad_vdot(state) == false);
    DRAKE_DEMAND(want_grad_u(state) == false);
    for (auto _ : state) {
      InvalidateState();
      // This requires both position and velocity kinematics.
      plant_->EvalVelocityKinematics(*context_);
    }
  }

  // Runs the MassMatrixViaID benchmark.
  // NOLINTNEXTLINE(runtime/references)
  void DoMassMatrixViaID(benchmark::State& state) {
    DRAKE_DEMAND(want_grad_vdot(state) == false);
    DRAKE_DEMAND(want_grad_u(state) == false);
    for (auto _ : state) {
      InvalidateState();
      plant_->CalcMassMatrixViaInverseDynamics(*context_, &mass_matrix_out_);
    }
  }

  // Runs the MassMatrix benchmark.
  // NOLINTNEXTLINE(runtime/references)
  void DoMassMatrix(benchmark::State& state) {
    DRAKE_DEMAND(want_grad_vdot(state) == false);
    DRAKE_DEMAND(want_grad_u(state) == false);
    for (auto _ : state) {
      InvalidateState();
      plant_->CalcMassMatrix(*context_, &mass_matrix_out_);
    }
  }

  // Runs the InverseDynamics benchmark.
  // NOLINTNEXTLINE(runtime/references)
  void DoInverseDynamics(benchmark::State& state) {
    DRAKE_DEMAND(want_grad_u(state) == false);
    for (auto _ : state) {
      InvalidateState();
      plant_->CalcInverseDynamics(*context_, desired_vdot_, external_forces_);
    }
  }

  // Runs the ForwardDynamics benchmark.
  // NOLINTNEXTLINE(runtime/references)
  void DoForwardDynamics(benchmark::State& state) {
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
  MultibodyForces<T> external_forces_{*plant_};
};

using CassieDouble = Cassie<double>;
using CassieAutoDiff = Cassie<AutoDiffXd>;
using CassieExpression = Cassie<Expression>;

template <typename T>
std::unique_ptr<MultibodyPlant<T>> Cassie<T>::MakePlant() {
  auto plant = std::make_unique<MultibodyPlant<double>>(0.0);
  Parser parser(plant.get());
  const auto& model = "drake/multibody/benchmarking/cassie_v2.urdf";
  parser.AddModels(FindResourceOrThrow(model));
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
    const RigidBody<T>& body = plant_->get_body(index);
    const RigidTransform<T> pose(RollPitchYaw<T>(0.1, 0.2, 0.3),
                                 Vector3<T>(0.4, 0.5, 0.6));
    plant_->SetFreeBodyPose(context_.get(), body, pose);
  }

  // Reset 'vdot'.
  desired_vdot_ = VectorX<T>::Constant(nv_, 0.5);

  // Reset 'u'.
  input_.GetMutableVectorData<T>()->SetFromVector(
      VectorX<T>::Constant(nu_, 0.5));

  // Reset 'tau'.
  external_forces_.mutable_generalized_forces() =
      VectorX<T>::LinSpaced(nv_, 0.01, 0.09);

  // Reset temporaries.
  mass_matrix_out_ = MatrixX<T>::Zero(nv_, nv_);
}

template <>  // NOLINTNEXTLINE(runtime/references)
void Cassie<double>::SetUpGradientsOrVariables(benchmark::State& state) {
  DRAKE_DEMAND(want_grad_q(state) == false);
  DRAKE_DEMAND(want_grad_v(state) == false);
  DRAKE_DEMAND(want_grad_vdot(state) == false);
  DRAKE_DEMAND(want_grad_u(state) == false);
}

template <>  // NOLINTNEXTLINE(runtime/references)
void Cassie<AutoDiffXd>::SetUpGradientsOrVariables(benchmark::State& state) {
  // For the quantities destined for InitializeAutoDiff, read their default
  // values (without any gradients). For the others, leave the matrix empty.
  VectorX<double> q, v, vdot, u;
  if (want_grad_q(state)) {
    q = math::DiscardGradient(plant_->GetPositions(*context_));
  }
  if (want_grad_v(state)) {
    v = math::DiscardGradient(plant_->GetVelocities(*context_));
  }
  if (want_grad_vdot(state)) {
    vdot = math::DiscardGradient(desired_vdot_);
  }
  if (want_grad_u(state)) {
    u = math::DiscardGradient(input_.get_vector_value<AutoDiffXd>().value());
  }

  // Initialize the desired gradients.
  VectorX<AutoDiffXd> q_grad, v_grad, vdot_grad, u_grad;
  std::tie(q_grad, v_grad, vdot_grad, u_grad) =
      math::InitializeAutoDiffTuple(q, v, vdot, u);

  // Write the gradients back to the plant.
  if (want_grad_q(state)) {
    plant_->SetPositions(context_.get(), q_grad);
  }
  if (want_grad_v(state)) {
    plant_->SetVelocities(context_.get(), v_grad);
  }
  if (want_grad_vdot(state)) {
    desired_vdot_ = vdot_grad;
  }
  if (want_grad_u(state)) {
    input_.GetMutableVectorData<AutoDiffXd>()->SetFromVector(u_grad);
  }
}

template <>  // NOLINTNEXTLINE(runtime/references)
void Cassie<Expression>::SetUpGradientsOrVariables(benchmark::State& state) {
  if (want_grad_q(state)) {
    const VectorX<Expression> q = MakeVectorVariable(nq_, "q");
    plant_->SetPositions(context_.get(), q);
  }
  if (want_grad_v(state)) {
    const VectorX<Expression> v = MakeVectorVariable(nv_, "v");
    plant_->SetVelocities(context_.get(), v);
  }
  if (want_grad_vdot(state)) {
    desired_vdot_ = MakeVectorVariable(nv_, "vd");
  }
  if (want_grad_u(state)) {
    const VectorX<Expression> u = MakeVectorVariable(nu_, "u");
    input_.GetMutableVectorData<Expression>()->SetFromVector(u);
  }
}

// All that remains is to add the sensible combinations of benchmark configs.
//
// For T=double, there's only a single config. We still use a range arg so
// that its correspondence with the non-double cases is apparent.
//
// For T=AutoDiff, the range arg sets which gradients to use, using a bitmask.
//
// For T=Expression, the range arg sets which variables to use, using a bitmask.

// NOLINTNEXTLINE(runtime/references)
BENCHMARK_DEFINE_F(CassieDouble, PositionKinematics)(benchmark::State& state) {
  DoPositionKinematics(state);
}
BENCHMARK_REGISTER_F(CassieDouble, PositionKinematics)
    ->Unit(benchmark::kMicrosecond)
    ->Arg(kWantNoGrad);

BENCHMARK_DEFINE_F(CassieDouble, CompositeBodyInertiaInWorld)
// NOLINTNEXTLINE(runtime/references)
(benchmark::State& state) {
  DoCompositeBodyInertiaInWorld(state);
}
BENCHMARK_REGISTER_F(CassieDouble, CompositeBodyInertiaInWorld)
    ->Unit(benchmark::kMicrosecond)
    ->Arg(kWantNoGrad);

// NOLINTNEXTLINE(runtime/references)
BENCHMARK_DEFINE_F(CassieDouble, SlowSystemJacobian)(benchmark::State& state) {
  DoSlowSystemJacobian(state);
}
BENCHMARK_REGISTER_F(CassieDouble, SlowSystemJacobian)
    ->Unit(benchmark::kMicrosecond)
    ->Arg(kWantNoGrad);

// NOLINTNEXTLINE(runtime/references)
BENCHMARK_DEFINE_F(CassieDouble, BlockSystemJacobian)(benchmark::State& state) {
  DoBlockSystemJacobian(state);
}
BENCHMARK_REGISTER_F(CassieDouble, BlockSystemJacobian)
    ->Unit(benchmark::kMicrosecond)
    ->Arg(kWantNoGrad);

// NOLINTNEXTLINE(runtime/references)
BENCHMARK_DEFINE_F(CassieDouble, PosAndVelKinematics)(benchmark::State& state) {
  DoPosAndVelKinematics(state);
}
BENCHMARK_REGISTER_F(CassieDouble, PosAndVelKinematics)
    ->Unit(benchmark::kMicrosecond)
    ->Arg(kWantNoGrad);

// NOLINTNEXTLINE(runtime/references)
BENCHMARK_DEFINE_F(CassieDouble, MassMatrixViaID)(benchmark::State& state) {
  DoMassMatrixViaID(state);
}
BENCHMARK_REGISTER_F(CassieDouble, MassMatrixViaID)
    ->Unit(benchmark::kMicrosecond)
    ->Arg(kWantNoGrad);

// NOLINTNEXTLINE(runtime/references)
BENCHMARK_DEFINE_F(CassieDouble, MassMatrix)(benchmark::State& state) {
  DoMassMatrix(state);
}
BENCHMARK_REGISTER_F(CassieDouble, MassMatrix)
    ->Unit(benchmark::kMicrosecond)
    ->Arg(kWantNoGrad);

// NOLINTNEXTLINE(runtime/references)
BENCHMARK_DEFINE_F(CassieDouble, InverseDynamics)(benchmark::State& state) {
  DoInverseDynamics(state);
}
BENCHMARK_REGISTER_F(CassieDouble, InverseDynamics)
    ->Unit(benchmark::kMicrosecond)
    ->Arg(kWantNoGrad);

// NOLINTNEXTLINE(runtime/references)
BENCHMARK_DEFINE_F(CassieDouble, ForwardDynamics)(benchmark::State& state) {
  DoForwardDynamics(state);
}
BENCHMARK_REGISTER_F(CassieDouble, ForwardDynamics)
    ->Unit(benchmark::kMicrosecond)
    ->Arg(kWantNoGrad);

BENCHMARK_DEFINE_F(CassieAutoDiff, PositionKinematics)
// NOLINTNEXTLINE(runtime/references)
(benchmark::State& state) {
  DoPositionKinematics(state);
}
BENCHMARK_REGISTER_F(CassieAutoDiff, PositionKinematics)
    ->Unit(benchmark::kMicrosecond)
    ->Arg(kWantNoGrad)
    ->Arg(kWantGradQ);

BENCHMARK_DEFINE_F(CassieAutoDiff, PosAndVelKinematics)
// NOLINTNEXTLINE(runtime/references)
(benchmark::State& state) {
  DoPosAndVelKinematics(state);
}
BENCHMARK_REGISTER_F(CassieAutoDiff, PosAndVelKinematics)
    ->Unit(benchmark::kMicrosecond)
    ->Arg(kWantNoGrad)
    ->Arg(kWantGradQ)
    ->Arg(kWantGradV)
    ->Arg(kWantGradX);

// NOLINTNEXTLINE(runtime/references)
BENCHMARK_DEFINE_F(CassieAutoDiff, MassMatrix)(benchmark::State& state) {
  DoMassMatrix(state);
}
BENCHMARK_REGISTER_F(CassieAutoDiff, MassMatrix)
    ->Unit(benchmark::kMicrosecond)
    ->Arg(kWantNoGrad)
    ->Arg(kWantGradQ)
    ->Arg(kWantGradV)
    ->Arg(kWantGradX);

// NOLINTNEXTLINE(runtime/references)
BENCHMARK_DEFINE_F(CassieAutoDiff, InverseDynamics)(benchmark::State& state) {
  DoInverseDynamics(state);
}
BENCHMARK_REGISTER_F(CassieAutoDiff, InverseDynamics)
    ->Unit(benchmark::kMicrosecond)
    ->Arg(kWantNoGrad)
    ->Arg(kWantGradQ)
    ->Arg(kWantGradV)
    ->Arg(kWantGradX)
    ->Arg(kWantGradVdot)
    ->Arg(kWantGradQ | kWantGradVdot)
    ->Arg(kWantGradV | kWantGradVdot)
    ->Arg(kWantGradX | kWantGradVdot);

// NOLINTNEXTLINE(runtime/references)
BENCHMARK_DEFINE_F(CassieAutoDiff, ForwardDynamics)(benchmark::State& state) {
  DoForwardDynamics(state);
}
BENCHMARK_REGISTER_F(CassieAutoDiff, ForwardDynamics)
    ->Unit(benchmark::kMicrosecond)
    ->Arg(kWantNoGrad)
    ->Arg(kWantGradQ)
    ->Arg(kWantGradV)
    ->Arg(kWantGradX)
    ->Arg(kWantGradU)
    ->Arg(kWantGradQ | kWantGradU)
    ->Arg(kWantGradV | kWantGradU)
    ->Arg(kWantGradX | kWantGradU);

BENCHMARK_DEFINE_F(CassieExpression, PositionKinematics)
// NOLINTNEXTLINE(runtime/references)
(benchmark::State& state) {
  DoPositionKinematics(state);
}
BENCHMARK_REGISTER_F(CassieExpression, PositionKinematics)
    ->Unit(benchmark::kMicrosecond)
    ->Arg(kWantNoGrad)
    ->Arg(kWantGradQ);

BENCHMARK_DEFINE_F(CassieExpression, PosAndVelKinematics)
// NOLINTNEXTLINE(runtime/references)
(benchmark::State& state) {
  DoPosAndVelKinematics(state);
}
BENCHMARK_REGISTER_F(CassieExpression, PosAndVelKinematics)
    ->Unit(benchmark::kMicrosecond)
    ->Arg(kWantNoGrad)
    ->Arg(kWantGradQ)
    ->Arg(kWantGradV)
    ->Arg(kWantGradX);

// NOLINTNEXTLINE(runtime/references)
BENCHMARK_DEFINE_F(CassieExpression, MassMatrix)(benchmark::State& state) {
  DoMassMatrix(state);
}
BENCHMARK_REGISTER_F(CassieExpression, MassMatrix)
    ->Unit(benchmark::kMicrosecond)
    ->Arg(kWantNoGrad)
    ->Arg(kWantGradQ)
    ->Arg(kWantGradV)
    ->Arg(kWantGradX);

// NOLINTNEXTLINE(runtime/references)
BENCHMARK_DEFINE_F(CassieExpression, InverseDynamics)(benchmark::State& state) {
  DoInverseDynamics(state);
}
BENCHMARK_REGISTER_F(CassieExpression, InverseDynamics)
    ->Unit(benchmark::kMicrosecond)
    ->Arg(kWantNoGrad)
    ->Arg(kWantGradQ)
    ->Arg(kWantGradV)
    ->Arg(kWantGradX)
    ->Arg(kWantGradVdot)
    ->Arg(kWantGradQ | kWantGradVdot)
    ->Arg(kWantGradV | kWantGradVdot)
    ->Arg(kWantGradX | kWantGradVdot);

// NOLINTNEXTLINE(runtime/references)
BENCHMARK_DEFINE_F(CassieExpression, ForwardDynamics)(benchmark::State& state) {
  DoForwardDynamics(state);
}
BENCHMARK_REGISTER_F(CassieExpression, ForwardDynamics)
    ->Unit(benchmark::kMicrosecond)
    ->Arg(kWantNoGrad)
    // N.B. MbP does not support forward dynamics with Variables in 'q'.
    ->Arg(kWantGradV)
    ->Arg(kWantGradU)
    ->Arg(kWantGradV | kWantGradU);

}  // namespace
}  // namespace multibody
}  // namespace drake
