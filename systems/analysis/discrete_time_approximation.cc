#include "drake/systems/analysis/discrete_time_approximation.h"

#include <memory>

#include <unsupported/Eigen/MatrixFunctions>

#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/simulator_config_functions.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

template <typename T>
std::unique_ptr<LinearSystem<T>> DiscreteTimeApproximation(
    const LinearSystem<T>& system, double time_period) {
  // Check that the original system is continuous.
  DRAKE_THROW_UNLESS(system.IsDifferentialEquationSystem());
  // Check that the discrete time_period is greater than zero.
  DRAKE_THROW_UNLESS(time_period > 0);

  const int ns = system.num_states();
  const int ni = system.num_inputs();

  Eigen::MatrixXd M(ns + ni, ns + ni);
  M << system.A(), system.B(), Eigen::MatrixXd::Zero(ni, ns + ni);

  Eigen::MatrixXd Md = (M * time_period).exp();

  auto Ad = Md.block(0, 0, ns, ns);
  auto Bd = Md.block(0, ns, ns, ni);
  auto& Cd = system.C();
  auto& Dd = system.D();

  return std::make_unique<LinearSystem<T>>(Ad, Bd, Cd, Dd, time_period);
}

template <typename T>
std::unique_ptr<AffineSystem<T>> DiscreteTimeApproximation(
    const AffineSystem<T>& system, double time_period) {
  // Check that the original system is continuous.
  DRAKE_THROW_UNLESS(system.IsDifferentialEquationSystem());
  // Check that the discrete time_period is greater than zero.
  DRAKE_THROW_UNLESS(time_period > 0);

  const int ns = system.num_states();
  const int ni = system.num_inputs();

  Eigen::MatrixXd M(ns + ni + 1, ns + ni + 1);
  M << system.A(), system.B(), system.f0(),
      Eigen::MatrixXd::Zero(ni + 1, ns + ni + 1);

  Eigen::MatrixXd Md = (M * time_period).exp();

  auto Ad = Md.block(0, 0, ns, ns);
  auto Bd = Md.block(0, ns, ns, ni);
  auto f0d = Md.block(0, ns + ni, ns, 1);
  auto& Cd = system.C();
  auto& Dd = system.D();
  auto& y0d = system.y0();

  return std::make_unique<AffineSystem<T>>(Ad, Bd, f0d, Cd, Dd, y0d,
                                           time_period);
}

namespace {

template <typename T>
class DiscreteTimeSystem final : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DiscreteTimeSystem);

  DiscreteTimeSystem(std::unique_ptr<System<T>> system, double time_period,
                     double time_offset,
                     const SimulatorConfig& integrator_config)
      : LeafSystem<T>(SystemTypeTag<DiscreteTimeSystem>{}),
        continuous_system_(std::move(system)),
        continuous_context_model_value_(
            continuous_system_->CreateDefaultContext()),
        time_period_(time_period),
        time_offset_(time_offset),
        integrator_config_(integrator_config) {
    this->Initialize();
  }

  // Scalar-converting copy constructor. See @ref system_scalar_conversion.
  template <typename U>
  explicit DiscreteTimeSystem(const DiscreteTimeSystem<U>& other)
      : DiscreteTimeSystem<T>(
            other.continuous_system_->template ToScalarType<T>(),
            other.time_period_, other.time_offset_, other.integrator_config_) {}

 private:
  template <typename U>
  friend class DiscreteTimeSystem;

  void Initialize() {
    // Set name.
    const std::string& name = continuous_system_->get_name();
    this->set_name(name.empty() ? "discrete-time approximation"
                                : "discrete-time approximated " + name);

    // Create a cache entry for the continuous system context where we can
    // safely modify it without a (non-thread-safe) mutable member.
    continuous_context_cache_entry_ = &this->DeclareCacheEntry(
        "continuous system context", *continuous_context_model_value_,
        &DiscreteTimeSystem<T>::CopyDiscreteContextToContinuousContext,
        {SystemBase::all_sources_ticket()});

    // Declare input ports.
    for (int i = 0; i < continuous_system_->num_input_ports(); ++i) {
      const InputPort<T>& port = continuous_system_->get_input_port(i);
      this->DeclareInputPort(port.get_name(), port.get_data_type(), port.size(),
                             port.get_random_type());
    }

    // Declare output ports.
    for (int i = 0; i < continuous_system_->num_output_ports(); ++i) {
      const OutputPort<T>& port = continuous_system_->get_output_port(i);
      if (port.get_data_type() == PortDataType::kVectorValued) {
        this->DeclareVectorOutputPort(
            port.get_name(), port.size(),
            [this, &port](const Context<T>& discrete_context,
                          BasicVector<T>* out) {
              const Context<T>& continuous_context =
                  continuous_context_cache_entry_->template Eval<Context<T>>(
                      discrete_context);
              out->SetFromVector(port.Eval(continuous_context));
            });
      } else if (port.get_data_type() == PortDataType::kAbstractValued) {
        this->DeclareAbstractOutputPort(
            port.get_name(),
            [&port] {
              return port.Allocate();
            },
            [this, &port](const Context<T>& discrete_context,
                          AbstractValue* out) {
              const Context<T>& continuous_context =
                  continuous_context_cache_entry_->template Eval<Context<T>>(
                      discrete_context);
              port.Calc(continuous_context, out);
            });
      }
    }

    // Declare parameters.
    for (int i = 0;
         i < continuous_context_model_value_->num_numeric_parameter_groups();
         ++i) {
      this->DeclareNumericParameter(
          continuous_context_model_value_->get_numeric_parameter(i));
    }
    for (int i = 0;
         i < continuous_context_model_value_->num_abstract_parameters(); ++i) {
      this->DeclareAbstractParameter(
          continuous_context_model_value_->get_abstract_parameter(i));
    }

    // Declare state.
    this->DeclareDiscreteState(continuous_system_->num_continuous_states());
    this->DeclarePeriodicDiscreteUpdateEvent(
        time_period_, time_offset_, &DiscreteTimeSystem<T>::DiscreteUpdate);
  }

  void DiscreteUpdate(const Context<T>& discrete_context,
                      DiscreteValues<T>* out) const {
    Simulator<T> simulator(*continuous_system_);
    ApplySimulatorConfig(integrator_config_, &simulator);
    auto& integrator = simulator.get_mutable_integrator();

    std::unique_ptr<Context<T>> continuous_context =
        continuous_context_cache_entry_
            ->template Eval<Context<T>>(discrete_context)
            .Clone();
    // TODO(wei-chen): Make the integrator and the integrated context a cache
    // variable.
    integrator.reset_context(continuous_context.get());
    integrator.Initialize();

    integrator.IntegrateWithMultipleStepsToTime(continuous_context->get_time() +
                                                time_period_);
    out->get_mutable_vector().SetFromVector(
        continuous_context->get_continuous_state_vector().CopyToVector());
  }

  void CopyDiscreteContextToContinuousContext(
      const Context<T>& discrete_context,
      Context<T>* continuous_context) const {
    // Copy time.
    continuous_context->SetTime(discrete_context.get_time());
    // Copy state.
    continuous_context->SetContinuousState(
        discrete_context.get_discrete_state_vector().value());
    // Copy parameters.
    continuous_context->get_mutable_parameters().SetFrom(
        discrete_context.get_parameters());
    // Copy accuracy.
    continuous_context->SetAccuracy(discrete_context.get_accuracy());
    // Copy fixed input port values.
    for (int i = 0; i < continuous_system_->num_input_ports(); ++i) {
      continuous_context->FixInputPort(
          i, *(this->EvalAbstractInput(discrete_context, i)));
    }
  }

  const std::unique_ptr<const System<T>> continuous_system_;
  const std::unique_ptr<const Context<T>> continuous_context_model_value_;
  const CacheEntry* continuous_context_cache_entry_;
  const double time_period_;
  const double time_offset_;
  const SimulatorConfig integrator_config_;
};

}  // namespace

namespace scalar_conversion {
template <>
struct Traits<DiscreteTimeSystem> : public NonSymbolicTraits {};
}  // namespace scalar_conversion

template <typename T>
std::unique_ptr<System<T>> DiscreteTimeApproximation(
    const System<T>& system, double time_period, double time_offset,
    const SimulatorConfig& integrator_config) {
  DRAKE_THROW_UNLESS(system.IsDifferentialEquationSystem());
  DRAKE_THROW_UNLESS(time_period > 0);
  return std::make_unique<DiscreteTimeSystem<T>>(
      system.Clone(), time_period, time_offset, integrator_config);
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    (static_cast<std::unique_ptr<LinearSystem<T>> (*)(
         const LinearSystem<T>&, double)>(&DiscreteTimeApproximation<T>),
     static_cast<std::unique_ptr<AffineSystem<T>> (*)(
         const AffineSystem<T>&, double)>(&DiscreteTimeApproximation<T>)));

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS((
    static_cast<std::unique_ptr<System<T>> (*)(const System<T>&, double, double,
                                               const SimulatorConfig&)>(
        &DiscreteTimeApproximation<T>)));

}  // namespace systems
}  // namespace drake
